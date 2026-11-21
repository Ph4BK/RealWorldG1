"""Rebuild the formal BEV from its retained PCD and JSON configuration.

Example: python RealWorldG1/tools/build_static_bev.py --output-dir /tmp/g1-bev
Outputs are generated only on request; existing outputs are not overwritten.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import shutil

import numpy as np


def decompress_lzf(data, expected_size):
    """Decode bounded LZF literals/back-references used by PCL binary_compressed.

    Format reference: PointCloudLibrary/pcl doc/tutorials/content/pcd_file_format.rst.
    This is an offline reader, not a sensor/control-loop dependency.
    """
    output = bytearray()
    index = 0
    while index < len(data):
        control = data[index]
        index += 1
        if control < 32:
            length = control + 1
            if index + length > len(data) or len(output) + length > expected_size:
                raise ValueError('Invalid LZF literal length')
            output.extend(data[index:index + length])
            index += length
        else:
            length = control >> 5
            offset = (control & 31) << 8
            if length == 7:
                if index >= len(data):
                    raise ValueError('Truncated LZF back-reference')
                length += data[index]
                index += 1
            if index >= len(data):
                raise ValueError('Truncated LZF offset')
            offset += data[index] + 1
            index += 1
            length += 2
            if offset > len(output) or len(output) + length > expected_size:
                raise ValueError('Invalid LZF back-reference range')
            for _ in range(length):
                output.append(output[-offset])  # Overlap is intentional in LZF.
    if len(output) != expected_size:
        raise ValueError('LZF decoded size differs from PCD header')
    return output


def read_pcd_xyz(source):
    """Read binary/ascii/PCL-LZF PCD without a ROS/Open3D dependency.

    The retained maps are little-endian binary PCD. Reject other encodings
    explicitly; do not mistake compressed payloads or malformed layouts for XYZ.
    """
    with Path(source).open('rb') as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD has no DATA header')
            words = line.decode('ascii').strip().split()
            if not words or words[0].startswith('#'):
                continue
            header[words[0].upper()] = words[1:]
            if words[0].upper() == 'DATA':
                break
        fields = header['FIELDS']
        sizes = list(map(int, header['SIZE']))
        types = header['TYPE']
        counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
        if not (len(fields) == len(set(fields)) == len(sizes) == len(types) == len(counts)):
            raise ValueError('Invalid PCD field layout')
        if any(n < 1 for n in counts):
            raise ValueError('Invalid PCD COUNT')
        for axis in ('x', 'y', 'z'):
            if axis not in fields or counts[fields.index(axis)] != 1:
                raise ValueError('PCD requires scalar x/y/z fields')
        number = int(header['POINTS'][0])
        if number <= 0 or number != int(header['WIDTH'][0]) * int(header['HEIGHT'][0]):
            raise ValueError('Invalid PCD dimensions')
        encoding = header['DATA'][0]
        payload = stream.read()
    if encoding in ('binary', 'binary_compressed'):
        formats = {('F', 4): '<f4', ('F', 8): '<f8',
                   ('I', 1): 'i1', ('I', 2): '<i2', ('I', 4): '<i4',
                   ('U', 1): 'u1', ('U', 2): '<u2', ('U', 4): '<u4'}
        try:
            dtype = np.dtype([(name, formats[(kind, size)], (count,))
                              for name, kind, size, count in zip(fields, types, sizes, counts)])
        except KeyError as exc:
            raise ValueError('Unsupported PCD field type/size') from exc
        if encoding == 'binary_compressed':
            if len(payload) < 8:
                raise ValueError('Truncated compressed PCD header')
            compressed, uncompressed = struct.unpack('<II', payload[:8])
            if compressed != len(payload) - 8 or uncompressed != number * dtype.itemsize:
                raise ValueError('Compressed PCD sizes differ from POINTS/layout')
            decoded = decompress_lzf(payload[8:], uncompressed)
            offset = 0
            xyz = {}
            # PCL stores all X values, then all Y values, etc. (SoA), not AoS.
            for name, size, kind, count in zip(fields, sizes, types, counts):
                if name in ('x', 'y', 'z'):
                    xyz[name] = np.frombuffer(decoded, dtype=formats[(kind, size)],
                                              count=number, offset=offset)
                offset += number * size * count
            return np.column_stack([xyz[axis] for axis in ('x', 'y', 'z')]).astype(float)
        if len(payload) != number * dtype.itemsize:
            raise ValueError('PCD binary payload size differs from POINTS')
        data = np.frombuffer(payload, dtype=dtype, count=number)
        return np.column_stack([data[axis][:, 0] for axis in ('x', 'y', 'z')]).astype(float)
    if encoding == 'ascii':
        data = np.loadtxt(payload.decode('ascii').splitlines(), ndmin=2)
        if data.shape != (number, sum(counts)):
            raise ValueError('PCD ASCII shape differs from header')
        offsets = np.cumsum([0] + counts[:-1])
        return data[:, [offsets[fields.index(axis)] for axis in ('x', 'y', 'z')]]
    raise ValueError('Unsupported PCD DATA encoding: ' + encoding)


def project_bev(points, config):
    """Shared offline projection; legacy histogram maps keep their exact edges."""
    points = np.asarray(points, dtype=float)
    low, high = config['height_range_m_inclusive']
    selected = points[np.isfinite(points).all(axis=1) &
                      (points[:, 2] >= low) & (points[:, 2] <= high)]
    res = float(config['resolution_m'])
    origin = np.asarray(config['origin_xy_m'], dtype=float)
    shape = tuple(config['shape_rows_cols'])
    threshold = config['min_points_per_cell']
    if (points.ndim != 2 or points.shape[1] != 3 or
            not np.isfinite([low, high, res]).all() or low > high or res <= 0 or
            origin.shape != (2,) or not np.isfinite(origin).all() or
            len(shape) != 2 or any(not isinstance(v, (int, np.integer)) or v <= 0 for v in shape) or
            not isinstance(threshold, (int, np.integer)) or isinstance(threshold, bool) or threshold < 1):
        raise ValueError('Invalid BEV projection configuration')
    if config.get('projection_method') not in (None, 'runtime_floor_half_open_v1'):
        raise ValueError('Unknown BEV projection method')
    if config.get('projection_method') == 'runtime_floor_half_open_v1':
        indices = np.floor((selected[:, :2] - origin) / res).astype(np.int64)
        valid = ((indices[:, 0] >= 0) & (indices[:, 0] < shape[1]) &
                 (indices[:, 1] >= 0) & (indices[:, 1] < shape[0]))
        counts = np.zeros(shape, dtype=np.int32)
        np.add.at(counts, (indices[valid, 1], indices[valid, 0]), 1)
        outside = int((~valid).sum())
        if outside != config['expected_height_points_outside_grid']:
            raise ValueError('Unexpected points outside the approved projection ROI')
    else:
        # Preserve the approved 803 map's original np.arange/histogram policy.
        edges = [np.arange(start, stop, res) for start, stop in
                 zip(origin, config['histogram_arange_stop_xy_m'])]
        counts = np.histogram2d(selected[:, 0], selected[:, 1], bins=edges)[0].T.astype(np.int32)
        outside = len(selected) - int(counts.sum())
        if outside:
            raise ValueError('Points outside the fixed map bounds')
    if counts.shape != shape:
        raise ValueError('Unexpected grid shape')
    return counts >= config['min_points_per_cell'], counts, dict(
        height_selected_points=len(selected), points_in_grid=int(counts.sum()),
        points_outside_grid=outside)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-dir', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'maps/full_lab_v1')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--counts', action='store_true')
    parser.add_argument('--preview', action='store_true')
    args = parser.parse_args()
    config = json.loads((args.map_dir / 'bev_config.json').read_text())
    source = args.map_dir / config['source']
    if hashlib.sha256(source.read_bytes()).hexdigest() != config['source_sha256']:
        raise ValueError('PCD checksum differs from the approved static map')
    names = ['bev_occupancy.npz']
    if args.counts:
        names.append('bev_counts.npz')
    if args.preview:
        names += ['bev_binary.png', 'bev_preview.png']
    for name in names:
        if (args.output_dir / name).exists():
            raise FileExistsError(args.output_dir / name)
    low, high = config['height_range_m_inclusive']
    res = config['resolution_m']
    origin = config['origin_xy_m']
    inherited = config.get('bev_source') == 'go2-grid'
    if inherited:
        if args.counts:
            raise ValueError('Cleaned occupancy has no point-count grid; refuse fabricated counts')
        grid_source = args.map_dir / config['bev_source_file']
        if hashlib.sha256(grid_source.read_bytes()).hexdigest() != config['bev_source_sha256']:
            raise ValueError('Cleaned grid checksum mismatch')
        with np.load(grid_source, allow_pickle=False) as grid:
            occ = grid['occ'].copy()
            if (occ.shape != tuple(config['shape_rows_cols']) or
                    not np.array_equal(grid['origin'], origin) or float(grid['res']) != res):
                raise ValueError('Cleaned grid geometry mismatch')
        projection = dict(height_selected_points=None, points_outside_grid=None)
    else:
        points = read_pcd_xyz(source)
        occ, counts, projection = project_bev(points, config)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if inherited:
        shutil.copyfile(grid_source, args.output_dir / names[0])
    else:
        np.savez_compressed(args.output_dir / names[0], occ=occ, origin=origin, res=res)
    if args.counts:
        np.savez_compressed(args.output_dir / 'bev_counts.npz', counts=counts,
                            origin=origin, res=res, z_range=[low, high])
    if args.preview:
        import os
        os.environ.setdefault('MPLCONFIGDIR', '/tmp/unigoal-matplotlib')
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from PIL import Image
        Image.fromarray(np.flipud(np.where(occ, 0, 255).astype(np.uint8))).save(
            args.output_dir / 'bev_binary.png')
        fig, ax = plt.subplots(figsize=(10, 10), layout='constrained')
        height, width = occ.shape
        ax.imshow(occ, origin='lower', cmap='gray_r', vmin=0, vmax=1,
                  interpolation='nearest', extent=[origin[0], origin[0] + width * res,
                                                   origin[1], origin[1] + height * res])
        ax.set(xlabel='Map X (m)', ylabel='Map Y (m)',
               title=(f'{args.map_dir.name} | Go2 cleaned grid, inherited without reprojection\n' if inherited else
                      f'{args.map_dir.name} | {low} <= Z <= {high} m | count >= {config["min_points_per_cell"]}\n') +
                     'Black: obstacle; white: free | no inflation')
        ax.set_aspect('equal')
        fig.savefig(args.output_dir / 'bev_preview.png', dpi=180)
        plt.close(fig)
    print(json.dumps(dict(selected_points=projection['height_selected_points'],
                          points_outside_grid=projection['points_outside_grid'], obstacle_cells=int(occ.sum()),
                          total_cells=int(occ.size), output=str(args.output_dir))))


if __name__ == '__main__':
    main()
