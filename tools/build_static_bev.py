"""Rebuild the formal BEV from its retained PCD and JSON configuration.

Example: python RealWorldG1/tools/build_static_bev.py --output-dir /tmp/g1-bev
Outputs are generated only on request; existing outputs are not overwritten.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import open3d as o3d


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
    points = np.asarray(o3d.io.read_point_cloud(str(source)).points)
    low, high = config['height_range_m_inclusive']
    selected = points[np.isfinite(points).all(axis=1) &
                      (points[:, 2] >= low) & (points[:, 2] <= high)]
    res = config['resolution_m']
    origin = config['origin_xy_m']
    # Preserve the exact histogram edges used for the approved preview.
    edges = [np.arange(start, stop, res) for start, stop in
             zip(origin, config['histogram_arange_stop_xy_m'])]
    counts = np.histogram2d(selected[:, 0], selected[:, 1], bins=edges)[0].T.astype(np.int32)
    if list(counts.shape) != config['shape_rows_cols'] or counts.sum() != len(selected):
        raise ValueError('Unexpected grid shape or points outside the fixed map bounds')
    occ = counts >= config['min_points_per_cell']
    args.output_dir.mkdir(parents=True, exist_ok=True)
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
               title=f'full_lab_v1 | {low} <= Z <= {high} m | count >= {config["min_points_per_cell"]}\n'
                     'Black: obstacle; white: free | no inflation')
        ax.set_aspect('equal')
        fig.savefig(args.output_dir / 'bev_preview.png', dpi=180)
        plt.close(fig)
    print(json.dumps(dict(selected_points=len(selected), obstacle_cells=int(occ.sum()),
                          total_cells=int(occ.size), output=str(args.output_dir))))


if __name__ == '__main__':
    main()
