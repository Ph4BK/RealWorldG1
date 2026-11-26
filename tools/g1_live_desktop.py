#!/usr/bin/env python3
"""G1-only desktop supervisor. VNC/websocket bind loopback; access via SSH tunnel."""
import argparse
import fcntl
import os
from pathlib import Path
import signal
import socket
import subprocess
import time


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--site', choices=('803', '619'), default='803',
                        help='Select the map and matching runtime configuration (default: 803)')
    parser.add_argument('--map-dir', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--obstacle-mode', choices=('dynamic', 'static_test'), default='dynamic')
    parser.add_argument('--record-stg-errors', action='store_true')
    parser.add_argument('--record-experiment', action='store_true')
    return parser.parse_args(argv)


def interface_command(root, args):
    """Resolve one map/config pair before starting desktop processes."""
    map_name = '619_go2base' if args.site == '619' else 'full_lab_v1'
    map_dir = args.map_dir or root/'maps'/map_name
    # The original 619 installation keeps maps beside RealWorldG1.
    legacy_map = root.parent/'maps'/map_name
    if args.map_dir is None and args.site == '619' and not map_dir.exists() and legacy_map.is_dir():
        map_dir = legacy_map
    config = args.config
    if config is None:
        runtime_config = map_dir/'runtime_live_bev.yaml'
        config = runtime_config if args.site == '619' or runtime_config.is_file() else root/'fastlio/config/live_bev.yaml'
    map_dir, config = map_dir.resolve(), config.resolve()
    required = [map_dir/name for name in ('map.pcd', 'bev_occupancy.npz', 'bev_config.json', 'metadata.yaml')]
    if args.site == '619':
        required.append(map_dir/'no_go.npz')
    missing = [str(path) for path in [*required, config] if not path.is_file()]
    if missing:
        raise SystemExit('Missing PosNav map/config files: ' + ', '.join(missing))
    command = ['bash', str(root/'tools/run_live_bev.sh'),
               '--map-dir', str(map_dir), '--config', str(config),
               '--obstacle-mode', args.obstacle_mode]
    if args.record_stg_errors:
        command.append('--record-stg-errors')
    if args.record_experiment:
        command.append('--record-experiment')
    return command


def wait_port(port, child, timeout=15):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if child.proc.poll() is not None:
            raise RuntimeError(f'Desktop component exited: {child.proc.returncode}')
        try:
            with socket.create_connection(('127.0.0.1',port),timeout=.2):return
        except OSError:time.sleep(.1)
    raise RuntimeError(f'Local desktop port {port} did not become ready')


def main():
    args=parse_args()
    root=Path(__file__).resolve().parents[1]
    app_command=interface_command(root, args)
    print('PosNav site=%s; map=%s; config=%s; obstacle_mode=%s' %
          (args.site, app_command[3], app_command[5], args.obstacle_mode), flush=True)
    from live_bev_node import Child
    run=root/'.cache/live_bev';run.mkdir(parents=True,exist_ok=True)
    lock=(root/'.cache/desktop.lock').open('w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('G1 live desktop is already running')
    for port in [5901,6081]:
        with socket.socket() as sock:
            # Match the servers' bind behavior: TIME_WAIT is not a live listener.
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:sock.bind(('127.0.0.1',port))
            except OSError:raise SystemExit(f'Port {port} is in use; refusing to replace another server')
    display=None
    for candidate in range(93,100):
        if not Path(f'/tmp/.X{candidate}-lock').exists() and not Path(f'/tmp/.X11-unix/X{candidate}').exists():
            display=candidate;break
    if display is None:raise SystemExit('No free desktop display')
    children=[]
    record_experiment=args.record_experiment
    def launch(name,args):
        prefixes=('EXPERIMENT_',) if name=='interface' and record_experiment else ()
        child=Child(args,run/(name+'.log'),relay_prefixes=prefixes);children.append(child);return child
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    signal.signal(signal.SIGINT,interrupted)
    signal.signal(signal.SIGHUP,interrupted)
    try:
        server=launch('xvfb',['Xvfb',f':{display}','-screen','0','1280x900x24','-nolisten','tcp'])
        deadline=time.monotonic()+10
        while not Path(f'/tmp/.X11-unix/X{display}').exists():
            if time.monotonic()>deadline or server.proc.poll() is not None:raise RuntimeError('Xvfb failed to start')
            time.sleep(.1)
        os.environ['DISPLAY']=f':{display}'
        runtime=run/'xdg';runtime.mkdir(mode=0o700,exist_ok=True);runtime.chmod(0o700)
        os.environ['XDG_RUNTIME_DIR']=str(runtime)
        launch('openbox',['openbox','--sm-disable'])
        vnc=launch('vnc',['x11vnc','-display',f':{display}','-listen','127.0.0.1','-rfbport','5901',
                          '-forever','-shared','-nopw','-noxdamage','-noshm','-noxrecord','-no6'])
        wait_port(5901,vnc)
        web=launch('web',['websockify','--web=/usr/share/novnc','127.0.0.1:6081','127.0.0.1:5901'])
        wait_port(6081,web)
        app=launch('interface',app_command)
        print('READY: tunnel laptop 6081 -> G1 127.0.0.1:6081; open http://127.0.0.1:6081/vnc.html?autoconnect=1&resize=scale',flush=True)
        while all(c.proc.poll() is None for c in children):time.sleep(.25)
        if any(c.proc.poll() is not None for c in children if c is not app):
            raise RuntimeError('Desktop component exited; inspect desktop logs')
        if app.proc.poll() not in (None,0):raise RuntimeError('Live BEV interface exited; inspect interface.log')
    except KeyboardInterrupt:pass
    finally:
        for child in reversed(children):child.close(timeout=35 if args.record_experiment else 5)
        lock.close()

if __name__=='__main__':main()
