#!/usr/bin/env python3
"""G1-only desktop supervisor. VNC/websocket bind loopback; access via SSH tunnel."""
import fcntl
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
from live_bev_node import Child


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
    root=Path(__file__).resolve().parents[1]
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
    def launch(name,args):
        child=Child(args,run/(name+'.log'));children.append(child);return child
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
        app=launch('interface',['bash',str(root/'tools/run_live_bev.sh')])
        print('READY: tunnel laptop 6081 -> G1 127.0.0.1:6081; open http://127.0.0.1:6081/vnc.html?autoconnect=1&resize=scale',flush=True)
        while all(c.proc.poll() is None for c in children):time.sleep(.25)
        if any(c.proc.poll() is not None for c in children if c is not app):
            raise RuntimeError('Desktop component exited; inspect desktop logs')
        if app.proc.poll() not in (None,0):raise RuntimeError('Live BEV interface exited; inspect interface.log')
    except KeyboardInterrupt:pass
    finally:
        for child in reversed(children):child.close()
        lock.close()

if __name__=='__main__':main()
