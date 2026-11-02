#!/usr/bin/env bash
# Single SSH connection owns both the G1 desktop and its loopback-only browser tunnel.
set -eo pipefail
G1_RECORD_OPTION=''
for G1_RECORD_ARG in "$@"; do
  case "$G1_RECORD_ARG" in
    --record-stg-errors|--record-experiment) G1_RECORD_OPTION+=" $G1_RECORD_ARG" ;;
    *) echo 'Usage: run_g1_live_bev.sh [--record-stg-errors] [--record-experiment]' >&2; exit 2 ;;
  esac
done
G1_TARGET=${G1_TARGET:-unitree@192.168.123.164}
# Use Windows SSH when available so the browser tunnel terminates in Windows,
# independent of WSL's NAT/mirrored mode and localhost forwarding.
if command -v ssh.exe >/dev/null 2>&1; then
  G1_SSH=ssh.exe
else
  G1_SSH=ssh
fi
printf '%s\n' 'After READY, open: http://127.0.0.1:6081/vnc.html?autoconnect=1&resize=scale'
printf '%s\n' 'Keep this terminal open. Closing RViz or pressing Ctrl+C stops this desktop session.'
exec "$G1_SSH" -t -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
  -L 127.0.0.1:6081:127.0.0.1:6081 "$G1_TARGET" \
  "docker start g1-posnav-humble >/dev/null && docker exec -it g1-posnav-humble bash /workspace/RealWorldG1/posnav/deploy/start_desktop.sh${G1_RECORD_OPTION}"
