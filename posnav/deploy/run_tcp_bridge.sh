#!/usr/bin/env bash
# Explicit opt-in forwarding; this script never starts navigation or sends a goal.
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source "$ROOT/posnav/deploy/env.sh"
source "$ROOT/.cache/nav_interfaces_ws/install/setup.bash"
G1_BRIDGE_CONFIG=${G1_BRIDGE_CONFIG:-$ROOT/fastlio/config/live_bev.yaml}
G1_BRIDGE_RUNTIME=${G1_BRIDGE_RUNTIME:-$ROOT/.cache/tcp_bridge}
mkdir -p "$G1_BRIDGE_RUNTIME"
if [[ ! -f "$G1_BRIDGE_RUNTIME/.token" ]]; then
  (umask 077; /usr/bin/python3 -c 'import secrets; print(secrets.token_urlsafe(32))' > "$G1_BRIDGE_RUNTIME/.token")
fi
export FIELD_BRIDGE_TOKEN="$(cat "$G1_BRIDGE_RUNTIME/.token")"
G1_BRIDGE_FRAME=$(/usr/bin/python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))["action"]["coordinate_frame"])' "$G1_BRIDGE_CONFIG")
exec /usr/bin/python3 "$ROOT/posnav/bridge/tcp_bridge.py" \
  --bind "${G1_BRIDGE_BIND:-192.168.123.164}" --port "${G1_BRIDGE_PORT:-8888}" \
  --scene-frame "$G1_BRIDGE_FRAME" --runtime-dir "$G1_BRIDGE_RUNTIME" "$@"
