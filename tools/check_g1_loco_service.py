#!/usr/bin/env python3
"""Issue only the five verified read-only G1 locomotion queries."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from RealWorldG1.src.g1_api_client import G1ReadOnlyLocoClient
from RealWorldG1.src.ros2_sport_transport import Ros2SportTransport


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check G1 APIs 7001-7005 without sending motion commands."
    )
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--max-pending", type=int, default=4)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    transport = Ros2SportTransport(depth=max(16, args.max_pending * 2))
    with G1ReadOnlyLocoClient(transport, max_pending=args.max_pending) as client:
        queries = [
            ("fsm_id", client.get_fsm_id_async),
            ("fsm_mode", client.get_fsm_mode_async),
            ("balance_mode", client.get_balance_mode_async),
            ("swing_height", client.get_swing_height_async),
            ("stand_height", client.get_stand_height_async),
        ]
        results = {}
        failed = False
        # Keep no more futures in flight than the configured capacity by
        # running the small fixed query set in capacity-sized batches.
        for start in range(0, len(queries), args.max_pending):
            batch = queries[start : start + args.max_pending]
            futures = {
                name: query(timeout=args.timeout) for name, query in batch
            }
            for name, future in futures.items():
                try:
                    results[name] = {"ok": True, "value": future.result()}
                except Exception as exc:
                    failed = True
                    results[name] = {
                        "ok": False,
                        "error": type(exc).__name__,
                        "message": str(exc),
                    }
    print(json.dumps(results, ensure_ascii=False, indent=2, sort_keys=True))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
