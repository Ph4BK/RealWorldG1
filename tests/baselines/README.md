# Historical compatibility fixtures

These files are immutable source snapshots used only by regression tests. They
are not runtime modules, launch alternatives or deployment payloads.

- `fine_previous.py`: tested 2026-11-24 final-approach implementation before the layered stop candidate.
- `fine_v2.py`: tested layered-stop implementation before entry alignment and lateral protection.
- `core_v3r2.py`: tested navigation core before the near-STG latch and coarse-turn profile.
- `round1_posnav_fmm/`: tested planner before the later-STG clearance gate.
- `field_planner.py`: robot snapshot before the planning/stop-band connection repair.

`manifest.json` records each source SHA256 after LF normalization. Tests compare
the real current controller or planner against these sources when the new
features are disabled. `frozen_baselines.baseline_path` defaults to these files;
the explicit `G1_*_BASELINE` overrides remain available for additional comparisons.
The field comparison also accepts the existing `G1_FIELD_SNAPSHOT` directory.

ROS tests still require their documented ROS 2 and Unitree interfaces. Historical
code is kept as data for the tests and is never installed by the navigation
packager. Raw robot logs, video and scene datasets are not fixtures here.
