# G1 localization dependencies

`FAST_LIO_LOCALIZATION_HUMANOID/` is an unmodified upstream dependency managed
as a fixed-version Git submodule:

- URL: https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID.git
- Upstream branch of origin: `humble` (not automatically tracked)
- Commit: `df4772ec4797172430e7efe990711d09a529f4ad`
- Downloaded: 2026-09-22

The parent repository records the exact commit as a gitlink; `.gitmodules`
records its path and upstream URL. Keep the gitlink and `.gitmodules` together.
Do not replace the dependency with a tracking-branch update or copy its Git
history and media into the parent repository as ordinary files.

Clone this project with `git clone --recurse-submodules`, or run the following
from the `RealWorldG1` repository root after an ordinary clone:

```bash
git submodule update --init --recursive
git -C third_party/FAST_LIO_LOCALIZATION_HUMANOID rev-parse HEAD
```

The second command must print the pinned commit above. Keep the submodule
working tree unmodified. Copy the needed source to a build workspace before
applying the production patches, in this order:

1. `../fastlio/patches/unitree_pointcloud2_and_build.patch`
2. `../fastlio/patches/localization_startup.patch`
3. `../fastlio/patches/localization_live_status.patch`

Patch paths above are relative to this README. Apply them at the root of the
copied upstream tree, containing `FAST_LIO/` and `open3d_loc/`.
`static_diagnostics.patch` is for historical diagnostics only; do not include
it in the production patch sequence. On 2026-10-20, the three production patches
were applied to the pinned upstream in a temporary directory, and all seven
affected source files matched the existing local build-workspace source.

The current packager reads patched source from `.cache/fastlio_ws/src/` and
performs additional Humble/Open3D compatibility changes for G1. The deployed
localization snapshot is retained at `../posnav/source/open3d_loc/`; it is
generated from the upstream plus patches and packaging changes, not a second
independently maintained upstream.

G1 bag-specific configuration lives in `../config/fastlio_g1_pointcloud2.yaml`.

The upstream `humble` checkout does **not** include `livox_ros_driver2`, although
FAST_LIO still requires that package at compile time, even for PointCloud2 input.
Mapping and localization build/runtime requirements are recorded in
`../docs/G1_FAST_LIO离线建图与定位.md`.
