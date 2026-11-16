# Chapter 3 staged Gazebo integration

The implementation plan and gate status are in [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md).
Current delivery contains G1 contracts/source identity, G2 scene/frame conversion,
the accepted G3 static scene preview and the new G4 four-vehicle motion/control test.
G3 passed user Linux SDF/loading checks and visual confirmation on 2026-11-16.
See [G4_MOTION.md](G4_MOTION.md) for the moving-vehicle entry point and parameter provenance.
G4 local checks pass, but Linux motion acceptance is pending. G5 and later mission/map/
learning modules are not implemented and must wait for G4. No checkpoints are loaded.

## Local verification

From this directory, using Python 3.10 or newer:

```bash
python3 -m unittest discover -s tests -v
sha256sum -c SHA256SUMS
```

The source lock is an inventory, not a runtime snapshot. It records 248 source
files from AUV-Thesis-Active. No live imports from that checkout or source changes
are made. The source identity tool only reads the checkout, compares original
constructor dt/roles and config horizon/map-sharing, and exclusively creates a new
lock file; do not replace the committed lock to conceal changes.

## Linux gate G3

On the Ubuntu host, stop any running old simulation, update the source clone:

```bash
git -C /home/legion/AUV-simulation/source pull --ff-only
bash /home/legion/AUV-simulation/source/tools/run_ch3_scene_check.sh --gui
```

The host wrapper reuses/configures crk-ros2-home and its output bind mount. It does
not start the single-vehicle controller. It runs all three sets of tests, generates
a new preview and invokes `gz sdf -k`, saving the exit code and output before opening
Gazebo. Without --gui, only the validation and export are performed.
The validation script explicitly sources `/opt/ros/jazzy/setup.bash` in its own
shell before looking up gz, so ROS-vendored executable/library/plugin paths are
available. Setup loaded by a previous build subprocess is not inherited by a
subsequent docker exec. Missing ROS setup and missing gz after setup have separate
error messages; a previous successful build does not require reinstalling Gazebo.

Expected: four static colored box markers (three search roles and an executor),
two brown obstacles, a yellow target marker and a cyan standby marker. Whole native
20 x 20 x 8 m workspace is rigidly translated by (-10,-10,-9), with no scaling.
The markers will not move even if Play is pressed: no AUV dynamics, sensors or
actor runs in this preview. A blue background does not simulate optical water effects.
The scene is explicitly an engineering fixture, not a generated thesis evaluation case.
Use Ctrl+C in the launching terminal when inspection is finished.

Inspect roles/obstacles/target for correct relative positions and check the saved
SDF-validation log. GUI success must be confirmed by the user; it is never inferred
from launching a process. Do not mark G3 complete or begin G4 based on XML tests alone.

Output directory:

```text
/home/legion/AUV-simulation/results/ch3/scenecheck_.../
  world.sdf
  scenario.json
  truth_scene.json
  resolved_config.json
  sdf_validation.log
  summary.json
```

summary.json must report linux_sdf_valid=true after validation; mission_executed,
checkpoint_loaded, performance_claim and gui_confirmed remain false. This is a
preflight report, not the task success summary from the single-boat example.
truth_scene.json contains privileged simulator geometry and must never be passed
to the map or actor as observations.

## Importing native scenes later

The converter consumes original `initial_agent_positions`, `initial_executor_wait_point`,
target initial position/velocity and AABB obstacles, without resampling. Use an
explicit native scenario manifest and select its scenario ID when multiple are present:

```bash
python3 -m crk_ch3.preview --manifest /mounted/path/scenarios.json --scenario-id ID
```

This command only exports static geometry and does not replace original scenario
admission, feasibility checks or source/checkpoint provenance. Geometry is transformed
consistently: positions rotate/translate, velocities only rotate, box sizes stay
unchanged and box poses carry rotation. No map, task, reward or algorithm runtime
is implemented by this converter.
