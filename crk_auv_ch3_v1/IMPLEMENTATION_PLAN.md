# CH3-Gazebo-v1 staged implementation

Each gate must pass before work proceeds to a dependent module. Failed checks are
fixed within their stage; they are not hidden by changing rewards or task rules.
Local tests do not establish ROS/Gazebo acceptance or research performance.

| Gate | Module | Required evidence | Status |
| --- | --- | --- | --- |
| G0 | Scope and immutable source baseline | Source inventory, contract, protected-directory before hash | Complete |
| G1 | Experiment and source contracts | Valid/invalid config tests, integral time steps, no checkpoint loading | Local checks passed: 3 tests; 248 source files locked |
| G2 | Coordinates and native scene inputs | Round trips, scene consistency, negative cases, no information sent to actor | Local checks passed: 4 tests |
| G3 | Static Gazebo scene preview | XML tests, Linux `gz sdf -k`, visual inspection of four role markers and obstacles | Passed: user Linux log and visual confirmation, 2026-11-16 |
| G4 | Physical AUV and synchronous world stepping | Model provenance/geometry checks, Linux plugins, four-state barrier, actuator signs | Implemented with literature ROV reference; local checks passed; Linux motion acceptance pending |
| G5 | Source-pinned mission port | Recorded-state parity of maps, events, observations, action composition, rewards, termination | Not started |
| G6 | ZERO four-vehicle closed loop | Linux search/discovery/handoff/capture, task/contact/engineering labels | Not started |
| G7 | B2, then B3 inference | Selected checkpoint/config hashes, native loading, four-agent context and gating parity | Not started |
| G8 | Paired evaluation | Fixed scenes/seeds/configs, complete outcome accounting, reproducible reports | Not started |

First baseline: original D2, ZERO residual, original four roles and 28D/3D
interfaces. The source configs and their identity are recorded, not interpreted as
validated Gazebo dynamics. B2/B3 and coupled-search variants remain unavailable
until their gates pass. No automatic training or checkpoint selection/loading.

Python retains sensing, shared maps, target motion, fixed reliable handoff,
task/reward/termination logic and policy. Gazebo eventually owns actual vehicle
motion. Preserve original step/event ordering, never integrate vehicle positions
twice, never expose unknown target/obstacle truth to policies. Measure commanded
and actual dynamics separately. Contact and original task collision remain distinct.

G3 deliberately uses static geometric role markers, not a real AUV model. It tests
scene transport only. G4 now uses a documented BlueROV2 Heavy literature-parameter
control reference with declared engineering approximations (G4_MOTION.md), not the
old uncalibrated layout or a validated AUV digital twin. No map/mission actor runs yet.

Results live on the Linux host under `/home/legion/AUV-simulation/results/ch3/`.
Figures/reports share that root. Keep the single-vehicle example operational.
Never modify AUV-Thesis-Active, protected CH3/CH4/CH5, old checkpoints or results.
The G1 source lock records code identity only; it is not an imported runtime snapshot
or permission to bypass the original provenance checks during later model loading.
