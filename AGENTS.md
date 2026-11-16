# Simulation project rules

- All Linux experiment outputs intended for inspection or manual export belong under
  `/home/legion/AUV-simulation`: runs in `results/`, figures in `figures/`, reports in `reports/`.
- Container runs must bind-mount that host directory; a private container directory
  with the same name does not satisfy the requirement. Do not silently fall back to `/root` or `/tmp`.
- Keep output readable by the host desktop user and preserve existing runs.
- Source code may remain separate from results. Do not upload experiment results,
  X11 authentication files, local Docker state, or checkpoints to Git without an explicit request.
- The current vehicle is an uncalibrated engineering demonstrator. Storage changes
  must not change physics, observations, actions, rewards, or mission criteria.
- For Chapter 3 integration, follow `crk_auv_ch3_v1/IMPLEMENTATION_PLAN.md` in order.
  Validate each new module before implementing a dependent module. A pending Linux
  or GUI gate requires actual evidence; local XML/unit tests cannot satisfy it.
  G3 passed Linux SDF/loading validation and user visual confirmation on 2026-11-16.
  G4 motion/control is authorized. G5+ must wait for real Linux G4 evidence.
