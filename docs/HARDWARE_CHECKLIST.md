# Separately gated hardware qualification (UNRUN)

Prompt 6 / actual robot integration remains on hold. The following work requires
separate authorization and measured evidence; this patch does not deploy or actuate.

- Pin controller image, WPILib, Java, native NT/HAL and vendor libraries independently
  for 2026 roboRIO and the experimental 2027 alpha-7 Systemcore profile. There is no
  official supported 2026 Systemcore profile in the checked matrix.
- Check current event/FMS/DS/communications rules and the actual season manual;
  an offseason map or a 2027 preview does not define a controller toolchain.
- Measure camera capture correction, synchronization uncertainty/jitter, reconnect
  behavior and capture/publication clocks using the real transport/controller.
- Measure mounts/calibration, fixed-origin/alliance transforms, signed target planes,
  floor/slopes/range limits, localization continuity and conservative uncertainty.
- Validate association, occlusion, identity crossings, source correlation and missed
  detections on labeled team data; establish sensing/coverage-loss policy.
- Measure footprint, piece dimensions/motion bounds, speed, latency and loaded braking
  distance before selecting margins, envelope horizons or planner budgets.
- Verify asynchronous cancellation/overload/CPU timing on the actual controller and
  prohibit model/network/file work on periodic. Reject stale replies under load.
- Independently test disable, DS loss, driver cancel, authority conflicts, stale safety
  sensing, localization resets and collision stop through the robot-owned executor.
- Validate current relative intake alignment, deliberate contact policy, mechanism
  readiness/possession feedback freshness and false-positive/false-negative behavior.
- Qualify low-speed following and bounded acquisition/recovery with supervised test
  procedures before any match use. Paths/forecasts/synthetic replay are insufficient.

No learned uncertainty calibration, trained-model accuracy, hardware stopping margin,
controller latency or possession reliability is claimed by the CPU tests.
