# Implementation report

## Delivered component

- Pure Java 17 immutable geometry, ego/object/snapshot/robot-fact contracts; explicit
  fixed origin, SI units, source/boot/calibration/mount/plane and robot-us domains.
- Bounded capture-time history, transactional raw-observation reprojection, resets,
  deterministic CP baseline / opt-in continuity-gated CV tracking, association,
  independent-confirmation/lifecycle/expiry and correlated-camera suppression.
- `frc-planner/1` SPI with typed results, cancellation/deadlines, separate geometry
  and trajectory types; off-loop geometry validator. Tested conservative physical,
  age/motion/timing/uncertainty obstacle-envelope construction (not point detections).
- `frc-prediction/1` SPI, model-specific uncertainty/domain gate, bounded-lag immutable
  histories/action candidates, accepted-command provenance, JSON schemas and Java/
  Python fixtures. Forecasts remain advisory and separate from observed state.
- Configurable pickup selection/planning/transit/approach/acquire/verify/recover,
  target locking/replan limits, bounded attempts, measured arrival/settling/mechanism/
  possession completion, duplicate-completion prevention and independent safety.
- Latest-request-wins bounded worker, observable cancellation/overload/failure/liveness;
  full bounded replay input/decision logs and a fake-I/O example with exact replay.
- Optional CVJ adapter with actual lifecycle admission and checked ns→us mapping;
  matching canonical producer geometry and typed unavailable/rejected outcomes.

## Passed verification (Temurin 17.0.20.1, CPU only)

| Command / suite | Evidence |
|---|---|
| `./scripts/test.sh` | Core compiles with `--release 17 -Xlint:all -Werror`; 61 tracker/history/time/worker/planner checks, 10 envelope checks, 12 pickup/safety test groups, 109 prediction assertions pass; local jar emitted. |
| `python3 scripts/verify_prediction_fixtures.py` | Java-emitted request/reply matched, Python JSON roundtrip, signed-plane acceptance and 15 negative vectors pass. |
| `./scripts/replay.sh` | Nine pickup decisions; exactly one measured completion. Actual absent/uncalibrated gate replies produce identical baseline decisions. Seven full input/decision events replay identically including correction and reset. |
| `./scripts/test-vision-adapter.sh` | CVJ protocol builds independently; optional consumer compiles against core+CVJ; 181 assertions pass. Final producer object golden SHA matches; unchanged unverified timing correctly abstains. Rerun against final CVJ source and exact protocol jar passes 181 assertions each; all 18 owner file hashes and 4 artifact hashes match. |
| `./scripts/test-planner-integration.sh` | Read-only A* sources compile against CURRENT core; 20 direct owner-envelope/backend/core-validator checks pass. Final owner commit fixes the three serialization warnings; strict `-Xlint:all -Werror` now passes against the recorded source content pins. |
| `jdeps -s build/frc-world-state-0.1.0.jar` | Only dependency is `java.base`; no vision/planner/model/vendor implementation required. |
| `javap -verbose … World` | Common class-file major version 61 (Java 17). |
| `git diff --check` | No whitespace errors. |

The jar normalizes ZIP entry timestamps for reproducibility. Build products are
ignored by Git; a content manifest records the exported source and fixture pins.
Gradle's `contractTest`/`check` entry points are supplied but not run in this task;
the dependency-free `javac` route is the proven build. A full third-party Draft
2020-12 schema validator was unavailable; the Python verifier explicitly supports
the schema subset used here plus cross-document semantic checks. No Java runtime
JSON wire parser is claimed: Java emission/Python consumption is proven.

## Acceptance coverage

| Requirement | Exercised evidence |
|---|---|
| Transforms/fixed origin, one alliance transform | Rotated robot/field roundtrip and fixed red→blue synthetic field tests. |
| Capture/publication separation, history bounds | Interpolated capture pose differs from publication pose; explicit out-of-history reject; future publication/capture; bounded history. |
| us/ns and epoch conversion | 2026 us vs alpha-7 ns, floor conversion, checked overflow; optional adapter exercises both mapped profiles and clock provenance. |
| Stale/out-of-order/duplicate/source restart | Capture age, per-source ordering, global delayed-camera high-water, sequence duplicates, source boot epoch reset, old epoch reject. |
| Empty/occluded/dead camera | Empty core observations retain occupancy until measurement expiry; unavailable adapter never asserts free; missing collision coverage rejects. |
| Identity/correlation/quality | Detection-ID changes preserve association, ambiguous-association abstention design, per-group interleaving regression, confidence/covariance/range gates; tentative propagation never confirms. |
| Corrections/resets | Raw reprojection preserves identity, zero/quarantined velocity, ego velocity consistency, large/same-time pose and history-gap reset design, explicit hard reset and full-log correction/reset replay. |
| Old async replies/task replacement | Worker drops overwritten/cancelled work; pickup preserves original request snapshot while rejecting old task/epoch/map/start drift/expiry; prediction gate rejects old IDs/maps/epochs and unsupported outputs. |
| Arrival/possession/recovery | Settled pose/velocity plus measured mechanism/possession; missing possession times out into bounded recovery/failure; disappearance/path success alone insufficient; one completion event. |
| Disable/cancel/collision/overrun | Independent supervisor reasons and collision stop while following; required sensing/coverage/envelope expiry conservatively hold/stop. |
| Missing/rejected model fallback | Gate rejection vectors and fake-I/O baseline decision equality; core has no predictor implementation dependency. |
| Java/Python/dependency direction | Exact schema/vector verification; isolated Java17 core/CVJ compile; read-only A*20-check integration; `jdeps` only `java.base`. |

## Cross-contract pins and remaining prerequisites

Exact implementation, schema, fixture and optional dependency source hashes are
recorded in `exports/contract-manifest.json`. Public repository/revision setup and
explicit local checkout overrides are documented in `DEPENDENCIES.md`. The core
build and replay need no external repository. Optional CVJ and A* integration checks
use those content pins and never require a private workspace path.

Producer ownership remains Custom-Vision. The optional adapter tests the exact
`objects_compact_covariance_selection.json` golden bytes and separately named
synthetic verified timing. It does not bypass deliberately unverified camera
correction. CVJ protocol source and its recorded local jar both passed 181 adapter
assertions; A* passed 20 direct envelope/backend/core-validator checks. Recorded jar
hashes identify tested build artifacts; they are not published Maven coordinates.

CVJ's adapter profiles are 2026.2.1 / Java 17 and 2027 alpha-7 / Java 25. The
core's source audit and A* legacy demo use 2026.2.2. These are distinct desktop
profiles; no unified deployed controller/vendor/toolchain version is claimed.

Prediction v1 intentionally omits heading/mechanism-state forecasts, full actuator
clipping/saturation/execution-deviation telemetry, training session storage and robot
integration. Accepted command provenance is not measured motion. Those extensions
need robot-owned logging/configuration and, for layout changes, a new schema version.
No learned uncertainty calibration or model accuracy is supplied.

## Failed / unrun / gated

No remaining failing **software acceptance** test at the recorded cut. Earlier
serialization warnings, tentative-propagation, delayed/correlated association,
transactional correction and validator boundary defects were corrected or explicitly
tracked above; tests were rerun after fixes. Hardware is entirely unrun: no camera,
Jetson/controller deployment, motors, GPU training, actual robot integration or event
qualification. See `HARDWARE_CHECKLIST.md`. Full-path validation of timed trajectories
is intentionally unsupported by the supplied geometry validator.

Official supported WPILib 2026 + Systemcore target: **not found** in the checked
[SystemcoreTesting matrix](https://github.com/wpilibsuite/SystemcoreTesting).
2026 roboRIO is the immediate profile; 2027 alpha-7 Systemcore is separately pinned.
Do not rewrite WPILib/HAL or change package visibility to manufacture another target.
