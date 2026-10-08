# Local API follow-up evidence

Local branch: `feat/usable-vision-target-selection`, based on published
`0e5b6c3f85d47205cde8b0c80e13fa79d35e95ab`. This follow-up is not pushed.

## Delivered

- `VisionTrackBridge` consumes CVJ's actual admitted immutable observation,
  including original lifecycle, clock and transport provenance. Origin/generation,
  facade configuration/mode, capture history and epoch gates remain distinct.
- `TargetSelector` provides selected persistent-object and weighted useful-cluster
  modes, measured-age/covariance gates, bounded approach search, independent path
  validation, physical-track deduplication and configured lock hysteresis.
- `PickupBindings` checks required robot/mechanism/possession/follower bindings,
  validates timestamped measured evidence, and cancels/stops both intent ports on
  failed feedback. It reuses the existing sole pickup controller.
- `package-controller.sh` builds original core and optional bridge binary/source
  Java 17 jars with a checksum manifest. No robot project or installer is modified.

See [API wiring and explicit prerequisites](USABILITY.md). Existing planner and
prediction v1 contracts, schemas and prediction fixture bytes are unchanged.

## Passed on Java 17.0.20.1, CPU only

| Command | Result |
|---|---|
| `./scripts/test.sh` | Strict `--release 17 -Xlint:all -Werror`; 61 tracker/worker, 10 envelope, 12 pickup/safety groups, 14 binding checks, 96 selector checks and 109 prediction assertions. |
| `python3 scripts/verify_prediction_fixtures.py` | Java/Python golden request/reply plus 15 rejection vectors. |
| `./scripts/replay.sh` | Identical absent/rejected-predictor baseline; nine pickup decisions, one measured completion, seven tracker input events with correction/reset. |
| `./scripts/test-vision-adapter.sh MATCHED_CVJ PINNED_PRODUCER MATCHED_PROTOCOL_JAR` | Source and exact binary each pass 181 normalization assertions plus 62 actual-admission/tracker/selector checks, both metadata time profiles. |
| `./scripts/test-planner-integration.sh PINNED_ASTAR` | 20 existing envelope checks plus 37 actual A*/selector checks against pinned public source. |
| `python3 scripts/export-contracts.py --producer PINNED_PRODUCER --vision-java MATCHED_CVJ --planner PINNED_ASTAR --artifacts` | All recorded source/artifact hashes match. |
| `./scripts/package-controller.sh MATCHED_PROTOCOL_JAR` | Four main-source-only Java 17 jars; expected package/bytecode content checked. |
| `jdeps -s build/controller/frc-world-state-core-0.1.0-local.jar` | `java.base` only. |
| `git diff --check` | Pass. |

The bridge tests exercise real decoder/session/mapping/admission over bounded fake
transport. Only timing qualification and I/O are synthetic. They include consume
once, retained replay, watchdog purges, invalid correction, history bounds, world
resets, boots, reconnects, synthetic delivery-gate rejection and capture-latency
coasting into selection. Actual facade mode/configuration fence behavior belongs
to CVJ's separate facade verification; this script compiles its protocol only.
Producer golden geometry is unchanged and hash checked.

Selector review found and fixed two defects: normal capture latency must allow
fresh coasting estimates, and a selection must expire with its chosen backend
reachability result. Regression checks preserve measurement/estimate distinction
and reject evidence expiring during later candidate evaluations, including a
hysteresis-retained candidate. The real A* suite also tests blocked goals, closed
corridors, detours, covariance margins and cancellation. No path result completes
pickup or changes possession.

## Dependency and remaining limits

Producer/planner public pins remain in `exports/contract-manifest.json`. Their
owners' changing feature working trees are not interchangeable with those pins:
content verification correctly rejected the newer producer fixture manifest and
A* geometry source. Checks were repeated against the declared public source
content. The golden object bytes remain identical.

CVJ's admitted envelope API is a separate local feature. The manifest records its
exact source and `0.2.0-local.1` protocol binary/source jar hashes. A null
`local_feature_revision` means the final owner commit is still pending, and the
older public CVJ baseline is explicitly incompatible with the new bridge API.
Do not claim a reproducible public-only bridge install until that source is
separately published with authorization. Core and pinned A* checks are independent
of that prerequisite.

No remaining failing software check at this recorded cut. Gradle entry points,
real follower/mechanism bindings, actual robot integration, deployment, hardware,
camera capture correction, controller timing/stopping margins and learned
calibration remain unrun or separately gated. The controller jars contain no
Python/model/GUI/server dependencies. CVJ owns its WPILib installer; no public
Maven/vendordep or online install is claimed here. Follow the separate
[hardware checklist](HARDWARE_CHECKLIST.md) only after authorization.
