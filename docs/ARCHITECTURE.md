# Architecture and invariants

## Dependency direction

`core` uses only `java.base`. External A* implementations implement
`PlannerBackend`; external model runtimes implement `Predictor`. The optional
vision adapter depends on core and Custom-Vision-Java's pure protocol module.
Neither backend, WPILib, NetworkTables, vendor motors nor robot project is a
core dependency. `jdeps -s` verifies the produced jar depends on `java.base` only.
The core compiles with `javac --release 17`; 2027 API/native adapters belong in
separate Java/toolchain profiles. There is no new learned architecture here.

## Robot authority, frames and time

`World.RobotPort` supplies authoritative immutable pose/velocity, enable, DS,
mode, command authority, driver cancellation, mechanism and possession facts.
Required-sensing validity/time must cover the actual mechanism/possession feedback.
Only the robot execution boundary can act on `PickupController.Decision` intents.
The library has no hardware, network or filesystem access in its control methods.

All geometry is SI in fixed `BLUE_FIELD`: configured blue origin, +X down field,
+Y left, CCW radians. Field identity includes season, map and geometry revision.
The robot boundary applies any alliance transformation exactly once; tracker,
planner and predictor never apply it again. Season does not select controller
hardware or WPILib version. Raw object geometry must retain its signed target
height/floor-plane, calibration, mount and source/boot assumptions.

Core observation/snapshot timestamps are nonnegative **robot monotonic us**.
They are neither Unix time, Jetson monotonic time nor unmapped NT metadata.
Capture and publication are separate values. `TimeDomains` explicitly preserves
2026 metadata us and floors 2027 alpha-7 metadata/mapped robot ns to us (<1us
backward rounding). JSON `_us` remains us. Signed clock offsets are handled by
CVJ's checked clock mapping, not treated as timestamps by `TimeDomains`.
`SearchBudget` uses `System.nanoTime` and measured elapsed solver duration; its
clock must not be compared with robot timestamp epochs.

## History, corrections and resets

`PoseHistory` bounds samples by both count and retained duration. Capture times
must lie inside explicit oldest/newest bounds before interpolation. Position
interpolation and shortest-angle interpolation happen only within that interval;
there is no boundary clamping. Pose covariance is a configured conservative
robot estimate, not invented access to WPILib estimator internals.

`WorldEngine` retains bounded raw relative observations, assigned identities and
source provenance. Explicit pose correction is transactional: evaluate corrected
poses into a bounded temporary history, then reproject raw observations. Track
identities survive a correction if retained raw history supports them; tracks
without retained capture history are removed. Velocities are set to zero and
quarantined until post-correction measurements. Existing snapshot history clears
and obstacle-map version advances, invalidating earlier geometry decisions.

Constant-position is the example/baseline configuration. Constant velocity is
an explicit opt-in requiring all localization corrections/revisions to be
reported by the robot port. Velocity inference also requires consistency of
capture-time pose displacement with integrated authoritative ego velocity.
Large configured translation/heading innovations, changed pose at the same
timestamp, or an ego gap beyond retained duration hard-reset the epoch/history.
Thresholds cannot identify every unannounced small correction; they are not a
substitute for that continuity contract. No guessed threshold is called safe.

Robot boot, field/geometry changes, reconnect, source boot/calibration/mount
changes and hard localization reset invalidate old goals/results through epochs.
`reset` clears tracks, raw evidence, sources and snapshots; pose history is cleared
for robot/field/localization resets and can be retained for a camera reconnect
in the same robot clock. The robot coordinator cancels worker jobs at epoch changes;
controller epoch checks independently cancel outstanding task requests. Source
restart rejects its triggering frame and requires resubmission in the new epoch.

## Deterministic tracking and occupancy

Configured caps limit sources, tracks, frames, raw evidence and snapshots. Apply
CVJ's bounded shared capture reorder buffer before the core; captures behind the
global high-water mark reject. Each source also gates sequence/capture monotonicity,
duplicates and boot/revision identity. Confidence/covariance/range/history and
localization gates precede association.

Association sorts observation IDs, then uses same-class nearest predicted position
inside a configured physical ceiling and uncertainty distance gate. Ties within
0.05m are conservatively ambiguous and skipped. Source-local detection/track IDs
are provenance hints rather than permanent field identities. The baseline cannot
resolve every crossing; it abstains when association is ambiguous. Confirmation
counts only admitted independent measurements; propagation never turns a
TENTATIVE track into a confirmed one. Estimates then become CONFIRMED/COASTING
and expire by **last measurement**, not fresh publication/estimate time.

A per-track, bounded correlation-group history suppresses correlated measurements
even when another camera group appears between them. Published provenance is
bounded separately. Empty/invalid/occluded frames, duplicate publications and
dead cameras never assert free occupancy. Expired/absent tracks mean unknown,
not a free-space guarantee. Collision coverage freshness is an independent
robot-owned input; stale required information conservatively holds/stops.

`ObstacleEnvelopeBuilder` requires positive configured physical radius, bounded
motion, positional covariance margin, capture age, timing uncertainty and latency
through the entire validity interval. CP/CV extrapolations are anchored back to
the last measured center; estimate timestamps do not freshen measurements. Missing
class geometry, stale information or unknown collision coverage rejects the whole
output. These are obstacle envelopes only; a backend adds the raw robot footprint
once. Static map geometry and deliberate intake-contact policy are robot/season
configuration; a selected target is never automatically removed as an obstacle.

## Tasks, supervision and asynchronous work

Pickup follows SELECT → PLANNING → TRANSIT → APPROACH → ACQUIRE → VERIFY, with
bounded RECOVER, failure, cancellation and latched SAFE_STOP states. Reachability
comes from an injected bounded deterministic scorer; selection uses cost/identity
ties, target locking and hysteresis. Meaningful target/map changes rate-limit
replanning. Planner replies bind the original request snapshot plus current
field/epoch/map/start-drift/expiry, allowing newer publication IDs without accepting
old task replies. A geometric path cannot complete pickup.

Fresh robot-relative alignment uses configured intake geometry and uncertainty
near pickup. Arrival pose/velocity, settling, measured mechanism-ready and verified
possession are required. Target disappearance, elapsed nominal time and planner
SUCCESS alone do not complete. Attempts/recoveries are bounded; completion fires
once. Driver cancel, disable/lost DS, authority conflict, mode/localization/sensing
loss, epoch change, loop overrun and collision risk stop/hold independently.

Collision supervision runs again on every following update using measured ego
velocity, configured stopping deceleration/latency, robot footprint and positional
uncertainty against current obstacle envelopes. The executor must still validate
and limit its accepted commands; this component is not a certified hardware safety
controller. Footprint, latency, uncertainty, stopping and speed bounds require later
measurements and qualification.

`LatestWorker` has one active job, one replaceable pending request and one latest
completion. New requests cancel active work and replace pending work; stale
completions drop. Submit/poll/close are bounded nonblocking operations. Metrics
expose replacement, cancellation, failures, dropped results, active age and worker
liveness. Injected backends must honor deadline/cancellation; a noncooperating
backend cannot safely be forcibly killed by Java. It blocks only its worker slot,
while requests expire and the control path remains independent. No parallel heavy
jobs or model runtimes are included.

Full `PlannerValidation` and `PredictionGate` validation belong in workers.
Planner validation checks bounded geometry/endpoints/bounds/collision with deadline
and cancellation. Timed trajectories require a separate qualified validator and
are rejected by this geometry-only gate. Predictions are advisory forecasts and
never update observed tracks/robot facts. Missing, uncalibrated or rejected replies
leave deterministic baseline behavior available; independent collision supervision
still applies to accepted forecasts.

## Replay

The bounded in-memory input log preserves full ego/frame data, identities, frame
outcomes, absolute corrected history, resets, snapshot outputs and decision IDs.
Derived resets are diagnostic events; replaying their triggering ego/frame input
reproduces them, so do not apply them twice. Persistent writers run off periodic.
A truncated ring needs an external initial checkpoint/full capture; it cannot
reconstruct omitted inputs. `FakeIoReplay` demonstrates exact complete-log replay
including correction/reset, and identical pickup decisions with absent and actual
gate-rejected optional predictions. Its facts and margins are synthetic.
