# Planner contract `frc-planner/1`

Owner: FRC-World-State. API: `org.frcworldstate.core.PlannerBackend` plus nested
`Geometry`/`World` types. PlannerBackend.java SHA-256:
`5f6c6e709516c522c2bfc765e8d7a241c274cb4ac1d2e1d03e5d0599980665aa`.
The content pin is independent of documentation and publication commits.

Requests carry request/task IDs, epoch, original snapshot ID, obstacle-map version,
field/map/geometry identity, start pose/velocity, goal/tolerances, raw robot footprint,
physical constraints, bounds, conservative circle obstacles, issued/expiry robot us,
monotonic search budget and cooperative cancellation. Obstacle envelopes must cover
the entire validity interval; they must include physical dimensions, capture-age,
uncertainty and bounded motion/timing/latency. `ObstacleEnvelopeBuilder` implements
this responsibility with explicit configuration and typed whole-output rejection.
A point detection or fresh estimate timestamp is insufficient.

Results carry matching identities, typed SUCCESS/NO_PATH/TIMEOUT/CANCELLED/
INVALID_INPUT/STALE_RESULT, measured solver ns, expiry, backend ID and detail.
`GeometricPath` is distinct from `TimedTrajectory`. No path result completes a task.
A SUCCESS needs at least two bounded geometry points, including stationary duplicate
endpoints if a backend explicitly supports that case. Timed trajectories are optional
contract data; the supplied core geometry validator rejects them until a separate
trajectory validator qualifies speeds, timing, kinematics and collision coverage.

`Result.matches` is called with the **saved original request snapshot**, not an
arbitrary latest publication. The controller separately gates current field/epoch/map,
meaningful target changes, start displacement/heading drift and expiry. Full path
collision validation is off the periodic path, bounded by search budget/cancellation.
Current collision risk is independently supervised during following.

A* owns its backend. Its public source repository is
[1086-On-The-Fly-A-Star](https://github.com/Antigro09/1086-On-The-Fly-A-Star).
Strict Java 17 envelope/backend/core-validator integration passed 20 checks against
the sources pinned in `exports/contract-manifest.json`. See `DEPENDENCIES.md` for
reproduction. This is desktop software evidence; the robot executor remains gated.
