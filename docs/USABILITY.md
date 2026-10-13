# Local controller API follow-up

This local feature adds the admitted-observation bridge, persistent-object target
selection and required pickup bindings. It reuses the existing tracker, planner
validator, safety supervisor and pickup controller. It adds no scheduler, follower,
motor implementation, learned model or robot project.

## Consume admitted observations

The optional adapter requires the CVJ feature API recorded in
`exports/contract-manifest.json`. The older public CVJ `ae67886…` baseline does
**not** provide the new `Measurement.admission()`, `clock()`, `transport()` or
`DeliveryGate` API. Use the explicitly matched feature checkout/jar; do not
reconstruct successful lifecycle or clock results from a packet.
The published CVJ handoff pin is `6639c8fc70c5d1b8b88e93711614c0b480b42efe`;
its implementation/source pin is separately recorded in the manifest.

`VisionTrackBridge` takes the configured `CustomVisionAdapter`, robot-owned
`WorldEngine` and the originating facade's current delivery gate:

```java
// These objects are configured and owned by the robot; this is wiring, not a robot program.
var bridge = new VisionTrackBridge(configuredAdapter, worldEngine,
    customVisionRig.observationDeliveryGate());
customVisionRig.rig().setObservationConsumer(bridge::acceptRobotNanoseconds);
```

The facade callback supplies its actual admitted immutable measurement and robot
nanoseconds. The bridge forwards its original admission and complete clock result,
checks current source generation and facade mode/configuration fences, normalizes
one time into core microseconds, and calls the bounded tracker. Direct client users
can instead supply their originating `VisionClient` and consume its measurement
drain once. Keep callback and manual observation insertion exclusive.

Record `Decision.normalization()` and `tracking()` separately: accepted geometry
can still fail capture-history bounds, epochs or duplicate gates. An insertion
count of zero never proves free space. Supply capture-time ego history first.
The robot owns hard resets and rebuilding bindings for source boot/reconnect,
calibration, mount, field or localization changes. No bridge resets the world
automatically. Low-level `adapt(Measurement, nowUs)` preserves historical
provenance but does not itself recheck a live delivery gate; prefer the bridge.

## Select a reachable useful target

Run one `TargetSelector` on a serialized bounded worker, using the injected
`PlannerBackend`, configured `SafetySupervisor` and robot microsecond clock. The
selector does not start a thread or run a periodic planner. One shared monotonic
search budget covers candidate generation, every planner call and validation.
Injected backends must cooperate with cancellation and that budget.

`SELECTED_OBJECT` requires a persistent World-State track ID; a raw camera's local
track ID is a different identity. `USEFUL_CLUSTER` considers collection disks
centered on eligible persistent tracks. Its finite candidate set is documented,
not an unrestricted continuous disk optimization. Configure these inputs:

- Maximum measurement age, confidence and covariance bounds; covariance expansion
  for collection coverage. Fresh coasting estimates qualify; tentative/stale
  tracks do not. Lifecycle, estimate time and last measured time remain distinct.
- Collection radius, robot-relative intake pickup point, bounded alternative
  approach headings, and pose/velocity tolerances.
- Class utility weights, travel cost per meter, switching improvement and minimum
  lock interval. Cluster utility sums unique physical track identities. Duplicate
  camera observations cannot increase that utility.
- Physical footprint, speed/acceleration/angular limits, fixed field bounds and
  candidate/provenance limits. Supply independently fresh conservative collision
  envelopes over the entire request validity interval.

For each configured anchor/heading, the selector places the intake point at the
anchor and asks the planner for a geometric approach. It validates the complete
path independently. Score is `sum(member class weights) - travelCostPerM *
validatedPathLengthM`; ties use persistent ID and approach order. Covariance and
goal tolerance must fit inside the collection disk. The current ego uncertainty
and configured safety margin expand obstacle envelopes once and shrink field
bounds once; the raw physical footprint is kept separate. These inputs require
later measurement qualification.

An invalid lock is released; a still-reachable lock uses configured hysteresis.
Task, epoch, map or field changes clear its context. Candidate overflow, timeout
or cancellation abstains instead of claiming the greatest partially searched
cluster. Missing configuration, safety/planner/clock bindings and unavailable
selected IDs return named outcomes.

Admit a worker result only when `selection.matches(currentSnapshot, taskId,
nowUs)` holds. Validity is bounded by the request, planner result, robot sensing
and member measurement freshness; backend IDs, duration and expiry are retained.
Selection is geometric evidence, not a motion command or pickup success. It
does not retain a follower-ready path: the robot task/follower must request and
validate its current path with the selected goal and recheck collision risk.
The disk is a useful region, not evidence that every member was collected.

## Bind pickup feedback and intents

`PickupBindings` constructs the existing sole `PickupController` after checking
required configuration and ports. Missing intake/footprint/constraint configuration,
authoritative facts, measured mechanism or possession feedback, bounded reachable
scorer, follower intent port or mechanism intent port fails with a named binding
error. No guessed geometry, generic follower or substitute possession evidence is
provided.

The robot supplies cached, timestamped `Evidence` and its authoritative
`RobotFacts`. `readFeedback(nowUs)` accepts only present, fresh, agreeing evidence.
A fresh false possession measurement is valid unsuccessful-acquisition evidence;
missing/stale/invalid evidence is a sensing failure. Failure cancels the current
task and attempts stop intents on both ports, including when one port throws.
It cannot resume an old path just because sensing later recovers.

Use `bindings.controller()` for the existing lifecycle. The robot still owns
bounded planner submission, current relative intake alignment, feedback updates
and dispatch through the required intent ports. Arrival/settling and verified
mechanism/possession remain required; target disappearance, planner success and
elapsed nominal duration do not complete a task.

## Local artifacts and evidence

```sh
./scripts/test.sh
./scripts/test-vision-adapter.sh /path/to/matched-CVJ /path/to/Custom-Vision [protocol.jar]
./scripts/test-planner-integration.sh /path/to/1086-On-The-Fly-A-Star
./scripts/package-controller.sh /path/to/matched-CVJ-protocol.jar
```

The package command creates original core and optional bridge Java 17 binary and
source jars plus a checksum manifest under `build/controller`. It compiles only
main Java sources and packages no WPILib/native libraries, planner/model runtime,
fixtures, tests, Python, Jetson code, GUI/server, datasets or weights. CVJ owns its
separate version-specific facade and offline installer. No public Maven/vendordep
URL is invented. These local jars can be used in a separately authorized robot
integration; no robot project is edited here.

The source test uses the actual decoder/session/clock/client with exact producer
golden geometry. Simulated timing evidence is explicitly marked; it is not camera
capture correction qualification. Both 2026 microsecond and alpha-7 nanosecond
metadata converge into capture microseconds. The planner test uses the actual
A* adapter/solver with bounded synthetic maps. All results are CPU software
evidence. Hardware work and actual robot integration remain on hold under the
separate [hardware checklist](HARDWARE_CHECKLIST.md).
