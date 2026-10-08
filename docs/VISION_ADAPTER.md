# Optional Custom-Vision-Java adapter

`adapters/custom-vision-java` consumes the zero-WPILib `org.customvision.protocol`
module and emits the core's immutable `World.ObservationFrame`. The dependency
direction is **adapter → CVJ protocol + World-State core**. The core has no vision,
NetworkTables, HAL, vendor, planner, or model dependency. Both modules compile
with Java 17 bytecode. Controller metadata units are selected explicitly in the
binding: WPILib 2026 microseconds or WPILib 2027 alpha-7 nanoseconds. JSON `_us`
fields remain exact microseconds in either profile. Season identity does not
select a controller or time unit.

## Admission and ordering

The public method is:

```java
Result adapt(Packet packet, SourceSession.Result lifecycleAdmission,
             ClockMapper.Result captureMapping, long nowRobotUs)
```

A measurement requires `Kind.ACCEPTED` and an exactly matching
`newlyAcceptedMeasurement`. Pending boot adoption, duplicate delivery,
status-only acceptance and rejected publications cannot emit measurements. Do
not repeatedly adapt `SourceSession.Status.currentPacket` during the robot loop.
The immutable lifecycle result is an explicit call contract; the robot-owned
receiver must supply the actual result of its admission policy.

Process each source's coherent publications in publication order. Apply its
invalidation and watchdog state before deduplicating captures or entering the
shared `CaptureReorderBuffer`. Remove affected pending source/boot/frame entries
when invalidation, disconnect, overload, revision change or reset occurs. Use
one bounded capture reorder buffer across cameras; the World-State core rejects
capture times behind its global high-water mark. Revalidate epoch, freshness and
capture-time pose-history bounds when a buffered frame reaches the core. Expire
source receipt liveness even when the queue is empty. A mapped timestamp alone
does not establish camera liveness.

The binding explicitly identifies full `SourceKey` namespace, pipeline and type,
the named protocol profile, active producer boot, robot clock epoch, connection
epoch, world epoch, metadata version, calibration and mount hashes, correlation
group and target-plane configuration revision. Rebuild the binding after an
authorized change. A retained old boot cannot replace it. A field-layout hash is
optional for capture-relative object observations; when configured it must
match, and the raw value is retained regardless. Robot field/map identity belongs
to the core's robot-owned field configuration.

## Geometry and uncertainty

Only `objects.targets` is normalized. Compact detection references, selection,
approach hints and repeated tracker metadata are not extra measurements. The
adapter requires current `observed:true`, `predicted:false` targets in
`robot_relative_at_capture_wpilib_nwu`, `motion_compensated:false`, and the
implemented `calibrated_ray_target_height_plane` method with `approximate:true`.
Its XY coordinates are already robot-relative meters with +X forward and +Y
left. The adapter does not convert optical axes, apply another mount transform,
apply alliance transformation, use a current robot pose or infer field geometry.
The core projects with authoritative capture-time pose after history-bound
checks. The signed target-height plane and tolerance are explicit configuration;
no floor plane or target height is guessed.

Missing calibration/mount revisions, missing metric coordinates, absent or
unsupported confidence, incompatible planes, nonfinite values, invalid estimate
flags, duplicate IDs and excessive target counts produce typed rejection. The
source-local `track_id` remains an observation association hint, not permanent
physical identity. `SourceStamp.sequence` is the capture `frame_id`; the raw
publication `packet_seq` is retained separately. Republishing the same capture
cannot generate an independent core measurement.

The producer's configured first-order XY covariance must be finite, symmetric
within the explicit wire-rounding tolerance and positive semidefinite within
that same numeric budget. Excessive variance is rejected. The adapter emits an
isotropic variance bound that dominates the reported covariance plus numeric
budget, with a separately configured conservative floor. That floor is an
assumption, not another independent measurement. It must cover measured
geometry/timing uncertainty for the intended operating domain. Confidence,
pixel residuals and association age are never converted into covariance. The
robot supplies its conservative localization uncertainty to the core; the
adapter does not claim access to estimator covariance. Configure a shared
correlation group for observations with a common exposure, detector or other
known dependence so that the tracker can avoid double counting them.

Capture nanoseconds from the already accepted clock mapping are floored by
integer division by 1000, never rounded forward. Publication uses checked
`ntTimestampNs + localToRobotOffsetNs` followed by the same conversion. Raw
metadata version, offset, JSON capture time and correction uncertainty must
match the mapped values. Capture and publication remain separate instants.
Unix and Jetson monotonic timestamps are preserved only as raw provenance;
neither is guessed to be robot time.

`UNAVAILABLE` covers dead cameras, missing/invalid families, empty observations
and lifecycle outcomes without a new measurement. It supplies no frame and
never claims free space or removes existing occupancy. `REJECTED` also supplies
no frame. Both retain the immutable decoded `Packet`, lifecycle result and full
`ClockMapper.Result`, plus configured geometry/correlation provenance. The
robot-owned receiver feeds source health into required-sensing safety policy;
the tracker may coast and expire estimates according to its documented rules.

The adapter performs bounded in-memory work only. It has no networking, file
I/O, executor, field estimator or motor access. Log its input and decision
identities through a bounded off-loop writer if persistent replay is needed.

## Reproducible CPU checks

After the core build, supply explicit dependency checkouts from `DEPENDENCIES.md`:

```sh
./scripts/test-vision-adapter.sh /path/to/Custom-Vision-Java /path/to/Custom-Vision
# Optional third argument: an explicitly supplied protocol jar.
./scripts/test-vision-adapter.sh /path/to/Custom-Vision-Java /path/to/Custom-Vision /path/to/protocol.jar
```

The scripts read dependency sources without editing them and do not fetch code.

The suite currently passes 181 assertions on local Temurin Java 17. It checks
both metadata versions, exact ns→us flooring, separate capture/publication times,
source/profile/boot/revision gates, missing/invalid/empty/dead-camera families,
estimated/nonfinite/out-of-domain geometry, confidence and covariance gates,
signed planes, capture-time field projection, duplicate capture independence,
pending adoption, real lifecycle admission and same-frame watchdog tombstones.

The exact producer coherent-string fixture
`objects_compact_covariance_selection.json` has SHA-256
`c0e5c0ea9241423e1d3aa8fe19373931cf3fab080ced9af2b9021d9eaf6fc6d9`.
Its unchanged timing is deliberately unverified and must reject with
`CAPTURE_CORRECTION_UNVERIFIED`. The accepted canonical-geometry check applies
a separately named **synthetic timing modification**. That test proves Java
geometry normalization, not camera exposure timing or hardware compatibility.
The producer's 27-fixture manifest records `matched_runtime`. Exact tested raw
schema/fixture and CVJ protocol content hashes are in
`exports/contract-manifest.json`; public sources and optional checkout overrides
are in `DEPENDENCIES.md`. The 181 assertions pass both against independently
compiled protocol source and the exact recorded local jar. Jar hashes identify
software evidence, not published binary packages.

CVJ's immediate adapter is WPILib 2026.2.1 / Java 17; the separately pinned alpha-7
adapter uses Java 25. The core time/geometry source audit references 2026.2.2, and
A*'s legacy demo also used 2026.2.2. These are distinct verified desktop profiles,
not a single deployed controller/vendor version. Source/schema hashes and tested
dependency revisions are recorded in `exports/contract-manifest.json`; a documentation-only
World-State commit does not require a future owner commit or a repinning loop.
No Maven artifact is published. Hardware qualification, Jetson/controller
deployment, actual robot integration and motor operation remain unrun and gated.
