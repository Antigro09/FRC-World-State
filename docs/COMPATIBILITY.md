# Controller, time and geometry compatibility audit

Checked 2026-10-08 UTC. This is a source audit for an offseason, controller-independent Java library. It is not controller, camera, robot or field qualification. Actual robot-code integration remains on hold.

## Target policy

| Requested target | Audited pin | Status and boundary |
| --- | --- | --- |
| WPILib 2026 + roboRIO | `allwpilib v2026.2.2` (`7ca35e5` tag commit prefix) | Source-audit pin; primary immediate family is 2026 +roboRIO. Final CVJ adapter separately pins2026.2.1 / Java 17. Keep its `edu.wpi.first.*` API adapter separate from the core. Source contracts checked; controller build/deploy and hardware behavior are unrun in this audit. |
| WPILib 2027 + Systemcore | `allwpilib v2027.0.0-alpha-7` (`83df3ee` release commit prefix) | Separately pinned experimental profile. Official testing matrix pairs Systemcore image 14 or newer with alpha-7 or newer; pin an exact image before hardware use. Alpha software is not a stable release or evidence of hardware readiness. |
| WPILib 2026 + Systemcore | None found in official supported matrix | Unsupported/unverified requested combination: the inspected official matrix lists only WPILib 2027 alphas. Do not create a fictitious 2026 Systemcore profile, rewrite HAL, or relabel a 2027 program as 2026. Reassess only when an official compatible release is published. |

The [SystemcoreTesting compatibility matrix](https://github.com/wpilibsuite/SystemcoreTesting#software-compatibility) is a mutable upstream page, audited on the date above. Its listed older image combinations use alpha-1/2 or alpha-5/6. It also states that its alpha software is incompatible with roboRIO and Control Hub. An offseason game's year does not select the controller toolchain. For an offseason event, confirm the event's FMS/Driver Station compatibility separately.

The [2026 source tag](https://github.com/wpilibsuite/allwpilib/releases/tag/v2026.2.2) and [2027 alpha-7 release](https://github.com/wpilibsuite/allwpilib/releases/tag/v2027.0.0-alpha-7) identify the exact versions audited. The [2027 migration notes](https://docs.wpilib.org/en/latest/docs/yearly-overview/yearly-changelog.html) specify Java 25 and document package and API changes. Building the common library with Java 17-compatible source does not prove that either controller adapter compiles against its selected WPILib distribution.

## Time adapter boundary

| Quantity | WPILib 2026.2.2 | WPILib 2027 alpha-7 |
| --- | --- | --- |
| `TimestampedString.timestamp`, `serverTime` | Local/server clock integer microseconds | Local/server clock integer nanoseconds |
| `NetworkTableInstance.getServerTimeOffset()` | Optional signed microseconds; add to local time to estimate server time | Optional signed nanoseconds; same sign convention |
| Robot raw time | `RobotController.getTime()` and `getFPGATime()` use microseconds | `RobotController.getTime()` and `getMonotonicTime()` use nanoseconds |
| `Timer` / pose-estimator floating timestamps | Seconds; default estimator epoch is FPGA time | Seconds; default estimator epoch is monotonic time |
| Custom-Vision JSON fields ending in `_us` | Microseconds | Microseconds; do not reinterpret as nanoseconds |

Sources: 2026 [TimestampedString](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.2/ntcore/src/generated/main/java/edu/wpi/first/networktables/TimestampedString.java), [NetworkTableInstance](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.2/ntcore/src/generated/main/java/edu/wpi/first/networktables/NetworkTableInstance.java), [Timer](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.2/wpilibj/src/main/java/edu/wpi/first/wpilibj/Timer.java), [RobotController](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.2/wpilibj/src/main/java/edu/wpi/first/wpilibj/RobotController.java); alpha-7 [TimestampedString](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/ntcore/src/generated/main/java/org/wpilib/networktables/TimestampedString.java), [NetworkTableInstance](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/ntcore/src/generated/main/java/org/wpilib/networktables/NetworkTableInstance.java), [Timer](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/wpilibj/src/main/java/org/wpilib/system/Timer.java), [RobotController](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/wpilibj/src/main/java/org/wpilib/system/RobotController.java).

Normalize once at ingress into the explicitly named core timestamp domain. Conversions are `seconds = us / 1_000_000.0` or `seconds = ns / 1_000_000_000.0`; converting a wire `_us` integer to internal nanoseconds requires checked multiplication by 1,000. Never subtract timestamps from different clocks merely because their units match. A signed clock offset is not a capture timestamp. The alpha-7 migration notes explicitly retain microseconds in NT network communication and datalog files despite the Java raw-API change.

Keep capture time, publication metadata and local receipt time distinct. `publish_unix_us` is wall-clock logging metadata. NT metadata timestamps describe value updates, not camera exposure. Require known server identity/epoch and valid clock synchronization before treating `capture_server_us` as robot-time history. A desktop NT server is not the robot clock. Treat local `serverTime` sentinel values 0/1 as unavailable. Clock epoch changes, reconnects and source boot changes need explicit invalidation; they must not create object velocity.

The alpha-7 [PubSubOption contract](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/ntcore/src/main/java/org/wpilib/networktables/PubSubOption.java) still expresses `periodic` in seconds and provides bounded polling storage. These options do not provide a camera clock mapping, an end-to-end latency guarantee or unlimited queue capacity. Networking and JSON/file work belong outside the periodic control path.

## Geometry and capture-history requirements

Use SI metres, metres/second, radians and radians/second. Robot-relative geometry follows WPILib NWU: +X forward, +Y left, +Z up; planar heading is counterclockwise positive about +Z. See the official [coordinate conventions](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html). Preserve the configured fixed field origin across alliance changes. Map an alliance-relative goal to the fixed field frame exactly once; do not flip already field-fixed observations, estimator poses or planner results again. Season, field layout and obstacle geometry are versioned configuration, not inferred from controller year.

For an already robot-relative object at capture, apply only the capture-time robot-to-field transform. In 2D:

```text
x_field = x_robot_pose + cos(heading) * x_relative - sin(heading) * y_relative
y_field = y_robot_pose + sin(heading) * x_relative + cos(heading) * y_relative
```

Do not apply the camera mount a second time. Retain the source/calibration/mount revisions and the signed target-plane height used for ranging. Zero target-plane Z means floor only when the robot origin is on that floor. An estimated plane intersection is not independently measured object height.

Before sampling pose history, require capture time to lie within explicit retained oldest/newest bounds in the same epoch. Both audited [2026 PoseEstimator](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.2/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java) and [alpha-7 PoseEstimator](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/wpimath/src/main/java/org/wpilib/math/estimator/PoseEstimator.java) clamp `sampleAt` to the available history. Their insertion and sampling APIs take seconds. Their `resetPosition`/`resetPose` clear internal pose and vision history. A nonempty sampled pose therefore does not prove that an old/future capture time was valid. The selected API supplies pose and noise-setting methods, not a public live estimator covariance getter; robot ports must provide an explicitly conservative uncertainty model when no measured estimator uncertainty is available.

Both [2026 TimeInterpolatableBuffer](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.2/wpimath/src/main/java/edu/wpi/first/math/interpolation/TimeInterpolatableBuffer.java) and [alpha-7 TimeInterpolatableBuffer](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/wpimath/src/main/java/org/wpilib/math/interpolation/TimeInterpolatableBuffer.java) return a boundary sample outside retained bounds. Do not use their `Optional` presence as a bounds check. Corrections must rebuild/reproject retained relative measurements, or use a documented odometry-frame model with a corrected field transform. Differencing corrected field positions across a pose jump invents target motion.

## Vision prerequisite inspected read-only

The existing raw baseline is `Custom-Vision 2aee0fc1794b31539b02000d16791d4eec8df29a`. Its [schema-2 NetworkTables contract](https://github.com/Antigro09/Custom-Vision/blob/2aee0fc1794b31539b02000d16791d4eec8df29a/docs/networktables.md), [localization conventions](https://github.com/Antigro09/Custom-Vision/blob/2aee0fc1794b31539b02000d16791d4eec8df29a/docs/localization.md) and [object geometry](https://github.com/Antigro09/Custom-Vision/blob/2aee0fc1794b31539b02000d16791d4eec8df29a/custom_vision/object_geometry.py) establish the existing behavior. It publishes coherent `result` JSON, boot/frame identity and `objects.targets` robot-relative geometry at host camera-read completion. The target plane and mount must be known; missing prerequisites reject metric geometry. Legacy pixel bearings have different signs from the metric positive-left bearing.

The optional adapter passes 181 assertions against both protocol source and the
recorded local protocol jar; the producer has 27 matched fixtures. Tested source,
fixture and artifact hashes are in `exports/contract-manifest.json`. See
`VISION_ADAPTER.md` and `DEPENDENCIES.md` for public sources and reproduction.

The actual CVJ desktop profile is 2026.2.1 / Java 17; this document's original geometry/
time source audit used 2026.2.2, as did A*'s legacy demo. Keep those pins separate.
The 2026.2.1 [Timer source](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.1/wpilibj/src/main/java/edu/wpi/first/wpilibj/Timer.java)
and final CVJ native interoperability evidence support its own profile; they do
not establish a unified deployed controller image/DS/vendor inventory.

Empty or invalid packets clear current vision selection, but are not evidence that space is free. Raw camera track IDs offer brief association, not persistent field identity through occlusion. World-state estimates and forecasts must retain their distinct provenance and must not be fed back as independent camera measurements.

## Evidence and separately gated hardware checklist

This audit performed primary-source inspection and read-only local prerequisite checks. No WPILib controller build, NT hardware loopback, camera measurement, deployment, training, actuator test or real robot-code integration was performed. Core replay/contract tests reported elsewhere are software evidence only.

Before any separately approved hardware work:

- Pin the exact controller image, WPILib, Java, NT runtime and compatible vendor libraries for each selected profile; verify the event's current rules and FMS/Driver Station support.
- Measure NT clock mapping, reconnect/boot behavior, API-unit conversion and capture/history bounds on the selected controller. Verify capture-to-publication and receipt times independently.
- Measure camera-read versus exposure delay, intrinsic calibration, mount and robot-origin/target-plane geometry. Validate fixed field/alliance transforms with known poses.
- Supply authoritative robot mode, command ownership, localization, velocity, mechanism and possession feedback. Validate any conservative uncertainty model.
- Measure footprint, speed, sensing latency, stopping distance and collision margins before selecting operational thresholds. Verify independent collision stop, disable/lost-DS behavior and bounded pickup recovery without assuming guessed constants are safe.

Until those gates are approved and evidenced, the artifact remains a pure-Java component and fake-I/O replay example.
