# FRC-World-State

A reusable **Java 17, controller-independent component**, not a robot project.
It owns immutable ego/object/snapshot types, deterministic tracking, planner and
predictor contracts, pickup task decisions and independent collision supervision.
Robot-owned adapters supply facts and execute intentions. This library cannot
command motors, redefine robot facts or declare success from a path alone.

The immediate integration target is **WPILib 2026 + roboRIO**; the final CVJ adapter
profile pins **2026.2.1 / Java 17**. The core source audit and A* legacy demo use
2026.2.2, which is recorded separately. A separately
pinned **2027.0.0-alpha-7 + Systemcore** adapter is experimental. There is no
official WPILib 2026 + Systemcore target in the inspected compatibility matrix;
this repository does not invent one. Common bytecode is Java 17 (major 61).
See [verified compatibility](docs/COMPATIBILITY.md) for source pins, Java 25 in
the 2027 profile, units and clock/history obligations.

## Build and verification

With a JDK 17+ and Python 3, no dependency download or GPU is required:

```sh
./scripts/test.sh
python3 scripts/verify_prediction_fixtures.py
./scripts/replay.sh
jdeps -s build/frc-world-state-0.1.0.jar
```

The first command builds the entire core without WPILib, CVJ, A*, model runtime,
or vendor libraries, runs the Java acceptance suites and writes the local jar.
The replay is scripted fake I/O on an empty synthetic map. It is software evidence,
not robot/camera/mechanism qualification. `build.gradle` also provides `contractTest`
and `check`; the dependency-free shell route is the tested build entry point.

Optional integration checks require explicitly supplied dependency checkouts:

```sh
./scripts/test-vision-adapter.sh /path/to/Custom-Vision-Java /path/to/Custom-Vision
./scripts/test-planner-integration.sh /path/to/1086-On-The-Fly-A-Star
```

See [dependency pins and setup](docs/DEPENDENCIES.md) for public sources, exact
content hashes and optional binary verification. These scripts do not fetch code
or assume another workspace's directory structure. The core needs no sibling
repository or published package. No GitHub Actions workflow is supplied; the
commands above are local software verification.

## Contracts and architecture

- [Architecture and reset/tracking decisions](docs/ARCHITECTURE.md)
- [Planner contract handoff](docs/PLANNER_CONTRACT.md)
- [Prediction contract, JSON schemas and golden vectors](docs/PREDICTION_CONTRACT.md)
- [Optional vision admission/geometry adapter](docs/VISION_ADAPTER.md)
- [Acceptance results and remaining limits](docs/IMPLEMENTATION_REPORT.md)
- [Separately gated hardware checklist](docs/HARDWARE_CHECKLIST.md)

Actual robot integration (prompt 6), deployment, motor operation and training are
on hold. See the hardware checklist before any separately authorized hardware work.
