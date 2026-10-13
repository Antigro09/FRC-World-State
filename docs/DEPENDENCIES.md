# Optional dependencies and content pins

The core, JSON contract tests and fake-I/O replay use only JDK 17+ and Python 3.
They require no external checkout, model runtime, WPILib installation or package
publication. Run the commands in the README from this repository's root.

The unchanged producer and planner contracts retain these public source revisions:

| Component | Public repository | Exact revision |
|---|---|---|
| Raw vision schemas and golden fixture | [Custom-Vision](https://github.com/Antigro09/Custom-Vision) | `fd093ef2948e7ac206727fff5cc7dc2a0288347b` |
| Geometric planner backend | [1086-On-The-Fly-A-Star](https://github.com/Antigro09/1086-On-The-Fly-A-Star) | `59ad897d895315a751df67c5751e30370850a784` |

Pin the revisions explicitly; a repository's default branch may contain a different
version. The admitted-observation follow-up requires CVJ's **local feature API**:
`VisionClient.Measurement` retains actual admission/full clock/transport provenance,
and `VisionClient.DeliveryGate` validates current originating-client and facade
eligibility. The older public baseline `ae67886da70223e9ee73683ee2bcd00dc3221469`
does not supply that API. Its public revision is retained as historical baseline
metadata, not a buildable pin for this bridge. The matching API is published on
[CVJ's feature/captain-api branch](https://github.com/Antigro09/Custom-Vision-Java/tree/feature/captain-api).
The final handoff is pinned to
`6639c8fc70c5d1b8b88e93711614c0b480b42efe`, with implementation revision
`12ee0ab5ede1e0d51c561f8a3d245a4e0822c926`. The handoff changes metadata only;
both revisions have the same verified protocol sources and jar bytes. The manifest
records those semantic source/artifact pins. Core-only tests and the A* check do
not require CVJ.

Clone the pinned feature sources into a directory you choose and pass those paths
to the scripts:

```sh
DEPS="$PWD/../frc-dependencies"
mkdir -p "$DEPS"
git clone https://github.com/Antigro09/Custom-Vision.git "$DEPS/Custom-Vision"
git -C "$DEPS/Custom-Vision" checkout --detach fd093ef2948e7ac206727fff5cc7dc2a0288347b
git clone https://github.com/Antigro09/Custom-Vision-Java.git "$DEPS/Custom-Vision-Java"
git -C "$DEPS/Custom-Vision-Java" checkout --detach 6639c8fc70c5d1b8b88e93711614c0b480b42efe
git clone https://github.com/Antigro09/1086-On-The-Fly-A-Star.git "$DEPS/1086-On-The-Fly-A-Star"
git -C "$DEPS/1086-On-The-Fly-A-Star" checkout --detach 59ad897d895315a751df67c5751e30370850a784
CVJ="$DEPS/Custom-Vision-Java"
python3 scripts/export-contracts.py \
  --producer "$DEPS/Custom-Vision" \
  --vision-java "$CVJ" \
  --planner "$DEPS/1086-On-The-Fly-A-Star"
./scripts/test.sh
./scripts/test-vision-adapter.sh "$CVJ" "$DEPS/Custom-Vision"
./scripts/test-planner-integration.sh "$DEPS/1086-On-The-Fly-A-Star"
```

Already available local checkouts are explicit overrides: supply their paths in
place of the cloned directories. The verifier rejects any recorded source content
mismatch. Neither test script fetches code or edits a dependency. The verifier
checks selected content hashes offline. Producer/planner public reachability was
verified for the baseline; the exact CVJ feature ref was verified after its
authorized publication. Its source/artifact pins are unchanged.
`--write` regenerates only this
repository's content hashes and retains external pins; it does not approve a new
dependency version.

`exports/contract-manifest.json` records core/adapter/schema/fixture hashes and
external content pins. Its CVJ artifact hashes identify the matched local protocol
binary/source jars tested with the follow-up; these are not Maven
coordinates or files promised in a source clone. An explicitly supplied protocol
jar can be tested as the third argument to `test-vision-adapter.sh` and used by
`package-controller.sh` to build original core/bridge local jars. To verify all
recorded jars in a populated CVJ build checkout, add `--artifacts` to the content
verifier. That optional binary check requires those exact build products.

The model consumes World-State's exported prediction schema. It is not a core
dependency. Model-specific uncertainty calibration, robot configuration and
physical execution evidence remain prerequisites for accepting learned output.

All source in this repository is original to this component; no third-party source
or binaries are vendored. WPILib source citations document audited contracts.
Optional dependencies remain separate and retain their own license and attribution
requirements. No new license for the original component is selected by this
publication.
