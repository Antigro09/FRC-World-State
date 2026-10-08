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
metadata, not a buildable pin for this bridge. No feature publication is claimed.
The manifest records exact matched feature source and local protocol jar hashes;
a null feature commit means the owner source commit is still pending. Core-only
tests and the A* check do not require CVJ.

Clone producer/planner into a directory you choose, supply an explicitly available
matched CVJ feature checkout, and pass those paths to the scripts:

```sh
DEPS="$PWD/../frc-dependencies"
mkdir -p "$DEPS"
git clone https://github.com/Antigro09/Custom-Vision.git "$DEPS/Custom-Vision"
git -C "$DEPS/Custom-Vision" checkout --detach fd093ef2948e7ac206727fff5cc7dc2a0288347b
git clone https://github.com/Antigro09/1086-On-The-Fly-A-Star.git "$DEPS/1086-On-The-Fly-A-Star"
git -C "$DEPS/1086-On-The-Fly-A-Star" checkout --detach 59ad897d895315a751df67c5751e30370850a784
CVJ=/path/to/matched-local-Custom-Vision-Java-feature
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
verified for the baseline; the CVJ follow-up is a separate local prerequisite.
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
