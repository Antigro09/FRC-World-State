#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
CVJ_CHECKOUT=${1:?Usage: test-vision-adapter.sh CVJ_CHECKOUT PRODUCER_CHECKOUT [CVJ_PROTOCOL_JAR]}
PRODUCER_CHECKOUT=${2:?Usage: test-vision-adapter.sh CVJ_CHECKOUT PRODUCER_CHECKOUT [CVJ_PROTOCOL_JAR]}
mkdir -p build/vision/cvj build/vision/adapter
find "$CVJ_CHECKOUT/protocol/src/main/java" -name '*.java' -print | sort > build/vision/cvj-sources.txt
find adapters/custom-vision-java/src -name '*.java' -print | sort > build/vision/adapter-sources.txt
# CVJ compiles independently of World-State; adapter is the only module that sees both.
javac --release 17 -Xlint:all -Werror -d build/vision/cvj @build/vision/cvj-sources.txt
javac --release 17 -Xlint:all -Werror -cp build/classes:build/vision/cvj -d build/vision/adapter @build/vision/adapter-sources.txt
java -ea -cp build/classes:build/vision/cvj:build/vision/adapter org.frcworldstate.vision.CustomVisionAdapterTest "$PRODUCER_CHECKOUT/protocol/fixtures"

# Optional third argument verifies the same adapter against the exact owner binary artifact.
if [ "$#" -ge 3 ]; then
    CVJ_PROTOCOL_JAR=$3
    mkdir -p build/vision/artifact-adapter
    javac --release 17 -Xlint:all -Werror -cp "build/classes:$CVJ_PROTOCOL_JAR" -d build/vision/artifact-adapter @build/vision/adapter-sources.txt
    java -ea -cp "build/classes:$CVJ_PROTOCOL_JAR:build/vision/artifact-adapter" org.frcworldstate.vision.CustomVisionAdapterTest "$PRODUCER_CHECKOUT/protocol/fixtures"
fi
