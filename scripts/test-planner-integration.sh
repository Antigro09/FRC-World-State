#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
ASTAR_CHECKOUT=${1:?Usage: test-planner-integration.sh ASTAR_CHECKOUT}
mkdir -p build/planner-cross
find "$ASTAR_CHECKOUT/src/main/java/pathplanning/geometric" -name '*.java' -print | sort > build/planner-cross/sources.txt
find "$ASTAR_CHECKOUT/src/worldStateAdapter/java" -name '*.java' -print | sort >> build/planner-cross/sources.txt
find "$ASTAR_CHECKOUT/src/integrationTest/java" -name '*.java' -print | sort >> build/planner-cross/sources.txt
find adapters/planner/src/test/java -name '*.java' -print | sort >> build/planner-cross/sources.txt
javac --release 17 -Xlint:all -Werror -cp build/classes -d build/planner-cross @build/planner-cross/sources.txt
java -ea -cp build/classes:build/planner-cross pathplanning.backend.WorldStateEnvelopeIntegrationSuite
java -ea -cp build/classes:build/planner-cross org.frcworldstate.planner.TargetSelectionIntegrationTest
