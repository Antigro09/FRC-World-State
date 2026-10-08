#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
mkdir -p build/classes build/test-classes
find src/main/java -name '*.java' -print | sort > build/main-sources.txt
find src/test/java -name '*.java' -print | sort > build/test-sources.txt
javac --release 17 -Xlint:all -Werror -d build/classes @build/main-sources.txt
javac --release 17 -Xlint:all -Werror -cp build/classes -d build/test-classes @build/test-sources.txt
java -ea -cp build/classes:build/test-classes org.frcworldstate.core.AcceptanceTest
jar --create --date=2026-10-08T00:00:00Z --file build/frc-world-state-0.1.0.jar -C build/classes .
