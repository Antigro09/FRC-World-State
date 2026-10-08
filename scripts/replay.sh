#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
./scripts/test.sh
mkdir -p build/example-classes
find examples/src/main/java -name '*.java' -print | sort > build/example-sources.txt
javac --release 17 -Xlint:all -Werror -cp build/classes -d build/example-classes @build/example-sources.txt
java -ea -cp build/classes:build/example-classes org.frcworldstate.example.FakeIoReplay
