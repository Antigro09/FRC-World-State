#!/bin/sh
# Local original Java libraries only. No download, installer, Maven publication or robot edits.
set -eu
cd "$(dirname "$0")/.."
CVJ_PROTOCOL_JAR=${1:?Usage: package-controller.sh CVJ_PROTOCOL_JAR}
rm -rf build/controller/core build/controller/bridge
mkdir -p build/controller/core build/controller/bridge
find src/main/java -name '*.java' -print | sort > build/controller/core-sources.txt
find adapters/custom-vision-java/src/main/java -name '*.java' -print | sort > build/controller/bridge-sources.txt
javac --release 17 -Xlint:all -Werror -d build/controller/core @build/controller/core-sources.txt
javac --release 17 -Xlint:all -Werror -cp "build/controller/core:$CVJ_PROTOCOL_JAR" -d build/controller/bridge @build/controller/bridge-sources.txt
jar --create --date=2026-10-08T00:00:00Z --file build/controller/frc-world-state-core-0.1.0-local.jar -C build/controller/core .
jar --create --date=2026-10-08T00:00:00Z --file build/controller/frc-world-state-vision-0.1.0-local.jar -C build/controller/bridge .
jar --create --date=2026-10-08T00:00:00Z --file build/controller/frc-world-state-core-0.1.0-local-sources.jar -C src/main/java .
jar --create --date=2026-10-08T00:00:00Z --file build/controller/frc-world-state-vision-0.1.0-local-sources.jar -C adapters/custom-vision-java/src/main/java .
python3 - "$CVJ_PROTOCOL_JAR" <<'PY'
import hashlib
import json
from pathlib import Path
import struct
import sys
import zipfile

root = Path('build/controller')
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
files = []
for path in sorted(root.glob('*.jar')):
    with zipfile.ZipFile(path) as archive:
        expected = 'org/frcworldstate/vision/' if '-vision-' in path.name else 'org/frcworldstate/core/'
        sources = '-sources.jar' in path.name
        for name in archive.namelist():
            if name.endswith('/') or name.startswith('META-INF/'):
                continue
            if not name.startswith(expected) or not name.endswith('.java' if sources else '.class'):
                raise ValueError('unexpected controller jar content: ' + name)
            if not sources:
                data = archive.read(name)
                if data[:4] != b'\xca\xfe\xba\xbe' or struct.unpack('>H', data[6:8])[0] != 61:
                    raise ValueError('controller common bytecode must be Java 17: ' + name)
    files.append({'path': path.name, 'bytes': path.stat().st_size, 'sha256': sha(path)})
manifest = {
    'format': 'frc-world-state-controller-local/1', 'published': False,
    'java_release': 17, 'files': files,
    'vision_protocol_input_sha256': sha(Path(sys.argv[1])),
    'core_dependency': 'java.base',
    'vision_adapter_required_external_libraries': ['Custom-Vision-Java protocol with admitted Measurement provenance and DeliveryGate'],
    'excluded': ['WPILib/native libraries', 'planner/model implementations', 'Python/Jetson/GUI/server',
                 'test classes', 'fixtures', 'weights', 'robot project and motor bindings'],
    'profiles': ['WPILib 2026 + roboRIO', 'WPILib 2027 alpha-7 + Systemcore experimental'],
    'hardware': 'UNRUN; actual robot integration ON HOLD',
}
(root / 'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
print('Built four local controller jars; manifest: build/controller/manifest.json')
PY
