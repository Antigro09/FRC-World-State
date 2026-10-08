#!/usr/bin/env python3
"""Verify content pins; optionally regenerate this repository's content hashes.

External checkouts are explicit, read-only inputs. Their paths and Git metadata
are never serialized. Public repository/revision pins are maintained separately
after verification against the remote; this offline tool cannot verify GitHub.
"""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MANIFEST = ROOT / 'exports/contract-manifest.json'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def hashes(paths):
    return {str(p.relative_to(ROOT)): sha(p) for p in sorted(paths)}


def verify_files(root, entries):
    for relative, expected in entries.items():
        path = Path(relative)
        if path.is_absolute() or '..' in path.parts:
            raise ValueError('invalid manifest path: ' + relative)
        if sha(root / path) != expected:
            raise ValueError('content hash mismatch: ' + relative)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--producer', type=Path, help='explicit Custom-Vision checkout')
    parser.add_argument('--vision-java', type=Path, help='explicit Custom-Vision-Java checkout')
    parser.add_argument('--planner', type=Path, help='explicit A* checkout')
    parser.add_argument('--artifacts', action='store_true', help='also verify recorded local CVJ jars; requires --vision-java')
    parser.add_argument('--write', action='store_true', help='regenerate only World-State hashes, retaining external pins')
    args = parser.parse_args()
    if args.artifacts and args.vision_java is None:
        parser.error('--artifacts requires --vision-java')
    manifest = json.loads(MANIFEST.read_text())
    world = manifest['world_state']
    current = {
        'source_sha256': hashes((ROOT / 'src/main/java').rglob('*.java')),
        'schema_sha256': hashes((ROOT / 'schemas').glob('*.json')),
        'fixture_sha256': hashes((ROOT / 'fixtures/prediction').glob('*.json')),
        'optional_adapter_sha256': hashes((ROOT / 'adapters/custom-vision-java/src/main/java').rglob('*.java')),
    }
    if not args.write:
        for key, entries in current.items():
            if entries != world[key]:
                raise ValueError('World-State pin mismatch: ' + key)
    for key, root in (('custom_vision', args.producer), ('custom_vision_java', args.vision_java), ('a_star', args.planner)):
        if root is not None:
            verify_files(root, manifest[key]['source_sha256'])
            print('Verified ' + key + ' source hashes')
    if args.artifacts:
        verify_files(args.vision_java, {item['path']: item['sha256'] for item in manifest['custom_vision_java']['artifacts']})
        print('Verified recorded CVJ artifact hashes (build products, not package coordinates)')
    if args.write:
        world.update(current)
        MANIFEST.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
        print('Updated World-State content hashes; external pins unchanged')
    else:
        print('Verified World-State core, adapter, schemas and fixtures')


if __name__ == '__main__':
    main()
