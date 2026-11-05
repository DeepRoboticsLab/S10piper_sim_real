#!/usr/bin/env python3
"""Offline file inventory and integrity checks. Never imports hardware code."""
import argparse
import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'verification/file_manifest.json'
SUMS = ROOT / 'SHA256SUMS'
EXCLUDED = {MANIFEST, SUMS}
PREFIX = '103/rl_deploy/'
KNOWN = {
    PREFIX+'policy/ppo/policy.onnx':
        'ab5a7b7ff15f84ab0d0d8fa028c4c556912c1b1eab0ee6f22231eb09ba8ea81d',
}


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def inventory():
    items = {}
    for base, dirs, files in os.walk(ROOT, followlinks=False):
        dirs[:] = [name for name in dirs if name not in {'.git', '__pycache__'} and
                   (Path(base)/name).relative_to(ROOT).as_posix() not in
                   {'.venv-sim', 'orin/piper/.venv', 'sim/logs', 'sim/bin', 'build',
                    '103/rl_deploy/build', '103/rl_deploy/build_vr',
                    '103/rl_deploy/install', '103/rl_deploy/install_vr',
                    '103/rl_deploy/log', '103/rl_deploy/log_vr',
                    '103/rl_deploy/build_drdds',
                    '103/rl_deploy/install_drdds',
                    '103/rl_deploy/log_drdds'}]
        for name in sorted(dirs + files):
            path = Path(base) / name
            if path in EXCLUDED:
                continue
            relative = path.relative_to(ROOT).as_posix()
            if path.is_symlink():
                items[relative] = {'type': 'symlink', 'target': os.readlink(path)}
            elif path.is_file():
                items[relative] = {'type': 'file', 'size': path.stat().st_size,
                                   'sha256': digest(path),
                                   'execute_bits': path.stat().st_mode & 0o111}
    return dict(sorted(items.items()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--create-manifest', action='store_true',
                        help='packager only: generate manifest after finalizing files')
    args = parser.parse_args()
    current = inventory()
    for name, expected in KNOWN.items():
        if current.get(name, {}).get('sha256') != expected:
            raise SystemExit('SOURCE_HASH_MISMATCH: '+name)
    if args.create_manifest:
        MANIFEST.write_text(json.dumps(current, indent=2, ensure_ascii=False)+'\n')
        lines = [entry['sha256']+'  '+name+'\n' for name,entry in current.items()
                 if entry['type']=='file']
        lines.append(digest(MANIFEST)+'  verification/file_manifest.json\n')
        SUMS.write_text(''.join(lines))
        print('MANIFEST_CREATED')
    else:
        expected = json.loads(MANIFEST.read_text())
        changes = sorted(name for name in set(current)|set(expected)
                         if current.get(name) != expected.get(name))
        if changes:
            raise SystemExit('PACKAGE_CHANGED:\n'+'\n'.join(changes))
    files = sum(entry['type']=='file' for entry in current.values())
    links = len(current)-files
    size = sum(entry.get('size', 0) for entry in current.values())
    print(f'PACKAGE_VERIFIED: {files} files, {links} symlinks, {size} bytes; '
          f'{len(KNOWN)} fixed policy hash and complete file manifest match; no hardware commands issued.')


if __name__=='__main__':
    main()
