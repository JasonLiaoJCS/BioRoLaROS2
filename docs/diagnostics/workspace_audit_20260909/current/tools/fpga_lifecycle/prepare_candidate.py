#!/usr/bin/env python3
"""Validate the baseline, then create a separate source candidate; never deploy/run."""
import hashlib
import json
from pathlib import Path
import shutil
import sys

package=Path(__file__).resolve().parent
source=Path(sys.argv[1]).resolve()
dest=Path(sys.argv[2]).resolve()
manifest=json.loads((package/'baseline-sha256.json').read_text())
if dest.exists() or source==dest or source in dest.parents:
    raise SystemExit('Destination must be a new directory outside the source project')
for name,digest in manifest.items():
    path=source/name
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
        raise SystemExit(f'Baseline changed: {path}; preserve changes and rebase this patch')
if (source/'include/driver_lifecycle.hpp').exists() or (source/'fpga-service').exists():
    raise SystemExit('driver_lifecycle.hpp already exists; inspect before applying')
shutil.copytree(source,dest,ignore=shutil.ignore_patterns('build','console-backups','.git'))
for folder in ('src','include'):
    for path in (package/folder).iterdir():
        shutil.copy2(path,dest/folder/path.name)
shutil.copy2(package/'fpga-service',dest/'fpga-service')
print(f'Candidate source only: {dest}; original source and deployed binary untouched')
print('Preserve the parent workspace third_party/robot_protos layout for a fresh CMake build.')
