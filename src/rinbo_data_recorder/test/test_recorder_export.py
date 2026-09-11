"""Filesystem-only export and cleanup contract tests."""
import importlib.util
from pathlib import Path
import tarfile

import pytest

ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('recorder_client',ROOT/'tools/rinbo_recorder_client.py')
client=importlib.util.module_from_spec(spec);spec.loader.exec_module(client)


def test_export_uses_acknowledged_lengths_and_keeps_original(tmp_path):
    run=tmp_path/'run';run.mkdir();data=run/'summary.csv';data.write_text('a\n1\n2\n')
    result=client.snapshot_export(dict(run_dir=str(run),files=[dict(name=data.name,path=str(data),bytes=4)],recording=True))
    assert result['recording']
    with tarfile.open(result['export']['path']) as archive:
        assert archive.extractfile('run/summary.csv').read()==b'a\n1\n'
    assert data.read_text()=='a\n1\n2\n'


def test_export_fails_on_truncation_and_does_not_overwrite(tmp_path):
    run=tmp_path/'run';run.mkdir();data=run/'summary.csv';data.write_text('x')
    with pytest.raises(RuntimeError,match='truncated'):
        client.snapshot_export(dict(run_dir=str(run),files=[dict(name=data.name,path=str(data),bytes=5)]))
    assert data.read_text()=='x'


def test_export_rejects_symlink(tmp_path):
    run=tmp_path/'run';run.mkdir();data=run/'summary.csv';data.symlink_to(tmp_path/'secret')
    with pytest.raises(RuntimeError,match='invalid_export'):
        client.snapshot_export(dict(run_dir=str(run),files=[dict(name=data.name,path=str(data),bytes=5)]))


def test_native_cleanup_targets_exclude_recorder_and_mirror():
    import sys
    sys.path.insert(0,str(ROOT/'src/rinbo_control'))
    from rinbo_control.connection_recovery import MOTION_NAMES
    assert 'rinbo_data_recorder' not in MOTION_NAMES
    assert 'rslip_recorder_mirror' not in MOTION_NAMES
