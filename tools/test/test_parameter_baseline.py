"""Filesystem-only recovery tests. No ROS nodes, services, or real site writes."""
import copy
import fcntl
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('parameter_baseline', ROOT / 'tools/parameter_baseline.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class StandardTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='rinbo-baseline-test-')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.bundle = self.root / 'bundle'
        shutil.copytree(module.BASELINE, self.bundle)
        self.site = self.root / 'site.yaml'
        shutil.copy2(self.bundle / 'native.yaml', self.site)
        self.settings = self.root / 'settings.json'
        self.settings.write_text(json.dumps({'plan': json.loads((self.bundle / 'manual-plan.json').read_text()),
                                            'ip': 'example-device', 'credential': 'test-only-placeholder'}))
        manifest = json.loads((self.bundle / 'manifest.json').read_text())
        manifest['binary_files'] = {}
        for name in manifest['source_files']:
            dest = self.root / name
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(self.bundle / 'sources' / name, dest)
        (self.bundle / 'manifest.json').write_text(json.dumps(manifest))
        self.manifest_hash = module.sha(self.bundle / 'manifest.json')
        self.calls, self.busy, self.read_mismatch = [], False, False
        self.standard = module.Standard(self.root, self.bundle, self.site, self.settings,
                                        self.root / 'backups', self.runner, self.manifest_hash)
        self.receipt = Path(str(self.site) + '.calibration.json')
        self.receipt.write_text('{"old_success":true}')

    def runner(self, args):
        self.calls.append(args)
        if args == ['assert-idle']:
            if self.busy: raise RuntimeError('mock motion active')
            return {'idle': True}
        assert args == ['status', '--json'], args
        try:
            document = yaml.safe_load(self.site.read_text())
            params = {}
            def flatten(values, prefix=''):
                out = {}
                for key, value in values.items():
                    name = prefix + '.' + key if prefix else key
                    if isinstance(value, dict): out.update(flatten(value, name))
                    else: out[name] = value
                return out
            for node, values in document['parameters'].items():
                if node != 'rinbo_manual': params[node] = flatten(values)
            params['rinbo_manual'] = copy.deepcopy(params['rinbo_standing'])
            params['rinbo_manual'].update(flatten(document['parameters']['rinbo_manual']))
            for values in params.values():
                values['hardware.disabled_legs'] = document['disabled_legs']
                values['hardware.max_disabled_legs'] = 6
            if self.read_mismatch and document['revision'] > 15:
                params['rinbo_cali']['kp'] = 999
            return dict(schema_version=document['schema_version'], revision=document['revision'],
                        disabled_legs=document['disabled_legs'], test_mode=document['test_mode'], parameters=params)
        except (yaml.YAMLError, TypeError, KeyError) as exc:
            raise RuntimeError('mock invalid config') from exc

    def change(self):
        doc = yaml.safe_load(self.site.read_text())
        doc['parameters']['rinbo_cali']['kp'] = .9
        doc['disabled_legs'] = []
        self.site.write_text(yaml.safe_dump(doc))

    def test_matching_standard_is_noop_and_preserves_receipts(self):
        before = self.site.read_bytes()
        self.assertFalse(any(self.standard.inspect().values()))
        result = self.standard.restore()
        self.assertFalse(result['changed'])
        self.assertEqual(before, self.site.read_bytes())
        self.assertTrue(self.receipt.exists())
        self.assertFalse((self.root / 'backups').exists())

    def test_restore_all_native_values_revision_backup_and_no_motor_actions(self):
        self.change()
        before = self.site.read_bytes()
        settings = self.settings.read_bytes()
        self.assertTrue(self.standard.inspect()['parameter_changes'])
        result = self.standard.restore()
        self.assertEqual(result['revision'], 16)
        self.assertEqual((Path(result['backup']) / 'site-before.yaml').read_bytes(), before)
        self.assertTrue((Path(result['backup']) / self.receipt.name).exists())
        self.assertFalse(self.receipt.exists())
        self.assertEqual(self.settings.read_bytes(), settings)
        self.assertFalse(any(self.standard.inspect().values()))
        self.assertTrue(all(c in (['assert-idle'], ['status', '--json']) for c in self.calls))

    def test_optional_plan_restores_only_plan_preserves_other_preferences(self):
        data = json.loads(self.settings.read_text())
        data['plan']['max_pwm'] = 400
        self.settings.write_text(json.dumps(data))
        before = self.site.read_bytes()
        self.assertFalse(any(self.standard.inspect().values()))
        self.assertTrue(self.standard.inspect(True)['plan_changes'])
        result = self.standard.restore(True)
        self.assertFalse(result['native_changed'])
        self.assertEqual(before, self.site.read_bytes())
        self.assertTrue(self.receipt.exists())
        restored = json.loads(self.settings.read_text())
        self.assertEqual(restored['plan']['max_pwm'], 80)
        self.assertEqual(restored['credential'], data['credential'])
        self.assertEqual(restored['ip'], data['ip'])

    def test_busy_process_does_not_change_files(self):
        self.change(); self.busy = True
        before = self.site.read_bytes()
        with self.assertRaisesRegex(RuntimeError, 'active'): self.standard.restore()
        self.assertEqual(before, self.site.read_bytes()); self.assertTrue(self.receipt.exists())

    def test_both_native_locks_refuse_concurrent_restore(self):
        self.change(); before = self.site.read_bytes()
        for suffix in ('.motion.lock', '.lock'):
            with open(str(self.site) + suffix, 'a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                with self.assertRaises(BlockingIOError): self.standard.restore()
            self.assertEqual(before, self.site.read_bytes()); self.assertTrue(self.receipt.exists())

    def test_tampered_baseline_fails_integrity_before_any_restore(self):
        with (self.bundle / 'native.yaml').open('a') as f: f.write('\n# accidental edit\n')
        with self.assertRaisesRegex(RuntimeError, '完整性'):
            module.Standard(self.root, self.bundle, self.site, self.settings, self.root / 'backups',
                            self.runner, self.manifest_hash)

    def test_bad_revision_requires_explicit_new_revision(self):
        self.site.write_text('revision: [broken\n')
        before = self.site.read_bytes()
        with self.assertRaisesRegex(RuntimeError, 'revision'): self.standard.restore()
        self.assertEqual(before, self.site.read_bytes())
        result = self.standard.restore(revision=99)
        self.assertEqual(result['revision'], 99)
        self.assertFalse(self.standard.inspect()['parameter_changes'])

    def test_revision_cannot_go_backwards(self):
        self.change()
        document = yaml.safe_load(self.site.read_text()); document['revision'] = 30
        self.site.write_text(yaml.safe_dump(document))
        with self.assertRaises(RuntimeError): self.standard.restore(revision=16)
        self.assertEqual(yaml.safe_load(self.site.read_text())['revision'], 30)
        self.assertEqual(self.standard.restore()['revision'], 31)

    def test_native_readback_mismatch_rolls_back_and_keeps_receipts_invalid(self):
        self.change(); self.read_mismatch = True
        before = self.site.read_bytes()
        with self.assertRaisesRegex(RuntimeError, '讀回'): self.standard.restore()
        self.assertEqual(before, self.site.read_bytes()); self.assertFalse(self.receipt.exists())
        self.assertEqual(len(list((self.root / 'backups').iterdir())), 1)

    def test_source_change_is_reported_not_overwritten(self):
        name = next(iter(self.standard.manifest['source_files']))
        code = self.root / name
        code.write_text(code.read_text() + '\n// future unrelated repair\n')
        before = code.read_bytes()
        self.assertIn(name, self.standard.inspect()['source_or_binary_changes'])
        self.change(); result = self.standard.restore()
        self.assertEqual(before, code.read_bytes()); self.assertIn(name, result['review_sources'])


if __name__ == '__main__': unittest.main()
