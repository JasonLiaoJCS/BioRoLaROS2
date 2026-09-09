#!/usr/bin/env python3
"""Inspect or explicitly restore the approved parameter standard. Never runs ROS motion."""
import argparse
import contextlib
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import yaml

ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / 'config/parameter_baselines/standard_20260909_r15'
MANIFEST_SHA256 = 'eb75690b53a00c149d202973e37ed7c4751347119fe6d5bae904537186c3ae5f'
SITE = Path('/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml')
SETTINGS = Path('/home/jetson/.local/state/rinbo_control/settings.json')
BACKUPS = Path('/home/jetson/.local/state/rinbo-control-backups')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def differences(expected, actual, prefix=''):
    if isinstance(expected, dict) and isinstance(actual, dict):
        out = []
        for key in sorted(expected.keys() | actual.keys()):
            label = prefix + '.' + key if prefix else key
            if key not in actual:
                out.append(f'{label}: 目前缺少；基準={expected[key]!r}')
            elif key not in expected:
                out.append(f'{label}: 目前新增={actual[key]!r}')
            else:
                out += differences(expected[key], actual[key], label)
        return out
    if expected != actual or isinstance(expected, bool) != isinstance(actual, bool):
        return [f'{prefix}: 目前={actual!r}；基準={expected!r}']
    return []


def control_values(effective):
    return {k: effective[k] for k in ('schema_version', 'disabled_legs', 'test_mode', 'parameters')}


class Standard:
    # Path/runner injection exists only for offline tests; the CLI has fixed live paths.
    def __init__(self, root=ROOT, bundle=BASELINE, site=SITE, settings=SETTINGS,
                 backups=BACKUPS, runner=None, manifest_hash=MANIFEST_SHA256):
        self.root, self.bundle, self.site = Path(root), Path(bundle), Path(site)
        self.settings, self.backups = Path(settings), Path(backups)
        self.runner = runner or self._run
        if sha(self.bundle / 'manifest.json') != manifest_hash:
            raise RuntimeError('基準 manifest 已變動，停止使用；請先核對原始標準，不能自行重新產生雜湊蓋過問題。')
        self.manifest = json.loads((self.bundle / 'manifest.json').read_text())
        for name, digest in self.manifest['files'].items():
            path = self.bundle / name
            if not path.is_relative_to(self.bundle) or '..' in Path(name).parts or sha(path) != digest:
                raise RuntimeError(f'基準副本完整性不符：{name}')
        self.expected = json.loads((self.bundle / 'effective.json').read_text())

    def _run(self, operation):
        result = subprocess.run([str(self.root / 'build/rinbo_fsm/rinbo_legs'), *operation],
                                text=True, capture_output=True, timeout=15)
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip() or '原生設定工具失敗')
        return json.loads(result.stdout)

    def read(self):
        return self.runner(['status', '--json'])

    def code_changes(self):
        out = []
        for name, expected in self.manifest['source_files'].items():
            p = self.root / name
            if not p.exists() or sha(p) != expected:
                out.append(name)
        for name, expected in self.manifest['binary_files'].items():
            relative = Path(name).relative_to(ROOT)
            p = self.root / relative
            if not p.exists() or sha(p) != expected:
                out.append(str(relative))
        return out

    def inspect(self, include_plan=False):
        errors = []
        try:
            live = self.read()
            changes = differences(control_values(self.expected), control_values(live))
        except (RuntimeError, ValueError, KeyError) as exc:
            changes = []
            errors.append('目前有效設定無法讀取：' + str(exc))
        plan_changes = []
        if include_plan:
            saved = json.loads(self.settings.read_text())
            plan_changes = differences(json.loads((self.bundle / 'manual-plan.json').read_text()),
                                       saved.get('plan'), 'manual_plan')
        return {'parameter_changes': changes, 'read_errors': errors,
                'plan_changes': plan_changes, 'source_or_binary_changes': self.code_changes()}

    @staticmethod
    def atomic_write(path, data, mode=0o600):
        fd, name = tempfile.mkstemp(prefix=path.name + '.baseline-', dir=path.parent)
        try:
            with os.fdopen(fd, 'wb') as stream:
                stream.write(data)
                stream.flush()
                os.fchmod(stream.fileno(), mode)
                os.fsync(stream.fileno())
            os.replace(name, path)
            fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        finally:
            Path(name).unlink(missing_ok=True)

    def restore(self, include_plan=False, revision=None):
        with contextlib.ExitStack() as stack:
            # Blocking the motion lock first allows the native idle command to
            # take its own configuration lock. A new MotionSession cannot run.
            motion = stack.enter_context(open(str(self.site) + '.motion.lock', 'a'))
            fcntl.flock(motion, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.runner(['assert-idle'])  # Read-only process gate; never stops anything.
            config = stack.enter_context(open(str(self.site) + '.lock', 'a'))
            fcntl.flock(config, fcntl.LOCK_EX | fcntl.LOCK_NB)
            inspection = self.inspect(include_plan)
            native_changed = bool(inspection['read_errors'] or inspection['parameter_changes'])
            plan_changed = bool(inspection['plan_changes'])
            if not native_changed and not plan_changed:
                return {'changed': False, 'message': '設定已符合基準，沒有寫檔或變更 revision／完成紀錄。',
                        'review_sources': inspection['source_or_binary_changes']}
            old = self.site.read_bytes()
            try:
                current = yaml.safe_load(old)
                current_revision = current['revision']
                if isinstance(current_revision, bool) or not isinstance(current_revision, int) or current_revision < 1:
                    raise ValueError('invalid revision')
            except (yaml.YAMLError, TypeError, KeyError, ValueError):
                if revision is None:
                    raise RuntimeError('無法辨識目前 revision。請確認最近編號後，用 --revision 新編號 明確指定；不會直接退回15。')
                current_revision = self.manifest['original_revision']
            next_revision = max(current_revision, self.manifest['original_revision']) + 1 if revision is None else revision
            if next_revision <= max(current_revision, self.manifest['original_revision']) or next_revision > 2**63 - 1:
                raise RuntimeError('新 revision 必須大於目前與基準編號，且在 int64 範圍內。')
            candidate = yaml.safe_load((self.bundle / 'native.yaml').read_text())
            candidate['revision'] = next_revision
            data = yaml.safe_dump(candidate, allow_unicode=True, sort_keys=False).encode()
            settings_old = self.settings.read_bytes() if plan_changed else None
            if plan_changed:
                settings = json.loads(settings_old)
                settings['plan'] = json.loads((self.bundle / 'manual-plan.json').read_text())
                settings_new = (json.dumps(settings, ensure_ascii=False, indent=2) + '\n').encode()
            self.backups.mkdir(parents=True, exist_ok=True)
            backup = Path(tempfile.mkdtemp(prefix='parameter-baseline-', dir=self.backups))
            (backup / 'site-before.yaml').write_bytes(old)
            receipts = [Path(str(self.site) + suffix) for suffix in ('.calibration.json', '.standing.json')]
            for receipt in receipts:
                if receipt.exists(): shutil.copy2(receipt, backup / receipt.name)
            if plan_changed: (backup / 'settings-before.json').write_bytes(settings_old)
            site_mode = self.site.stat().st_mode & 0o777
            settings_mode = self.settings.stat().st_mode & 0o777 if plan_changed else 0o600
            site_written, plan_written = False, False
            try:
                if native_changed:
                    for receipt in receipts: receipt.unlink(missing_ok=True)
                    site_written = True
                    self.atomic_write(self.site, data, site_mode)
                    after = self.read()  # Parses config without initializing ROS.
                    mismatch = differences(control_values(self.expected), control_values(after))
                    if mismatch or after['revision'] != next_revision:
                        raise RuntimeError('恢復後原生讀回與基準不符：' + '; '.join(mismatch))
                if plan_changed:
                    if self.settings.read_bytes() != settings_old:
                        raise RuntimeError('操作台偏好在恢復期間變動，停止覆寫。')
                    plan_written = True
                    self.atomic_write(self.settings, settings_new, settings_mode)
                result = {'changed': True, 'native_changed': native_changed, 'plan_changed': plan_changed,
                          'revision': next_revision if native_changed else current_revision,
                          'backup': str(backup), 'review_sources': inspection['source_or_binary_changes'],
                          'hardware_actions': [], 'restored_at': datetime.datetime.now().astimezone().isoformat()}
                (backup / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
                return result
            except BaseException:
                if site_written: self.atomic_write(self.site, old, site_mode)
                if plan_written: self.atomic_write(self.settings, settings_old, settings_mode)
                # Failed/uncertain updates never revive historical completion receipts.
                raise


def main():
    parser = argparse.ArgumentParser(description='重要參數基準：檢查／預覽／明確恢復。不上電、不啟停任何服務。')
    parser.add_argument('operation', choices=['check', 'restore'])
    parser.add_argument('--apply', action='store_true', help='實際寫入恢復；未指定時只預覽')
    parser.add_argument('--include-plan', action='store_true', help='另包含儲存的L2動作計畫，保留其他偏好與連線資料')
    parser.add_argument('--revision', type=int, help='只有目前revision損壞或需要明確新編號時使用，必須遞增')
    args = parser.parse_args()
    if args.operation != 'restore' and (args.apply or args.revision is not None):
        parser.error('--apply／--revision 只適用 restore')
    try:
        standard = Standard()
        if args.operation == 'restore' and args.apply:
            result = standard.restore(args.include_plan, args.revision)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            if result['changed']:
                print('設定已恢復；沒有上電或啟停程序。若原生設定改變，下次需重新完成校正／站立。')
            else:
                print(result['message'])
            if result['review_sources']:
                print('另有來源／執行檔差異需核對；本指令沒有回復整支程式。')
            return 0
        result = standard.inspect(args.include_plan)
        print('基準：' + standard.manifest['baseline_id'])
        if args.operation == 'restore': print('僅預覽，未寫入；確認差異後加 --apply 才會恢復。')
        for key in ('read_errors', 'parameter_changes', 'plan_changes'):
            for item in result[key]: print(item)
        if result['source_or_binary_changes']:
            print('來源／執行檔有變動（不代表參數一定改了，需比對基準來源）：')
            for name in result['source_or_binary_changes']: print('  ' + name)
        if not any(result.values()): print('符合標準基準；未改動任何設定。')
        return 2 if any(result.values()) else 0
    except (OSError, RuntimeError, ValueError, KeyError, subprocess.TimeoutExpired, yaml.YAMLError) as exc:
        print('未完成：' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
