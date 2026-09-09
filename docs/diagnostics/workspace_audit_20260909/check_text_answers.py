"""Read a restoration decision sheet; never writes robot settings or runs ROS."""
import argparse
import json
from pathlib import Path
import re
import sys


def main():
    audit_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description='檢查純文字恢復表；不修改機器人設定。')
    parser.add_argument('file', nargs='?', type=Path,
                        default=audit_dir.parents[1] / 'workspace_restore_answers_20260909.txt')
    args = parser.parse_args()
    items = json.loads((audit_dir / 'decision-items.json').read_text())['items']
    known = {x['id']: x for x in items}
    seen, selected, errors = set(), {}, []
    try:
        lines = args.file.read_text(encoding='utf-8-sig').splitlines()
    except OSError as exc:
        print(f'無法讀取表單：{exc}', file=sys.stderr)
        return 1
    for number, line in enumerate(lines, 1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        match = re.fullmatch(r'\s*([A-Z]\d{2})\s*=\s*([ABCabc]?)\s*(?:\|\s*(.*))?', line)
        if not match:
            errors.append(f'第 {number} 行格式不符：請填「C01 = A」，或留白「C01 =」。備註用 | 分隔。')
            continue
        item_id, choice, note = match.groups()
        if item_id not in known or item_id in seen:
            errors.append(f'第 {number} 行：{item_id} 是未知或重複編號。')
            continue
        seen.add(item_id)
        if choice:
            selected[item_id] = (choice.upper(), note or '')
        elif note:
            errors.append(f'第 {number} 行：{item_id} 有備註但未選 A／B／C。')
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        print('檢查未通過；沒有修改任何設定。', file=sys.stderr)
        return 1
    print(f'格式正確：已選 {len(selected)} / {len(items)} 項；未選項目維持現況。')
    print('以下只是選擇摘要，沒有執行任何修改：')
    for item in items:
        if item['id'] in selected:
            choice, note = selected[item['id']]
            print(f"{item['id']}={choice}（{item['title']}）" + (f'；{note}' if note else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
