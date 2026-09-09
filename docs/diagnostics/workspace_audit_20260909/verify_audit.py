"""Read-only source integrity and offline restoration-form verification."""
from pathlib import Path
from collections import Counter
import csv
import datetime
import hashlib
import json
import re
import subprocess

ROOT = Path('/home/jetson/rinbo_ros_ws')
OUT = Path(__file__).resolve().parent
DOC = ROOT / 'docs'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
inventory = json.loads((OUT / 'inventory.json').read_text())
data = json.loads((OUT / 'decision-items.json').read_text())
items = data['items']
assert len(items) == len({x['id'] for x in items}) == 92
assert all(x['choice'] == '' and x['note'] == '' for x in items)
assert all((ROOT / source).exists() for x in items for source in x['sources'])
checks = Counter()
for x in inventory:
    current = ROOT / x['path']
    if x.get('local_sha256'):
        assert sha(current) == x['local_sha256'], x['path']
        assert sha(OUT / 'current' / x['path']) == x['local_sha256'], x['path']
        checks['current_source_hashes_unchanged'] += 1
    elif not x['local_exists']:
        assert not current.exists(), x['path']
    if x.get('upstream_sha256'):
        assert sha(OUT / 'upstream' / x['path']) == x['upstream_sha256'], x['path']
        checks['upstream_snapshot_hashes'] += 1
for p in (OUT / 'site').iterdir():
    assert p.read_bytes() == (Path('/home/jetson/redrhex_site') / p.name).read_bytes()
    checks['site_files_unchanged'] += 1
for path, expected in json.loads((OUT / 'binary-manifest.json').read_text()).items():
    assert sha(Path(path)) == expected, path
    checks['binaries_unchanged'] += 1
with (DOC / 'workspace_restore_form_20260909.csv').open(encoding='utf-8-sig', newline='') as f:
    rows = list(csv.DictReader(f))
assert len(rows) == 92
assert [r['編號'] for r in rows] == [x['id'] for x in items]
assert all(not r['選擇A/B/C'] and not r['備註'] for r in rows)
page = (DOC / 'workspace_restore_form_20260909.html').read_text()
assert re.findall(r'<article data-id="([A-Z]\d+)"', page) == [x['id'] for x in items]
assert len(re.findall('data-choice=', page)) == 92
assert not re.search(r'<(?:script|link)[^>]+(?:src|href)=', page)
embedded = json.loads(re.search(r'<script id="audit-data" type="application/json">(.*?)</script>', page, re.S)[1])
assert embedded == data
script = re.search(r'<script>(.*?)</script>', page, re.S)[1]
# Run the actual page script with a minimal DOM; no browser/robot network.
harness = r'''
const vm=require('node:vm'),assert=require('node:assert/strict');
const input=JSON.parse(require('node:fs').readFileSync(0,'utf8'));
const nodes={};
const get=id=>nodes[id]||(nodes[id]={textContent:'',value:'',addEventListener(){},scrollIntoView(){}});
get('audit-data').textContent=JSON.stringify(input.data);
const saved={C01:{choice:'A',note:'保留備註'},C02:{choice:'Z',note:''},UNKNOWN:{choice:'A'}};
const document={getElementById:get,querySelectorAll:()=>[]};
const context=vm.createContext({document,localStorage:{getItem:()=>JSON.stringify(saved),setItem(){}},Blob,URL,setTimeout});
vm.runInContext(input.script,context);
const evaluate=s=>vm.runInContext(s,context);
assert.equal(get('count').textContent,'已選 1 / 92');
assert.equal(evaluate('Object.hasOwn(choices,"UNKNOWN")'),false);
assert.equal(evaluate('buildReply(data.items,{})'),'');
assert.equal(evaluate('selectedPayload(data.items,{}).decisions.length'),0);
evaluate("choices={C01:{choice:'A',note:'自訂'},C02:{choice:'B'},C03:{choice:'C'},C04:{choice:'Z'}};refresh()");
assert.equal(get('count').textContent,'已選 3 / 92');
get('reply').onclick();
assert.equal(get('output').value,'C01=A；自訂\nC02=B\nC03=C');
assert.equal(evaluate('selectedPayload(data.items,choices).decisions.length'),3);
assert.equal(evaluate('selectedPayload(data.items,choices).metadata.site_revision'),14);
console.log('PASS: actual inline script, valid/invalid choices, draft restore, count, selected-only JSON and text export');
'''
result = subprocess.run(['node', '-e', harness], input=json.dumps({'data': data, 'script': script}), text=True, capture_output=True, check=True)
missing_links = []
for name in ['workspace_audit_20260909_zh_TW.md', 'workspace_restore_form_20260909.md']:
    for target in re.findall(r'\]\(([^)]+)\)', (DOC / name).read_text()):
        if target.startswith(('https://', 'http://', '#')):
            continue
        target = re.sub(r':\d+$', '', target.split('#')[0])
        if not (DOC / target).exists():
            missing_links.append(target)
assert not missing_links, missing_links
verification = {
    'verified_at': datetime.datetime.now().astimezone().isoformat(),
    'result': 'pass', 'decision_items': 92, 'initial_choices': 'all blank',
    'inventory_counts': dict(Counter(x['status'] for x in inventory)),
    'integrity_checks': dict(checks), 'all_source_and_document_links_exist': True,
    'csv_and_embedded_html_match_json': True, 'form_smoke_test': result.stdout.strip(),
    'form_validation_limit': 'Minimal DOM execution; no visual browser layout test.',
    'hardware_actions': 'none', 'control_changes': 'none',
    'tests_scope': 'Audit artifacts only; no new motor/control runtime test.'
}
(OUT / 'audit-verification.json').write_text(json.dumps(verification, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(verification, ensure_ascii=False, indent=2))
