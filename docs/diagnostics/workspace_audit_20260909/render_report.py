from pathlib import Path
import csv,html,json,collections,hashlib,datetime,re
from audit_rows import ROWS,GROUPS
ROOT=Path('/home/jetson/rinbo_ros_ws');OUT=ROOT/'docs/diagnostics/workspace_audit_20260909';DOC=ROOT/'docs'
BASE=json.loads((OUT/'baseline.json').read_text());SITE=json.loads((OUT/'effective-site.json').read_text());INV=json.loads((OUT/'inventory.json').read_text())
META={'audit_id':'workspace_audit_20260909','generated_at':datetime.datetime.now().astimezone().isoformat(),'upstream_commit':BASE['upstream_commit'],'local_head':BASE['local_head'],'site_revision':SITE['revision'],'site_sha256':SITE['hash'],'changes_authorized':False,'choices_meaning':{'A':'恢復GitHub舊版此項','B':'保留本次盤點現況','C':'交由助理評估此項後調整'}}
DATA={'metadata':META,'items':ROWS};(OUT/'decision-items.json').write_text(json.dumps(DATA,ensure_ascii=False,indent=2)+'\n')
with (DOC/'workspace_restore_form_20260909.csv').open('w',encoding='utf-8-sig',newline='') as f:
 w=csv.writer(f);w.writerow(['編號','範圍','項目','GitHub舊版','目前盤點版本','影響與相依','建議（不是已選）','選擇A/B/C','備註','程式或設定位置','可確認修改來源'])
 for x in ROWS:w.writerow([x['id'],x['group'],x['title'],x['old'],x['current'],x['effect'],x['recommendation'],'','', '; '.join(x['sources']),x['provenance']])
with (OUT/'inventory.csv').open('w',encoding='utf-8-sig',newline='') as f:
 keys=sorted(set().union(*(x.keys() for x in INV)));w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(INV)
def clean(x):return str(x).replace('|','／').replace('\n',' ')
def source_links(row):
 return '、'.join(f'[{Path(s).name}]({s if s.startswith("/") else str(ROOT/s)})' for s in row['sources'])
form=['# 恢復選擇表：GitHub 舊版與目前 Orin（92 項）\n',
'本表固定對照 GitHub `ddcbce9fecdf039af17784385839eb55baeed2a3` 與 Orin 現場 revision 14。**本輪只盤點，沒有變更控制程式、設定、執行檔或啟停服務。**\n',
'每項三選一：**A＝恢復 GitHub 舊版此項；B＝保留本次盤點現況；C＝交由我評估後決定原版／新版／其他合理設定。** 空白表示尚未決定，維持現況。建議欄不代表已替你選擇。\n',
'可逐組回覆，例如 `C01=A、C02=B、C03=C`，也可回「C01～C14 全部 C，唯 C06 要 A」。這只是回覆格式示例，不是預選。可附註精確數值；後續收到你的選擇才實作與測試。\n',
'舊版不存在的功能，A 表示要回到不使用該新增功能的設計方向，不是把數值設成 0，更不是任意刪檔。與 Bridge／FSM／GUI 相依的項目會成套處理；若選項互相衝突，會先指出具體衝突。C 不授權猜測硬體比例，也不授權自動上電或運動。L3 目前故障的要求持續有效，其他項目選 A 不會附帶解除 L3。\n',
'[開啟離線勾選表](workspace_restore_form_20260909.html) · [Excel/CSV](workspace_restore_form_20260909.csv) · [完整診斷與檔案清單](workspace_audit_20260909_zh_TW.md)\n']
for group in GROUPS.values():
 rows=[x for x in ROWS if x['group']==group];form.append('\n## '+group+'\n\n| 編號 | 項目 | GitHub 舊版 | 目前盤點版本 | 建議與相依 | 你的選擇 |\n|---|---|---|---|---|---|\n')
 for x in rows:form.append('| '+' | '.join(clean(v) for v in [x['id'],x['title'],x['old'],x['current'],x['recommendation']+'；'+x['effect'],'□ A　□ B　□ C'])+' |\n')
(DOC/'workspace_restore_form_20260909.md').write_text(''.join(form))
report='''# Orin 全工作區與 GitHub 舊版詳細比對（2026-09-09）

本輪只新增這份比對、來源快照及恢復表，沒有修改任何控制源碼／現場設定／執行檔，没有重新建置、啟停服務或上電。後續收到各項 A/B/C 選擇再修改。

## 比對基準與證據範圍

- GitHub：[ShuWei-Yang/rinbo_ros_ws](https://github.com/ShuWei-Yang/rinbo_ros_ws/tree/ddcbce9fecdf039af17784385839eb55baeed2a3)，本次用 `ls-remote` 確認 main 仍是 `ddcbce9fecdf039af17784385839eb55baeed2a3`。
- 本機 origin 是另一個倉庫 `JasonLiaoJCS/BioRoLaROS2`，HEAD `20a0151`，不能把本機 HEAD 當成使用者 GitHub 的舊版。
- 目前原生設定 revision **14**，SHA256 `c657c377f47f4277a6b4f1b7e2abb08e1f68eca36c5541df5a28d8e6587dda90`。從原生 `rinbo_legs status --json` 讀取有效值，含 YAML 未寫出的預設值；未啟動 ROS 動作。
- 保存 GitHub 全部 37 個樹項目（36 檔＋1 gitlink）的基準，對照現行檔案。共同檔案完整文字 diff；重要控制、保護、啟動與停止路徑做人工語意核對。新增套件做檔案盤點及參數擷取，**不宣稱已逐行證明所有新增程式都沒有缺陷**。
- 共同檔案有 **13 檔內容差異、9 檔只有換行差、6 檔完全相同**；另有 **8 個 GitHub 檔案與1個子模組在本機不存在**。另盤點 230 個本機新增的 src/tools/入口檔案（含測試及FPGA候選材料）；數量不是功能數。
- 25 份 YAML 共 1709 筆展開設定，另有原生有效參數、逐腳GUI偏好與編譯期常數。備用profile不是生效參數；CSV不等於所有C++常數，重要常數也在下方項目中列出。
- 只讀 `/proc` 快照觀察到 Bridge PID **117000**，使用 `redrhex_safe.yaml` 並覆寫 `core_ip:=192.168.30.254`；ROS domain 99、Jetson .8、Core .254:50051。執行檔 SHA256 與既有 Bridge 部署記錄一致。沒有觀察到 Calibration／Standing／Tripod／Manual 或策略程序；這不是整個系統的即時健康保證，也沒有以此代替電源回讀。沒有另做 ROS runtime parameter dump，Bridge參數列為該程序的檔案／argv契約。

## 先看最重要的差異

| 項目 | GitHub 舊版 | 目前現況 | 表單 |
|---|---|---|---|
| Calibration / Standing KP、KD、前饋 | 0.35 / 0.002 / 0.02 | 0.08 / 0.006 / 0.005 | C01～C03、S01～S03 |
| Calibration / Standing PWM | 500 | 80，原生驗證器也卡80 | C06、S06 |
| Calibration / Standing 摩擦／速度濾波 | 無／無 | 40 PWM補償／20ms | C04～C05、S04～S05 |
| Standing 到位 | 200counts；不要求低速連續到位 | 1000counts＋500counts/s以下持續0.3s | S08～S09 |
| Standing 保持 | KP0.1、±300，無速度阻尼 | KP0.08、KD0.006、±80，保持失位>12000停止 | S11～S12 |
| Tripod 係數／目標ratio | 0.38 / 0.003 / 0.005；8→1 | 已恢復相同 | T01～T04、T10 |
| Tripod起步 | 4s三次曲線 | 8s五次曲線與平滑進入 | T06～T07 |
| Tripod零點與加速 | 兩次實際位置重設；每callback減ratio | 已恢復兩次重設；依dt維持名目每秒減0.2 | T08～T11 |
| Tripod輸出 | cap3300、無slew | 相同；250/s設定值未啟用 | T12 |
| 逐腳與RL輸出 | GitHub沒有等價控制器 | 仍有各自80 cap及250/s slew | M03、R03 |
| 逐腳追蹤錯誤文字 | 沒有此功能 | 實際12000，文字仍說5000 | M04 |
| 屏蔽設定 | 固定六腿 | 原生L3；備用RL YAML另有[]、L1 | G01、R02 |
| 通訊與停止 | 直接轉送、缺新版握手 | source/age/arbiter/ack、停止與off重送 | G06～G10、B01～B08 |

目前儲存的逐腳計畫另為 **L2、90°/s、加速度10°/s²、保持/勻速10秒、PWM80**。這只是GUI偏好，未表示正在執行；與截圖中較早的5°/s不是同一份目前設定。這組速度與80/250輸出限制的相容性值得獨立評估，不能以Tripod修正已完成就認定Manual也已一致。

## 能確認是我改過什麼，哪些不能歸因

本機大量修改尚未提交，git沒有這些差異的逐行作者。以下依本次對話、保存的源碼與部署紀錄說明；其餘僅標示存在差異，不冒稱全部是我做的。

| 可追溯紀錄 | 改動內容／狀態 | 證據 |
|---|---|---|
| 9/8方向／到位修正 | 反向行程辨識、Standing停穩/阻尼、尋零與站立平滑參考、TRACE、Tripod負相位修正 | [方向診斷](motor_direction_audit_zh_TW.md) |
| 9/9 13:12，revision9 | Tripod有限位置誤差警告模式、保留無效回饋停止 | [位置警告部署](diagnostics/tripod_position_warning_20260909/deployment.json) |
| 9/9 19:53，revision11 | 到位／等待限制可調，Standing1000counts、60s；Cali60/60/15s；Control Panel選15 | [限制部署](diagnostics/motion_limits_20260909/deployment.json) |
| 9/9 20:16，revision12 | Tripod cap3300與Bridge3300驗證契約 | [PWM部署](diagnostics/tripod_pwm3300_20260909/deployment.json) |
| 9/9連線整理修正 | 按1整理過期自有紀錄、重用有效程序；不登入就操作／不清除所有SSH | [連線整理](connection_recovery_20260909_zh_TW.md) |
| 9/9 21:09，revision13 | Tripod額外slew關閉，保留可選開關；Manual250/s仍保留 | [slew部署](diagnostics/tripod_gait_compare_20260909/deployment.json) |
| 9/9 21:42，revision14 | KP=.38、KD=.003、摩擦0、濾波5ms、實際基準、8→1與名目每秒-.2；Cali/Standing設定不變 | [最新恢復部署](diagnostics/tripod_restore_20260909/deployment.json) |
| FPGA console生命週期 | EOF/HUP/ERR/CPU修正候選與背景/監看交接，文件明確標示未部署 | [候選交接](../tools/fpga_lifecycle/README.md) |

**特別更正「所有不同都是最近改的」的理解：**本機已提交HEAD的Tripod本來就是8秒起步、target ratio=2；GitHub是4秒、target=1。Cali、Recorder與Bridge的上游→本機HEAD差異主要是註解與格式；Standing內容在忽略換行後相同。舊panel、pid_test、microstrain在本機HEAD也不存在，沒有證據說是本次助理刪除。其他未提交新增套件的歷史作者不能只憑檔案mtime判定。

## 恢復相依與不能照抄的地方

1. **原生FSM與Bridge握手要成套。**只換回原版Bridge，現在的FSM等不到epoch/active ack；只换回舊FSM，也不會自動符合新版Bridge的來源及命令契約。
2. **500 PWM不是只改YAML。**Cali/Standing/Manual另有原生80的硬驗證；如果選恢復500，後續需修改對應模式驗證器並確認Manual是否連動，不能讓UI顯示500而實際仍80。
3. **L3不會因其他選項恢復舊版而解除。**這是你仍有效的故障腳要求。RL的備用mask也不代表原生已解除L3；需先指定準備使用哪對profile。
4. **舊GUI有可證實的入口不一致。**它呼叫`rinbo_tripod_rslip`，同一GitHub CMake只產生`rinbo_tripod`。選舊GUI不等於逐字照抄就能用。
5. **舊版本沒有RL、Manual、FPGA driver或Windows source。**沒有的功能不能捏造「舊參數」；A是恢復不使用新增功能的方向。FPGA需另一份sbRIO基準，Windows需桌面App原始碼。
6. **README不是硬體規格。**舊README仍混用Corgi/ROS1說明，源碼實際是ROS2。兩種counts/rev與方向需要硬體證據，選C也不會直接猜測統一。
7. **不同停止是不同範圍。**停止Tripod、Manual完成轉sensors、Bridge退出off、全部關電、急停解除不能互換；ERROR90這类紀錄不一致不能用清除文件假裝關電成功。

## 各項詳細比較與程式位置

以下同恢復表使用固定編號；建議不是已選擇。源碼連結指向目前檔案；同時保留本輪current/upstream快照，避免之後調整時失去比對基準。
'''
for group in GROUPS.values():
 report+='\n### '+group+'\n\n'
 for x in [x for x in ROWS if x['group']==group]:
  report+=f"**{x['id']} — {x['title']}**\n\n- 舊版：{x['old']}。\n- 現況：{x['current']}。\n- 影響：{x['effect']}\n- 建議：{x['recommendation']}。\n- 位置：{source_links(x)}。\n\n"
report+='''
## 全部檔案差異與參數附件

| 共同／缺席檔案 | 結果 |
|---|---|
'''
labels={'modified':'內容有差異','line_endings_only':'僅CRLF/LF換行差異','identical':'完全相同','absent_locally':'本機不存在','gitlink_absent':'子模組入口不存在'}
for x in INV:
 if x['status']!='local_only':report+=f"| `{x['path']}` | {labels.get(x['status'],x['status'])} |\n"
report+='''
完整機器可讀附件：

- [逐檔SHA256／GitHub→本機HEAD→工作檔對照](diagnostics/workspace_audit_20260909/inventory.csv)
- [GitHub→目前共同檔案完整差異](diagnostics/workspace_audit_20260909/upstream-to-current.patch)
- [GitHub→本機已提交HEAD差異](diagnostics/workspace_audit_20260909/upstream-to-local-head.patch)
- [25份YAML／1709筆設定清單](diagnostics/workspace_audit_20260909/all-yaml-parameters.csv)：每列保留來源檔，避免混淆profile；不是全部都在運行。
- [原生FSM有效參數](diagnostics/workspace_audit_20260909/effective-site.json)：包含預設值，原生下一次啟動依此讀取。
- [目前逐腳偏好](diagnostics/workspace_audit_20260909/panel-preferences.json)、[程序唯讀快照](diagnostics/workspace_audit_20260909/process-snapshot.json)、[本機執行檔雜湊](diagnostics/workspace_audit_20260909/binary-manifest.json)。
- [互動恢復表](workspace_restore_form_20260909.html)、[Markdown恢復表](workspace_restore_form_20260909.md)、[Excel/CSV恢復表](workspace_restore_form_20260909.csv)。

本輪驗證的是比對基準、來源／檔案完整性、欄位與表單匯出，不是新一次馬達控制測試。上一輪Tripod部署有258項離線檢查通過；它不能代替所有RL profile或真實機構的驗證。審核完整性結果保存在 `diagnostics/workspace_audit_20260909/audit-verification.json`。

## 如何回覆

你可以先回 **C01～C14（Calibration）**，再回Standing、Tripod，或直接逐項回覆。每項A/B/C，必要時加一句自己的數字或要求；沒有回覆的項目維持現況。選C的項目我會依對應控制路徑、現有證據及離線測試選擇，不會把其他模式一起改掉；缺硬體資訊時會明列缺口。
'''
(DOC/'workspace_audit_20260909_zh_TW.md').write_text(report)
# Standalone form: no server, no external resources, no robot-facing API.
esc=html.escape
sections=[]
for group in GROUPS.values():
 cards=[]
 for x in [x for x in ROWS if x['group']==group]:
  cards.append(f'''<article data-id="{x['id']}" data-search="{esc(' '.join([x['id'],group,x['title'],x['old'],x['current']]))}">
+<h3>{x['id']} · {esc(x['title'])}</h3><div class="compare"><div><b>GitHub 舊版</b><p>{esc(x['old'])}</p></div><div><b>目前盤點版本</b><p>{esc(x['current'])}</p></div></div>
+<p>{esc(x['effect'])}</p><p class="advice">建議（尚未替你選）：{esc(x['recommendation'])}</p>
+<label>你的選擇 <select data-choice="{x['id']}" aria-label="{x['id']} 選擇"><option value="">待選</option><option value="A">A 恢復舊版</option><option value="B">B 保留新版</option><option value="C">C 交由助理評估</option></select></label>
+<label class="note">備註／指定數值 <input data-note="{x['id']}" aria-label="{x['id']} 備註" placeholder="可留白；例如只調此模式，不改其他階段"></label>
+<details class="sources"><summary>程式位置與來源</summary><p>{'<br>'.join(esc(s) for s in x['sources'])}</p><p>{esc(x['provenance'])}</p></details></article>'''.replace('\n+','\n'))
 sections.append(f'<details class="group" {"open" if group==GROUPS["C"] else ""}><summary>{esc(group)}（{len(cards)}項）</summary>'+''.join(cards)+'</details>')
page=r'''<!doctype html><html lang="zh-Hant"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Rinbo 恢復選擇表</title>
<style>*{box-sizing:border-box}body{font:16px/1.6 system-ui,sans-serif;margin:0;background:#f3f5f7;color:#17212e}main{max-width:1080px;margin:auto;padding:24px}h1{font-size:28px}h3{margin:0 0 10px}button,select,input{font:inherit;padding:8px;border:1px solid #78899a;border-radius:5px}button{background:#154d75;color:white;cursor:pointer}input{max-width:100%;background:white}.bar{position:sticky;top:0;background:#e7eef4;padding:12px;z-index:2;border-bottom:1px solid #ccd}.bar>div{max-width:1080px;margin:auto;display:flex;gap:12px;align-items:center;flex-wrap:wrap}article{background:white;border:1px solid #cbd5df;padding:18px;margin:12px 0;border-radius:8px}article.chosen{border-left:5px solid #25724d}.compare{display:grid;grid-template-columns:1fr 1fr;gap:16px}.compare>div{background:#f4f6fa;padding:12px}.compare p{margin:4px 0}.advice{color:#415268}.group>summary{font-size:20px;padding:12px;cursor:pointer;font-weight:600}.note{display:block;margin-top:12px}.note input{width:70%}.sources{font-size:13px;margin-top:12px;overflow-wrap:anywhere}textarea{width:100%;min-height:150px;font:15px/1.5 monospace}.muted{color:#4c6075}#storage{font-size:13px}[hidden]{display:none!important}@media(max-width:650px){.compare{grid-template-columns:1fr}main{padding:12px}.note input{width:100%}}@media print{.bar,button{display:none}article{break-inside:avoid}}
</style><div class="bar"><div><b id="count"></b><input id="search" aria-label="搜尋編號或項目" placeholder="搜尋編號、PWM、Standing…"><button id="open-all" type="button">展開全部</button><button id="download" type="button">匯出已選 JSON</button><button id="reply" type="button">產生文字回覆</button></div></div><main>
<h1>Rinbo 恢復選擇表</h1><p><b>92項，所有選擇先留白。</b> A＝恢復GitHub此項；B＝保留這次盤點現況；C＝交由助理評估原版／新版／其他合理值。未選項目保持現況。</p>
<p>此離線表只整理你的回覆，沒有連接機器人、沒有更改設定功能。填完可匯出JSON，或把文字回覆貼回對話。相依與衝突由後續實作處理；其他項目選A不附帶解除故障L3。沒有舊版的功能選A代表不使用新增功能，不是把數字設0。</p>
<p class="muted">基準：GitHub ddcbce9 · Orin revision14 · 本次只做比對。C不包含自動上電或動作授權。請勿將「建議」誤認為已選。</p><p id="storage"></p>
__SECTIONS__
<h2>可貼回對話的回覆</h2><p>按「產生文字回覆」後，複製下面內容。也可不使用此頁，直接在對話回覆編號與A/B/C。</p><textarea id="output" aria-label="文字回覆" readonly></textarea></main>
<script id="audit-data" type="application/json">__DATA__</script><script>
'use strict';
const data=JSON.parse(document.getElementById('audit-data').textContent);
const known=new Set(data.items.map(x=>x.id));
const key=data.metadata.audit_id+':'+data.metadata.site_sha256;
function validChoice(c){return ['A','B','C'].includes(c);}
function buildReply(items,choices){return items.filter(x=>validChoice(choices[x.id]?.choice)).map(x=>x.id+'='+choices[x.id].choice+(choices[x.id].note?'；'+choices[x.id].note:'')).join('\n');}
function selectedPayload(items,choices){return {metadata:data.metadata,decisions:items.filter(x=>validChoice(choices[x.id]?.choice)).map(x=>({id:x.id,title:x.title,choice:choices[x.id].choice,note:choices[x.id].note||''}))};}
let choices={};
try{const stored=JSON.parse(localStorage.getItem(key)||'{}');for(const [id,x] of Object.entries(stored)){if(known.has(id)&&x&&typeof x==='object')choices[id]={choice:validChoice(x.choice)?x.choice:'',note:typeof x.note==='string'?x.note:''};}document.getElementById('storage').textContent='選擇會嘗試保存在這台瀏覽器；匯出檔案可保留正式回覆。';}catch(e){document.getElementById('storage').textContent='此瀏覽器無法儲存草稿，離開前請匯出JSON或複製文字。';}
function refresh(){const n=data.items.filter(x=>validChoice(choices[x.id]?.choice)).length;document.getElementById('count').textContent='已選 '+n+' / '+data.items.length;document.querySelectorAll('article').forEach(el=>el.classList.toggle('chosen',validChoice(choices[el.dataset.id]?.choice)));}
function save(){try{localStorage.setItem(key,JSON.stringify(choices));}catch(e){document.getElementById('storage').textContent='草稿未儲存，請匯出或複製文字。';}refresh();}
document.querySelectorAll('[data-choice]').forEach(el=>{const id=el.dataset.choice;el.value=choices[id]?.choice||'';el.addEventListener('change',()=>{choices[id]={...(choices[id]||{}),choice:el.value};save();});});
document.querySelectorAll('[data-note]').forEach(el=>{const id=el.dataset.note;el.value=choices[id]?.note||'';el.addEventListener('input',()=>{choices[id]={...(choices[id]||{}),note:el.value};save();});});
document.getElementById('reply').onclick=()=>{const text=buildReply(data.items,choices);document.getElementById('output').value=text||'尚未選擇項目；請先選A/B/C。';document.getElementById('output').scrollIntoView({behavior:'smooth'});};
document.getElementById('download').onclick=()=>{const blob=new Blob([JSON.stringify(selectedPayload(data.items,choices),null,2)],{type:'application/json;charset=utf-8'});const url=URL.createObjectURL(blob);const a=document.createElement('a');a.href=url;a.download='rinbo-restore-choices-20260909.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
document.getElementById('open-all').onclick=()=>document.querySelectorAll('details.group').forEach(el=>el.open=true);
document.getElementById('search').addEventListener('input',event=>{const q=event.target.value.trim().toLowerCase();document.querySelectorAll('article').forEach(el=>el.hidden=!el.dataset.search.toLowerCase().includes(q));document.querySelectorAll('details.group').forEach(el=>{el.hidden=![...el.querySelectorAll('article')].some(a=>!a.hidden);if(q&&!el.hidden)el.open=true;});});
refresh();
</script></html>'''
page=page.replace('__SECTIONS__',''.join(sections)).replace('__DATA__',json.dumps(DATA,ensure_ascii=False).replace('<','\\u003c'))
(DOC/'workspace_restore_form_20260909.html').write_text(page)
print('Rendered',len(ROWS),'items; all choices blank; report, MD, CSV, HTML generated')
