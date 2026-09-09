from pathlib import Path
import json
root=Path('/home/jetson/rinbo_ros_ws');out=root/'docs/diagnostics/restore_choices_20260909'
items=json.loads((root/'docs/diagnostics/workspace_audit_20260909/decision-items.json').read_text())['items']
choices=json.loads((out/'selections.json').read_text())['choices']
results={
'C04':'改為 0 PWM。恢復舊版前饋 0.02 後，先不疊加尚未量測的 40 PWM 摩擦補償。',
'C05':'改為可獨立設定的 5 ms 濾波；原始 encoder／速度仍供保護及歸零判斷。缺欄位的舊設定保留 20 ms，避免默默改動舊檔。',
'C08':'保留 0.5 秒平滑尋零起步；最高速度仍 36°/s。恢復出力不需要同時恢復速度跳變。',
'C09':'保留 60 秒伺服定位等待。沒有等待不足的新證據，不取消逾時。',
'C10':'保留 60 秒尋 Hall 等待；名目每圈 10 秒已有餘裕。',
'C11':'保留停穩／歸零各 15 秒及有界重送，不把送出 reset 當成成功。',
'C12':'保留速度 <500 counts/s 持續 0.3 秒、歸零回讀 |counts|≤100 的流程。',
'S04':'改為 0 PWM；理由同 C04，避免恢復較高前饋後又疊加固定摩擦補償。',
'S05':'改為獨立 5 ms 濾波；不改原始 encoder、比例、符號。',
'S07':'保留平滑尋零與半圈加減速，半圈約 5.5 秒、最高 36°/s、終點 180°。',
'S08':'保留 1000 counts（約 6.51°），先前 218 counts 誤差已在容差內；S09 另按 A 取消停穩條件。',
'S10':'保留尋 Hall／半圈各 60 秒。此次主要恢復出力及到位判定，沒有根據延長到無期限。',
'S11':'保持控制改為 KP=min(階段 KP,0.1)=0.1，KD=0.002；保留速度阻尼，保持上限 min(階段上限,300)=300 PWM。行走到位上限仍 500。',
'S12':'保留 12000 counts（約 78.13°）保持失位停止；這是大幅失位，不是一般到位容差。',
'T16':'保留新版 2 秒減速、5 秒停止期限；到 ratio=1 後持續 RUNNING，直到停止輸入。未恢復依 callback 加 ratio 的舊停止。',
'M02':'新增完整 rinbo_manual 控制區塊：KP=.35、KD=.002、K_FF=.02、摩擦0、濾波5ms。之後調 Standing 控制係數不再連動 Manual；供電／通訊仍共用 Standing 契約。',
'M03':'Manual 現場上限與計畫允許上界改為500，保留250 PWM/s變化率。實際上限=min(計畫,現場)。目前儲存計畫80保留，選13可明確調整到500。',
'M04':'修正實際12000卻寫5000的錯誤文字，停止當筆記錄腿名、帶符號error、target/actual counts與實際limit；門檻仍12000，無效回饋仍停止。',
'M05':'保留逐腳對齊／收尾位置差≤2°、速度≤5°/s；這是Manual不同於Standing的動作流程，沒有把S09選項連動過來。',
'M06':'保留你儲存的L2 90°/s、加速度10°/s²、10秒、PWM80與1～90速度／加速度範圍。新增前饋超過cap提示，不暗改已選动作。前饋約276.48 PWM，高於80；可自行用13調出力或2降速度。10°/s²的前饋增率約30.72 PWM/s，低於保留的250 slew；完整負載追蹤仍待量測。',
'R02':'評估後保留未啟用的備用profile檔案；原生L3繼續屏蔽。full_feedback兩份=[]、sensor_v2兩份=[L1]並不適合直接視為本機L3設定。準備啟用RL時須指定配對profile，再依L3要求及模型觀測契約一起核對。',
'R03':'保留目前轉換40 PWM/(rad/s)、基礎cap80與slew250；它不是Tripod的PD，沒有選定profile或速度到PWM量測，不能用原生500／3300代替。',
'R04':'保留現有數字但不認證為硬體規格；54984.83 counts/rev、ABAD1000 counts/rad仍待編碼器、減速比及實測確認。未猜測新比例。',
'R05':'保留base12秒／site30秒等各自初始化到位設定；目前未選定RL啟動profile，沒有把Standing位置完成模式移植到策略控制。',
'R06':'保留base與site原有動作限幅；它們的速度／伺服角度單位和PWM不同，需選定模型／profile才能判定適合數字。',
'R09':'保留各profile供電與通訊門檻及功能；base18～30V/3A、site18～42V/5A未被互相覆蓋。原生18～42V/5A保持。',
'O01':'保留robot.sh文字操作台，不恢復PyQt GUI。你目前使用SSH／VS Code；舊GUI的Tripod執行檔名稱也不一致，沒有必要重新引入它。',
'O02':'保留目前Manual逐腳入口及診斷，不恢復會直接發布命令的舊PID測試／單腿RSLIP程式。之後若需要特定正弦辨識實驗，再做符合現有握手的測試入口。',
'O03':'保留目前未引入microstrain子模組的狀態。沒有IMU型號、driver來源與裝置資料，不猜測安裝未知commit。',
}
assert set(results)=={k for k,v in choices.items() if v=='C'}
rows=[]
for x in items:
 c=choices[x['id']]
 if c=='B':action='保留盤點行為／設定；其必要相依元件隨本次建置更新。'
 elif c=='A':
  action='恢復：'+x['old']+'。'
  if x['id']=='C13':action='舊版伺服目標與容差原本就相同，數字不變；保留已修正的帶符號差值計算，避免溢位。'
  if x['id']=='S09':action='設 settle_time_s=0，半圈參考時間結束且位置進入S08容差即完成；不要求低速度或連續0.3秒。DONE日誌清楚標示position_only。'
 else:action=results[x['id']]
 rows.append({'id':x['id'],'choice':c,'title':x['title'],'result':action})
(out/'decisions-applied.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n')
dep=json.loads((out/'deployment.json').read_text()) if (out/'deployment.json').exists() else None
status=(f"已於 {dep['deployed_at']} 更新現場 revision {dep['revision']}。SHA256 `{dep['site_sha256']}`。備份：`{dep['backup']}`。" if dep else '程式修改完成，正在離線編譯／驗證；尚未更新現場設定或原生執行檔。')
text='''# 依恢復表選擇調整的結果（2026-09-09）

'''+status+'''

92項：A 10項、B 53項、C 29項。保存使用者原始填寫檔；只整理M02漏等號、R09換行，兩者均為C。未自動上電、校正、站立、執行Tripod或策略；未啟停Bridge／Core／FPGA。

## 實際控制設定

| 模式 | KP / KD / K_FF | 摩擦 | 濾波 | 最大PWM |
|---|---|---|---|---|
| Calibration | .35 / .002 / .02 | 0 | 5ms | 500 |
| Standing 尋零／到位 | .35 / .002 / .02 | 0 | 5ms | 500 |
| Standing 保持 | .1 / .002 / 無速度前饋 | 0 | 5ms | 300（也受階段上限限制） |
| Tripod | .38 / .003 / .005 | 0 | 5ms | 3300，額外slew關閉 |
| Manual 逐腳 | .35 / .002 / .02 | 0 | 5ms | 現場500；有效值=min(計畫,現場) |

目前儲存的Manual計畫仍為L2、90°/s、加速度10°/s²、10秒、PWM80。這是待執行計畫，不表示正在動作。前饋`0.02 × 90 × 55296/360 = 276.48 PWM`已超過80；操作台新增明確提示，選13可調出力到現場500或選2降低速度。沒有自動將下次計畫改成500。Manual仍保留250 PWM/s變化率；目前10°/s²的前饋變化約30.72 PWM/s，並非僅此項就超過250。PID與負載合計仍需實機驗證。

摩擦補償選0是為了讓恢復的前饋與PD可以單獨辨識；5ms濾波是控制回饋的短時間平滑，並非已量得最佳值。原始位置／速度仍供保護，不掩蓋無效資料。

## 到位與保護

- Calibration：servo/Hall各60秒、停穩/歸零各15秒；Hall後低速<500counts/s連續0.3秒，再確認原始歸零|counts|≤100。
- Standing：平滑半圈約5.5秒；位置誤差<1000counts（按55296counts/rev約6.51°）即可完成，不再要求低速0.3秒。設定`settle_time_s=0`，若以後設正數則重新要求速度與持續時間。尋Hall/到位各60秒，保持失位>12000counts約78.13°仍停止。
- Manual：對齊/收尾2°、5°/s條件保留；持續動作誤差>12000counts仍停止，錯誤文字改為實際門檻且記錄當筆數字；既有encoder跳變/過速、SIGINT、失聯與首因保留。
- Tripod：8秒起步、8→1、名目每秒ratio減0.2，到1後持續跑；保持2秒減速/5秒停止期限。有限位置誤差仍只警告；無效回饋仍停止。
- 原生L3屏蔽、單一動作限制、Bridge PWM3300、18～42V/5筆、腿5A/25筆、馬達/電源失聯、來源及啟用ack、急停與關電契約均保留。bus30A停止原本關閉，沒有宣稱它已開啟。

## 程式與操作入口

- 現場值：`/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml`。新增`parameters.rinbo_manual`完整控制區塊；舊版YAML若沒有此區塊，保留原先Standing繼承行為。新區塊缺欄位或拼錯會拒絕，避免悄悄繼承。
- 參數與型別／範圍驗證：`src/rinbo_fsm/src/robot_config.cpp`；500/3300硬上界：`safety_invariants.hpp`。
- Calibration控制：`rinbo_cali.cpp`；Standing到位與保持：`rinbo_standing.cpp`；Manual獨立參數與停止文字：`rinbo_manual.cpp`；Manual計畫與最終輸出限幅：`manual_motion.hpp`。
- Control Panel：照常用`./robot.sh`；選7調上限、選13快調出力、選15調到位限制。`robot.sh`直接載入`src/rinbo_control`，新開操作台即使用新程式。舊的install內Python副本不是這個入口，本次沒有將它當成已更新的入口。
- 只讀核對：`build/rinbo_fsm/rinbo_legs status --json`。請勿用全模式`tune-motion`只改一個階段；它仍調Cali/Standing/Tripod，而新Manual控制區塊獨立。

因Calibration／Standing實際設定改變，原先成功紀錄不能沿用。部署會備份並失效舊紀錄；下次由你手動執行Calibration→Standing→Tripod。這不是自動重跑，也不是假造完成。

Windows不需改Tripod停止命令或Bridge上限：Bridge仍3300。若Windows把Cali/Standing期望值硬寫80，需改為讀取原生有效設定；目前為500。site revision/hash也應動態核對，不沿用revision14；新增rinbo_manual區塊不可被舊模板覆寫掉。這台Orin沒有Windows App原始碼，未宣稱已修改Windows。

## 每一項的處理

| 編號 | 選擇 | 項目 | 結果與理由 |
|---|---|---|---|
'''
for x in rows:text+='| '+' | '.join(str(x[k]).replace('|','／') for k in ['id','choice','title','result'])+' |\n'
text+='''
## 離線驗證與現場確認範圍

驗證涵蓋舊YAML相容、新Manual設定隔離、500PWM從設定到最終輸出、L3屏蔽、Standing位置完成模式、Calibration歸零流程、保護與停止回歸。只使用隔離ROS domain231與localhost模擬回讀；沒有啟動真實控制入口。結果與完整日誌見同目錄diagnostics/restore_choices_20260909。

Sim2Real的C項目前完成的是評估與保留決策，沒有宣稱已完成硬體參數校準。備用profile不能直接當成L3屏蔽的現場設定。要調整或啟用它，還需指定RL/lowlevel兩個profile，以及馬達／encoder型號、齒比、輸出軸一圈counts差、伺服角度與counts對應、模型觀測契約。IMU也需實際型號與驅動來源。未取得這些資料前，C不被解讀成猜一個比例。

本次離線通過不等於實機PID已調到最佳。最小下一次量測：先檢查載入係數/上限及L3；手動校正一次記錄Hall/歸零，再Standing記錄target/actual/PWM及DONE模式。若方向相反、未隨目標移動、長時間飽和而誤差持續增加，或供電／通訊報錯，使用Q／停止並保留首因日誌；不在本次自動執行。
'''
if dep:
 native=sum(dep['native_tests'].values())
 text+=f"\n測試結果：原生{native}項GTest與1項退役入口檢查通過（11個CTest套件）；Control Panel 256通過、1跳過。\n"
(root/'docs/restore_choices_20260909_zh_TW.md').write_text(text)
print('Rendered 92 item results')
