後續政策更新：使用者已明確選擇「有限位置誤差只警告」，且已部署至Orin revision 9。請以 [最新政策與部署紀錄](tripod_position_warning_20260909_zh_TW.md) 為準；下文的舊位置自動停止要求及尚未部署描述是先前階段的歷史紀錄。

本次已完成 Tripod 原始碼修正與離線驗證；尚未部署、上電或執行真實動作。現場 YAML、Calibration／Standing 控制原始碼及既有執行檔保留。另加入只調整 Tripod 的操作入口，模型建議不構成新的啟動禁令。

**證據與根因界線**

原始日誌：`/tmp/rslip-launcher/20260909-121824-216/rinbo_tripod.log`。首個停止原因保存為：

```
TRIPOD SAFETY STOP: hard position error: L2 counts=18008.929688 hard_limit=18000
```

日誌檔名是工作階段建立時間，不是動作開始時間：STARTUP 約 12:21:50.548824，RUNNING 12:21:58.548966，B 組啟動 12:22:00.158084，最後 PWM 行 12:22:00.759108（UTC+8）。安全停止行本身沒有獨立時間戳。

診斷時 PID 45873 仍以 SAFETY_STOP 存活，stdin 為 `/dev/null`，不是這次 SSH 終端退出。boot ID 為 `b0008f58-5739-453d-be5f-aa3fa7d3bd72`。使用原 flags 重編舊程式，object 逐位元一致；再使用原連結輸入重連，與 `/proc/45873/exe` 的 SHA256 完全相同：`bba154b0b7fa84ed7770853b294e5be12327c23d8d9ad702a18b0464dc6d47f5`。因此下列程式缺陷確實存在於本次執行版本，並非只存在於尚未部署的原始碼。

| 判定 | 證據／限制 |
|---|---|
| 已確認：觸發硬位置保護 | L2 的誤差**絕對值**超過 18000；舊日誌沒有保存其正負號，不可自行稱為正向落後。 |
| 已確認：控制需求與輸出限幅衝突 | 現場 k_ff=.005、摩擦補償40、PWM80。ratio8 時中心目標速度約14064.8 counts/s，零誤差前饋約110.3；飛行平台約27492.8 counts/s，前饋約177.5。即使誤差為零，控制式也要求超過80。實際日誌多腿反覆到80，與此一致。這證明設定／控制式不相容，不等於已辨識真實馬達模型。 |
| 已確認：目標切換有跳變 | 舊 STARTUP→RUNNING 用 actual 重設 home，最多可抹去一般門檻範圍內的誤差；B 組啟動又重設 home。舊 STARTUP 終點速度非零，但 B 等待時速度突然變零，啟動時又跳成非零。 |
| 已確認：slew 實作不符設定 | `max(1,250*max(dt,.001))` 在1kHz允許每秒1000 PWM，而非設定250。ratio_step 也原本依 callback 次數改變，受排程速度影響。 |
| 已確認：故障診斷漏幀 | 位置保護在 debug 發佈之前 return；raw_pwm 其實已經被限幅，後續 SAFETY_STOP debug 又被零值覆蓋。 |
| 尚未確認：物理上的第一原因 | 缺少本次 controller debug bag，以及觸發時 raw／target／帶符號誤差／實際速度。無法把 L2 唯一歸因於機械受阻、馬達能力、回饋錯誤、零點或方向。 |

L2 屬於 **Group A**。A 日誌順序為 R1、L2、R3；B 為 L1、R2、L3。B 啟動時 L2 沒有換組，也沒有進入 B 啟動分支。共同負載或電源影響仍需同步電流／電壓與回饋資料，不把時間相近當作因果。

**座標、歷史與跨圈對照**

| 項目 | Calibration／Standing | Tripod（保留歷史定義） |
|---|---|---|
| counts/rev | 55296 | 54984.83 |
| 正向移動的 counts | 左腿 raw 下降，右腿 raw 上升 | 左腿 raw 上升，右腿 raw 下降 |
| 正 PWM 命令 direction | 左 false、右 true | 左 true、右 false |
| 控制位置 | 以 Hall／各腿起始 raw 計算行程 | 左 actual=raw，右 actual=-raw |
| 零點 | Calibration 建立 Hall 基準；Standing 找 Hall 後移動半圈27648 counts | 擷取 Tripod 起始 actual；STARTUP 前進54984.83 counts；之後保留這個**計畫終點**為 home，不再用 actual 消除誤差 |
| 位置誤差 | 正向座標下的目標－實際 | `target_counts - normalized_actual_counts`，不取最短角度差 |
| 圈數 | Standing 有限行程 | `floor(tau/period)` 展開整圈，位置持續累加2π；保留負相位及多圈 |
| 相位 | 不使用 A/B 相位 | period=.403516855584082；中心約.0551277043；B 延後半週期.2017584278 |

Tripod 的感測與命令符號一起反轉，內部可以一致；這不是單憑符號就能判定的接線錯誤。沒有設計證明可確認應向哪個物理方向走，故未改方向。

可讀取的 HEAD 歷史：`600dacf`（2026-05-15，Add tripod gait controller）首次加入54984.83與目前方向；`e7c6a24`、`6bacbaa` 分別加入 Calibration／Standing 的55296。提交與現有文件沒有提供齒比、編碼器型號或一圈實測依據。lowlevel_bridge 的54984.83只是引用Tripod，不能反過來作硬體證明。`git log --all` 遇到既有損壞的 Codex checkpoint ref；未修復或覆寫該 ref，歷史結論限於可遍歷的 HEAD。

兩個比例差311.17 counts/rev，約0.56%。以約一至數圈的這次動作而言，僅此差異不足以解釋18009 counts；仍未據此盲目統一比例。

**修改內容**

1. STARTUP 改為靜止到靜止的五次曲線，保持原有一圈行程、使用者指定時間及既有對準門檻。取消本次開發中曾考慮的額外200-count／0.3s對準門檻與2s等待禁令。
2. RUNNING 保留計畫 home，A/B 各自以連續位置與速度進入既有軌跡；起步相位平滑段結束後恢復原有相位與半週期差。沒有將多圈位置改為最短角差。
3. B 等待時仍維持停用，但誤差對固定 home 計算，不再跟著 actual 改成零；等待腿不累積未送出的 slew 命令。
4. slew 改為 `250*實際dt`；ratio_step 保留原1kHz定義，換算成經過時間。非正時間間隔明確停止，不虛構1ms。
5. SIGINT 從最後一筆目標位置／速度減速，2s後停用；保留5s截止及即時故障停止。STARTUP 中途停止不再跳入 RUNNING 軌跡；原本已慢於ratio10的運動也不會因停止而加速。
6. 模型只顯示 `TRIPOD_REFERENCE_WARNING` 和需求數值，不禁止啟動、不自動改參數。真正的位置、回饋及電源保護仍照原設定運作。
7. 故障前先保存完整當筆資料，再執行保護。既有 `/rinbo/controller_debug` 訊息格式不變，raw_pwm 現在是限幅前值。文字日誌平時約10Hz，另保留最多100筆、約50Hz的最近控制資料，首次故障一次輸出 `TRIPOD_PREFAULT`、`TRIPOD_FAULT_FRAME`。含腿名、分組、原始／正規化／目標 counts、origin/home、帶符號誤差、phase/tau、目標／實際／濾波速度、前饋、限幅前後與slew後PWM、飽和及slew受限時間、source seq／stamp／arrival、保護次數／時間／門檻。當筆故障輸出值標為零；第一原因與第一幀不被後續原因／零值覆蓋。未收到可計算資料時明示無可用幀；非動作回饋觸發時可用 frame_s 與 observation_s 辨認資料年齡。
8. 安全停止後收到SIGINT可退出，程式回傳2保留失敗語意；正常停止回傳0。停用命令若發送失敗仍保留故障並使舊完成紀錄失效，不宣稱硬體已停妥或已關電。
9. `rinbo_legs tune-tripod [--dry-run] KEY=VALUE...`：只調整Tripod，不改 Calibration／Standing；有效舊完成紀錄可重新綁定新設定雜湊。失效／缺失／不同boot紀錄不能被重新建立；寫入時仍檢查動作鎖及執行中的動作。預覽可在動作進行中使用。不增加確認視窗或模型禁令。

**保護與數值**

| 項目 | 本次結果 |
|---|---|
| 一般位置門檻 | 保留9000 counts；至少10筆且持續至少0.5s，恢復後重新計時 |
| 硬位置上限 | 保留18000 counts；嚴格 `>`，非有限值立即停止，不等待0.5s |
| PWM／slew | 上限80；250 PWM/s；故障停用不等待slew |
| L3／測試模式 | L3屏蔽、supported_leg_test保留，沒有重新分組或啟用L3 |
| 電源 | 現場18–42V、腿電流5A／25筆、bus ch7／腿ch1–6；總線電流停止仍依原設定false |
| 回饋 | motor stale .25s／source age .10s，power stale .5s／source age .35s；relay、發布者及仲裁檢查保留 |
| 控制值 | 現場kp=.08、kd=.006、k_ff=.005、friction=40、fade=153.6均未套用更改 |
| 正常停止／急停 | 2s減速、5s截止；故障／急停立即停用；不將Tripod停用等同馬達電源已關閉 |

若54984.83確實是一個輸出軸整圈：9000≈58.93°、18000≈117.85°；若實際為55296：分別58.59°、117.19°。這是**附帶齒比／每圈定義假設的換算**，不能在沒有規格時稱為已校準輸出軸角度。兩種解釋下都不是很小的誤差，目前沒有擴大門檻的證據；機構允許角度仍待確認。未修改共享 safety_invariants 的12000常數（它是可設定一般門檻的上界，不是Tripod的18000硬上限）。

可供操作者選擇的較慢起點是 `startup_duration=20 start_ratio=40 target_ratio=40`，不是新強制預設。依目前控制式計算：STARTUP最大5154.8 counts/s、前饋65.8；RUNNING最大5498.6 counts/s、前饋67.5、前饋最大變化約190.2 PWM/s。速度約36°/s（仍以Tripod比例假設換算），接近已使用的 Standing 36°/s，沒有增加PWM。模型只用來選擇量測起點；必須觀察實際追蹤才能決定是否逐步加快。原STARTUP8s／ratio8→4仍可選擇，會顯示需求超限警告及保留原保護。

**離線驗證與交付範圍**

建置在 `/tmp/rinbo-tripod-fix-build`，只執行模擬測試及 `rinbo_legs --dry-run`；沒有替換 `build/` 或 `install/` 的現場執行檔。測試網域固定 `ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1`，直接餵入模擬回饋，未啟動 Bridge／FPGA，也沒有實際馬達模型，不能證明負載下可追蹤。

共49項測試全部通過：Tripod23項、專用調參4項、Calibration14項、Standing8項。測試結果與完整證據見 [診斷資料](diagnostics/tripod_20260909/)、[測試結果](diagnostics/tripod_20260909/test-results.json)。涵蓋正常追蹤、兩組切換、原始方向、負相位／跨圈、短暫／持續／硬位置誤差、無效資料、64種屏蔽組合、slew、停止與錯誤發布、只調Tripod及完成紀錄有效性；Calibration／Standing原測試回歸。此次修改相對本次工作開始時的差異另保存在診斷資料，避免混入其他工作階段既有修改。

Windows 配合方式見 [Tripod操作交接](tripod_windows_handoff_20260909_zh_TW.md)。現場程式仍為舊版本；只有部署候選版後，新調參命令與診斷標記才可使用。

**最小現場確認，尚未執行**

1. 提供馬達／編碼器型號、每馬達圈計數及解碼倍率、減速比；標記輸出軸的物理正方向，確認Tripod究竟應與Standing同向還是反向。
2. 在你確認支撐、供電與可手動轉動條件後，保持馬達驅動停用、只讀編碼器，記錄輸出軸標記位置的raw counts，再手動轉動**恰好一圈**回到標記，讀取raw counts差值；同時記錄是哪一腿及轉向。不可回驅的減速機不強扳，改採經你確認的受控量測方案。避免把Hall觸發、encoder reset或32位溢位混入差值。
3. 方向與比例確認後，再由你選擇一次短程Tripod測試：保留L3屏蔽，先觀察STARTUP，再觀察A起步和B接入。保存原始日誌及controller debug；若位置誤差持續增大、PWM長時間受限而速度追不上、方向與指定相反、卡住／碰撞，立即停止；軟／硬位置、無效回饋與電源保護仍自動生效。
4. 比較L2 target/actual/error、原始速度與PWM、B啟動前後電流／電壓，才區分負載、受阻、回饋和控制器問題。這不是目前已完成的實機修復驗收。
