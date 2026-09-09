# 恢復選擇表：GitHub 舊版與目前 Orin（92 項）
本表固定對照 GitHub `ddcbce9fecdf039af17784385839eb55baeed2a3` 與 Orin 現場 revision 14。**本輪只盤點，沒有變更控制程式、設定、執行檔或啟停服務。**
每項三選一：**A＝恢復 GitHub 舊版此項；B＝保留本次盤點現況；C＝交由我評估後決定原版／新版／其他合理設定。** 空白表示尚未決定，維持現況。建議欄不代表已替你選擇。
可逐組回覆，例如 `C01=A、C02=B、C03=C`，也可回「C01～C14 全部 C，唯 C06 要 A」。這只是回覆格式示例，不是預選。可附註精確數值；後續收到你的選擇才實作與測試。
舊版不存在的功能，A 表示要回到不使用該新增功能的設計方向，不是把數值設成 0，更不是任意刪檔。與 Bridge／FSM／GUI 相依的項目會成套處理；若選項互相衝突，會先指出具體衝突。C 不授權猜測硬體比例，也不授權自動上電或運動。L3 目前故障的要求持續有效，其他項目選 A 不會附帶解除 L3。
[開啟離線勾選表](workspace_restore_form_20260909.html) · [Excel/CSV](workspace_restore_form_20260909.csv) · [完整診斷與檔案清單](workspace_audit_20260909_zh_TW.md)

## Calibration 校正

| 編號 | 項目 | GitHub 舊版 | 目前盤點版本 | 建議與相依 | 你的選擇 |
|---|---|---|---|---|---|
| C01 | KP 位置修正係數 | 0.35 | 0.08 | C：依實測選值；不要把 Tripod 的 0.38 自動套到這裡；變更位置誤差造成的 PWM；與 KD、前饋和 PWM 上限一起檢視。 | □ A　□ B　□ C |
| C02 | KD 速度修正係數 | 0.002 | 0.006 | C：與該階段速度回饋共同評估；現值為原版 3 倍；實際影響也取決於速度濾波。 | □ A　□ B　□ C |
| C03 | K_FF 速度前饋 | 0.02 | 0.005 | C：與 PWM／摩擦補償成套评估；相同目標速度下前饋為原版四分之一；不是位置保護。 | □ A　□ B　□ C |
| C04 | 額外摩擦補償 | 無，0 PWM | 40×tanh(target_velocity/153.6) PWM | C：量測後決定，不能以固定補償取代控制調校；速度接近零時平滑歸零；目前 80 PWM 上限下影響比例大。 | □ A　□ B　□ C |
| C05 | 控制速度濾波 | 原始位置差分，無濾波 | 固定 20 ms；原始速度仍供停穩等判斷 | C：比較原始／短時間常數；不要直接改共用預設影響別的模式；減少速度尖峰，也增加回饋延遲；不等於輸出 slew。 | □ A　□ B　□ C |
| C06 | 主驅動最大 PWM | 500 | 80，且原生共用硬上限也為 80 | C：與增益、負載和該模式一起評估，不沿用 Tripod 3300；只改 YAML 到 500 會被拒絕；需同步驗證器，Standing 上限還影響 Manual。 | □ A　□ B　□ C |
| C07 | 尋 Hall 巡航速度 | 0.2π rad/s＝36°/s＝5529.6 counts/s | 相同 | B：相同，無須回復；這個速度並未被降低；不可把較低 PWM 誤認成目標速度變慢。 | □ A　□ B　□ C |
| C08 | 尋 Hall 起步曲線 | 直接切到固定速度的線性位置參考 | 0.5 秒加速到同一巡航速度 | B：保留平滑起步；速度上限不變；移除 ramp 會恢復起步速度跳變。 | □ A　□ B　□ C |
| C09 | 伺服定位等待上限 | 無明確 timeout | 60 秒；可設範圍 >0～600 秒 | B：目前 60 秒；若確有慢速需求可選 C；原版可能永遠等不到定位；目前逾時報出第一個原因。 | □ A　□ B　□ C |
| C10 | 主馬達尋 Hall 等待上限 | 無明確 timeout | 60 秒；可設範圍 >0～600 秒 | B：目前 60 秒；位置／尋零等待，不是通訊失聯期限。 | □ A　□ B　□ C |
| C11 | 停穩／歸零等待上限 | 無明確 timeout | 各階段 15 秒；可設範圍 >0～600 秒 | B：15 秒；與歸零確認一起保留；停止與歸零各有期限；操作台等待预算也依這些設定計算。 | □ A　□ B　□ C |
| C12 | Hall 後停穩與歸零完成判定 | 速度 <500 counts/s 持續 0.3 秒，送一次 reset 就 DONE | 先停穩，再等待 ／raw position／≤100 counts 且低速回讀；允許有界 reset 重送 | B：保留回讀確認；原版送出命令不代表歸零已完成；恢復可能影響下一步站立原點。 | □ A　□ B　□ C |
| C13 | 伺服中立目標／到位容差 | [740,2565,3283,1944,2071,989]；容差 100 | 相同；差值改用較寬的帶符號整數計算 | B：保留既有目標與數值修正；硬體零點不由程式猜；全域 servo_control_mode 仍可能控制所有已接伺服。 | □ A　□ B　□ C |
| C14 | 只校正選中腳與成功紀錄 | 固定全六腿，沒有分腳成功憑據 | 支援 --plan 選中腳；其他主驅動不參與；保存已確認腳位紀錄 | B：保留逐腳校正能力；與 robot.sh 逐腳控制、L3 屏蔽相依；不能單獨刪除憑據流程。 | □ A　□ B　□ C |

## Standing 站立

| 編號 | 項目 | GitHub 舊版 | 目前盤點版本 | 建議與相依 | 你的選擇 |
|---|---|---|---|---|---|
| S01 | KP 位置修正係數 | 0.35 | 0.08 | C：依實測選值；不要把 Tripod 的 0.38 自動套到這裡；變更位置誤差造成的 PWM；與 KD、前饋和 PWM 上限一起檢視。 | □ A　□ B　□ C |
| S02 | KD 速度修正係數 | 0.002 | 0.006 | C：與該階段速度回饋共同評估；現值為原版 3 倍；實際影響也取決於速度濾波。 | □ A　□ B　□ C |
| S03 | K_FF 速度前饋 | 0.02 | 0.005 | C：與 PWM／摩擦補償成套评估；相同目標速度下前饋為原版四分之一；不是位置保護。 | □ A　□ B　□ C |
| S04 | 額外摩擦補償 | 無，0 PWM | 40×tanh(target_velocity/153.6) PWM | C：量測後決定，不能以固定補償取代控制調校；速度接近零時平滑歸零；目前 80 PWM 上限下影響比例大。 | □ A　□ B　□ C |
| S05 | 控制速度濾波 | 原始位置差分，無濾波 | 固定 20 ms；原始速度仍供停穩等判斷 | C：比較原始／短時間常數；不要直接改共用預設影響別的模式；減少速度尖峰，也增加回饋延遲；不等於輸出 slew。 | □ A　□ B　□ C |
| S06 | 主驅動最大 PWM | 500 | 80，且原生共用硬上限也為 80 | C：與增益、負載和該模式一起評估，不沿用 Tripod 3300；只改 YAML 到 500 會被拒絕；需同步驗證器，Standing 上限還影響 Manual。 | □ A　□ B　□ C |
| S07 | 尋 Hall／半圈軌跡 | 尋零直接36°/s；半圈約5秒，終點速度直接歸零 | 尋零0.5秒加速；半圈加減速，約5.5秒；仍36°/s／180° | B：保留平滑切換；最高速度與半圈終點相同，時間與速度連續性不同。 | □ A　□ B　□ C |
| S08 | 到位容許位置誤差 | <200 counts，約1.30° | <1000 counts，約6.51°（以55296 counts/rev換算） | B：依先前明確要求放寬後的值；這是到位門檻；不是 Tripod 的9000/18000追蹤門檻。 | □ A　□ B　□ C |
| S09 | 到位停穩條件 | 只檢查到時且位置到位 | 還需速度 <500 counts/s、連續0.3秒 | B：保留停穩確認；避免高速經過目標就被判定完成；可分別在備註指定速度／時間。 | □ A　□ B　□ C |
| S10 | 尋 Hall／轉半圈等待上限 | 均無明確 timeout | 均60秒；可設 >0～600秒 | B：保留60秒，有需求再選 C 分別調整；兩個階段分別计時；未完成會留下實際位置、誤差和速度。 | □ A　□ B　□ C |
| S11 | 到位後保持出力 | 0.1×位置誤差，PWM限幅±300，沒有速度阻尼 | min(KP,0.1)×誤差−KD×濾波速度；現值0.08/0.006，限幅±80 | C：保持阻尼，與上限／KP成套評估；不只是到位門檻；它影響等候 Tripod 前的姿態保持。 | □ A　□ B　□ C |
| S12 | 保持位置遺失保護 | 無 | >12000 counts 時停止，約78.13°；可設上界55296 counts | C：區分保持失效與一般到位，不只改錯誤文字；此停止仍存在；未被 Tripod 的 warn_only 影響。 | □ A　□ B　□ C |
| S13 | Standing 完成與退出 | 全6腳DONE後持續保持，印ALL LEGS STANDING | 全部健康腳DONE後持續保持，寫成功紀錄；SIGINT走新版停止 | B：保留新版停止及可驗證完成；正常完成不是自動退出；恢復舊版需同步六腿條件與流程。 | □ A　□ B　□ C |

## Tripod 步態

| 編號 | 項目 | GitHub 舊版 | 目前盤點版本 | 建議與相依 | 你的選擇 |
|---|---|---|---|---|---|
| T01 | KP | 0.38 | 0.38 | B：已與原版相同；已恢復，無額外改值。 | □ A　□ B　□ C |
| T02 | KD | 0.003 | 0.003 | B：已與原版相同；使用者已確認不是0.03。 | □ A　□ B　□ C |
| T03 | K_FF | 0.005 | 0.005 | B：已與原版相同；速度前饋相同。 | □ A　□ B　□ C |
| T04 | 摩擦補償 | 0 | 0 | B：已與原版相同；已取消先前40 PWM補償。 | □ A　□ B　□ C |
| T05 | 速度回饋 | 原始差分 | 5 ms濾波；設0可用原始差分 | B：先保留5ms；要純原版比較可選A；5ms為離線折衷，尚非實機最佳值。 | □ A　□ B　□ C |
| T06 | 起步時間 | 4秒 | 8秒 | B：依你明確指定8秒；8秒也已存在本機已提交HEAD；不是全部由最近修改引入。 | □ A　□ B　□ C |
| T07 | 起步曲線／銜接 | 三次曲線，起步末端帶步態速度 | 五次靜止到靜止，接短暫平滑相位進入 | B：保留平滑銜接，若選A須與起步時間一起驗證；總行程仍一圈；不只是延長時間。 | □ A　□ B　□ C |
| T08 | 起步結束的位置基準 | 各腿實際位置 | 相同，另留TRIPOD_REBASE紀錄 | B：已恢復原版；起步残差會留紀錄，但不再追補到規劃終點。 | □ A　□ B　□ C |
| T09 | Group B 啟動基準 | B組啟動時再取B組實際位置 | 相同，但略過L3；只做一次 | B：已恢復原版；不重設A組、不重設硬體encoder、不每次跟著實際位置改零點。 | □ A　□ B　□ C |
| T10 | T-ratio 起點／目標 | 8→1 | 8→1 | B：已恢復原版；到1後持續RUNNING；T-ratio是時間縮放，不是位置保護threshold。 | □ A　□ B　□ C |
| T11 | 加速計時方式 | 每callback減0.0002；1kHz時約35秒由8到1 | 同名ratio_step=-0.0002，乘dt/0.001；每秒減0.2 | B：保留原版名目速度與新版時間計算；原版速度取決於回讀頻率；目前用實際經過時間。 | □ A　□ B　□ C |
| T12 | 最大PWM／額外slew | 3300；無額外PWM slew | 3300；slew OFF，250/s僅為未啟用的儲存值 | B：已與原版出力上限一致；不能看到設定裡250就誤判它仍限制Tripod。 | □ A　□ B　□ C |
| T13 | 步態多項式／A、B分組 | A=R1,L2,R3；B=L1,R2,L3；B延後半週期 | 相同基本軌跡與分組；L3不輸出 | B：相同；L3另看G01；常數與平滑進入後的軌跡已做數值比對；不是重新設計A/B。 | □ A　□ B　□ C |
| T14 | 負相位與跨圈 | floor後負相位再減一圈，會重複扣圈 | 移除多減的一圈，保留多圈位置，不採最短角度差 | B：保留修正；此為已確認邊界錯誤；正常活動腳通常不走負相位，非所有實跑異常的原因。 | □ A　□ B　□ C |
| T15 | 有限位置誤差保護 | 沒有這組軟／硬位置停止 | 現場warn_only；軟9000、10筆且0.5秒；硬18000在stop模式才停 | B：保留你選擇的warn_only與診斷；警告值不代表目前會自動停；無效數字仍獨立停止。 | □ A　□ B　□ C |
| T16 | 正常停止與執行時間 | STOPPING每callback ratio+0.002直到10；沒有RUNNING總時限 | 從最後參考2秒減速、5秒期限；RUNNING仍無總時限 | B：依你明確指定保留新版停止；目前不會在到target ratio後自動完成；仍依停止輸入結束。 | □ A　□ B　□ C |
| T17 | 觸發前後診斷 | pid/data與一般日誌，無完整首因／觸發框 | TRACE/history、target/actual、raw/filtered速度、原始/限幅PWM、飽和時間、首個原因 | B：保留可核對紀錄；追蹤已確認的命令限制問題；不更改控制目標。 | □ A　□ B　□ C |

## 共用設定、座標與保護

| 編號 | 項目 | GitHub 舊版 | 目前盤點版本 | 建議與相依 | 你的選擇 |
|---|---|---|---|---|---|
| G01 | L3及全域主驅動屏蔽 | 固定六腿，沒有共享mask | 原生FSM L3屏蔽，supported_leg_test | B：保留L3；不把其他項A當成解除L3；選A會涉及恢復六腿；你目前明確說L3仍故障。Servo全域模式不是逐腳主驅動mask。 | □ A　□ B　□ C |
| G02 | 編碼器比例／方向／零點 | Cali/Standing 55296且左反號；Tripod54984.83且右反號 | 兩套历史慣例均保留；未盲目統一 | B：先保留；實體規格未確認；比例差約0.56%；C不能用猜測取代齒比、編碼器或一圈量測。 | □ A　□ B　□ C |
| G03 | Cali/Standing反方向行程保護 | 沒有 | 相對起點反向超過500 counts即停止 | B：保留辨識；要調數字可選C；不是禁止所有反向PWM；是辨識實際encoder總行程方向。 | □ A　□ B　□ C |
| G04 | 供電電壓範圍 | FSM未檢查PowerState | 現場18～42V；超界5筆觸發；原生可設定硬界18～42V | B：保留供電檢查；動作限制與供電界線是不同層；上電前工具另有門檻。 | □ A　□ B　□ C |
| G05 | 腿電流／bus電流保護 | FSM未檢查 | 腿5A連續25筆；可設硬界10A/100筆；bus30A設定但stop_on_bus_current_limit=false | B：保留現場腿保護；C需硬體額定依據；不能宣稱bus30A目前會自動停止；單位是回讀電流，非PWM。 | □ A　□ B　□ C |
| G06 | 馬達／電源回讀失聯期限 | 沒有完整watchdog | 馬達0.25s、電源0.5s；初次資料最多2s；motor來源年齡0.10s、power0.35s | B：保留有效通訊檢查；失聯時舊命令不可視為有效；与Bridge100ms命令watchdog是不同方向。 | □ A　□ B　□ C |
| G07 | 來源身分／序號／QoS | 一般訂閱；Cali/Standing/Tripod queue10 | 最新資料QoS、唯一Bridge來源GID／序號／時間檢查；啟動graph準備最多8s | B：保留整組，不能只換舊Bridge；部分命令與回讀握手欄位依賴新版Bridge。 | □ A　□ B　□ C |
| G08 | 馬達啟用握手 | 收到回讀就能開始產生有效命令 | 等待Bridge epoch、disabled rearm及相關active ack；ready5s、heartbeat/ack0.25s | B：保留新版Bridge/FSM配套；用來區分送出命令與Bridge接受命令；兩端必須相容。 | □ A　□ B　□ C |
| G09 | 唯一設定來源與可調範圍 | 大多C++常數，各檔獨立 | 固定site YAML；未知參數／-p／params-file覆寫被拒絕；有原子tune入口 | B：保留單一來源；數值按各行決定；恢復常數可能讓GUI顯示值與實際值分離；改超出硬上界必須連驗證器一起修改。 | □ A　□ B　□ C |
| G10 | 成功紀錄與單一動作限制 | 無跨程序成功憑據／排他鎖 | revision/hash/boot ID綁定；未完成不能偽造；共享鎖排除同時動作 | B：保留，優化錯誤說明可選C；與Control Panel重試、校正快取、Windows核對相依。 | □ A　□ B　□ C |

## Bridge 通訊與停止

| 編號 | 項目 | GitHub 舊版 | 目前盤點版本 | 建議與相依 | 你的選擇 |
|---|---|---|---|---|---|
| B01 | Core目標IP與環境變數 | 程式setenv CORE_IP=192.168.30.12 | 啟動參數實際192.168.30.254；YAML預設.2；CORE_MASTER_ADDR=.254:50051；Jetson=.8 | B：保留現場位址；原版IP不是當前設備位址；不能逐字回復造成連錯。 | □ A　□ B　□ C |
| B02 | Bridge motor PWM／servo範圍驗證 | 直接轉送，Bridge無此數值檢查 | PWM 0～3300；servo encoder0～65535，無效命令拒絕 | B：保留驗證與3300；Tripod3300與Bridge3300已一致；Windows仍需期望值3300。 | □ A　□ B　□ C |
| B03 | 命令失聯與過期 | 無100ms命令watchdog／時戳上限 | motor命令timeout100ms、max age100ms；power age200ms | B：保留；不應把UI動作時長當成有效命令期限。 | □ A　□ B　□ C |
| B04 | 單一發布者／rearm／ack | 任何發布者命令可直接轉送 | 唯一來源；換來源先5筆停用命令；epoch及相關ack | B：保留兩端配套；現行FSM不能直接搭配完全原版Bridge，否則等不到ack。 | □ A　□ B　□ C |
| B05 | 軟體急停與上電命令epoch | 沒有目前的鎖定與跨epoch機制 | estop鎖定；false不直接解除；上電命令限制來源與順序，全關電命令有独立處理 | B：保留；保留急停、關電語意，不用位置誤差放寬來清除急停狀態。 | □ A　□ B　□ C |
| B06 | 回讀時間戳與重播處理 | 直接將sbRIO stamp換成ROS stamp | ROS stamp採Jetson收到新回讀時間；保留seq；完全重播封包不轉送 | B：保留已驗證的來源時間契約；兩台時鐘不同時可避免錯判；也不是已量到完整端到端延遲。 | □ A　□ B　□ C |
| B07 | 停止重送／心跳／關閉 | 主迴圈結束後shutdown，缺明確停用重送 | 停用重送20ms；心跳50ms；輸出狀態20ms；退出8筆disabled+3筆off | B：保留；Bridge退出與只停止Tripod不同；舊版原始碼不等於已證實安全關電。 | □ A　□ B　□ C |
| B08 | 通訊主迴圈／消息數值映射 | 主迴圈1000Hz；六腿按原順序直接映射enable/dir/voltage | 主迴圈仍1000Hz；保留主要訊息映射，新增驗證與診斷鏡像 | B：核心映射相同；名目1000Hz不是實測收到的封包率；PWM未被換算成百分比。 | □ A　□ B　□ C |

## 逐腳動作

| 編號 | 項目 | GitHub 舊版 | 目前盤點版本 | 建議與相依 | 你的選擇 |
|---|---|---|---|---|---|
| M01 | 逐腳動作模式與參考 | 無rinbo_manual；舊PID測試不是等價入口 | 相對位移／定點／速度／相位；平滑對齊、加減速與收尾 | B：保留逐腳操作；選A代表回到不使用此新增入口；不能直接以舊PID测试取代。 | □ A　□ B　□ C |
| M02 | 逐腳KP/KD/FF與摩擦 | 無此控制器 | 沿用Standing設定：0.08/0.006/0.005、摩擦40、20ms濾波 | C：評估拆成獨立參數，避免互相牽動；調Standing係數會同時影響此路徑；需先拆設定才能完全獨立。 | □ A　□ B　□ C |
| M03 | 逐腳PWM cap與slew | 無此控制器 | min(plan cap,Standing cap)=80；硬slew250 PWM/s | C：用逐腳需求評估，不直接套Tripod3300；Tripod移除slew不影響Manual；可能限制快速動作，需個別評估。 | □ A　□ B　□ C |
| M04 | 逐腳追蹤門檻／錯誤文字 | 無此控制器 | 實際>12000 counts停止，但訊息仍寫>5000 | C：修正文案對齊12000；是否改門檻另註明；已確認訊息與實際程式不一致；本輪不改，先列入恢復決策。 | □ A　□ B　□ C |
| M05 | 逐腳對齊完成條件 | 無此控制器 | 對齊後位置差≤2°、速度≤5°/s；超過不進入主動作 | C：依實際對齊需求評估；與持續追蹤門檻不同。 | □ A　□ B　□ C |
| M06 | 逐腳時間／速度／加速度 | 無此控制器 | 原生可填時間1～60s、速度/加速度1～90；GUI目前L2速度90°/s、加速度10°/s²、10s、PWM80 | C：檢視目前90°/s需求與80/250相容性；這是儲存的下次動作設定，不代表正在執行；GUI預設10°/s而原生Plan預設30。 | □ A　□ B　□ C |

## 操作台與連線流程

| 編號 | 項目 | GitHub 舊版 | 目前盤點版本 | 建議與相依 | 你的選擇 |
|---|---|---|---|---|---|
| U01 | 文字Control Panel與步驟 | GitHub只有另一套PyQt GUI | robot.sh：選脚/計畫/五階段執行/快捷與收藏/現場模式/到位限制 | B：保留文字入口；目前文字台主動作是Manual，沒有直接複製Windows Tripod整套流程。 | □ A　□ B　□ C |
| U02 | 第1步連線自動整理 | 舊GUI只Popen本機Bridge | 按1核對並正常停止已識別動作、整理過期owned紀錄、重建自有SSH；重用有效Core/driver/Bridge | B：保留已明確選擇的第1步行為；不清除無關SSH；登入本身不動作。第3步仍拒絕其他動作占用。 | □ A　□ B　□ C |
| U03 | 操作台消失／子程序生命週期 | 舊GUI在closeEvent主動送SIGINT，無parent-death wrapper | guardian用parent-death SIGINT通知子程序；含其啟動的Bridge | B：保留受控子程序語意；獨立背景服務另選C；與Windows nohup wrapper不同；不能混稱關閉任意監看都會關Bridge。 | □ A　□ B　□ C |
| U04 | 正常完成／失敗後供電流程 | 舊GUI按鈕直接發power command；reset全關 | Manual正常完成用sensors模式保留感測器、關relay；執行階段失败/取消嘗試all-off | B：保留清楚區分；如要統一流程可選C；Tripod單獨SIGINT不等同此Manual流程，也不等同整機停止。 | □ A　□ B　□ C |
| U05 | 上電前工具門檻 | 舊GUI沒有同等核對 | power tool健康腿電流須<3A，與FSM動作時5A不同 | C：依上電前與運轉中用途分别評估；不同階段／層的門檻；不要只改FSM後以為所有地方都放寬。 | □ A　□ B　□ C |
| U06 | 校正快取／重試與錯誤說明 | 沒有sensor_epoch與原生成功紀錄核對 | 本次操作台完成校正、sensor_epoch未變、原生紀錄有效才沿用；失敗不自動重跑 | B：保留紀錄核對；精簡流程可選C；選A需重做整個狀態判斷，不是清除一個error旗標。 | □ A　□ B　□ C |

## Sim2Real／策略與轉接

| 編號 | 項目 | GitHub 舊版 | 目前盤點版本 | 建議與相依 | 你的選擇 |
|---|---|---|---|---|---|
| R01 | Sim2Real整體與所選profile | GitHub没有這三套redrhex套件 | 有redrhex_msgs、lowlevel_bridge、rl_controller與多組profile；目前未觀察到運行中的策略程序 | B：保留原始碼；啟用profile需另確認；沒有可直接恢復的舊版RL數字；A代表停用新增路徑而非填0。 | □ A　□ B　□ C |
| R02 | Sim2Real的腿位mask | 無RL設定 | 原生FSM=L3；site full_feedback兩份=[]；sensor_v2兩份=[L1] | C：在選定profile後對齊L3，不能現在盲改全部模板；这些檔案不自動等同原生site mask；必須按實際launch配置配對。沒有因此修改L3。 | □ A　□ B　□ C |
| R03 | RL速度到PWM轉換／限幅 | 無 | 基礎rinbo adapter：40 PWM/(rad/s)、cap80、slew250/s | C：確認實際profile及馬達關係後調整；不是Tripod PD公式，也不是SBReal已量測的馬達模型；部分profile不同。 | □ A　□ B　□ C |
| R04 | RL encoder／方向／伺服換算 | 無RL對照 | 基礎54984.83 counts/rev、encoder左負右正；ABAD1000 counts/rad為待校準值 | C：量測後設定，不猜比例與方向；引用Tripod的counts常數不是硬體規格證據；方向還有命令sign hook。 | □ A　□ B　□ C |
| R05 | RL初始站姿到位 | 無 | base：2s參考/12s timeout/.12rad/.25rad/s/.5s；site兩份timeout30s | C：按選定profile評估，不改Cali/Standing；不是rinbo_standing；Control Panel選15的Sim2Real頁只改明確選中的profile。 | □ A　□ B　□ C |
| R06 | RL主驅動與ABAD動作限幅 | 無 | base：主速度30rad/s、slew120rad/s²；ABAD角0.7rad、slew6rad/s；site rig通常12與1 | C：選定profile後統一操作需求；不同profile數字差異很大，完整值另附CSV；不宣稱base目前生效。 | □ A　□ B　□ C |
| R07 | RL資料、推論、姿態等保護 | 無 | base有姿態0.7rad、sensor0.10s、cmd0.25s、推論8ms／loop30ms等；profile有更嚴格契約 | B：保留有效資料；閾值C需個別評估；硬體觀測缺失、ONNX契約或mask不符不能由提高PWM修復。 | □ A　□ B　□ C |
| R08 | RL policy檔案／契約／啟動使能 | 無 | ONNX觀測/動作契約、hash與啟動輸出檢查；基礎enable_policy_on_start=false、enable_motor_output_on_start=false | B：保留契約檢查與明確啟動；模型／硬體校準未證實；本輪不載入模型或執行推論／動作。 | □ A　□ B　□ C |
| R09 | RL供電與通訊門檻 | 無 | base adapter18～30V/3A/3筆；site rig18～42V/5A；state0.25s、power0.35s、cmd0.10s | C：確認所選profile；保留供電／通訊功能；再次說明profile與native FSM不同；不是全部平台已統一成同一數字。 | □ A　□ B　□ C |

## 記錄、監控與訊息

| 編號 | 項目 | GitHub 舊版 | 目前盤點版本 | 建議與相依 | 你的選擇 |
|---|---|---|---|---|---|
| D01 | 資料記錄內容與CSV介面 | trigger後依pid/data寫單一CSV；actual右腿反號 | summary.csv/events.csv/metadata；100Hz摘要；原始motor state、命令、power、debug、首因；auto_start預設true | B：保留完整記錄；需要舊格式可選C加相容匯出；欄名／座標不能直接拿舊分析程式套用；CSV摘要不是每筆控制回讀。 | □ A　□ B　□ C |
| D02 | 浏览器監控／錄製 | 無rinbo_monitor | 唯讀狀態、原始/轉送PWM、錄製；port8088、顯示過期0.5s | B：保留獨立監看；顯示門檻不會改控制器保護；頁面無上電／馬達控制路由。 | □ A　□ B　□ C |
| D03 | ROS消息契約 | 9個基本.msg | 基本.msg內容相同（只有換行差）；新增ControllerDebugStamped、SafetyEventStamped | B：保留新增診斷型別；不是把原來motor/command欄位重編碼；新增型別需相容建置。 | □ A　□ B　□ C |

## 舊版其他套件／建置

| 編號 | 項目 | GitHub 舊版 | 目前盤點版本 | 建議與相依 | 你的選擇 |
|---|---|---|---|---|---|
| O01 | 舊PyQt GUI與啟動名稱 | 有rinbo_panel；Tripod按鈕呼叫rinbo_tripod_rslip，但同倉庫CMake只產生rinbo_tripod | 本機沒有此package；Windows桌面App也不是這份原始碼 | C：若要恢復舊GUI，保留外觀但修正入口與新版握手；已確認舊GUI／CMake入口名稱不一致；整包照抄不能保證可用。 | □ A　□ B　□ C |
| O02 | 舊PID測試／單腿RSLIP | 有rinbo_pid_test：正弦5000 counts、0.2Hz、PWM500；另有rinbo_traj_rslip | 本機沒有這兩支；另有rinbo_sin_sweep退役stub及Manual入口 | C：如需測PID，做相容獨立測試入口；原版測試直接發布motor命令；不同於新版逐腳控制，不適合未核對直接混用。 | □ A　□ B　□ C |
| O03 | IMU子模組 | gitlink src/microstrain_inertial | 本機src沒有此子模組 | C：若需要IMU，先確認實際型號、來源與安裝位置；GitHub樹只有指向commit；沒有同倉庫.gitmodules，不能假裝已拿到完整IMU驅動內容。 | □ A　□ B　□ C |
| O04 | 建置與README | ROS2 ament CMake；README混有Corgi/ROS1 catkin說明 | 新增yaml-cpp/OpenSSL/診斷消息、robot_config靜態庫與測試；README改為當前入口 | B：保留當前建置；只按所選控制項調整；不能用舊README判定整包應改回ROS1；恢復來源要同步相依套件與執行檔名稱。 | □ A　□ B　□ C |

## FPGA 與 Windows 邊界

| 編號 | 項目 | GitHub 舊版 | 目前盤點版本 | 建議與相依 | 你的選擇 |
|---|---|---|---|---|---|
| F01 | FPGA console生命週期候選 | 本GitHub沒有FPGA driver原始碼，無法以此倉庫定義原版 | tools/fpga_lifecycle是依sbRIO歷史正式源碼製作的候選；文件標示未部署 | B：保留候選供另案部署決策，不自動替換驅動；候選修正EOF/HUP/ERR/忙迴圈、背景/監看責任及off流程；不能當成目前sbRIO已運行版本。 | □ A　□ B　□ C |
| F02 | Windows期待值／停止入口 | 本GitHub的PyQt GUI不是現用Windows程式；無Windows source | 已有交接prompt；Bridge expected PWM需3300、site revision/hash動態讀取；Tripod stop≠all-off | C：Windows提供當前程式後另比對；Orin先保留現況；無法在這台Orin核對Windows目前實作；不能說已替Windows修改完成。 | □ A　□ B　□ C |
