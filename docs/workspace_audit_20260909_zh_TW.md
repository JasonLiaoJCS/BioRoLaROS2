# Orin 全工作區與 GitHub 舊版詳細比對（2026-09-09）

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

### Calibration 校正

**C01 — KP 位置修正係數**

- 舊版：0.35。
- 現況：0.08。
- 影響：變更位置誤差造成的 PWM；與 KD、前饋和 PWM 上限一起檢視。
- 建議：C：依實測選值；不要把 Tripod 的 0.38 自動套到這裡。
- 位置：[rinbo_cali.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_cali.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**C02 — KD 速度修正係數**

- 舊版：0.002。
- 現況：0.006。
- 影響：現值為原版 3 倍；實際影響也取決於速度濾波。
- 建議：C：與該階段速度回饋共同評估。
- 位置：[rinbo_cali.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_cali.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**C03 — K_FF 速度前饋**

- 舊版：0.02。
- 現況：0.005。
- 影響：相同目標速度下前饋為原版四分之一；不是位置保護。
- 建議：C：與 PWM／摩擦補償成套评估。
- 位置：[rinbo_cali.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_cali.cpp)、[motion_effort.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/motion_effort.hpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**C04 — 額外摩擦補償**

- 舊版：無，0 PWM。
- 現況：40×tanh(target_velocity/153.6) PWM。
- 影響：速度接近零時平滑歸零；目前 80 PWM 上限下影響比例大。
- 建議：C：量測後決定，不能以固定補償取代控制調校。
- 位置：[motion_effort.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/motion_effort.hpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**C05 — 控制速度濾波**

- 舊版：原始位置差分，無濾波。
- 現況：固定 20 ms；原始速度仍供停穩等判斷。
- 影響：減少速度尖峰，也增加回饋延遲；不等於輸出 slew。
- 建議：C：比較原始／短時間常數；不要直接改共用預設影響別的模式。
- 位置：[control_velocity.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/control_velocity.hpp)、[rinbo_cali.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_cali.cpp)。

**C06 — 主驅動最大 PWM**

- 舊版：500。
- 現況：80，且原生共用硬上限也為 80。
- 影響：只改 YAML 到 500 會被拒絕；需同步驗證器，Standing 上限還影響 Manual。
- 建議：C：與增益、負載和該模式一起評估，不沿用 Tripod 3300。
- 位置：[rinbo_cali.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_cali.cpp)、[safety_invariants.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/safety_invariants.hpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**C07 — 尋 Hall 巡航速度**

- 舊版：0.2π rad/s＝36°/s＝5529.6 counts/s。
- 現況：相同。
- 影響：這個速度並未被降低；不可把較低 PWM 誤認成目標速度變慢。
- 建議：B：相同，無須回復。
- 位置：[rinbo_cali.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_cali.cpp)。

**C08 — 尋 Hall 起步曲線**

- 舊版：直接切到固定速度的線性位置參考。
- 現況：0.5 秒加速到同一巡航速度。
- 影響：速度上限不變；移除 ramp 會恢復起步速度跳變。
- 建議：B：保留平滑起步。
- 位置：[motor_tracking.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/motor_tracking.hpp)、[rinbo_cali.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_cali.cpp)。

**C09 — 伺服定位等待上限**

- 舊版：無明確 timeout。
- 現況：60 秒；可設範圍 >0～600 秒。
- 影響：原版可能永遠等不到定位；目前逾時報出第一個原因。
- 建議：B：目前 60 秒；若確有慢速需求可選 C。
- 位置：[rinbo_cali.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_cali.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**C10 — 主馬達尋 Hall 等待上限**

- 舊版：無明確 timeout。
- 現況：60 秒；可設範圍 >0～600 秒。
- 影響：位置／尋零等待，不是通訊失聯期限。
- 建議：B：目前 60 秒。
- 位置：[rinbo_cali.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_cali.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**C11 — 停穩／歸零等待上限**

- 舊版：無明確 timeout。
- 現況：各階段 15 秒；可設範圍 >0～600 秒。
- 影響：停止與歸零各有期限；操作台等待预算也依這些設定計算。
- 建議：B：15 秒；與歸零確認一起保留。
- 位置：[rinbo_cali.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_cali.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**C12 — Hall 後停穩與歸零完成判定**

- 舊版：速度 <500 counts/s 持續 0.3 秒，送一次 reset 就 DONE。
- 現況：先停穩，再等待 |raw position|≤100 counts 且低速回讀；允許有界 reset 重送。
- 影響：原版送出命令不代表歸零已完成；恢復可能影響下一步站立原點。
- 建議：B：保留回讀確認。
- 位置：[rinbo_cali.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_cali.cpp)。

**C13 — 伺服中立目標／到位容差**

- 舊版：[740,2565,3283,1944,2071,989]；容差 100。
- 現況：相同；差值改用較寬的帶符號整數計算。
- 影響：硬體零點不由程式猜；全域 servo_control_mode 仍可能控制所有已接伺服。
- 建議：B：保留既有目標與數值修正。
- 位置：[rinbo_cali.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_cali.cpp)。

**C14 — 只校正選中腳與成功紀錄**

- 舊版：固定全六腿，沒有分腳成功憑據。
- 現況：支援 --plan 選中腳；其他主驅動不參與；保存已確認腳位紀錄。
- 影響：與 robot.sh 逐腳控制、L3 屏蔽相依；不能單獨刪除憑據流程。
- 建議：B：保留逐腳校正能力。
- 位置：[rinbo_cali.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_cali.cpp)、[robot_config.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/robot_config.cpp)。


### Standing 站立

**S01 — KP 位置修正係數**

- 舊版：0.35。
- 現況：0.08。
- 影響：變更位置誤差造成的 PWM；與 KD、前饋和 PWM 上限一起檢視。
- 建議：C：依實測選值；不要把 Tripod 的 0.38 自動套到這裡。
- 位置：[rinbo_standing.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_standing.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**S02 — KD 速度修正係數**

- 舊版：0.002。
- 現況：0.006。
- 影響：現值為原版 3 倍；實際影響也取決於速度濾波。
- 建議：C：與該階段速度回饋共同評估。
- 位置：[rinbo_standing.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_standing.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**S03 — K_FF 速度前饋**

- 舊版：0.02。
- 現況：0.005。
- 影響：相同目標速度下前饋為原版四分之一；不是位置保護。
- 建議：C：與 PWM／摩擦補償成套评估。
- 位置：[rinbo_standing.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_standing.cpp)、[motion_effort.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/motion_effort.hpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**S04 — 額外摩擦補償**

- 舊版：無，0 PWM。
- 現況：40×tanh(target_velocity/153.6) PWM。
- 影響：速度接近零時平滑歸零；目前 80 PWM 上限下影響比例大。
- 建議：C：量測後決定，不能以固定補償取代控制調校。
- 位置：[motion_effort.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/motion_effort.hpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**S05 — 控制速度濾波**

- 舊版：原始位置差分，無濾波。
- 現況：固定 20 ms；原始速度仍供停穩等判斷。
- 影響：減少速度尖峰，也增加回饋延遲；不等於輸出 slew。
- 建議：C：比較原始／短時間常數；不要直接改共用預設影響別的模式。
- 位置：[control_velocity.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/control_velocity.hpp)、[rinbo_standing.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_standing.cpp)。

**S06 — 主驅動最大 PWM**

- 舊版：500。
- 現況：80，且原生共用硬上限也為 80。
- 影響：只改 YAML 到 500 會被拒絕；需同步驗證器，Standing 上限還影響 Manual。
- 建議：C：與增益、負載和該模式一起評估，不沿用 Tripod 3300。
- 位置：[rinbo_standing.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_standing.cpp)、[safety_invariants.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/safety_invariants.hpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**S07 — 尋 Hall／半圈軌跡**

- 舊版：尋零直接36°/s；半圈約5秒，終點速度直接歸零。
- 現況：尋零0.5秒加速；半圈加減速，約5.5秒；仍36°/s／180°。
- 影響：最高速度與半圈終點相同，時間與速度連續性不同。
- 建議：B：保留平滑切換。
- 位置：[motor_tracking.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/motor_tracking.hpp)、[rinbo_standing.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_standing.cpp)。

**S08 — 到位容許位置誤差**

- 舊版：<200 counts，約1.30°。
- 現況：<1000 counts，約6.51°（以55296 counts/rev換算）。
- 影響：這是到位門檻；不是 Tripod 的9000/18000追蹤門檻。
- 建議：B：依先前明確要求放寬後的值。
- 位置：[rinbo_standing.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_standing.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**S09 — 到位停穩條件**

- 舊版：只檢查到時且位置到位。
- 現況：還需速度 <500 counts/s、連續0.3秒。
- 影響：避免高速經過目標就被判定完成；可分別在備註指定速度／時間。
- 建議：B：保留停穩確認。
- 位置：[rinbo_standing.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_standing.cpp)、[robot_config.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/robot_config.cpp)。

**S10 — 尋 Hall／轉半圈等待上限**

- 舊版：均無明確 timeout。
- 現況：均60秒；可設 >0～600秒。
- 影響：兩個階段分別计時；未完成會留下實際位置、誤差和速度。
- 建議：B：保留60秒，有需求再選 C 分別調整。
- 位置：[rinbo_standing.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_standing.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**S11 — 到位後保持出力**

- 舊版：0.1×位置誤差，PWM限幅±300，沒有速度阻尼。
- 現況：min(KP,0.1)×誤差−KD×濾波速度；現值0.08/0.006，限幅±80。
- 影響：不只是到位門檻；它影響等候 Tripod 前的姿態保持。
- 建議：C：保持阻尼，與上限／KP成套評估。
- 位置：[rinbo_standing.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_standing.cpp)。

**S12 — 保持位置遺失保護**

- 舊版：無。
- 現況：>12000 counts 時停止，約78.13°；可設上界55296 counts。
- 影響：此停止仍存在；未被 Tripod 的 warn_only 影響。
- 建議：C：區分保持失效與一般到位，不只改錯誤文字。
- 位置：[rinbo_standing.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_standing.cpp)、[robot_config.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/robot_config.cpp)。

**S13 — Standing 完成與退出**

- 舊版：全6腳DONE後持續保持，印ALL LEGS STANDING。
- 現況：全部健康腳DONE後持續保持，寫成功紀錄；SIGINT走新版停止。
- 影響：正常完成不是自動退出；恢復舊版需同步六腿條件與流程。
- 建議：B：保留新版停止及可驗證完成。
- 位置：[rinbo_standing.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_standing.cpp)、[robot_config.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/robot_config.cpp)。


### Tripod 步態

**T01 — KP**

- 舊版：0.38。
- 現況：0.38。
- 影響：已恢復，無額外改值。
- 建議：B：已與原版相同。
- 位置：[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**T02 — KD**

- 舊版：0.003。
- 現況：0.003。
- 影響：使用者已確認不是0.03。
- 建議：B：已與原版相同。
- 位置：[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**T03 — K_FF**

- 舊版：0.005。
- 現況：0.005。
- 影響：速度前饋相同。
- 建議：B：已與原版相同。
- 位置：[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**T04 — 摩擦補償**

- 舊版：0。
- 現況：0。
- 影響：已取消先前40 PWM補償。
- 建議：B：已與原版相同。
- 位置：[motion_effort.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/motion_effort.hpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**T05 — 速度回饋**

- 舊版：原始差分。
- 現況：5 ms濾波；設0可用原始差分。
- 影響：5ms為離線折衷，尚非實機最佳值。
- 建議：B：先保留5ms；要純原版比較可選A。
- 位置：[control_velocity.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/control_velocity.hpp)、[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)。

**T06 — 起步時間**

- 舊版：4秒。
- 現況：8秒。
- 影響：8秒也已存在本機已提交HEAD；不是全部由最近修改引入。
- 建議：B：依你明確指定8秒。
- 位置：[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**T07 — 起步曲線／銜接**

- 舊版：三次曲線，起步末端帶步態速度。
- 現況：五次靜止到靜止，接短暫平滑相位進入。
- 影響：總行程仍一圈；不只是延長時間。
- 建議：B：保留平滑銜接，若選A須與起步時間一起驗證。
- 位置：[tripod_reference.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/tripod_reference.hpp)、[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)。

**T08 — 起步結束的位置基準**

- 舊版：各腿實際位置。
- 現況：相同，另留TRIPOD_REBASE紀錄。
- 影響：起步残差會留紀錄，但不再追補到規劃終點。
- 建議：B：已恢復原版。
- 位置：[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)。

**T09 — Group B 啟動基準**

- 舊版：B組啟動時再取B組實際位置。
- 現況：相同，但略過L3；只做一次。
- 影響：不重設A組、不重設硬體encoder、不每次跟著實際位置改零點。
- 建議：B：已恢復原版。
- 位置：[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)。

**T10 — T-ratio 起點／目標**

- 舊版：8→1。
- 現況：8→1。
- 影響：到1後持續RUNNING；T-ratio是時間縮放，不是位置保護threshold。
- 建議：B：已恢復原版。
- 位置：[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**T11 — 加速計時方式**

- 舊版：每callback減0.0002；1kHz時約35秒由8到1。
- 現況：同名ratio_step=-0.0002，乘dt/0.001；每秒減0.2。
- 影響：原版速度取決於回讀頻率；目前用實際經過時間。
- 建議：B：保留原版名目速度與新版時間計算。
- 位置：[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)。

**T12 — 最大PWM／額外slew**

- 舊版：3300；無額外PWM slew。
- 現況：3300；slew OFF，250/s僅為未啟用的儲存值。
- 影響：不能看到設定裡250就誤判它仍限制Tripod。
- 建議：B：已與原版出力上限一致。
- 位置：[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**T13 — 步態多項式／A、B分組**

- 舊版：A=R1,L2,R3；B=L1,R2,L3；B延後半週期。
- 現況：相同基本軌跡與分組；L3不輸出。
- 影響：常數與平滑進入後的軌跡已做數值比對；不是重新設計A/B。
- 建議：B：相同；L3另看G01。
- 位置：[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)、[tripod_reference.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/tripod_reference.hpp)。

**T14 — 負相位與跨圈**

- 舊版：floor後負相位再減一圈，會重複扣圈。
- 現況：移除多減的一圈，保留多圈位置，不採最短角度差。
- 影響：此為已確認邊界錯誤；正常活動腳通常不走負相位，非所有實跑異常的原因。
- 建議：B：保留修正。
- 位置：[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)。

**T15 — 有限位置誤差保護**

- 舊版：沒有這組軟／硬位置停止。
- 現況：現場warn_only；軟9000、10筆且0.5秒；硬18000在stop模式才停。
- 影響：警告值不代表目前會自動停；無效數字仍獨立停止。
- 建議：B：保留你選擇的warn_only與診斷。
- 位置：[tripod_reference.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/tripod_reference.hpp)、[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**T16 — 正常停止與執行時間**

- 舊版：STOPPING每callback ratio+0.002直到10；沒有RUNNING總時限。
- 現況：從最後參考2秒減速、5秒期限；RUNNING仍無總時限。
- 影響：目前不會在到target ratio後自動完成；仍依停止輸入結束。
- 建議：B：依你明確指定保留新版停止。
- 位置：[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)、[tripod_reference.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/tripod_reference.hpp)。

**T17 — 觸發前後診斷**

- 舊版：pid/data與一般日誌，無完整首因／觸發框。
- 現況：TRACE/history、target/actual、raw/filtered速度、原始/限幅PWM、飽和時間、首個原因。
- 影響：追蹤已確認的命令限制問題；不更改控制目標。
- 建議：B：保留可核對紀錄。
- 位置：[rinbo_tripod.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_tripod.cpp)。


### 共用設定、座標與保護

**G01 — L3及全域主驅動屏蔽**

- 舊版：固定六腿，沒有共享mask。
- 現況：原生FSM L3屏蔽，supported_leg_test。
- 影響：選A會涉及恢復六腿；你目前明確說L3仍故障。Servo全域模式不是逐腳主驅動mask。
- 建議：B：保留L3；不把其他項A當成解除L3。
- 位置：[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)、[disabled_legs.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/disabled_legs.hpp)。

**G02 — 編碼器比例／方向／零點**

- 舊版：Cali/Standing 55296且左反號；Tripod54984.83且右反號。
- 現況：兩套历史慣例均保留；未盲目統一。
- 影響：比例差約0.56%；C不能用猜測取代齒比、編碼器或一圈量測。
- 建議：B：先保留；實體規格未確認。
- 位置：[rinbo_cali.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_cali.cpp)、[rinbo_standing.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_standing.cpp)、[tripod_reference.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/tripod_reference.hpp)。

**G03 — Cali/Standing反方向行程保護**

- 舊版：沒有。
- 現況：相對起點反向超過500 counts即停止。
- 影響：不是禁止所有反向PWM；是辨識實際encoder總行程方向。
- 建議：B：保留辨識；要調數字可選C。
- 位置：[motor_tracking.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/motor_tracking.hpp)、[rinbo_cali.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_cali.cpp)、[rinbo_standing.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_standing.cpp)。

**G04 — 供電電壓範圍**

- 舊版：FSM未檢查PowerState。
- 現況：現場18～42V；超界5筆觸發；原生可設定硬界18～42V。
- 影響：動作限制與供電界線是不同層；上電前工具另有門檻。
- 建議：B：保留供電檢查。
- 位置：[safety_invariants.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/safety_invariants.hpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**G05 — 腿電流／bus電流保護**

- 舊版：FSM未檢查。
- 現況：腿5A連續25筆；可設硬界10A/100筆；bus30A設定但stop_on_bus_current_limit=false。
- 影響：不能宣稱bus30A目前會自動停止；單位是回讀電流，非PWM。
- 建議：B：保留現場腿保護；C需硬體額定依據。
- 位置：[safety_invariants.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/safety_invariants.hpp)、[rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml)。

**G06 — 馬達／電源回讀失聯期限**

- 舊版：沒有完整watchdog。
- 現況：馬達0.25s、電源0.5s；初次資料最多2s；motor來源年齡0.10s、power0.35s。
- 影響：失聯時舊命令不可視為有效；与Bridge100ms命令watchdog是不同方向。
- 建議：B：保留有效通訊檢查。
- 位置：[safety_invariants.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/safety_invariants.hpp)、[ros_input_guard.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/ros_input_guard.hpp)。

**G07 — 來源身分／序號／QoS**

- 舊版：一般訂閱；Cali/Standing/Tripod queue10。
- 現況：最新資料QoS、唯一Bridge來源GID／序號／時間檢查；啟動graph準備最多8s。
- 影響：部分命令與回讀握手欄位依賴新版Bridge。
- 建議：B：保留整組，不能只換舊Bridge。
- 位置：[ros_input_guard.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/ros_input_guard.hpp)、[latest_state_qos.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/latest_state_qos.hpp)、[bridge_input_discovery.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/bridge_input_discovery.hpp)。

**G08 — 馬達啟用握手**

- 舊版：收到回讀就能開始產生有效命令。
- 現況：等待Bridge epoch、disabled rearm及相關active ack；ready5s、heartbeat/ack0.25s。
- 影響：用來區分送出命令與Bridge接受命令；兩端必須相容。
- 建議：B：保留新版Bridge/FSM配套。
- 位置：[motor_arbiter_handshake.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/motor_arbiter_handshake.hpp)、[rinbo_ros_bridge.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/src/rinbo_ros_bridge.cpp)。

**G09 — 唯一設定來源與可調範圍**

- 舊版：大多C++常數，各檔獨立。
- 現況：固定site YAML；未知參數／-p／params-file覆寫被拒絕；有原子tune入口。
- 影響：恢復常數可能讓GUI顯示值與實際值分離；改超出硬上界必須連驗證器一起修改。
- 建議：B：保留單一來源；數值按各行決定。
- 位置：[robot_config.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/robot_config.cpp)、[robot_config.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/robot_config.hpp)。

**G10 — 成功紀錄與單一動作限制**

- 舊版：無跨程序成功憑據／排他鎖。
- 現況：revision/hash/boot ID綁定；未完成不能偽造；共享鎖排除同時動作。
- 影響：與Control Panel重試、校正快取、Windows核對相依。
- 建議：B：保留，優化錯誤說明可選C。
- 位置：[robot_config.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/robot_config.cpp)。


### Bridge 通訊與停止

**B01 — Core目標IP與環境變數**

- 舊版：程式setenv CORE_IP=192.168.30.12。
- 現況：啟動參數實際192.168.30.254；YAML預設.2；CORE_MASTER_ADDR=.254:50051；Jetson=.8。
- 影響：原版IP不是當前設備位址；不能逐字回復造成連錯。
- 建議：B：保留現場位址。
- 位置：[rinbo_ros_bridge.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/src/rinbo_ros_bridge.cpp)、[redrhex_safe.yaml](/home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/config/redrhex_safe.yaml)。

**B02 — Bridge motor PWM／servo範圍驗證**

- 舊版：直接轉送，Bridge無此數值檢查。
- 現況：PWM 0～3300；servo encoder0～65535，無效命令拒絕。
- 影響：Tripod3300與Bridge3300已一致；Windows仍需期望值3300。
- 建議：B：保留驗證與3300。
- 位置：[motor_output_limits.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/src/motor_output_limits.hpp)、[redrhex_safe.yaml](/home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/config/redrhex_safe.yaml)。

**B03 — 命令失聯與過期**

- 舊版：無100ms命令watchdog／時戳上限。
- 現況：motor命令timeout100ms、max age100ms；power age200ms。
- 影響：不應把UI動作時長當成有效命令期限。
- 建議：B：保留。
- 位置：[rinbo_ros_bridge.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/src/rinbo_ros_bridge.cpp)、[redrhex_safe.yaml](/home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/config/redrhex_safe.yaml)。

**B04 — 單一發布者／rearm／ack**

- 舊版：任何發布者命令可直接轉送。
- 現況：唯一來源；換來源先5筆停用命令；epoch及相關ack。
- 影響：現行FSM不能直接搭配完全原版Bridge，否則等不到ack。
- 建議：B：保留兩端配套。
- 位置：[rinbo_ros_bridge.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/src/rinbo_ros_bridge.cpp)、[motor_arbiter_handshake.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/motor_arbiter_handshake.hpp)。

**B05 — 軟體急停與上電命令epoch**

- 舊版：沒有目前的鎖定與跨epoch機制。
- 現況：estop鎖定；false不直接解除；上電命令限制來源與順序，全關電命令有独立處理。
- 影響：保留急停、關電語意，不用位置誤差放寬來清除急停狀態。
- 建議：B：保留。
- 位置：[rinbo_ros_bridge.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/src/rinbo_ros_bridge.cpp)、[power_command_epoch_guard.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/src/power_command_epoch_guard.hpp)。

**B06 — 回讀時間戳與重播處理**

- 舊版：直接將sbRIO stamp換成ROS stamp。
- 現況：ROS stamp採Jetson收到新回讀時間；保留seq；完全重播封包不轉送。
- 影響：兩台時鐘不同時可避免錯判；也不是已量到完整端到端延遲。
- 建議：B：保留已驗證的來源時間契約。
- 位置：[rinbo_ros_bridge.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/src/rinbo_ros_bridge.cpp)。

**B07 — 停止重送／心跳／關閉**

- 舊版：主迴圈結束後shutdown，缺明確停用重送。
- 現況：停用重送20ms；心跳50ms；輸出狀態20ms；退出8筆disabled+3筆off。
- 影響：Bridge退出與只停止Tripod不同；舊版原始碼不等於已證實安全關電。
- 建議：B：保留。
- 位置：[rinbo_ros_bridge.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/src/rinbo_ros_bridge.cpp)、[redrhex_safe.yaml](/home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/config/redrhex_safe.yaml)。

**B08 — 通訊主迴圈／消息數值映射**

- 舊版：主迴圈1000Hz；六腿按原順序直接映射enable/dir/voltage。
- 現況：主迴圈仍1000Hz；保留主要訊息映射，新增驗證與診斷鏡像。
- 影響：名目1000Hz不是實測收到的封包率；PWM未被換算成百分比。
- 建議：B：核心映射相同。
- 位置：[rinbo_ros_bridge.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/src/rinbo_ros_bridge.cpp)。


### 逐腳動作

**M01 — 逐腳動作模式與參考**

- 舊版：無rinbo_manual；舊PID測試不是等價入口。
- 現況：相對位移／定點／速度／相位；平滑對齊、加減速與收尾。
- 影響：選A代表回到不使用此新增入口；不能直接以舊PID测试取代。
- 建議：B：保留逐腳操作。
- 位置：[manual_motion.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/manual_motion.hpp)、[rinbo_manual.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_manual.cpp)。

**M02 — 逐腳KP/KD/FF與摩擦**

- 舊版：無此控制器。
- 現況：沿用Standing設定：0.08/0.006/0.005、摩擦40、20ms濾波。
- 影響：調Standing係數會同時影響此路徑；需先拆設定才能完全獨立。
- 建議：C：評估拆成獨立參數，避免互相牽動。
- 位置：[rinbo_manual.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_manual.cpp)、[robot_config.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/robot_config.cpp)。

**M03 — 逐腳PWM cap與slew**

- 舊版：無此控制器。
- 現況：min(plan cap,Standing cap)=80；硬slew250 PWM/s。
- 影響：Tripod移除slew不影響Manual；可能限制快速動作，需個別評估。
- 建議：C：用逐腳需求評估，不直接套Tripod3300。
- 位置：[rinbo_manual.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_manual.cpp)、[safety_invariants.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/safety_invariants.hpp)。

**M04 — 逐腳追蹤門檻／錯誤文字**

- 舊版：無此控制器。
- 現況：實際>12000 counts停止，但訊息仍寫>5000。
- 影響：已確認訊息與實際程式不一致；本輪不改，先列入恢復決策。
- 建議：C：修正文案對齊12000；是否改門檻另註明。
- 位置：[rinbo_manual.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_manual.cpp)、[safety_invariants.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/safety_invariants.hpp)。

**M05 — 逐腳對齊完成條件**

- 舊版：無此控制器。
- 現況：對齊後位置差≤2°、速度≤5°/s；超過不進入主動作。
- 影響：與持續追蹤門檻不同。
- 建議：C：依實際對齊需求評估。
- 位置：[rinbo_manual.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_manual.cpp)。

**M06 — 逐腳時間／速度／加速度**

- 舊版：無此控制器。
- 現況：原生可填時間1～60s、速度/加速度1～90；GUI目前L2速度90°/s、加速度10°/s²、10s、PWM80。
- 影響：這是儲存的下次動作設定，不代表正在執行；GUI預設10°/s而原生Plan預設30。
- 建議：C：檢視目前90°/s需求與80/250相容性。
- 位置：[manual_motion.hpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/manual_motion.hpp)、[plans.py](/home/jetson/rinbo_ros_ws/src/rinbo_control/rinbo_control/plans.py)、[settings.json](/home/jetson/.local/state/rinbo_control/settings.json)。


### 操作台與連線流程

**U01 — 文字Control Panel與步驟**

- 舊版：GitHub只有另一套PyQt GUI。
- 現況：robot.sh：選脚/計畫/五階段執行/快捷與收藏/現場模式/到位限制。
- 影響：目前文字台主動作是Manual，沒有直接複製Windows Tripod整套流程。
- 建議：B：保留文字入口。
- 位置：[robot.sh](/home/jetson/rinbo_ros_ws/robot.sh)、[console.py](/home/jetson/rinbo_ros_ws/src/rinbo_control/rinbo_control/console.py)。

**U02 — 第1步連線自動整理**

- 舊版：舊GUI只Popen本機Bridge。
- 現況：按1核對並正常停止已識別動作、整理過期owned紀錄、重建自有SSH；重用有效Core/driver/Bridge。
- 影響：不清除無關SSH；登入本身不動作。第3步仍拒絕其他動作占用。
- 建議：B：保留已明確選擇的第1步行為。
- 位置：[connection_recovery.py](/home/jetson/rinbo_ros_ws/src/rinbo_control/rinbo_control/connection_recovery.py)、[runtime.py](/home/jetson/rinbo_ros_ws/src/rinbo_control/rinbo_control/runtime.py)、[sbrio.py](/home/jetson/rinbo_ros_ws/src/rinbo_control/rinbo_control/sbrio.py)。

**U03 — 操作台消失／子程序生命週期**

- 舊版：舊GUI在closeEvent主動送SIGINT，無parent-death wrapper。
- 現況：guardian用parent-death SIGINT通知子程序；含其啟動的Bridge。
- 影響：與Windows nohup wrapper不同；不能混稱關閉任意監看都會關Bridge。
- 建議：B：保留受控子程序語意；獨立背景服務另選C。
- 位置：[guardian.py](/home/jetson/rinbo_ros_ws/src/rinbo_control/rinbo_control/guardian.py)、[runtime.py](/home/jetson/rinbo_ros_ws/src/rinbo_control/rinbo_control/runtime.py)。

**U04 — 正常完成／失敗後供電流程**

- 舊版：舊GUI按鈕直接發power command；reset全關。
- 現況：Manual正常完成用sensors模式保留感測器、關relay；執行階段失败/取消嘗試all-off。
- 影響：Tripod單獨SIGINT不等同此Manual流程，也不等同整機停止。
- 建議：B：保留清楚區分；如要統一流程可選C。
- 位置：[runtime.py](/home/jetson/rinbo_ros_ws/src/rinbo_control/rinbo_control/runtime.py)。

**U05 — 上電前工具門檻**

- 舊版：舊GUI沒有同等核對。
- 現況：power tool健康腿電流須<3A，與FSM動作時5A不同。
- 影響：不同階段／層的門檻；不要只改FSM後以為所有地方都放寬。
- 建議：C：依上電前與運轉中用途分别評估。
- 位置：[rinbo_power_tool.py](/home/jetson/rinbo_ros_ws/src/redrhex_lowlevel_bridge/redrhex_lowlevel_bridge/rinbo_power_tool.py)、[runtime.py](/home/jetson/rinbo_ros_ws/src/rinbo_control/rinbo_control/runtime.py)。

**U06 — 校正快取／重試與錯誤說明**

- 舊版：沒有sensor_epoch與原生成功紀錄核對。
- 現況：本次操作台完成校正、sensor_epoch未變、原生紀錄有效才沿用；失敗不自動重跑。
- 影響：選A需重做整個狀態判斷，不是清除一個error旗標。
- 建議：B：保留紀錄核對；精簡流程可選C。
- 位置：[runtime.py](/home/jetson/rinbo_ros_ws/src/rinbo_control/rinbo_control/runtime.py)、[feedback.py](/home/jetson/rinbo_ros_ws/src/rinbo_control/rinbo_control/feedback.py)。


### Sim2Real／策略與轉接

**R01 — Sim2Real整體與所選profile**

- 舊版：GitHub没有這三套redrhex套件。
- 現況：有redrhex_msgs、lowlevel_bridge、rl_controller與多組profile；目前未觀察到運行中的策略程序。
- 影響：沒有可直接恢復的舊版RL數字；A代表停用新增路徑而非填0。
- 建議：B：保留原始碼；啟用profile需另確認。
- 位置：[lowlevel_bridge.yaml](/home/jetson/rinbo_ros_ws/src/redrhex_lowlevel_bridge/config/lowlevel_bridge.yaml)、[redrhex_policy.yaml](/home/jetson/rinbo_ros_ws/src/redrhex_rl_controller/config/redrhex_policy.yaml)。

**R02 — Sim2Real的腿位mask**

- 舊版：無RL設定。
- 現況：原生FSM=L3；site full_feedback兩份=[]；sensor_v2兩份=[L1]。
- 影響：这些檔案不自動等同原生site mask；必須按實際launch配置配對。沒有因此修改L3。
- 建議：C：在選定profile後對齊L3，不能現在盲改全部模板。
- 位置：[redrhex_policy_full_feedback_rig.yaml](/home/jetson/redrhex_site/redrhex_policy_full_feedback_rig.yaml)、[redrhex_policy_sensor_v2_suspended_experimental.yaml](/home/jetson/redrhex_site/redrhex_policy_sensor_v2_suspended_experimental.yaml)、[rinbo_leg_mask.py](/home/jetson/rinbo_ros_ws/src/redrhex_rl_controller/redrhex_rl_controller/rinbo_leg_mask.py)。

**R03 — RL速度到PWM轉換／限幅**

- 舊版：無。
- 現況：基礎rinbo adapter：40 PWM/(rad/s)、cap80、slew250/s。
- 影響：不是Tripod PD公式，也不是SBReal已量測的馬達模型；部分profile不同。
- 建議：C：確認實際profile及馬達關係後調整。
- 位置：[rinbo_ros_backend.py](/home/jetson/rinbo_ros_ws/src/redrhex_lowlevel_bridge/redrhex_lowlevel_bridge/rinbo_ros_backend.py)、[lowlevel_bridge.yaml](/home/jetson/rinbo_ros_ws/src/redrhex_lowlevel_bridge/config/lowlevel_bridge.yaml)。

**R04 — RL encoder／方向／伺服換算**

- 舊版：無RL對照。
- 現況：基礎54984.83 counts/rev、encoder左負右正；ABAD1000 counts/rad為待校準值。
- 影響：引用Tripod的counts常數不是硬體規格證據；方向還有命令sign hook。
- 建議：C：量測後設定，不猜比例與方向。
- 位置：[rinbo_ros_backend.py](/home/jetson/rinbo_ros_ws/src/redrhex_lowlevel_bridge/redrhex_lowlevel_bridge/rinbo_ros_backend.py)、[lowlevel_bridge.yaml](/home/jetson/rinbo_ros_ws/src/redrhex_lowlevel_bridge/config/lowlevel_bridge.yaml)。

**R05 — RL初始站姿到位**

- 舊版：無。
- 現況：base：2s參考/12s timeout/.12rad/.25rad/s/.5s；site兩份timeout30s。
- 影響：不是rinbo_standing；Control Panel選15的Sim2Real頁只改明確選中的profile。
- 建議：C：按選定profile評估，不改Cali/Standing。
- 位置：[sim_motion_limits.py](/home/jetson/rinbo_ros_ws/src/rinbo_control/rinbo_control/sim_motion_limits.py)、[redrhex_policy.yaml](/home/jetson/rinbo_ros_ws/src/redrhex_rl_controller/config/redrhex_policy.yaml)。

**R06 — RL主驅動與ABAD動作限幅**

- 舊版：無。
- 現況：base：主速度30rad/s、slew120rad/s²；ABAD角0.7rad、slew6rad/s；site rig通常12與1。
- 影響：不同profile數字差異很大，完整值另附CSV；不宣稱base目前生效。
- 建議：C：選定profile後統一操作需求。
- 位置：[redrhex_policy.yaml](/home/jetson/rinbo_ros_ws/src/redrhex_rl_controller/config/redrhex_policy.yaml)、[safety_filter.py](/home/jetson/rinbo_ros_ws/src/redrhex_rl_controller/redrhex_rl_controller/safety_filter.py)。

**R07 — RL資料、推論、姿態等保護**

- 舊版：無。
- 現況：base有姿態0.7rad、sensor0.10s、cmd0.25s、推論8ms／loop30ms等；profile有更嚴格契約。
- 影響：硬體觀測缺失、ONNX契約或mask不符不能由提高PWM修復。
- 建議：B：保留有效資料；閾值C需個別評估。
- 位置：[rl_controller_node.py](/home/jetson/rinbo_ros_ws/src/redrhex_rl_controller/redrhex_rl_controller/rl_controller_node.py)、[redrhex_policy.yaml](/home/jetson/rinbo_ros_ws/src/redrhex_rl_controller/config/redrhex_policy.yaml)。

**R08 — RL policy檔案／契約／啟動使能**

- 舊版：無。
- 現況：ONNX觀測/動作契約、hash與啟動輸出檢查；基礎enable_policy_on_start=false、enable_motor_output_on_start=false。
- 影響：模型／硬體校準未證實；本輪不載入模型或執行推論／動作。
- 建議：B：保留契約檢查與明確啟動。
- 位置：[policy_validation.py](/home/jetson/rinbo_ros_ws/src/redrhex_rl_controller/redrhex_rl_controller/policy_validation.py)、[redrhex_policy.yaml](/home/jetson/rinbo_ros_ws/src/redrhex_rl_controller/config/redrhex_policy.yaml)。

**R09 — RL供電與通訊門檻**

- 舊版：無。
- 現況：base adapter18～30V/3A/3筆；site rig18～42V/5A；state0.25s、power0.35s、cmd0.10s。
- 影響：再次說明profile與native FSM不同；不是全部平台已統一成同一數字。
- 建議：C：確認所選profile；保留供電／通訊功能。
- 位置：[lowlevel_bridge.yaml](/home/jetson/rinbo_ros_ws/src/redrhex_lowlevel_bridge/config/lowlevel_bridge.yaml)、[rinbo_ros_backend.py](/home/jetson/rinbo_ros_ws/src/redrhex_lowlevel_bridge/redrhex_lowlevel_bridge/rinbo_ros_backend.py)。


### 記錄、監控與訊息

**D01 — 資料記錄內容與CSV介面**

- 舊版：trigger後依pid/data寫單一CSV；actual右腿反號。
- 現況：summary.csv/events.csv/metadata；100Hz摘要；原始motor state、命令、power、debug、首因；auto_start預設true。
- 影響：欄名／座標不能直接拿舊分析程式套用；CSV摘要不是每筆控制回讀。
- 建議：B：保留完整記錄；需要舊格式可選C加相容匯出。
- 位置：[rinbo_data_recorder.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_data_recorder/src/rinbo_data_recorder.cpp)、[logging_tripod_safety.yaml](/home/jetson/rinbo_ros_ws/src/rinbo_data_recorder/config/logging_tripod_safety.yaml)。

**D02 — 浏览器監控／錄製**

- 舊版：無rinbo_monitor。
- 現況：唯讀狀態、原始/轉送PWM、錄製；port8088、顯示過期0.5s。
- 影響：顯示門檻不會改控制器保護；頁面無上電／馬達控制路由。
- 建議：B：保留獨立監看。
- 位置：[server.py](/home/jetson/rinbo_ros_ws/src/rinbo_monitor/rinbo_monitor/server.py)、[panel.html](/home/jetson/rinbo_ros_ws/src/rinbo_monitor/rinbo_monitor/panel.html)。

**D03 — ROS消息契約**

- 舊版：9個基本.msg。
- 現況：基本.msg內容相同（只有換行差）；新增ControllerDebugStamped、SafetyEventStamped。
- 影響：不是把原來motor/command欄位重編碼；新增型別需相容建置。
- 建議：B：保留新增診斷型別。
- 位置：[CMakeLists.txt](/home/jetson/rinbo_ros_ws/src/rinbo_msgs/CMakeLists.txt)、[ControllerDebugStamped.msg](/home/jetson/rinbo_ros_ws/src/rinbo_msgs/msg/ControllerDebugStamped.msg)、[SafetyEventStamped.msg](/home/jetson/rinbo_ros_ws/src/rinbo_msgs/msg/SafetyEventStamped.msg)。


### 舊版其他套件／建置

**O01 — 舊PyQt GUI與啟動名稱**

- 舊版：有rinbo_panel；Tripod按鈕呼叫rinbo_tripod_rslip，但同倉庫CMake只產生rinbo_tripod。
- 現況：本機沒有此package；Windows桌面App也不是這份原始碼。
- 影響：已確認舊GUI／CMake入口名稱不一致；整包照抄不能保證可用。
- 建議：C：若要恢復舊GUI，保留外觀但修正入口與新版握手。
- 位置：[rinbo_control_panel.py](/home/jetson/rinbo_ros_ws/docs/diagnostics/workspace_audit_20260909/upstream/src/rinbo_panel/scripts/rinbo_control_panel.py)、[CMakeLists.txt](/home/jetson/rinbo_ros_ws/docs/diagnostics/workspace_audit_20260909/upstream/src/rinbo_fsm/CMakeLists.txt)。

**O02 — 舊PID測試／單腿RSLIP**

- 舊版：有rinbo_pid_test：正弦5000 counts、0.2Hz、PWM500；另有rinbo_traj_rslip。
- 現況：本機沒有這兩支；另有rinbo_sin_sweep退役stub及Manual入口。
- 影響：原版測試直接發布motor命令；不同於新版逐腳控制，不適合未核對直接混用。
- 建議：C：如需測PID，做相容獨立測試入口。
- 位置：[rinbo_pid_test.cpp](/home/jetson/rinbo_ros_ws/docs/diagnostics/workspace_audit_20260909/upstream/src/rinbo_pid_test/src/rinbo_pid_test.cpp)、[rinbo_sin_sweep.cpp](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/rinbo_sin_sweep.cpp)。

**O03 — IMU子模組**

- 舊版：gitlink src/microstrain_inertial。
- 現況：本機src沒有此子模組。
- 影響：GitHub樹只有指向commit；沒有同倉庫.gitmodules，不能假裝已拿到完整IMU驅動內容。
- 建議：C：若需要IMU，先確認實際型號、來源與安裝位置。
- 位置：[inventory.json](/home/jetson/rinbo_ros_ws/docs/diagnostics/workspace_audit_20260909/inventory.json)。

**O04 — 建置與README**

- 舊版：ROS2 ament CMake；README混有Corgi/ROS1 catkin說明。
- 現況：新增yaml-cpp/OpenSSL/診斷消息、robot_config靜態庫與測試；README改為當前入口。
- 影響：不能用舊README判定整包應改回ROS1；恢復來源要同步相依套件與執行檔名稱。
- 建議：B：保留當前建置；只按所選控制項調整。
- 位置：[CMakeLists.txt](/home/jetson/rinbo_ros_ws/src/rinbo_fsm/CMakeLists.txt)、[CMakeLists.txt](/home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/CMakeLists.txt)、[README.md](/home/jetson/rinbo_ros_ws/README.md)。


### FPGA 與 Windows 邊界

**F01 — FPGA console生命週期候選**

- 舊版：本GitHub沒有FPGA driver原始碼，無法以此倉庫定義原版。
- 現況：tools/fpga_lifecycle是依sbRIO歷史正式源碼製作的候選；文件標示未部署。
- 影響：候選修正EOF/HUP/ERR/忙迴圈、背景/監看責任及off流程；不能當成目前sbRIO已運行版本。
- 建議：B：保留候選供另案部署決策，不自動替換驅動。
- 位置：[README.md](/home/jetson/rinbo_ros_ws/tools/fpga_lifecycle/README.md)、[console.cpp](/home/jetson/rinbo_ros_ws/tools/fpga_lifecycle/baseline/console.cpp)、[console.cpp](/home/jetson/rinbo_ros_ws/tools/fpga_lifecycle/src/console.cpp)。

**F02 — Windows期待值／停止入口**

- 舊版：本GitHub的PyQt GUI不是現用Windows程式；無Windows source。
- 現況：已有交接prompt；Bridge expected PWM需3300、site revision/hash動態讀取；Tripod stop≠all-off。
- 影響：無法在這台Orin核對Windows目前實作；不能說已替Windows修改完成。
- 建議：C：Windows提供當前程式後另比對；Orin先保留現況。
- 位置：[windows_bridge_pwm3300_prompt_20260909_zh_TW.md](/home/jetson/rinbo_ros_ws/docs/windows_bridge_pwm3300_prompt_20260909_zh_TW.md)、[tripod_restore_20260909_zh_TW.md](/home/jetson/rinbo_ros_ws/docs/tripod_restore_20260909_zh_TW.md)。


## 全部檔案差異與參數附件

| 共同／缺席檔案 | 結果 |
|---|---|
| `README.md` | 內容有差異 |
| `src/microstrain_inertial` | 子模組入口不存在 |
| `src/rinbo_data_recorder/CMakeLists.txt` | 內容有差異 |
| `src/rinbo_data_recorder/package.xml` | 內容有差異 |
| `src/rinbo_data_recorder/src/rinbo_data_recorder.cpp` | 內容有差異 |
| `src/rinbo_fsm/CMakeLists.txt` | 內容有差異 |
| `src/rinbo_fsm/package.xml` | 內容有差異 |
| `src/rinbo_fsm/src/rinbo_cali.cpp` | 內容有差異 |
| `src/rinbo_fsm/src/rinbo_standing.cpp` | 內容有差異 |
| `src/rinbo_fsm/src/rinbo_tripod.cpp` | 內容有差異 |
| `src/rinbo_msgs/CMakeLists.txt` | 內容有差異 |
| `src/rinbo_msgs/README.md` | 完全相同 |
| `src/rinbo_msgs/msg/Header.msg` | 僅CRLF/LF換行差異 |
| `src/rinbo_msgs/msg/LegCmd.msg` | 僅CRLF/LF換行差異 |
| `src/rinbo_msgs/msg/LegState.msg` | 僅CRLF/LF換行差異 |
| `src/rinbo_msgs/msg/MotorCmdStamped.msg` | 僅CRLF/LF換行差異 |
| `src/rinbo_msgs/msg/MotorStateStamped.msg` | 僅CRLF/LF換行差異 |
| `src/rinbo_msgs/msg/PowerCmdStamped.msg` | 僅CRLF/LF換行差異 |
| `src/rinbo_msgs/msg/PowerStateStamped.msg` | 僅CRLF/LF換行差異 |
| `src/rinbo_msgs/msg/ServoCmd.msg` | 僅CRLF/LF換行差異 |
| `src/rinbo_msgs/msg/ServoState.msg` | 僅CRLF/LF換行差異 |
| `src/rinbo_msgs/package.xml` | 完全相同 |
| `src/rinbo_panel/CMakeLists.txt` | 本機不存在 |
| `src/rinbo_panel/launch/rinbo_control_panel.launch` | 本機不存在 |
| `src/rinbo_panel/package.xml` | 本機不存在 |
| `src/rinbo_panel/scripts/rinbo_control_panel.py` | 本機不存在 |
| `src/rinbo_pid_test/CMakeLists.txt` | 本機不存在 |
| `src/rinbo_pid_test/package.xml` | 本機不存在 |
| `src/rinbo_pid_test/src/rinbo_pid_test.cpp` | 本機不存在 |
| `src/rinbo_pid_test/src/rinbo_traj_rslip.cpp` | 本機不存在 |
| `src/rinbo_ros_bridge/CMakeLists.txt` | 內容有差異 |
| `src/rinbo_ros_bridge/cmake/common.cmake` | 完全相同 |
| `src/rinbo_ros_bridge/launch/rinbo_ros_bridge.launch` | 完全相同 |
| `src/rinbo_ros_bridge/package.xml` | 內容有差異 |
| `src/rinbo_ros_bridge/scripts/rinbo_ros_bridge.sh` | 完全相同 |
| `src/rinbo_ros_bridge/src/CMakeLists.txt` | 完全相同 |
| `src/rinbo_ros_bridge/src/rinbo_ros_bridge.cpp` | 內容有差異 |

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
