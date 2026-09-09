# Tripod 控制與位置基準恢復紀錄（2026-09-09）

本次依使用者確認，恢復 KP=0.38、KD=0.003。KD 不是最初口述的 0.03；使用者確認採 GitHub 的 0.003。比對基準固定為 [ShuWei-Yang/rinbo_ros_ws，ddcbce9](https://github.com/ShuWei-Yang/rinbo_ros_ws/blob/ddcbce9fecdf039af17784385839eb55baeed2a3/src/rinbo_fsm/src/rinbo_tripod.cpp)。

**已於 21:42:43 +0800 部署並讀回確認，現場 revision 14。258 項離線檢查全部通過。** 設定 SHA256：`c657c377f47f4277a6b4f1b7e2abb08e1f68eca36c5541df5a28d8e6587dda90`。備份位於 `/home/jetson/.local/state/rinbo-control-backups/tripod-restore-pa9q_9em`。下次手動啟動載入新設定，本次未啟動真實動作。

## 採用設定與差異

| 項目 | 本次採用 | 說明 |
|---|---|---|
| KP / KD / K_FF | 0.38 / 0.003 / 0.005 | GitHub 控制係數；沒有新增積分項 |
| 摩擦補償 | 0 PWM | 恢復原版無額外摩擦補償；目前沒有實測支持固定加 40 PWM |
| 控制用速度濾波 | 0.005 秒 | 新增 Tripod 獨立參數 `velocity_filter_time_constant_s`；0 可還原原始差分 |
| 起步 | 8 秒 | 保留目前單圈、靜止到靜止的五次曲線與短暫平滑步態銜接；原版 4 秒三次曲線沒有一併恢復 |
| 起步後位置基準 | 各腿當下實際位置 | 恢復原版做法；不再以起步規劃終點為原點 |
| Group B 基準 | B 組啟動當下再次取實際位置 | 原版做法；僅 L1、R2 更新，L3 仍屏蔽；A 組不重設 |
| T-ratio | 8 → 1 | 到 1 後持續 RUNNING，等手動停止；1 是時間縮放比，不是位置保護門檻 |
| 加速 | `ratio_step=-0.0002`，依實際 dt 換算 | 名目每秒 -0.2，約 35 秒由 8 到 1；原版是每次 callback -0.0002，只有 1 kHz 時同速 |
| 最大 PWM | 3300 | 與 GitHub、Bridge 上限一致；上限不是每次固定輸出 3300 |
| 額外 PWM slew | 關閉 | 保持已完成的修正；不加入原先 250 PWM/s 限制 |
| L3 | disabled / supported_leg_test | 主驅動 enable=false、PWM=0；沒有解除腳位屏蔽 |
| 手動停止 | 沿用新版 | 2 秒減速、5 秒停止期限，保留停止命令與回報；不改成 Tripod 自行切斷整機電源 |
| 位置誤差 | 有限值警告，不自動停 | 沿用現場設定；9000/18000 counts 與記錄機制未變 |
| 供電、有效通訊、無效資料、急停 | 沿用新版 | 原有電壓／過流、來源／時效、arbiter ack、第一個停止原因均保留 |

主要出力公式（最後仍有 ±3300 限幅）：

```text
PWM = 0.38 × (target_position - actual_position)
    + 0.003 × (target_velocity - filtered_actual_velocity)
    + 0.005 × target_velocity
```

單位是 encoder counts、counts/s、帶正負號 PWM。Tripod 的左右符號、方向、54984.83 counts/rev、A={R1,L2,R3}、B={L1,R2,L3}、步態多項式及多圈位置都未改動。硬體 counts/rev 與物理方向仍缺規格／量測證據，不把上述數值當成已校準硬體規格。

## 為何採 5 ms 濾波

原始位置差分容易放大雜訊；低通濾波能抑制尖峰，也會引入延遲。這兩者是取捨，不能只因濾波比較平滑就判定比較好。參考 [MathWorks 的差分說明](https://www.mathworks.com/help/signal/ug/take-derivatives-of-a-signal.html)及[一階濾波時間常數說明](https://uk.mathworks.com/help/simscape/ug/filtering-input-signals-and-providing-time-derivatives.html)。

`tools/evaluate_tripod_velocity_filter.py` 擷取正式 C++ 步態函式，搭配正式 `ControlVelocity`，比較 0、5、20 ms。以理想跟隨軌跡、整數編碼器 counts、1 ms 控制週期，分別模擬每 1 ms 回傳及每 5 ms 分批回傳。**5 ms 封包情境是模擬假設，不是 sbRIO 實測規格；此評估沒有馬達模型，不能證明閉迴路穩定。**

下表將速度估計 RMS 誤差乘 KD=0.003，呈現它會造成多少 PWM 修正誤差：

| 情境 | 原始差分 | 5 ms | 20 ms |
|---|---:|---:|---:|
| ratio 5.9，1 ms 回傳 | 1.25 | 0.63 | 2.28 |
| ratio 5.9，5 ms 分批 | 159.68 | 20.43 | 6.05 |
| ratio 1，1 ms 回傳 | 2.32 | 21.74 | 72.75 |
| ratio 1，5 ms 分批 | 928.37 | 122.32 | 86.02 |

因此採 5 ms 作為初始折衷：比 20 ms 更少拖延快速步態修正，比原始差分更能抑制分批回傳尖峰。不做沒有實測依据的自動切換或任意調大 KD。實際每筆回饋時間、編碼器步進與負載量測後，才可能判定 0、5 或其他時間常數何者較合適。

Calibration／Standing 仍呼叫原本 20 ms 的預設濾波，控制係數、到位條件與起步程序不變。舊現場設定若缺少新欄位，讀取時仍使用 20 ms；本次套用會明確寫入 0.005，不以默默更換舊檔預設值的方式調整。

## 基準切換的確切意義

起步規劃是走一整圈，但結束時如果某腿尚差 100 counts，RUNNING 將從該腿**實際到達的位置**開始，不再追補這 100 counts。這會讓位置目標在切換點重設；沒有宣稱規劃位置絕對連續，也沒有修改 encoder 或套最短角度差。

`TRIPOD_REBASE` 每次列出階段、腿、實際位置、前目標、殘差、新基準、tau、ratio。STARTUP 的最後一筆 target/error 仍先經過原有檢查和完整診斷流程；即使重設基準也不覆蓋該筆誤差或第一個停止原因。B 組重設只在它首次啟動時做一次，之後不持續追著實際位置重設。

## 程式與操作入口

- 現場生效設定：`/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml`，`parameters.rinbo_tripod_rslip`。
- 控制載入、T-ratio、原點切換：`src/rinbo_fsm/src/rinbo_tripod.cpp`。
- PWM 公式：`src/rinbo_fsm/src/motion_effort.hpp`，`MotionEffort::command()`。
- 速度濾波：`src/rinbo_fsm/src/control_velocity.hpp`，`ControlVelocity::update()`。
- 設定預設／驗證／原子寫入：`src/rinbo_fsm/src/robot_config.cpp`。
- 起步、平滑銜接、正常停止參考：`src/rinbo_fsm/src/tripod_reference.hpp`，本次沒有更動。

查看生效參數（純讀取、不啟動 ROS 動作）：

```bash
/home/jetson/rinbo_ros_ws/build/rinbo_fsm/rinbo_tripod --check-config
/home/jetson/rinbo_ros_ws/build/rinbo_fsm/rinbo_legs status --json
```

下列是日後離線調整濾波的介面範例，**不是本次要求使用者另外執行的步驟**：

```bash
# 預覽原始差分設定，不寫入
/home/jetson/rinbo_ros_ws/build/rinbo_fsm/rinbo_legs tune-tripod --dry-run velocity_filter_time_constant_s=0
# 動作停止後才可套用；0.005 是本次採用的 5 ms
/home/jetson/rinbo_ros_ws/build/rinbo_fsm/rinbo_legs tune-tripod velocity_filter_time_constant_s=0.005
```

此入口只改 Tripod，不用會同時調整其他階段的 `tune-motion`。保留合法的 Calibration／Standing 紀錄綁定；不創造原本不存在、已失效或不同開機的成功紀錄。

## Windows 配合內容

啟動／停止按鈕與命令路徑不必更換。桌面 App 若有參數白名單、固定預期值或自行產生 override，請用以下內容交接：

```text
請同步 Orin Tripod 設定契約：kp=0.38、kd=0.003、k_ff=0.005、
friction_pwm=0、velocity_filter_time_constant_s=0.005、startup_duration=8、
start_ratio=8、target_ratio=1、ratio_step=-0.0002、max_pwm=3300。
保留 L3 disabled / supported_leg_test、safety.enable_pwm_slew_limit=false、
有限位置誤差 warn_only、新版停止與所有既有有效通訊／供電檢查。
讀取 Orin 當下 revision/hash，不要硬編碼旧 revision，也不要覆蓋上述參數。
Bridge 的 motor_command_max_pwm 預期值仍應為 3300，不能沿用 80。
ratio 到達 1 是持續運轉階段，不是「操作已完成」，沒有執行時間上限。
停止 Tripod 按鈕沿用既有 SIGINT 與停止回報核對，不等同全部關電。
如有顯示加速速度，顯示 T-ratio 每秒降低 0.2，不把 -0.0002 誤當每秒值。
啟動成功需記錄實際 PID/boot ID/設定 revision/hash；錯誤仍保留第一個原因。
```

Windows App 原始碼不在此工作區，本次沒有宣稱已修改或驗證 Windows 二進位。

## 離線驗證與部署

結果與雜湊以 `docs/diagnostics/tripod_restore_20260909/verification.json`、`deployment.json` 為準。保存修改前源碼／設定、差異、完整建置與測試輸出。未啟動真實 Tripod、FPGA、Bridge，未上電或發送動作。

最終通過 183 項原生 GTest、1 項退役入口檢查、74 項控制台 pytest，共 258 項。首次新增的加速測試因模擬時鐘截斷及以 callback 次數計算期望值而失敗；修正測試時間轉換、改對照實際模擬經過時間後，完整原生回歸再跑一次全部通過。未為此放寬測試容差或控制保護。

五個 FSM 執行檔已重新建置／連結，因它們共用設定讀取器，需要同時認得新的 Tripod 濾波欄位。Calibration／Standing 動作源碼與現場參數、正常停止曲線、Bridge 源執行檔／設定均另以修改前 SHA256 或語義比較確認未變。

新增測試涵蓋：實際原點重設、切換前誤差保存、B 組僅重設一次、8 秒起步、8→1 的 35 秒加速、到 1 持續運轉、3300 PWM、L3 屏蔽、正常停止、速度濾波／原始模式、非法參數，以及其他階段設定不變。既有方向、多圈、全部 64 種腿位組合、無效資料、通訊、供電／arbiter 與首因保存回歸一起執行。

## 尚待實機確認

本次沒有證明實機在 ratio 1 已穩定。後續由使用者確認現場條件後，手動啟動一次，觀察 8 秒起步與兩次 `TRIPOD_REBASE`，再記錄 ratio 6、5.9、4 至 1 的各腿 target/actual、原始／濾波速度、PWM、飽和時間與電流。若步態明顯失序、機械受阻或持續偏離，按既有停止；通訊／供電／無效資料故障仍走原有停止與錯誤回報。這些步驟本次未自動執行。
