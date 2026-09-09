# 重要參數基準：請勿隨一般程式修改變動

**重要資料。這是使用者於2026-09-09選定並要求保護的標準設定。後續修程式、改介面、重建或部署時，不得順便調整這些基礎參數。**

基準名稱：`standard_20260909_r15`。原始現場revision **15**，SHA256 `5866b8ee7bb6a7733c0757fb0d7dd35995a1f78e3a5ec4defe6507c130c13129`。這表示已確認的設定與離線驗證版本，不表示已取得硬體規格或量得最佳PID。

## 先看主要數值

| 模式 | KP | KD | K_FF | 摩擦PWM | 速度濾波 | PWM上限 |
|---|---:|---:|---:|---:|---:|---:|
| Calibration | 0.35 | 0.002 | 0.02 | 0 | 0.005秒 | 500 |
| Standing 尋零／到位 | 0.35 | 0.002 | 0.02 | 0 | 0.005秒 | 500 |
| Standing 保持 | min(KP,0.1)=0.1 | 0.002 | 無速度前饋 | 0 | 0.005秒 | min(階段上限,300)=300 |
| Manual 逐腳 | 0.35 | 0.002 | 0.02 | 0 | 0.005秒 | 現場500，再取與計畫上限的較小值 |
| Tripod | 0.38 | 0.003 | 0.005 | 0 | 0.005秒 | 3300，額外slew關閉 |

- 原生L3屏蔽，`supported_leg_test`。不因恢復其他舊版數字而解除L3。
- Calibration：servo定位60秒、Hall搜尋60秒、停穩／reset等待各15秒；低速<500counts/s持續0.3秒，再確認歸零|raw counts|≤100。伺服目標依L1,L2,L3,R1,R2,R3順序為 `[740,2565,3283,1944,2071,989]`，容差100。
- Standing：半圈參考180°、最高36°/s、平滑加減速，約5.5秒；位置誤差<1000counts（約6.51°）即可完成。`settle_time_s=0`表示不要求低速與連續停穩；速度欄位500仍保存但此模式不使用。尋Hall／轉半圈各60秒；保持失位>12000counts（約78.13°）停止。
- Manual：位置／速度／相位／相對動作、原有平滑對齊；對齊與收尾誤差≤2°、速度≤5°/s；追蹤誤差>12000counts停止，訊息必須使用真實門檻；PWM slew為250/s。計畫時間1～60秒、速度／加速度1～90，預設計畫PWM80。
- Tripod：8秒起步、起始ratio8、目標1、ratio_step=-0.0002（按dt/0.001換算，名目每秒減0.2）；起步後和Group B啟動各以實際位置重設基準；8→1約35秒，到1後持續RUNNING直到停止。A=R1,L2,R3；B=L1,R2,L3，保留L3屏蔽與多圈位置。
- Tripod正常停止：2秒減速、5秒期限；`slowdown_step=0.002`是保留的舊欄位，不能將它誤當成新版減速的實際計時方法。
- Tripod有限位置誤差只警告；軟9000counts、10筆且0.5秒、硬18000counts在啟用stop模式時才造成停止。無效回饋仍停止。slew欄位250/s仍存檔但開關false，並未限制Tripod輸出。
- Calibration／Standing／Manual使用55296counts/rev；Tripod使用54984.83。Cali/Standing正向的原始counts左減右增；Tripod保留其相反慣例與配套命令方向。不要自行統一；硬體比例仍缺型號、齒比、一圈量測證据。
- 原生供電18～42V，超界5筆；腿電流5A、25筆；bus30A欄位存在，但bus過流停止開關false。馬達失聯0.25秒、電源0.5秒，初始資料2秒；來源年齡與arbiter/ack等完整值見下方。

## 唯一來源與容易混淆的地方

現場實際設定是 `/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml`，位於ROS工作區外。`.cpp`內的`declare_parameter(...,80.0)`是預設值；正式入口會注入現場YAML，不能只看那個80便認定還沒有調整。

`parameters.rinbo_manual`是獨立控制區塊；供電／通訊仍採Standing契約。新版基準會明確保存所有有效預設值，避免以後程式預設改變而悄悄連動。`hardware.*`由頂層mask產生，不重複寫入各區塊。`effective.json`另保存包含這些派生值的完整讀回結果。

日常動作計畫不是永久控制參數。當時儲存的是L2速度90°/s、加速度10°/s²、10秒、PWM80；現場上限500不會自動把該計畫改成500。此計畫另存`manual-plan.json`，一般check/restore不檢查或還原它；只有加`--include-plan`才包含它。

## 檢查與快速恢復（SSH／VS Code終端可用）

先進入工作區：

```bash
cd /home/jetson/rinbo_ros_ws
```

只檢查，不改設定、不要求停止正在看的監看頁：

```bash
python3 tools/parameter_baseline.py check
```

只預覽恢復差異：

```bash
python3 tools/parameter_baseline.py restore
```

明確恢復原生基礎設定：

```bash
python3 tools/parameter_baseline.py restore --apply
```

如果連儲存的日常動作計畫也要回到當時那份，先預覽再明確套用：

```bash
python3 tools/parameter_baseline.py restore --include-plan
python3 tools/parameter_baseline.py restore --include-plan --apply
```

套用時會取得動作／設定鎖、使用原生唯讀程序檢查；若有動作占用，會停止這次恢復而不是關掉它。工具自動備份到 `/home/jetson/.local/state/rinbo-control-backups/parameter-baseline-*`，原子寫入新設定，再用已部署原生工具讀回驗證。**不會上電、啟動動作、關掉SSH、重啟Bridge或FPGA。**

若原生設定已符合基準，不會重寫、增加revision或刪完成紀錄。若有變更，使用比目前更大的revision，並使舊校正／站立紀錄失效，不會把revision退回15。格式不同或revision/hash改變本身不代表參數漂移。若原生讀回不符，會恢復寫入前的檔案，但不復活已失效的成功紀錄。

若連revision都損壞無法辨識，工具會要求明確指定新編號。例如最近有效編號為20，就使用`restore --apply --revision 21`；不要未核對就照抄這個範例編號。現場檔案完全遺失時，先由維護者從副本重建並核對revision，本工具不猜最後編號。

輸出碼：0表示本次檢查符合／明確恢復成功；2表示唯讀檢查發現參數或來源差異需核對；1表示無法完成。來源／執行檔變動不等於一定改了參數，但必須審查。恢復命令只處理設定，若來源還有差異會列出，不會將整支程式回滾。

## 以後修改程式的規則

工作區根目錄的[AGENTS.md](../AGENTS.md)會要求後續助理先讀本文件，開始與完成工作時執行check。沒有你明確授權，就不能改重要值或透過共用程式間接改變行為。已存在的差異要先記錄，不能擅自重設你的調機設定。

只有你明確說「更新永久標準」，才建立新的有版本基準；舊版、修改原因、離線結果都保留。不提供會把當下設定直接覆蓋成標準的capture命令，也不可改雜湊來掩蓋差異。

這些規則與檢查提供持續可核對的保護，不會鎖住你的編輯器，也不能保證其他未遵守AGENTS.md的程式永遠不會改檔。若意外修改，可依上方流程比對／恢復。

## 副本與程式固定值

[基準目錄](../config/parameter_baselines/standard_20260909_r15/manifest.json)包含原始YAML、完整有效值、明寫預設值的標準YAML、可選動作計畫、20份重要來源快照、4份僅供參考的RL profile。工具先驗證固定manifest及副本SHA256；基準副本被意外修改時拒絕使用。

- [完整標準YAML](../config/parameter_baselines/standard_20260909_r15/native.yaml)
- [完整有效參數JSON](../config/parameter_baselines/standard_20260909_r15/effective.json)
- [原始revision15檔案](../config/parameter_baselines/standard_20260909_r15/site-original.yaml)
- [92項選擇與理由](restore_choices_20260909_zh_TW.md)

程式固定值需要來源快照審查與重建，不由restore覆寫整支程式，避免撤銷後續修復。主要位置：

| 內容 | 程式位置 |
|---|---|
| 原生500／Tripod3300與共用硬界 | `src/rinbo_fsm/src/safety_invariants.hpp` |
| 參數載入、驗證、Manual隔離、成功紀錄 | `src/rinbo_fsm/src/robot_config.cpp` |
| 摩擦／PD／前饋公式 | `src/rinbo_fsm/src/motion_effort.hpp` |
| 控制速度濾波公式 | `src/rinbo_fsm/src/control_velocity.hpp` |
| 尋零0.5秒、半圈平滑參考、反向500counts | `src/rinbo_fsm/src/motor_tracking.hpp` |
| 伺服目標、Hall停穩與歸零 | `src/rinbo_fsm/src/rinbo_cali.cpp` |
| Standing到位及保持300上限 | `src/rinbo_fsm/src/rinbo_standing.cpp` |
| Tripod參考、跨圈、分組與停止 | `src/rinbo_fsm/src/rinbo_tripod.cpp`、`tripod_reference.hpp` |
| Manual最終PWM邊界、方向、對齊 | `src/rinbo_fsm/src/manual_motion.hpp`、`rinbo_manual.cpp` |
| Bridge與來源／通訊保護 | `src/rinbo_ros_bridge/src/rinbo_ros_bridge.cpp`及FSM guards |
| 操作台計畫與可調範圍 | `src/rinbo_control/rinbo_control/plans.py`、`motion_limits.py` |

共用硬界原文（名稱是程式識別字；實際現場值可較低，完整現場值見YAML）：

```cpp
inline constexpr double kHardMinBusVoltageV = 18.0;
inline constexpr double kHardMaxBusVoltageV = 42.0;
inline constexpr double kHardMaxLegCurrentA = 10.0;
inline constexpr double kHardMaxBusCurrentA = 30.0;
inline constexpr double kHardMaxPowerStaleS = 0.5;
inline constexpr double kHardMaxPowerRequiredAfterS = 2.0;
inline constexpr int kHardMaxVoltageTripSamples = 5;
inline constexpr int kHardMaxCurrentTripSamples = 100;
inline constexpr int kPowerBusVoltageChannel = 7;
inline constexpr std::array<int, 6> kLegCurrentChannels = {1, 2, 3, 4, 5, 6};
inline constexpr double kHardMaxMotorStateStaleS = 0.25;
inline constexpr double kHardMaxMotorStateRequiredAfterS = 2.0;
inline constexpr double kHardMaxMotorSourceAgeS = 0.10;
inline constexpr double kHardMaxPowerSourceAgeS = 0.35;
inline constexpr double kHardMaxPwm = 500.0;
inline constexpr double kHardMaxTripodPwm = 3300.0;
inline constexpr double kHardMaxMotorArbiterReadyTimeoutS = 5.0;
inline constexpr double kHardMaxMotorArbiterHeartbeatStaleS = 0.25;
inline constexpr double kHardMaxMotorCommandAckTimeoutS = 0.25;
inline constexpr double kHardMaxPositionErrorCounts = 12000.0;
inline constexpr double kHardMaxPwmSlewRatePerS = 250.0;
inline constexpr int kHardMaxPositionErrorTripSamples = 10;
```

Bridge的固定設定副本如下。**core_ip=.2是檔案預設；當時啟動參數覆寫為sbRIO192.168.30.254，Jetson192.168.30.8、port50051、ROS_DOMAIN_ID99。** 此工具不改Bridge檔案／執行參數，也不重啟服務；若Bridge項目漂移，用來源快照和本表單獨修正並驗證。Windows也應讀取有效參數，不把Cali/Standing80或舊revision硬寫在探測器。

```yaml
rinbo_ros2_bridge:
  ros__parameters:
    core_ip: "192.168.30.2"
    motor_command_timeout_ms: 100
    motor_command_max_age_ms: 100
    motor_command_rearm_disabled_samples: 5
    motor_disable_resend_period_ms: 20
    motor_arbiter_heartbeat_period_ms: 50
    motor_output_status_period_ms: 20
    motor_shutdown_disabled_packets: 8
    power_shutdown_off_packets: 3
    motor_command_max_pwm: 3300.0
    motor_command_min_servo_encoder: 0
    motor_command_max_servo_encoder: 65535
    power_command_max_age_ms: 200
```

RL/Sim2Real的4份profile僅為參考快照，沒有被批准為硬體標準，不會自動restore。它們可能屏蔽L1或沒有屏蔽，不能直接當成原生L3設定。齒比、encoder比例、ABAD1000counts/rad、模型契約與IMU型號未確認的事項仍需現場資料，不可因存在快照就認定已校準。

## 完整原生標準設定

下列內容來自固定副本，涵蓋所有可設定的有效值（包含原始YAML未寫出的預設值）。恢復時只有revision換成遞增編號，其餘依此標準；Manual的共用保護由Standing區塊衍生。

```yaml
# IMPORTANT: fixed standard, not the live site file. All effective defaults pinned.
schema_version: 1
revision: 15
disabled_legs:
- L3
test_mode: supported_leg_test
parameters:
  rinbo_cali:
    friction_pwm: 0
    friction_velocity_counts_s: 153.6
    k_ff: 0.02
    kd: 0.002
    kp: 0.35
    max_pwm: 500
    safety:
      current_trip_samples: 25
      hall_search_timeout_s: 60
      leg_current_channels:
      - 1
      - 2
      - 3
      - 4
      - 5
      - 6
      max_bus_current: 30
      max_bus_voltage: 42
      max_current: 5
      min_bus_voltage: 18
      motor_arbiter_heartbeat_stale_s: 0.25
      motor_arbiter_node_name: rinbo_ros2_bridge
      motor_arbiter_ready_timeout_s: 5
      motor_command_ack_timeout_s: 0.25
      motor_state_required_after_s: 2
      motor_state_source_max_age_s: 0.1
      motor_state_timeout_s: 0.25
      power_bus_voltage_channel: 7
      power_guard_enabled: true
      power_required_after_seconds: 2
      power_stale_seconds: 0.5
      power_state_source_max_age_s: 0.35
      require_power_relay: true
      servo_homing_timeout_s: 60
      stop_on_bus_current_limit: false
      stop_on_current_limit: true
      stop_on_power_stale: true
      stop_on_voltage: true
      stop_timeout_s: 15
      voltage_trip_samples: 5
    velocity_filter_time_constant_s: 0.005
  rinbo_manual:
    friction_pwm: 0
    friction_velocity_counts_s: 153.6
    k_ff: 0.02
    kd: 0.002
    kp: 0.35
    max_pwm: 500
    velocity_filter_time_constant_s: 0.005
  rinbo_standing:
    friction_pwm: 0
    friction_velocity_counts_s: 153.6
    k_ff: 0.02
    kd: 0.002
    kp: 0.35
    max_pwm: 500
    safety:
      current_trip_samples: 25
      hall_search_timeout_s: 60
      hold_error_counts: 12000
      leg_current_channels:
      - 1
      - 2
      - 3
      - 4
      - 5
      - 6
      max_bus_current: 30
      max_bus_voltage: 42
      max_current: 5
      min_bus_voltage: 18
      motor_arbiter_heartbeat_stale_s: 0.25
      motor_arbiter_node_name: rinbo_ros2_bridge
      motor_arbiter_ready_timeout_s: 5
      motor_command_ack_timeout_s: 0.25
      motor_state_required_after_s: 2
      motor_state_source_max_age_s: 0.1
      motor_state_timeout_s: 0.25
      position_tolerance_counts: 1000
      power_bus_voltage_channel: 7
      power_guard_enabled: true
      power_required_after_seconds: 2
      power_stale_seconds: 0.5
      power_state_source_max_age_s: 0.35
      require_power_relay: true
      rotate_timeout_s: 60
      settle_time_s: 0
      settle_velocity_counts_s: 500
      stop_on_bus_current_limit: false
      stop_on_current_limit: true
      stop_on_power_stale: true
      stop_on_voltage: true
      voltage_trip_samples: 5
    velocity_filter_time_constant_s: 0.005
  rinbo_tripod_rslip:
    friction_pwm: 0
    friction_velocity_counts_s: 153.6
    k_ff: 0.005
    kd: 0.003
    kp: 0.38
    max_pwm: 3300
    ratio_step: -0.0002
    safety:
      current_trip_samples: 25
      enable_pwm_slew_limit: false
      enabled: true
      leg_current_channels:
      - 1
      - 2
      - 3
      - 4
      - 5
      - 6
      max_bus_current: 30
      max_bus_voltage: 42
      max_current: 5
      max_position_error_counts: 9000
      min_bus_voltage: 18
      motor_arbiter_heartbeat_stale_s: 0.25
      motor_arbiter_node_name: rinbo_ros2_bridge
      motor_arbiter_ready_timeout_s: 5
      motor_command_ack_timeout_s: 0.25
      motor_state_required_after_seconds: 2
      motor_state_source_max_age_s: 0.1
      motor_state_stale_seconds: 0.25
      position_error_trip_samples: 10
      position_error_trip_seconds: 0.5
      power_bus_voltage_channel: 7
      power_required_after_seconds: 2
      power_stale_seconds: 0.5
      power_state_source_max_age_s: 0.35
      pwm_slew_rate_per_sec: 250
      require_power_relay: true
      stop_on_bus_current_limit: false
      stop_on_current_limit: true
      stop_on_over_voltage: true
      stop_on_position_error: false
      stop_on_power_stale: true
      stop_on_voltage_sag: true
      voltage_trip_samples: 5
    shutdown:
      slowdown_duration_s: 2
      timeout_s: 5
    slowdown_step: 0.002
    start_ratio: 8
    startup_duration: 8
    stop_servo_control_mode: 0
    target_ratio: 1
    velocity_filter_time_constant_s: 0.005
```

## 可選的日常動作計畫副本

只在明確加`--include-plan`時使用，並保留其他偏好／連線資料。此檔不是上電或執行指令。

```json
{
  "duration_s": 10.0,
  "max_pwm": 80.0,
  "max_speed_deg_s": 90.0,
  "acceleration_deg_s2": 10.0,
  "legs": {
    "L2": {
      "mode": "velocity",
      "speed_deg_s": 90.0
    }
  }
}
```

## 建立時的驗證

10項檔案層離線測試通過，另以正式C++設定驗證器對暫存設定完成一次誤改→恢復→讀回比對（revision19→20，Standing PWM123→500）。本機真實設定、原生執行檔與控制器來源均未改動，沒有執行真實恢復或動作。完整紀錄見[驗證結果](diagnostics/parameter_baseline_20260909/verification.json)。

若check列出程式來源差異，可在VS Code將基準目錄`sources/`內對應的快照與現行檔案比較，只修回意外改動的數值；不要整支覆蓋，以免撤銷後續修復。
