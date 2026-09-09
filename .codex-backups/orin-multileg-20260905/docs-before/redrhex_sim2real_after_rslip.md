# R-Slip 一鍵啟動後：Tripod → Sim2Real policy

適用情境：已依 [《R-Slip 實驗完整操作流程 v4.0》](https://drive.google.com/file/d/16q2kQjzAQV3-ogL6IlfXm7PLFOJYXCLi/view?usp=sharing)完成一鍵啟動，**尚未執行舊版第 5 節的 raw `/power/command`**，並要在同一次實驗完成 Cali／Standing／Tripod Check，再接到 RedRHex policy。第一次準備模型、校正與封裝請先看[完整手冊](redrhex_sim2real_sbrio.md)；一般從零操作看[最短說明書](redrhex_sim2real_quickstart.md)。

目前的 `DIAGNOSTIC_ONLY.onnx` 不能接 policy。若要測 RL，先回[最簡說明書](redrhex_sim2real_quickstart.md#你現在先做什麼)取得並封裝新 bundle；若只測三個 FSM，做本頁第 0～3 節後直接到第 7 節關電。

## 先記住三件事

1. `rinbo_standing` 必須先 `Ctrl+C` 並完全消失，policy 才能接管；兩者不可共存。
2. R-Slip 已啟動的 `rinbo_ros_bridge` 是唯一 final bridge，要一直保留到最後斷電；接續時不可再啟動第二份。
3. L1 是左前腳，Main 與 SL1 必須維持實體隔離；本流程使用同一個 L1 mask。

## 0. R-Slip v4.0 的兩個相容性 gate

舊版第 5 節用固定 `seq: 1`、零 timestamp 發送 `/power/command`。新版 final bridge 會拒絕這種 `power=true`；上電必須改用 `rinbo_power_tool`，它會產生 fresh、單調的 seq/stamp，並在 3 秒內等待精確 `/power/state` acknowledgement。

Launcher 最好把 final bridge 命令更新成：

```bash
test -n "${CORE_MASTER_ADDR:-}" || { echo "ERROR: CORE_MASTER_ADDR 未設定"; exit 1; }
test -n "${CORE_LOCAL_IP:-}" || { echo "ERROR: CORE_LOCAL_IP 未設定"; exit 1; }
RSLIP_SBRIO_HOST="${CORE_MASTER_ADDR%:*}"
ros2 run rinbo_ros_bridge rinbo_ros_bridge --ros-args \
  --params-file "$(ros2 pkg prefix rinbo_ros_bridge)/share/rinbo_ros_bridge/config/redrhex_safe.yaml" \
  -p core_ip:="$RSLIP_SBRIO_HOST"
```

若 launcher 尚未帶 `redrhex_safe.yaml`，只接受**本 workspace 本次 rebuild 的新版 binary**：它會從 R-Slip 已設定的 `CORE_MASTER_ADDR` 解析 sbRIO IP，且 compiled defaults 與安全 YAML 相同。啟動後在 `R-Slip Orin1 Bridge` 核對：

```bash
tail -n 100 /tmp/rslip_ros_bridge.log | grep 'Motor arbiter:'
```

必須顯示正確 `CORE_IP`、`timeout=100ms`、`max_age=100ms`、`rearm=5`、`status=20ms`、`max_pwm=80.0`。再於 Orin2 核對：

```bash
ros2 param get /rinbo_ros2_bridge power_command_max_age_ms
ros2 param get /rinbo_ros2_bridge motor_shutdown_disabled_packets
```

必須分別是 `200` 與 `8`。舊 binary、數值不符或無法核對都是 **NO-GO**；先停止 R-Slip、更新 launcher／rebuild，再重新一鍵啟動。不可用「另開一份安全 bridge」補救。

## 1. Orin2 環境

在 `R-Slip Orin2 Manual` 執行：

```bash
export REDRHEX_WS=/home/jetson/rinbo_ros_ws
export REDRHEX_SITE_CTRL=/home/jetson/redrhex_site/redrhex_policy_full_feedback_rig.yaml
export REDRHEX_SITE_BRIDGE=/home/jetson/redrhex_site/lowlevel_bridge_full_feedback_rig.yaml
export REDRHEX_BAD_LEG=L1
export REDRHEX_SBRIO_IP="${REDRHEX_SBRIO_IP:-192.168.30.2}"
export REDRHEX_ORIN_WIRED_IP="${REDRHEX_ORIN_WIRED_IP:-192.168.30.8}"
export CORE_MASTER_ADDR="${CORE_MASTER_ADDR:-${REDRHEX_SBRIO_IP}:50051}"
export CORE_LOCAL_IP="${CORE_LOCAL_IP:-$REDRHEX_ORIN_WIRED_IP}"
export CORE_IP="${CORE_IP:-$REDRHEX_SBRIO_IP}"
export ROS_DOMAIN_ID=99
source /opt/ros/humble/setup.bash
source "$REDRHEX_WS/install/setup.bash"
```

若 R-Slip 現場使用不同 IP，這裡必須填與原 final bridge 完全相同的 sbRIO／Orin 有線 IP；不可只為了讓 checker 通過而沿用預設值。

要進入 policy 接管，site YAML、verified policy、真實 golden 與校正 ack 必須事先完成。現有 training `play.py` 尚未接上 golden recorder；Main 單腿安全 mapping CLI 也尚未完成。缺少真實 evidence 時，policy 是 **NO-GO**，不可填假路徑或直接把 ack 改成 `true`。

目前下載的 `policy_sensor_v2_run10_seed42_DIAGNOSTIC_ONLY.onnx` 明確是 `diagnostic_only_not_deployable`／`quality_rejected`，而且 I/O 不符合現有 runtime。它只能留在 `redrhex_models/incoming/`，不可寫入 site YAML。

新候選 `incoming/sensor_v2_drive_1mNVkQYh_20260901/policy.onnx` 雖然沒有 rejected 標記且附有 sidecar，仍是 `[1,60,36] + [1,3] → [1,12] + [1,3]`。現有 runtime 不支援，recorded golden 是 0，且 site calibration 仍未完成；因此它同樣不可寫入 active YAML，也不可進入本頁 policy 接管。

- 本次目標包含 policy：尚未取得並封裝 quality-approved bundle 時，停在本節，不要執行第 2 節上電。
- 本次只測 Cali／Standing／Tripod：可依第 2、3 節完成 FSM 測試，之後直接執行第 7 節斷電；不要執行第 4～6 節的 policy 接管。

本次目標包含 policy 時，在任何 R-Slip power command 前先做離線 policy gate：

```bash
(
  set -e
  test -f "$REDRHEX_SITE_CTRL" -a -f "$REDRHEX_SITE_BRIDGE"
  ros2 run redrhex_rl_controller preflight_check \
    --config "$REDRHEX_SITE_CTRL" \
    --bridge-config "$REDRHEX_SITE_BRIDGE" \
    --disabled-legs "$REDRHEX_BAD_LEG"
)
```

兩個命令都必須 exit `0`，且 Preflight 的 `checks[].ok` 全部是 `true`。失敗就回[最短操作說明書的模型準備步驟](redrhex_sim2real_quickstart.md#你現在先做什麼)，不要進行包含 policy 的上電流程。第 4 節在 policy 接管前仍會再跑一次 Preflight，防止期間設定或檔案改變。只測三個 FSM 時不要求 policy Preflight，但第 2、3 節的網路、power、L1 mask、健康腿與 E-stop 安全條件仍全部適用。

## 2. 用新版工具上電

不要再貼 R-Slip v4.0 原第 5 節的 raw `/power/command`。執行：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool sequence --include-relay \
  --confirm-relay --disabled-leg "$REDRHEX_BAD_LEG"
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status --disabled-leg "$REDRHEX_BAD_LEG"
```

必須看到 `digital=true, signal=true, power=true`，且每一步都有唯一 final bridge 的 fresh、單調 exact ack；relay 要連續 3 筆通過 18–30 V 與五隻健康腿 3 A gate。Nonzero exit、timeout 或 relay state `UNKNOWN` 都是 **NO-GO**。

## 3. 完成 R-Slip Tripod Check

一次只能跑一個 FSM；全部使用同一份 L1 設定：

```bash
export REDRHEX_FSM_CFG="$REDRHEX_WS/src/rinbo_fsm/config/l1_degraded_test.yaml"
ros2 run rinbo_fsm rinbo_cali --ros-args --params-file "$REDRHEX_FSM_CFG"
```

看到 Calibration `DONE` 後按 `Ctrl+C`，等提示字元返回，再確認：

```bash
pgrep -af 'rinbo_(cali|standing|tripod)'
timeout 3s ros2 topic echo /rinbo/motor_output_enabled --once
```

必須沒有 FSM，output 必須是 `false`。接著：

```bash
ros2 run rinbo_fsm rinbo_standing --ros-args --params-file "$REDRHEX_FSM_CFG"
```

Standing 穩定後按 `Ctrl+C`，重做上面的 FSM／output 檢查；通過後才跑：

```bash
ros2 run rinbo_fsm rinbo_tripod --ros-args --params-file "$REDRHEX_FSM_CFG"
```

完成短時間 Tripod Check 後按 `Ctrl+C`，必須等到 `Fully stopped`、提示字元返回，並再次確認 FSM 無輸出、motor output 為 `false`。任何 `SAFETY STOP` 都不要接 policy；保持支撐並直接到第 7 節斷電。

## 4. 準備 policy takeover

這一節是 **R-Slip 同次接續專用路徑**：FSM 完全退出後可暫時維持 relay 開啟，但期間必須持續沒有 FSM command publisher，且 motor output 保持 `false`。下方 handoff checker 或 Preflight 任一失敗就立即執行第 7 節關電；兩者都通過後才可啟動 policy stack。一般從零路徑仍必須以 relay 關閉完成只讀驗收，不可混用。

若 Tripod Check 後 encoder 參考位置可能改變，或現場 SOP 要求，先再跑一次 Cali：

```bash
ros2 run rinbo_fsm rinbo_cali --ros-args --params-file "$REDRHEX_FSM_CFG"
# 看到 DONE 後按 Ctrl+C，等 process 返回
pgrep -af 'rinbo_(cali|standing|tripod)'
timeout 3s ros2 topic echo /rinbo/motor_output_enabled --once
```

必須沒有 FSM，output 必須是 `false`。接著一定要再到 Standing：

```bash
ros2 run rinbo_fsm rinbo_standing --ros-args --params-file "$REDRHEX_FSM_CFG"
```

姿態穩定後按 `Ctrl+C`，等 process 返回，再執行：

```bash
pgrep -af 'rinbo_(cali|standing|tripod)'
timeout 3s ros2 topic echo /rinbo/motor_output_enabled --once
```

只有「沒有任何 FSM 輸出」且 motor output 是 `false` 才能準備接管。Standing 只是交接前姿態，不可留著 hold；機器人仍須由測試架支撐。

確認原 final bridge 只有一份，且不要重啟它：

```bash
FINAL_BRIDGE_BIN="$(ros2 pkg prefix rinbo_ros_bridge)/lib/rinbo_ros_bridge/rinbo_ros_bridge"
pgrep -af "^${FINAL_BRIDGE_BIN}( |$)"
```

`pgrep` 必須只有一行。

本 repo **沒有**這台實機的 IMU driver package／launch 命令。請在獨立 terminal 使用現場已驗證的實際命令啟動 `/imu/data`；若團隊尚未提供明確命令、publisher node 或 `frame_id`，本次就是 **NO-GO**，不可用 fake IMU。啟動後只用下列 read-only checker 作為 R-Slip → policy 的 authoritative handoff gate：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_bringup_check --strict \
  --max-motor-command-publishers 0 --require-power-relay-on \
  --require-motor-output-disabled --require-imu
```

它只訂閱、不發布命令；必須 exit `0`。它同時要求同一筆 fresh power feedback 的 `digital=true, signal=true, power=true`、`/motor/command` publisher 為 0、`/rinbo/motor_output_enabled` 唯一且 fresh false、IMU 唯一且 fresh。Nonzero exit 即 **NO-GO**，不要用零散 topic 檢查取代。

最後驗證 policy artifact：

```bash
ros2 run redrhex_rl_controller preflight_check \
  --config "$REDRHEX_SITE_CTRL" --bridge-config "$REDRHEX_SITE_BRIDGE" \
  --disabled-legs "$REDRHEX_BAD_LEG"
```

Preflight 的 `checks[].ok` 必須全部是 `true`。

## 5. 啟動 policy stack，不啟第二 final bridge

另開一個 Orin terminal，使用第 1 節相同環境後執行：

```bash
ros2 launch redrhex_rl_controller redrhex_policy_bringup.launch.py \
  safety_profile:=custom config:="$REDRHEX_SITE_CTRL" \
  bridge_config:="$REDRHEX_SITE_BRIDGE" disabled_legs:="$REDRHEX_BAD_LEG" \
  bridge_rinbo_allow_enable:=true start_bridge:=true use_fake_sensors:=false
```

這個 launch 的 `start_bridge:=true` 只啟動 `redrhex_lowlevel_bridge`（12 路轉換與安全層），**不會**啟動第二份 `rinbo_ros_bridge` final arbiter。

馬達仍未啟用時，先在 Orin2 完成 smoke gate：

```bash
(
  set -e
  for topic in /imu/data /joint_states /redrhex/motor_commands /motor/command; do
    ros2 topic info "$topic" -v
  done
)
```

先確認四個關鍵 topic 都只有預期的一個 publisher，再讀取每個必要 topic；前一個 timeout 時整段會立即停止：

```bash
(
  set -e
  for topic in \
    /joint_states \
    /redrhex/observation \
    /redrhex/observation_imputation_mask \
    /redrhex/lowlevel_disabled_legs \
    /redrhex/rinbo_motor_command_preview \
    /rinbo/motor_output_enabled \
    /redrhex/lowlevel_output_enabled \
    /redrhex/lowlevel_diagnostics \
    /redrhex/diagnostics; do
    timeout 3s ros2 topic echo "$topic" --once
  done
)
```

必須確認：四個關鍵 topic 都只有預期的一個 publisher；`/joint_states` 有 12 joints；observation `0:3=[0,0,0]`、`6:9≈[0,-1,0]`、`39:42=[0,0,0]`；mask 正確；disabled leg 是 L1；preview 全腿 disabled，且 low-level diagnostics 的 L1 preview PWM 為 0；兩層 output 都是 `false`；diagnostics 沒有 ERROR、stale 或 deadline miss。任一項不通過都不可執行第 6 節。

## 6. 交給 policy

在 Orin2 依序執行：

```bash
ros2 topic echo /redrhex/state_machine_state
# 看到 INIT_STAND 後按 Ctrl+C
```

只有看到 `INIT_STAND` 且沒有安全錯誤，才單獨執行：

```bash
ros2 topic pub --once /redrhex/enable_motors std_msgs/msg/Bool "{data: true}"
```

接著只觀察 state：

```bash
ros2 topic echo /redrhex/state_machine_state
# 看到 POLICY_READY 後按 Ctrl+C
```

只有看到 `POLICY_READY`，才查參數：

```bash
ros2 param get /redrhex_rl_controller commands.profile
ros2 param get /redrhex_rl_controller commands.fixed_forward_vx
```

只有參數分別是 `fixed_forward` 與 `0.22`，而且 diagnostics 沒有 ERROR，才單獨執行：

```bash
ros2 topic pub --once /redrhex/enable_policy std_msgs/msg/Bool "{data: true}"
```

不需要鍵盤或 `/cmd_vel`。第一次測試 3 秒後會自動退出 policy：

```bash
(
  set -e
  sleep 4
  timeout 3s ros2 topic echo /redrhex/state_machine_state --once
  timeout 3s ros2 topic echo /rinbo/motor_output_enabled --once
  timeout 3s ros2 topic echo /redrhex/lowlevel_output_enabled --once
  timeout 3s ros2 topic echo /redrhex/diagnostics --once
)
```

必須回到 `INIT_STAND`，兩層 output 都是 `false`，且 diagnostics 沒有 deadline miss。否則立即使用實體 E-stop／主電源切斷。

## 7. 停止

Policy launch 還在時先執行：

```bash
ros2 topic pub --once /redrhex/enable_policy std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /redrhex/enable_motors std_msgs/msg/Bool "{data: false}"
timeout 3s ros2 topic echo /rinbo/motor_output_enabled --once
timeout 3s ros2 topic echo /redrhex/lowlevel_output_enabled --once
ros2 run redrhex_lowlevel_bridge rinbo_power_tool off
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status --disabled-leg "$REDRHEX_BAD_LEG"
```

若 handoff checker／Preflight 在 policy launch 前失敗，跳過前四個 RL topic 命令，直接執行 `rinbo_power_tool off` 與 `status`。

若 policy stack 已啟動，兩層 output 必須先是 `false`；無論是否啟動過 policy，power status 都必須變成三個 `false`。然後在已存在的 policy launch 按 `Ctrl+C`、停止 IMU driver；最後回 Windows 執行 `停止 R-Slip.cmd`，讓原 final bridge 與 sbRIO driver 最後退出。

若曾 assert E-stop，final bridge 的 E-stop 是 process-lifetime sticky；同一個 process 收到 `false` 也不會解除。先實體急停／斷電、盡可能執行 `rinbo_power_tool off`，排除危害並停止 policy，再用 `停止 R-Slip.cmd` 結束原 final bridge。之後重新一鍵啟動，只保留新的一份 final bridge，並從網路、Preflight、只讀驗收與 disabled rearm 重做；不可對原 process 只執行 `estop_tool clear --confirm-clear` 後繼續。
