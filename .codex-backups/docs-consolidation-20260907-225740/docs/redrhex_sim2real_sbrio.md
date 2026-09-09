> 2026-09-05 R-Slip 更新：Calibration／Standing／Tripod 與 power tool 的屏蔽操作請改用 [Orin 多腿操作卡](orin_multileg_operation.md)。本頁舊 R-Slip `--params-file`、`--disabled-leg`、腿名環境變數與單腿限制已被取代；ONNX policy 的獨立 contract 仍照原規則。

# RedRHex Sim2Real 完整操作手冊

適用情境：機器人固定在測試架，使用 6 個 Main Drive encoder、6 個 ABAD encoder 與 IMU；L1（左前腳）已斷電且不能動。本流程只驗證資料、方向、安全保護與 policy 執行，不代表已可落地行走。

如果你只是收到一顆新的 `.onnx`，不要從頭讀本頁；先照[最簡操作說明書](redrhex_sim2real_quickstart.md)把 bundle 放進固定資料夾並完成第一個 gate。本頁只用來查校正、完整封裝命令與排錯細節。

## 先選正確入口

| 目前狀態 | 要看的文件 |
|---|---|
| 已完成 R-Slip 一鍵啟動，尚未執行舊版 raw power command | [R-Slip 一鍵啟動後：Tripod → Policy](redrhex_sim2real_after_rslip.md) |
| 平常重測已封裝的同一顆 policy | [最短操作說明書](redrhex_sim2real_quickstart.md) |
| 查看、切換或清除故障腿 mask | [`rinbo_leg_mask` 单腿屏蔽操作手册](redrhex_disabled_leg_operation.md) |
| 第一次校正、換 policy、換故障腿或排錯 | 本手冊 |

R-Slip 接續版會自己完成新版上電、Cali、Standing、Tripod 與 policy 接管。R-Slip launcher 已有一份 final bridge；走該路徑時不可再啟動第二份 `rinbo_ros_bridge`，也不可回頭執行一般路徑的 sensor-only 上電步驟。

## 開始前先確認

第一次做第 3 步的**受限校正**前，只要求：

- [ ] sbRIO `192.168.30.254:50051` 可連線（sbRIO 端的 `grpccore` 與 `fpga_driver` 已啟動）。
- [ ] `/home/jetson/redrhex_site` 內已有兩份 site YAML，兩邊都宣告 L1 disabled。
- [ ] 機器人已固定，L1/SL1 已實體隔離，限流電源與 E-stop 可用。
- [ ] 使用第 3 步規定的低功率、逐腿、3 A guard 校正方式。

進入第 4 步封裝前，還必須完成：

- [ ] 12 路 encoder mapping 與 IMU mounting 已用真機量測，五個 calibration gate 都是 `true`。
- [ ] 已取得同 checkpoint 的 ONNX、TorchScript、golden NPZ 與 training source 完整 bundle。

進入第 5 步前，還必須完成：

- [ ] 第 4 步 package 成功，site YAML 已寫入新的 verified path 與 SHA。

進入第 6 步前，還必須完成：

- [ ] `preflight_check` 的所有 `checks[].ok` 都是 `true`。

進入第 7 步前，還必須完成：

- [ ] 只讀 launch 已通過，而且兩層 output 都是 `false`。

缺少 mapping 或 verified policy 時，只能停在受限校正／離線準備階段，不能啟動 RL policy。

目前 workspace 中 SHA256 為
`972cbaaefaa7213f38aeff5ea88e20a18b028bda1d025cb84d5e2b75e312b9bf`
的舊 `policy.onnx` 雖然是 `[1,56] -> [1,12]`，但缺少 contract metadata，reference action 最大值約為 `457.469`。**禁止上機，也不可藉由放寬 action、PWM 或電流上限使用。**

2026-09-01 從 Drive 下載的
`/home/jetson/redrhex_models/incoming/policy_sensor_v2_run10_seed42_DIAGNOSTIC_ONLY.onnx`
（SHA256 `d89d30cc0faa1a3e0bee20e680f88fa38f68251ea2c8e33fc63a5bd87b40e3bd`）也禁止上機。它的 metadata 是 `diagnostic_only_not_deployable`／`quality_rejected`，I/O 是兩個輸入 `[1,60,36] + [1,3]` 與兩個輸出 `[1,12] + [1,3]`，不符合目前 runtime contract。不要把它改名、裁掉第二個 output 或寫入 active YAML；詳細下一步見[最短操作說明書的「你現在先做什麼」](redrhex_sim2real_quickstart.md#你現在先做什麼)。

同日取得的新候選位於 `/home/jetson/redrhex_models/incoming/sensor_v2_drive_1mNVkQYh_20260901/`，ONNX SHA256 是 `b1754a92cb2ea37623a793d127f56d3247f16ae295185854a8c8565081ff00f9`。它的 ONNX 結構、sidecar contract hashes 與 CPU smoke inference 通過，且沒有 rejected 標記；但仍是 sensor-v2 的 2-input／2-output contract，現有 runtime gate 失敗。Sidecar 只有 4 個 random parity samples、0 個 recorded samples，並缺少 sensor-v2 runtime、同 checkpoint TorchScript、真實 golden、training source、完整 filter/reset/L1-disabled 規則；其 `hardware_ready=true` 也與 site calibration gates 全為 `false` 相衝突。它仍是 **NO-GO**，active YAML 不可更新。

## 固定安全規則

1. L1 是**左前腳**；右後腳是 R3。測試前仍須依實機標籤確認。
2. L1 Main 與 SL1 必須保持實體隔離。Servo control mode 是全域值，軟體 mask 不能取代實體斷電。
3. 硬體模式一次最多停用一腳；controller、bridge、FSM、artifact 與 launch 必須使用同一腿名。
4. 同一時間只能有一個 `/motor/command` publisher，以及一個 final bridge。
5. 真機一律使用 `use_fake_sensors:=false`；E-stop／主電源切斷永遠優先於軟體命令。
6. L1 mask 只豁免 L1 的已知錯誤。其餘五腳、bus voltage、stale telemetry、relay 與 final-arbiter 保護仍然有效。

三個 FSM 的 `/motor/state` 與 `/power/state` 都使用 reliable、volatile 的
`KeepLast(1)` latest-value QoS；callback 入口時間用於 source-age 與 arrival
watchdog。這只避免啟動時回放舊佇列，不會放寬 motor 的 `0.10 s` source-age、
power stale、publisher identity/GID、sequence 或 timestamp 單調性保護。

建議起始限制如下。硬體額定值若更低，必須再降低；不要為了避開 trip 而直接放寬。

| 保護 | 起始值 |
|---|---:|
| 每隻健康腿電流 | 3 A |
| bus voltage（ch7） | 18–30 V |
| Main Drive PWM | 80 |
| Main target velocity | 1.0 rad/s |
| Main measured velocity | 2.0 rad/s |
| ABAD target position | 0.18 rad |
| hardware action clip | 0.35 |
| raw action hard stop | 1.5 |
| CPU inference p99 | 6 ms |
| runtime inference | 8 ms，連續 3 筆即停止 |
| control-loop tick | 12 ms，連續 3 筆即停止 |

`/motor/state` 目前沒有馬達溫度或 driver fault 欄位，所以 YAML 的 `55 °C` 尚不是有效軟體保護；仍須使用 driver 保護、限流電源與人工溫度監看。Bus current 目前只監看，不會自動 trip。

## 操作順序

第一次使用依序完成：

```text
0 環境 → 1 Build/網路 → 2 site YAML → 3 真機校正
→ 4 policy 驗證與封裝 → 5 Preflight → 6 只讀驗收
→ 7 三秒測試 → 8 停止
```

換 policy 時重做第 4～8 步；修改 calibration、fixed-forward 速度、decoder 或故障腿時，也必須重新封裝。

## 第 0 步：每個 terminal 載入環境

```bash
export REDRHEX_WS=/home/jetson/rinbo_ros_ws
export REDRHEX_SITE_CTRL=/home/jetson/redrhex_site/redrhex_policy_full_feedback_rig.yaml
export REDRHEX_SITE_BRIDGE=/home/jetson/redrhex_site/lowlevel_bridge_full_feedback_rig.yaml
export REDRHEX_BAD_LEG=L1
export REDRHEX_SBRIO_IP=192.168.30.254
export REDRHEX_ORIN_WIRED_IP=192.168.30.8
export CORE_MASTER_ADDR="${REDRHEX_SBRIO_IP}:50051"
export CORE_LOCAL_IP="$REDRHEX_ORIN_WIRED_IP"
export CORE_IP="$REDRHEX_SBRIO_IP"
export ROS_DOMAIN_ID=99

source /opt/ros/humble/setup.bash
[ -f "$REDRHEX_WS/install/setup.bash" ] && source "$REDRHEX_WS/install/setup.bash"
```

IMU driver 與所有 ROS process 都必須使用相同的 `ROS_DOMAIN_ID`。

## 第 1 步：Build、網路與實體安全

### 執行

```bash
(
  set -e
  cd "$REDRHEX_WS"
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install
  source install/setup.bash

  colcon test --packages-select \
    redrhex_rl_controller redrhex_lowlevel_bridge rinbo_fsm rinbo_ros_bridge
  colcon test-result --verbose
)
```

上面全部成功後，刷新目前 terminal：

```bash
source /opt/ros/humble/setup.bash
source "$REDRHEX_WS/install/setup.bash"
```

先檢查網卡與 route：

```bash
ip -4 -brief address
ip route get "$REDRHEX_SBRIO_IP"
```

確認 route 的來源 IP 正確後，再做唯一的連線 hard gate：

```bash
nc -zvw2 "$REDRHEX_SBRIO_IP" 50051
```

### 通過條件

- Build 與 test 沒有 failure。
- Route 走 Orin 有線介面，來源 IP 等於 `$REDRHEX_ORIN_WIRED_IP`（預設 `192.168.30.8`）。
- TCP 50051 成功。
- 機器人已固定、L1/SL1 已隔離、限流電源與實體 E-stop 可用。
- 沒有舊 FSM、RL launch 或 servo probe 在背景發布 command。

若網路或實體安全任一項不通過，停止；不要送 power command。

## 第 2 步：建立 site-local YAML

第一次才執行：

```bash
mkdir -p /home/jetson/redrhex_site /home/jetson/redrhex_models

cp -n "$REDRHEX_WS/src/redrhex_rl_controller/config/redrhex_policy_full_feedback_rig.yaml" \
  "$REDRHEX_SITE_CTRL"
cp -n "$REDRHEX_WS/src/redrhex_lowlevel_bridge/config/lowlevel_bridge_full_feedback_rig.yaml" \
  "$REDRHEX_SITE_BRIDGE"

code "$REDRHEX_SITE_CTRL" "$REDRHEX_SITE_BRIDGE"
```

兩份 YAML 都必須保留：

```yaml
hardware:
  disabled_legs: ["L1"]
  max_disabled_legs: 1
```

只在實測完成後填入或確認下列內容：

| 檔案 | 必須填入的內容 |
|---|---|
| Controller | ONNX path/SHA、Bridge hash、IMU publisher/frame/mount/upright quaternion、action sign/offset |
| Bridge | Main zero/sign/direction/PWM scale、ABAD zero/sign/counts-per-rad、calibration acknowledgement |

順序不可混用：

- Policy order：`[R1,R2,R3,L1,L2,L3]`
- Rinbo／Bridge order：`[L1,L2,L3,R1,R2,R3]`

`commands.fixed_forward_vx` 必須是 `0.22`；play-forward 的 bias/residual/clip scale 必須是 `1.0/0.04/0.30`。這些是 artifact contract，不是臨場 tuning 參數。

## 第 3 步：校正 12 路 encoder 與 IMU

已完成可信校正且硬體未改動，可跳到第 4 步。L1/SL1 不可為校正而重新上電。

### 3.1 啟動唯一 final bridge 與只讀 feedback

本章只適用一般從零／第一次校正路徑。若 R-Slip launcher 已啟動 final bridge，請回到[專用接續流程](redrhex_sim2real_after_rslip.md)，不要混用本章的上電命令。

Terminal A：

```bash
ros2 run rinbo_ros_bridge rinbo_ros_bridge --ros-args \
  --params-file "$(ros2 pkg prefix rinbo_ros_bridge)/share/rinbo_ros_bridge/config/redrhex_safe.yaml" \
  -p core_ip:="$REDRHEX_SBRIO_IP"
```

Terminal B：只開 digital 與 sensor power，不開 relay。

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool sequence
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status \
  --disabled-leg "$REDRHEX_BAD_LEG"

ros2 run redrhex_lowlevel_bridge biorola_bringup_check \
  --message-timeout-s 5.0 --require-power-state --strict
ros2 run redrhex_lowlevel_bridge biorola_fault_diag snapshot
timeout 3s ros2 topic echo /motor/state --once
timeout 3s ros2 topic echo /power/state --once
```

通過條件：

- Power status 是 `digital=true`、`signal=true`、`power=false`。
- `/motor/state` 與 `/power/state` 各只有一個 publisher，stamp/seq 單調。
- Motor 最大間隔 `<0.25 s`，power 最大間隔 `<0.35 s`。
- Power mapping 是 `ch1=L1 ... ch6=R3, ch7=bus`；L1 的 ch1 不可當成 bus voltage。

### 3.2 Main Drive encoder

轉換式：

```text
position_rad = (raw_count - zero_count) × sign × 2π / counts_per_rev
```

需要小角度主動量測時才開 relay：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool sequence \
  --include-relay --confirm-relay --disabled-leg "$REDRHEX_BAD_LEG"
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status \
  --disabled-leg "$REDRHEX_BAD_LEG"
```

通過條件：`power=true`、ch7 為 18–30 V、五隻健康腿各低於 3 A。依序小角度正反轉五隻健康腿，記錄並填入：

```text
main_position_counts_per_rev
main_encoder_zero_counts_rinbo_order
main_encoder_sign_rinbo_order
main_direction_positive_rinbo_order
```

目前沒有專用 Main 單腿低功率 mapping CLI；只能使用現場已核准且有限制 PWM／電流的既有方法。沒有可重複量測證據，不可把 `main_drive_calibrated` 改成 `true`。

### 3.3 ABAD encoder 與 command mapping

ABAD 順序是 `[SL1,SL2,SL3,SR1,SR2,SR3] = [L1,L2,L3,R1,R2,R3]`。

每次只測一顆健康 servo；例如 SL2：

```bash
ros2 run redrhex_lowlevel_bridge biorola_servo_probe test sl2 \
  --delta 20 --max-abs-delta 20 \
  --require-power-state --require-power-on --max-current-a 3.0 \
  --confirm-motion
```

用至少兩個已知角度求：

```text
counts_per_rad = abs(count_2 - count_1) / abs(angle_2 - angle_1)
```

將五隻健康腿的實測結果填入：

```text
abad_encoder_zero_rinbo_order
abad_sign_rinbo_order
abad_encoder_counts_per_rad_rinbo_order
```

完成後立刻關閉 relay：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool sensors
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status \
  --disabled-leg "$REDRHEX_BAD_LEG"
```

通過條件是 power status 回到 `digital=true`、`signal=true`、`power=false`。五隻健康腿的 zero/sign/scale、限位與 command mapping 都通過後，才可設定：

```yaml
rinbo:
  main_drive_calibrated: true
  publish_abad_joint_feedback: true
  abad_feedback_calibrated: true
  require_abad_command_calibration: true
  abad_command_calibrated: true
```

Controller 的 Main/ABAD action sign 與 zero offset 也通過後，才把 `action.hardware_mapping_calibrated` 設為 `true`。

### 3.4 IMU frame 與 mounting

本 repo 沒有這台實機的 IMU driver package／launch 命令。先用現場已驗證的命令啟動真實 `/imu/data`；若 package、launch、publisher node 或 `frame_id` 尚未確認，本次就是 **NO-GO**，不可使用 fake IMU。啟動後執行：

```bash
ros2 topic info /imu/data -v
timeout 3s ros2 topic echo /imu/data --once
timeout 5s ros2 topic hz /imu/data

ros2 run redrhex_rl_controller imu_alignment_tool \
  --topic /imu/data --duration-s 5 --min-samples 200
```

通過條件：

- `/imu/data` 只有一個 publisher，建議穩定至少 50 Hz。
- Publisher node 與 `header.frame_id` 已原樣填入 controller YAML。
- Stationary gyro RMS `≤0.03 rad/s`，stamp 單調，orientation/gyro covariance 可用。
- 正常架空站姿的對齊結果可重複，且 observation `6:9` 會接近 `[0,-1,0]`。

至少量兩次且結果一致後，才把 mounting/upright quaternion 寫入同一份 controller YAML，並設定：

```yaml
observation:
  imu_alignment_calibrated: true
```

四元數順序是 `[x,y,z,w]`。若結果無法重現，保持 acknowledgement 為 `false`。

### 3.5 校正結束

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool off
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status \
  --disabled-leg "$REDRHEX_BAD_LEG"
```

必須看到 `digital=false`、`signal=false`、`power=false`。接著停止 IMU driver，最後停止 terminal A 的 final bridge，再做離線封裝。

## 第 4 步：離線驗證並封裝 policy

開始本步前，Bridge YAML 的 Main、ABAD feedback、ABAD command，以及 Controller YAML 的 IMU alignment、hardware mapping 共五個 calibration gate 都必須是基於真機量測的 `true`。任一項仍為 `false` 時，只能先執行 `check_onnx_io.py` 分類模型；不要算 Bridge hash、compare 或 package，請先回第 3 步完成校正。

### 4.1 先分類收到的模型

| 類型 | 放置位置 | 可以寫入 active YAML？ |
|---|---|---|
| `DIAGNOSTIC_ONLY`／`quality_rejected` | `redrhex_models/incoming/` | 不可以 |
| 尚未比對的 source bundle | `redrhex_models/incoming/<tag>/` | 不可以 |
| 本機 compare/package 全部通過的 artifact | `redrhex_models/policy_verified_<tag>.onnx` | Preflight 全通過後才可以 |

目前下載的 sensor-v2 diagnostic 模型不能靠重新命名或刪除額外 output 轉成 deployment model，因為它的 input feature、history、normalizer 與目前 56/280D observation builder 都不同，而且 artifact 自己已標示 quality rejected。

正確作法二選一：

1. **建議路線**：由訓練端匯出符合下節 56/280D contract 的 quality-approved 完整 bundle。
2. **新 sensor-v2 專案**：先取得 36D feature order、60-frame history/reset、normalizer、command/action 語意、quality-approved ONNX/TorchScript/golden，再另外實作與測試新的 runtime contract。現在這顆 rejected artifact 不能作為部署模型。

兩條路都必須提供同 checkpoint 的 ONNX、TorchScript、golden NPZ、training Git SHA 與三份 training source；只有單一 ONNX 不可 package。

目前 site controller YAML 指向尚不存在的 `/home/jetson/redrhex_models/policy_verified.onnx`，`expected_sha256` 是空白；Main、ABAD、IMU 與 hardware mapping acknowledgement 也仍未完成。這些狀態允許依第 3 步做受限校正，但 policy Preflight／launch／enable 都是 **NO-GO**。只有 `package_verified_policy.py` 成功後，才把新 verified path 與 SHA 寫入 site YAML。

新模型統一使用以下目錄，不要把 raw ONNX 直接放在 verified 區：

```text
/home/jetson/redrhex_models/incoming/<tag>/       # 完整 source bundle
/home/jetson/redrhex_models/policy_verified_<tag>.onnx
/home/jetson/redrhex_models/policy_verified_<tag>.json
```

### 4.2 Policy 必須符合的現有 contract

Runtime 只接受 `.onnx`；TensorRT `.engine` 不能直接放入本流程。ONNX 必須是 `float32`，輸入 `[1,56]` 或 `[1,280]`，輸出 `[1,12]` residual action。

| Index | 56D observation |
|---:|---|
| 0–2 | base linear velocity；架空 profile 固定補 `[0,0,0]` |
| 3–5 | 對齊 policy frame 的 IMU gyro |
| 6–8 | projected gravity |
| 9–14 | `sin(main position)` |
| 15–20 | `cos(main position)` |
| 21–26 | `main velocity / 2π` |
| 27–32 | `ABAD position / 0.61096` |
| 33–38 | ABAD velocity |
| 39–41 | `POLICY_RUN` 時為 `[0.22,0,0]` |
| 42–43 | 1 Hz gait phase sin/cos |
| 44–55 | 兩步延遲、訓練 clip 後的 12D action |

280D 只能是：

```text
[current 56, previous-1, previous-2, previous-3, previous-4]
```

架空 profile 沒有真實 base linear velocity，因此只能做資料流與架空動作驗證。若訓練時的 observation、座標、單位、normalizer 或 action 語意不同，回訓練端修改／重訓，不可在真機端猜 permutation。

### 4.3 準備完整輸入檔

必須來自同一 checkpoint：

其中 `policy.pt` 必須是 `torch.jit.load()` 可載入的 TorchScript export，不是訓練用 PPO checkpoint。

```bash
export REDRHEX_ARTIFACT_TAG=填入本次唯一tag
export REDRHEX_BUNDLE_DIR="/home/jetson/redrhex_models/incoming/${REDRHEX_ARTIFACT_TAG}"
export REDRHEX_SOURCE_ONNX="$REDRHEX_BUNDLE_DIR/policy.onnx"
export REDRHEX_SOURCE_PT="$REDRHEX_BUNDLE_DIR/policy.pt"
export REDRHEX_GOLDEN_NPZ="$REDRHEX_BUNDLE_DIR/redrhex_golden_v2.npz"
export REDRHEX_TRAINING_SHA="$(tr -d '[:space:]' < "$REDRHEX_BUNDLE_DIR/training_git_sha.txt")"
export REDRHEX_TRAINING_ENV_SOURCE="$REDRHEX_BUNDLE_DIR/training_env.py"
export REDRHEX_TRAINING_ENV_CONFIG_SOURCE="$REDRHEX_BUNDLE_DIR/training_env_config.py"
export REDRHEX_TRAINING_PLAY_SOURCE="$REDRHEX_BUNDLE_DIR/training_play.py"
export REDRHEX_VERIFIED_ONNX="/home/jetson/redrhex_models/policy_verified_${REDRHEX_ARTIFACT_TAG}.onnx"
export REDRHEX_REPORT="/home/jetson/redrhex_models/policy_verified_${REDRHEX_ARTIFACT_TAG}.json"
export PYTHONPATH="$REDRHEX_WS/src/redrhex_rl_controller${PYTHONPATH:+:$PYTHONPATH}"
```

Source ONNX metadata 必須精確包含：

```text
redrhex.observation_contract = redrhex_obs56_v2_action_lag2
redrhex.action_contract = redrhex_stage5_residual_v1
redrhex.policy_input_layout = single_56 或 current_to_oldest_5x56
redrhex.normalizer = embedded
redrhex.training_action_clip = 1.0
redrhex.training_git_sha = 與 REDRHEX_TRAINING_SHA 相同的 commit
redrhex_artifact_status = deployment_approved（若 exporter 有提供）
redrhex_quality_status = quality_approved（若 exporter 有提供）
```

最後兩個 status key 為選填以相容既有 exporter；但只要 key 存在，就必須是明確 approved 值。`pending_review`、未知值或任何 rejected／diagnostic 值都會被檢查器拒絕。

Golden NPZ 必須由真實 simulator play loop 匯出，包含 fixed-forward `0.22`、zero base velocity、phase、reset/warmup、L1 disabled 與兩步 action lag；不可用候選 ONNX 反向生成 expected output。

### 4.4 檢查、比對與封裝

```bash
python3 "$REDRHEX_WS/src/redrhex_rl_controller/scripts/check_onnx_io.py" \
  "$REDRHEX_SOURCE_ONNX" \
  --expected-obs-dim 56 --expected-action-dim 12 \
  --max-reference-action 1.5
```

只有 exit code 是 `0` 且看到 `ONNX I/O check OK` 才執行：

```bash
ros2 run redrhex_rl_controller bridge_config_hash \
  --bridge-config "$REDRHEX_SITE_BRIDGE" \
  --disabled-legs "$REDRHEX_BAD_LEG"
```

把第二個命令輸出的 hash 填入 `$REDRHEX_SITE_CTRL` 的 `policy.expected_bridge_config_sha256`，再執行：

```bash
(
  set -e
  python3 "$REDRHEX_WS/src/redrhex_rl_controller/scripts/compare_onnx_with_torch.py" \
    --onnx "$REDRHEX_SOURCE_ONNX" \
    --torchscript "$REDRHEX_SOURCE_PT" \
    --golden-vectors "$REDRHEX_GOLDEN_NPZ" \
    --controller-config "$REDRHEX_SITE_CTRL" \
    --bridge-config "$REDRHEX_SITE_BRIDGE" \
    --disabled-legs "$REDRHEX_BAD_LEG" \
    --training-git-sha "$REDRHEX_TRAINING_SHA" \
    --training-env-source "$REDRHEX_TRAINING_ENV_SOURCE" \
    --training-env-config-source "$REDRHEX_TRAINING_ENV_CONFIG_SOURCE" \
    --training-play-source "$REDRHEX_TRAINING_PLAY_SOURCE" \
    --report-json "$REDRHEX_REPORT"

  python3 "$REDRHEX_WS/src/redrhex_rl_controller/scripts/package_verified_policy.py" \
    "$REDRHEX_SOURCE_ONNX" "$REDRHEX_VERIFIED_ONNX" \
    --torchscript "$REDRHEX_SOURCE_PT" \
    --golden-vectors "$REDRHEX_GOLDEN_NPZ" \
    --controller-config "$REDRHEX_SITE_CTRL" \
    --bridge-config "$REDRHEX_SITE_BRIDGE" \
    --disabled-legs "$REDRHEX_BAD_LEG" \
    --training-git-sha "$REDRHEX_TRAINING_SHA" \
    --training-env-source "$REDRHEX_TRAINING_ENV_SOURCE" \
    --training-env-config-source "$REDRHEX_TRAINING_ENV_CONFIG_SOURCE" \
    --training-play-source "$REDRHEX_TRAINING_PLAY_SOURCE" \
    --golden-report "$REDRHEX_REPORT"

  sha256sum "$REDRHEX_VERIFIED_ONNX"
)
```

這個小括號區塊是 fail-fast：compare 失敗時不會繼續 package；package 失敗時也不會把檔案當成 verified。

將新檔案的完整路徑與 packager 印出的 SHA 寫回同一份 controller YAML：

```yaml
policy:
  onnx_path: "/home/jetson/redrhex_models/policy_verified_本次tag.onnx"
  expected_sha256: "packager輸出的64位SHA256"
```

輸出檔名必須是全新且不可覆寫。目前已驗證的是 CPU execution provider；CUDA/TensorRT 尚未驗證。

## 第 5 步：Preflight

```bash
ros2 run redrhex_rl_controller preflight_check \
  --config "$REDRHEX_SITE_CTRL" \
  --bridge-config "$REDRHEX_SITE_BRIDGE" \
  --disabled-legs "$REDRHEX_BAD_LEG"
```

只有以下全部成立才可繼續：

- JSON 中所有 `checks[].ok` 都是 `true`。
- Profile 是 `full_feedback_rig`，policy/Bridge SHA 與 metadata 一致。
- Main、ABAD、IMU 與 hardware mapping calibration gate 都通過。
- Command 是 `fixed_forward=0.22`，lease 是 3 秒。
- Controller、Bridge 與 artifact 都是 L1 disabled。
- Bridge YAML 的 `allow_enable` 仍是 `false`。
- CPU inference p99 `≤6 ms`。

任一項為 `false` 就停。修改任何 calibration、decoder、fixed-forward 或 disabled leg 後，回第 4 步重新封裝。

## 第 6 步：只讀啟動與 topic 驗收

本節是**一般從零路徑**。日常重測先依第 0、1、3.1、3.4、5 步啟動唯一 final bridge、sensor-only power 與真實 IMU；到本節時保持它們運行，不要再啟動第二份 bridge，也不要重送 power sequence。先用 `rinbo_power_tool status --disabled-leg "$REDRHEX_BAD_LEG"` 確認 `digital=true`、`signal=true`、`power=false`。R-Slip 同次接續時 relay 仍開啟，必須改走專用接續版，不可混用本節。

啟動只讀 policy stack：

```bash
ros2 launch redrhex_rl_controller redrhex_policy_bringup.launch.py \
  safety_profile:=custom \
  config:="$REDRHEX_SITE_CTRL" \
  bridge_config:="$REDRHEX_SITE_BRIDGE" \
  disabled_legs:="$REDRHEX_BAD_LEG" \
  bridge_rinbo_allow_enable:=false \
  start_bridge:=true \
  use_fake_sensors:=false
```

`start_bridge:=true` 啟動的是 12 路轉換與安全層 `redrhex_lowlevel_bridge`，不是第二份 `rinbo_ros_bridge` final arbiter。

另一個 terminal：

```bash
for topic in /imu/data /joint_states /redrhex/motor_commands /motor/command; do
  ros2 topic info "$topic" -v
done

for topic in /imu/data /joint_states /redrhex/observation; do
  timeout 5s ros2 topic hz "$topic"
done

for topic in \
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
```

通過條件：

- IMU、`/joint_states`、controller command 與 final `/motor/command` 都只有預期的一個 publisher。
- `/joint_states` 有 6 Main + 6 ABAD；順序、單位、方向與靜止速度正確。
- IMU 與 joint 最大間隔 `<0.10 s`，observation 約 125 Hz。
- Observation `0:3=[0,0,0]` 且 imputation mask 有標示；`6:9≈[0,-1,0]`；此時 `39:42=[0,0,0]`。
- Disabled leg 是 L1；preview 是全腿 `enable=false`、`servo_control_mode=0`，diagnostics 中 L1 preview PWM 為 0。
- `/rinbo/motor_output_enabled` 與 `/redrhex/lowlevel_output_enabled` 都是 `false`。
- Diagnostics 沒有 ERROR、stale、publisher mismatch 或 deadline miss。

完成後在 launch terminal 按 `Ctrl+C`，等 process 完全返回。Final bridge 保持運行。若 mapping、IMU 或 gravity 不對，修正 YAML 後回第 4 步重新封裝。

## 第 7 步：三秒 fixed-forward 架空測試

先再跑一次第 5 步 Preflight，並確認所有 FSM、servo probe 與只讀 launch 都已停止。

### 7.1 開 relay 並啟動 active stack

```bash
(
  set -e
  ros2 run redrhex_lowlevel_bridge rinbo_power_tool sequence \
    --include-relay --confirm-relay --disabled-leg "$REDRHEX_BAD_LEG"
  ros2 run redrhex_lowlevel_bridge rinbo_power_tool status \
    --disabled-leg "$REDRHEX_BAD_LEG"
)
```

兩個命令都必須 exit `0`，而且 status 必須是 `digital=true`、`signal=true`、`power=true`。任一條件不符就執行第 8 步，不要啟動 active stack。全部通過後才執行：

```bash
ros2 launch redrhex_rl_controller redrhex_policy_bringup.launch.py \
  safety_profile:=custom \
  config:="$REDRHEX_SITE_CTRL" \
  bridge_config:="$REDRHEX_SITE_BRIDGE" \
  disabled_legs:="$REDRHEX_BAD_LEG" \
  bridge_rinbo_allow_enable:=true \
  start_bridge:=true \
  use_fake_sensors:=false
```

唯一允許的 active override 是 `bridge_rinbo_allow_enable:=true` 與同一個 L1 mask；不可在 launch 臨時覆寫 calibration acknowledgement。

### 7.2 依 state 啟用

另一個 terminal：

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

參數必須分別是 `fixed_forward` 與 `0.22`，五隻健康腳也必須在 12 秒內到位：wrapped error `≤0.12 rad`、速度 `≤0.25 rad/s`，並穩定至少 `0.50 s`。L1 不參與到位 gate。全部通過後才單獨執行：

```bash
ros2 topic pub --once /redrhex/enable_policy std_msgs/msg/Bool "{data: true}"
```

不需要鍵盤，也不要發布 `/cmd_vel`。

### 7.3 驗收自動停止

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

必須回到 `INIT_STAND`，兩層 output 都是 `false`，且沒有 deadline miss。若仍在 `POLICY_RUN` 或任一 output 為 `true`，立即使用實體 E-stop／主電源切斷，再執行第 8 步。

架空、base velocity 補零且 L1 disabled 時，動作不一定像六腳落地模擬。本次通過標準是 input、方向、mask、限幅、ack 與三秒自動停止正確，不是落地步態品質。

## 第 8 步：停止與收工

RL launch 還在運行時先執行：

```bash
ros2 topic pub --once /redrhex/enable_policy std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /redrhex/enable_motors std_msgs/msg/Bool "{data: false}"
timeout 3s ros2 topic echo /rinbo/motor_output_enabled --once
timeout 3s ros2 topic echo /redrhex/lowlevel_output_enabled --once

ros2 run redrhex_lowlevel_bridge rinbo_power_tool off
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status \
  --disabled-leg "$REDRHEX_BAD_LEG"
```

通過條件：兩層 output 先變成 `false`，再收到 `digital=false`、`signal=false`、`power=false` acknowledgement。之後依序：

1. 在 RL launch terminal 按 `Ctrl+C` 並等 process 返回。
2. 停止 IMU driver。
3. 最後才停止 final bridge。

不要先殺 final bridge；它負責最後的 timeout 與 disable。若 ROS 無回應，先按實體 E-stop／切主電源，再盡可能執行：

```bash
ros2 run redrhex_rl_controller estop_tool assert
ros2 run redrhex_lowlevel_bridge rinbo_power_tool off
```

E-stop 對同一個 final bridge process 是 sticky。排除危害並確認全斷電後，必須重啟唯一 final bridge，再重新通過網路、Preflight 與只讀驗收。

## Calibration、Standing、Tripod 的 L1 降級測試

本节是当前现场 L1 流程；若故障腿不是 L1，完整依[单腿屏蔽操作手册](redrhex_disabled_leg_operation.md)操作并使用 `rinbo_leg_mask commands` 产生的命令，不要沿用本手册第 0 步的 `REDRHEX_BAD_LEG=L1`。

三个 FSM 必须使用同一份 site-local 配置。先保持 relay off；第一次依[单腿屏蔽操作手册](redrhex_disabled_leg_operation.md#第一次准备)准备 Controller／Bridge site YAML，再用 `rinbo_leg_mask auto L1` 统一三层 mask；所选 FSM 文件缺失时会从模板自动建立。之后每次运行先检查：

```bash
export REDRHEX_FSM_CFG="${REDRHEX_FSM_CFG:-/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml}"
test -f "$REDRHEX_FSM_CFG"
ros2 run redrhex_rl_controller rinbo_leg_mask verify --scope fsm
```

任一命令失败或显示 `MISMATCH` 都不要开 relay。不要把 `REDRHEX_FSM_CFG` 改回仓库里的 L1 专用模板来绕过检查。

只保留唯一 final bridge；RL stack 與 servo probe 必須停止。開 relay並確認 exact acknowledgement：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool sequence \
  --include-relay --confirm-relay --disabled-leg "$REDRHEX_BAD_LEG"
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status \
  --disabled-leg "$REDRHEX_BAD_LEG"
```

依序執行，一次只能跑一個：

```bash
ros2 run rinbo_fsm rinbo_cali --ros-args --params-file "$REDRHEX_FSM_CFG"
# 看到 DONE 後按 Ctrl+C，等 process 返回

timeout 3s ros2 topic echo /rinbo/motor_output_enabled --once
# 必須是 false

ros2 run rinbo_fsm rinbo_standing --ros-args --params-file "$REDRHEX_FSM_CFG"
# 看到 HEALTHY LEGS STANDING；結束時按 Ctrl+C，等 process 返回

timeout 3s ros2 topic echo /rinbo/motor_output_enabled --once
# 必須是 false

ros2 run rinbo_fsm rinbo_tripod --ros-args --params-file "$REDRHEX_FSM_CFG"
# 結束時按 Ctrl+C，等待 Fully stopped 與 process 返回
```

每一個 FSM 啟動後都必須明確看到 `DEGRADED MODE`，且清單与 `rinbo_leg_mask status` 显示的是同一腿；沒看到或腿名错误就立即 `Ctrl+C`，不可改用未載入參數檔的裸指令重試。任何 `SAFETY STOP` 都不可接著跑下一個 FSM；按 `Ctrl+C`、等 process 返回，再依第 8 步關機。被屏蔽腿不參與完成與 position-error gate，但五隻健康腿的 error、3 A、18–30 V、PWM 80 與 stale telemetry 保護仍保留。

## 日後更換故障腿

不要再手工分别编辑 FSM、Controller 与 Bridge YAML。保持 output 为 `false`、relay off 且相关节点全部退出后，依 [`rinbo_leg_mask` 单腿屏蔽操作手册](redrhex_disabled_leg_operation.md)使用日常自动流程；若所选 FSM 文件缺失，`auto` 会从模板建立：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask auto L2
ros2 run redrhex_rl_controller rinbo_leg_mask commands
```

mask 只能在 motor output 为 `false`、relay off 且相关节点全退出后改变；它不是 `ros2 param set` 热切换。损坏腿的 Main 与对应 SL/SR servo 都必须实体隔离。Controller／Bridge mask 改变或 Bridge hash pin 失配时，工具会清空旧 policy SHA，并把 Main／ABAD／hardware mapping calibration acknowledgement 复位为 `false`。必须根据真机重新校正，再用新 mask 重新 compare、package、Preflight；不能拿绑定 L1 的 packaged artifact，只在 launch 改成另一腿，也不能直接把 acknowledgement 手改回 `true`。

## 腿部 mapping

| 腿 | 位置 | Rinbo index | Power ch | Policy index | Main joint | ABAD joint/servo |
|---|---|---:|---:|---:|---|---|
| L1 | 左前 | 0 | 1 | 3 | `Revolute_18` | `Revolute_17` / SL1 |
| L2 | 左中 | 1 | 2 | 4 | `Revolute_23` | `Revolute_22` / SL2 |
| L3 | 左後 | 2 | 3 | 5 | `Revolute_24` | `Revolute_21` / SL3 |
| R1 | 右前 | 3 | 4 | 0 | `Revolute_15` | `Revolute_14` / SR1 |
| R2 | 右中 | 4 | 5 | 1 | `Revolute_7` | `Revolute_6` / SR2 |
| R3 | 右後 | 5 | 6 | 2 | `Revolute_12` | `Revolute_11` / SR3 |

實機標籤若與表格不一致，保持 output disabled，逐腿低功率確認 wiring；不要猜。

## 快速排錯

| 現象 | 立即處理 |
|---|---|
| ONNX shape 正確但 Preflight 失敗 | 回訓練端補齊 metadata、TorchScript、golden NPZ 與 training source |
| Reference action 是數百 | 禁止上機；模型/action contract 不相容 |
| `policy_sha256` 失敗 | 用新檔名重新 package，將實際 SHA 寫入 site YAML |
| 等不到 IMU | 檢查 topic、唯一 publisher、frame 與 `ROS_DOMAIN_ID` |
| Gravity 不是 `[0,-1,0]` | Output 保持關閉，重做 IMU alignment並重新 package |
| 缺少 ABAD joints | 檢查 `/motor/state`、Bridge calibration 與 joint 名稱 |
| Disabled-leg mismatch | Controller、Bridge、artifact、FSM 與 launch 全部統一後重新封裝 |
| 卡在 `INIT_STAND` | 查 output ack、healthy-leg mapping、位置誤差與速度；不要先放寬 tolerance |
| `POLICY_RUN` command 仍為 0 | 檢查 state、`commands.profile` 與 `fixed_forward_vx` |
| 一啟用就過流／欠壓 | 立即停機；檢查卡滯、方向、供電與 channel mapping |
| 動作不像模擬 | 先驗證資料 contract；要改善落地步態，需 velocity estimator 或同 sensor/dropout contract 重訓 |
