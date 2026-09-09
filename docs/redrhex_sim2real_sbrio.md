# Sim2Real：從模型檔到真機架空測試

> 2026-09-07：目前 36 V 吊掛測試的保護門檻已調整，請以[吊掛測試保護調整](suspended_safety_tuning_zh_TW.md)及現場設定為準；本文件下方的舊數字可能仍描述初始保守設定。

[回到入口](README.md)｜[單腳測試](manual_leg_control_zh_TW.md)｜[腿部啟用／屏蔽](orin_multileg_operation.md)

Sim2Real 是把模擬中訓練的控制模型接到真機。`policy` 就是控制模型；ONNX 是模型檔案格式；Bridge 是轉換 ROS 與硬體訊息的程式。

**先確認模型能走哪條驗證流程，再做真機校正與上電。** 本頁主流程使用 `full_feedback_rig`：6 路主馬達編碼器、6 路側擺伺服（ABAD）編碼器和真實 IMU，機器人全程架空。這是資料與動作驗證，並非落地行走教學。

## 先判斷你在哪一步

| 現在的情況 | 下一步 |
|---|---|
| 只收到一顆新 ONNX | 看下方「模型格式與檔案位置」；先做離線分類，不要先上電 |
| 模型是 56／280 維，配套資料齊全 | 第 0～2 步準備環境與現場設定；第 3 步完成真機量測；第 4 步完整比對與封裝 |
| 模型是 sensor-v2 | 看下方 sensor-v2 說明；現有 packager 尚不接受它，不能直接照第 4～7 步部署 |
| 換新模型，但硬體校正仍有效 | 第 4 步重新驗證封裝，再走第 5～8 步 |
| 同一顆模型已驗證，要重測 | 第 0 步載入環境 → 第 5 步檢查 → 第 6 步準備回讀並驗收 → 第 7 步測試 → 第 8 步停止 |
| 已用 Windows R-Slip 跑完 Tripod | 看第 6 步的「從 R-Slip 接續」，保留唯一 final bridge |
| 只是校正、站立或測單腳 | 看[單腳測試](manual_leg_control_zh_TW.md)或[腿部操作](orin_multileg_operation.md)，不需要 RL 模型 |

查細節：[模型資料](#model-files) → [環境](#environment) → [現場設定](#site-config) → [硬體校正](#calibration) → [模型封裝](#package) → [上機前檢查](#preflight) → [只讀驗收](#read-only) → [三秒測試](#motion-test) → [停止](#stop)。

<a id="model-files"></a>

## 模型格式與檔案位置

| 格式 | 目前程式狀態 | 要做什麼 |
|---|---|---|
| 單一輸入 `[1,56]` 或 `[1,280]`，輸出 `[1,12]` | 有完整比對與封裝流程 | 依本頁執行；尺寸正確仍不代表模型內容合格 |
| sensor-v2：`sensor_history [1,60,36]`、`command [1,3]`，輸出 action 與速度估計 | 已有 runner、歷史資料組裝及控制節點分支；現有 `package_verified_policy.py` 明確拒絕 sensor-v2 bundle | 只依下面的離線檢查分類；需完成相應驗證／封裝整合，或由訓練端提供相容的 56／280 維完整模型資料 |
| `DIAGNOSTIC_ONLY`／`quality_rejected` | 不可部署 | 保留作分析，不可改名當成驗證通過 |

不要用 reshape（硬改資料尺寸）、補零、刪除輸出或修改通過標記來繞過規格。檔名含 `sensor_v2_suspended_experimental` 的範本也不代表 sensor-v2 已完成部署驗證。

每次新模型使用一個不重複的 `<tag>`，例如 `run11_seed42_v1`：

```text
/home/jetson/redrhex_models/
├── incoming/<tag>/
│   ├── policy.onnx
│   ├── policy.onnx.json          # 有提供時保留；不能取代完整驗證
│   ├── policy.pt                 # 同一 checkpoint 的 TorchScript export
│   ├── redrhex_golden_v2.npz      # 真實模擬軌跡，不是人工假資料
│   ├── training_git_sha.txt
│   ├── training_env.py
│   ├── training_env_config.py
│   └── training_play.py
├── policy_verified_<tag>.onnx    # 第 4 步封裝成功才會產生
└── policy_verified_<tag>.json    # 比對報告
```

`checkpoint` 是同一次訓練保存的模型版本；`golden` 是用來核對模型與控制結果的模擬紀錄。缺檔就向訓練端索取，不能自行補成通過。模型放在 `incoming/` 不會自動啟用。

一般 56／280 維模型的第一次離線檢查（把路徑換成本次模型）：

```bash
python3 /home/jetson/rinbo_ros_ws/src/redrhex_rl_controller/scripts/check_onnx_io.py \
  /home/jetson/redrhex_models/incoming/本次tag/policy.onnx \
  --expected-obs-dim 56 --expected-action-dim 12 --max-reference-action 1.5
```

sensor-v2 使用不同的離線檢查，需要匯出者提供的 sidecar 與可信 SHA256；SHA256 是檔案指紋，用來核對是否為同一檔案：

```bash
python3 /home/jetson/rinbo_ros_ws/src/redrhex_rl_controller/scripts/check_onnx_io.py \
  /home/jetson/redrhex_models/incoming/本次tag/policy.onnx \
  --contract experimental-sensor-v2 \
  --expected-sha256 填入本次模型的64位SHA256 \
  --sidecar /home/jetson/redrhex_models/incoming/本次tag/policy.onnx.json
```

通過這個離線檢查不會產生上機許可；sensor-v2 不接以下 56／280 維封裝步驟。

## 開始前：名單與硬體條件

本頁保留既有 **L1 已實體隔離**的單腿架空 RL 範本。它只適用於現場、模型、controller YAML、bridge YAML 全部同為 L1 disabled 的情況。先用 `ros2 run rinbo_fsm rinbo_legs status` 查實際名單；環境尚未載入時先做第 0 步。

**若現場屏蔽的是 L1、L3，或任何與模型不同的組合，就不能沿用本頁的 L1 範例上電。** 先完成硬體處理與對應模型驗證；不要為了符合範例而解除故障腿。R-Slip 支援多腿屏蔽，並不代表既有 RL 範本也支援。

每次實測都需符合：

- 機身可靠固定、腿部有轉動空間、實體急停可用；禁用腿的主馬達與對應伺服依現場方式隔離。
- sbRIO core／FPGA 已啟動，連線和真實回讀正常；IMU 使用實際驅動，不使用 fake sensors。
- 同時只有一份 final `rinbo_ros_bridge`。動作程式輪流使用，不能同時有 FSM、手動工具與 RL 輸出。
- 觀察命令用[監控面板](rinbo_monitor.md)，不要直接訂閱 `/motor/command`，以免破壞它只有 Bridge 一個接收者的要求。
- sbRIO／FPGA 失聯時的硬體 watchdog 與伺服停用行為需有現場實測證據；軟體檢查不會替代它。

`rinbo_cali` 的主馬達歸零不等於完成 RL 的 Main／ABAD／IMU 比例、方向與座標校正。第 3 步仍須有可重複的量測證據。

下面細節使用幾個程式用語：`observation` 是模型輸入，`action` 是模型輸出，`contract` 是雙方必須一致的規格，`gate` 是必須通過的檢查，`artifact` 是模型及其驗證資料。`IMU` 是姿態／角速度感測器，`relay` 是馬達電源繼電器。

既有架空範本的主要限制如下。硬體額定值若更低，必須再降低；不要為了避開 trip 而直接放寬。

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

`/motor/state` 目前沒有馬達溫度或 driver fault 欄位，所以 YAML 的 `55 °C` 尚不是有效軟體保護；仍須使用 driver 保護、限流電源與人工溫度監看。匯流排總電流目前只監看，不會自動觸發停止。

<a id="environment"></a>

## 第 0 步：每個終端機載入環境

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

## 第 1 步：編譯、網路與實體準備

已安裝並編譯的工作區不用每次重編。首次安裝或改過程式時，在動作停止後編譯：

```bash
cd "$REDRHEX_WS"
colcon build --symlink-install
source "$REDRHEX_WS/install/setup.bash"
```

開發驗證方式見 [RL 套件 README](../src/redrhex_rl_controller/README.md)；不要在實機動作期間跑建置或測試。

先檢查網卡與 route：

```bash
ip -4 -brief address
ip route get "$REDRHEX_SBRIO_IP"
```

確認路由的來源 IP 正確後，再檢查通訊埠是否可連線：

```bash
nc -zvw2 "$REDRHEX_SBRIO_IP" 50051
```

### 通過條件

- 所需套件已成功建置，修改過的程式已完成相應驗證。
- Route 走 Orin 有線介面，來源 IP 等於 `$REDRHEX_ORIN_WIRED_IP`（預設 `192.168.30.8`）。
- TCP 50051 成功。
- 機器人已固定、L1/SL1 已隔離、限流電源與實體 E-stop 可用。
- 沒有舊 FSM、RL launch 或 servo probe 在背景發布 command。

若網路或實體安全任一項不通過，停止；不要送 power command。

<a id="site-config"></a>

## 第 2 步：建立現場設定 YAML

第一次才執行：

```bash
mkdir -p /home/jetson/redrhex_site /home/jetson/redrhex_models

cp -n "$REDRHEX_WS/src/redrhex_rl_controller/config/redrhex_policy_full_feedback_rig.yaml" \
  "$REDRHEX_SITE_CTRL"
cp -n "$REDRHEX_WS/src/redrhex_lowlevel_bridge/config/lowlevel_bridge_full_feedback_rig.yaml" \
  "$REDRHEX_SITE_BRIDGE"

code "$REDRHEX_SITE_CTRL" "$REDRHEX_SITE_BRIDGE"
```

僅在現場符合本頁 L1 範本時，兩份 RL YAML 都使用以下設定。FSM 名單另由 `rinbo_legs` 管理，三者須一致：

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

本頁既有架空範本的 `commands.fixed_forward_vx` 是 `0.22`；play-forward 的 bias/residual/clip scale 是 `1.0/0.04/0.30`。這些設定與模型驗證綁定；修改後要重新比對與封裝。

<a id="calibration"></a>

## 第 3 步：校正編碼器與 IMU

已有可重複的量測證據、校正與硬體均未改動，可跳到第 4 步。L1/SL1 不可為校正而重新上電。

### 3.1 啟動唯一 final bridge 與只讀 feedback

第一次校正先停止所有動作，確認馬達已停用。若已有 final bridge 就沿用；若剛跑完 R-Slip 且校正、模型都已驗證，改看第 6 步「從 R-Slip 接續」。

Terminal A：

```bash
ros2 run rinbo_ros_bridge rinbo_ros_bridge --ros-args \
  --params-file "$(ros2 pkg prefix rinbo_ros_bridge)/share/rinbo_ros_bridge/config/redrhex_safe.yaml" \
  -p core_ip:="$REDRHEX_SBRIO_IP"
```

Terminal B：只開 digital 與 sensor power，不開 relay。

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool sequence
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status

ros2 run redrhex_lowlevel_bridge rinbo_bringup_check \
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

### 3.2 主馬達編碼器

轉換式：

```text
position_rad = (raw_count - zero_count) × sign × 2π / counts_per_rev
```

需要小角度主動量測時才開 relay：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool relay --confirm-relay
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
```

通過條件：`power=true`、ch7 為 18–30 V、五隻健康腿各低於 3 A。依序小角度正反轉五隻健康腿，記錄並填入：

```text
main_position_counts_per_rev
main_encoder_zero_counts_rinbo_order
main_encoder_sign_rinbo_order
main_direction_positive_rinbo_order
```

已有[單腳測試工具](manual_leg_control_zh_TW.md)，但它不會自動量測或證明 RL 的比例、方向與 zero mapping。使用現場核准、具出力／電流限制的量測方法，記錄實際角度與回讀。沒有可重複證據，不可把 `main_drive_calibrated` 改成 `true`。

### 3.3 側擺伺服的角度回讀與命令對應

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
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
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

### 3.4 IMU 座標與安裝方向

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
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
```

必須看到 `digital=false`、`signal=false`、`power=false`。接著停止 IMU driver，最後停止 terminal A 的 final bridge，再做離線封裝。

<a id="package"></a>

## 第 4 步：離線驗證並封裝 policy

開始本步前，Bridge YAML 的 Main、ABAD feedback、ABAD command，以及 Controller YAML 的 IMU alignment、hardware mapping 共五個 calibration gate 都必須是基於真機量測的 `true`。任一項仍為 `false` 時，只能先執行 `check_onnx_io.py` 分類模型；不要算 Bridge hash、compare 或 package，請先回第 3 步完成校正。

### 4.1 確認模型已分類

先完成本頁開頭的「模型格式與檔案位置」。以下只封裝符合 56／280 維規格的完整模型資料；sensor-v2、diagnostic 或缺少訓練證據的檔案停在離線檢查。

### 4.2 模型的輸入與輸出規格

本節的已封裝流程使用 `.onnx`；TensorRT `.engine` 不能直接放入。ONNX 必須是 `float32`，輸入 `[1,56]` 或 `[1,280]`，輸出 `[1,12]` residual action。

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

<a id="preflight"></a>

## 第 5 步：Preflight（上機前檢查）

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

<a id="read-only"></a>

## 第 6 步：只讀啟動與資料驗收

### 一般啟動

第 0 步的環境需在每個終端機載入。依第 3.1 步準備唯一 final bridge 和感測器電源，依第 3.4 步準備真實 IMU。已在執行的程序沿用，不重開；用 `rinbo_power_tool status` 確認 `digital=true`、`signal=true`、`power=false`，並通過第 5 步。

### 從 R-Slip 接續

已做完 Tripod 時，先依[腿部操作](orin_multileg_operation.md)停止 FSM。若參考零點可能改變，或現場流程要求，先重新校正。接著完成 Standing，姿態穩定後停止並等待程序退出。不要讓 Standing 留著保持出力。機身需持續由測試架支撐。

保持原本唯一 final bridge，準備真實 IMU；原流程 relay 還開著時，先做交接檢查：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_bringup_check --strict \
  --max-motor-command-publishers 0 --require-power-relay-on \
  --require-motor-output-disabled --require-imu
```

必須成功退出；失敗就依第 8 步關電、處理原因。名單仍須符合本頁 RL 規格，不能直接接多腿 FSM 的結果。

接著關 relay、保留感測器，統一從下面的只讀驗收開始：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool sensors
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
```

確認 `digital=true`、`signal=true`、`power=false`，重新通過第 5 步。這個接續方式保留通訊，但仍會重新驗收 policy 輸入；不需要重跑 power sequence。

### 兩種入口從這裡合流

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

<a id="motion-test"></a>

## 第 7 步：三秒 fixed-forward 架空測試

先再跑一次第 5 步 Preflight，並確認所有 FSM、servo probe 與只讀 launch 都已停止。

### 7.1 開 relay 並啟動 active stack

```bash
(
  set -e
  ros2 run redrhex_lowlevel_bridge rinbo_power_tool relay --confirm-relay
  ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
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

<a id="stop"></a>

## 第 8 步：停止與收工

RL launch 還在運行時先執行：

```bash
ros2 topic pub --once /redrhex/enable_policy std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /redrhex/enable_motors std_msgs/msg/Bool "{data: false}"
timeout 3s ros2 topic echo /rinbo/motor_output_enabled --once
timeout 3s ros2 topic echo /redrhex/lowlevel_output_enabled --once

ros2 run redrhex_lowlevel_bridge rinbo_power_tool off
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
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

## 查表：腿部與模型順序

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
| Disabled-leg mismatch | 停止上機；依實際故障狀態準備一致設定與模型，再重新封裝，不能為通過檢查而解除故障腿 |
| 卡在 `INIT_STAND` | 查 output ack、healthy-leg mapping、位置誤差與速度；不要先放寬 tolerance |
| `POLICY_RUN` command 仍為 0 | 檢查 state、`commands.profile` 與 `fixed_forward_vx` |
| 一啟用就過流／欠壓 | 立即停機；檢查卡滯、方向、供電與 channel mapping |
| 動作不像模擬 | 先驗證資料 contract；要改善落地步態，需 velocity estimator 或同 sensor/dropout contract 重訓 |

## 設定與程式在哪裡

| 要查什麼 | 位置 |
|---|---|
| 模型啟動與設定檢查 | [redrhex_policy_bringup.launch.py](../src/redrhex_rl_controller/launch/redrhex_policy_bringup.launch.py) |
| 真機控制主流程 | [rl_controller_node.py](../src/redrhex_rl_controller/redrhex_rl_controller/rl_controller_node.py) |
| 感測資料轉成模型輸入 | [observation_builder.py](../src/redrhex_rl_controller/redrhex_rl_controller/observation_builder.py) |
| 執行 ONNX | [policy_onnx_runner.py](../src/redrhex_rl_controller/redrhex_rl_controller/policy_onnx_runner.py) |
| 模型輸出轉成馬達目標 | [action_decoder.py](../src/redrhex_rl_controller/redrhex_rl_controller/action_decoder.py) |
| 狀態與安全限制 | [state_machine.py](../src/redrhex_rl_controller/redrhex_rl_controller/state_machine.py)、[safety_filter.py](../src/redrhex_rl_controller/redrhex_rl_controller/safety_filter.py) |
| 檢查、比對、封裝 | [scripts/](../src/redrhex_rl_controller/scripts)、[preflight_check.py](../src/redrhex_rl_controller/redrhex_rl_controller/preflight_check.py) |
| 訓練端錄製真實軌跡 | [golden_recorder.py](../src/redrhex_rl_controller/redrhex_rl_controller/golden_recorder.py)；需整合進實際模擬 play loop |
| RL 設定範本 | [controller config/](../src/redrhex_rl_controller/config)、[bridge config/](../src/redrhex_lowlevel_bridge/config) |
| Rinbo 訊息轉換 | [rinbo_ros_backend.py](../src/redrhex_lowlevel_bridge/redrhex_lowlevel_bridge/rinbo_ros_backend.py) |
| 最終 sbRIO 通訊橋接 | [rinbo_ros_bridge.cpp](../src/rinbo_ros_bridge/src/rinbo_ros_bridge.cpp) |
| 模型腿部規格 | [degraded_mode.py](../src/redrhex_rl_controller/redrhex_rl_controller/degraded_mode.py)；日常 FSM 名單用[腿部管理](orin_multileg_operation.md) |

RL 套件仍保留 `rinbo_leg_mask.py` 的舊跨設定工具；它預期舊 FSM YAML 格式，不是目前 `rinbo_legs` 多腿名單的管理入口。
