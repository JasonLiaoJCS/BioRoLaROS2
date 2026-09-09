# RedRHex Sim2Real 最簡操作說明書

> 最重要的一句：每顆新 ONNX 都先放到 `/home/jetson/redrhex_models/incoming/<新tag>/policy.onnx`；通過完整驗證後產生的 `/home/jetson/redrhex_models/policy_verified_<新tag>.onnx` 才能上機。

這份是日常入口，只告訴你「檔案放哪裡、下一步做什麼」。第一次校正或排錯才看[完整技術手冊](redrhex_sim2real_sbrio.md)；從 R-Slip 一鍵啟動接到 policy 時，最後走[R-Slip → Policy 操作流程](redrhex_sim2real_after_rslip.md)。要查看、切換或解除被屏蔽的腿，完整安全條件見[`rinbo_leg_mask` 單腿屏蔽操作手冊](redrhex_disabled_leg_operation.md)。不要手工在多份 YAML 之間改腿名，也不要用 `ros2 param set` 熱切換。

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask auto L2    # 範例：屏蔽 L2
ros2 run redrhex_rl_controller rinbo_leg_mask auto none  # 恢復六腿
ros2 run redrhex_rl_controller rinbo_leg_mask auto       # 互動選單
```

## 架空人工監看：最短實驗流程

這個流程只適用於機器人架空、現場有人握著 E-stop 的低速測試。實驗設定已固定為：`L1` 斷電停用、前進命令 `vx=0.22 m/s`、主驅動目標上限 `1 rad/s`、量測上限 `2 rad/s`、ABAD `0.18 rad`、單腿 `3 A`、匯流排 `18–30 V`、policy 最長運轉 `3 秒`。任何異音、異味、方向錯誤或機架碰撞都立刻按 E-stop。

目前收到的 sensor-v2 檔是 `[1,60,36] + [1,3] → [1,12] + [1,3]`，現有 runtime 只接受單一 `[1,56]` 或 `[1,280]` 輸入及單一 `[1,12]` 輸出，**現在不可啟動**。必須先由訓練端以同一 checkpoint 重新匯出並完成 verified package；不可用 reshape、補零或刪 output 代替。

### 1. 建立獨立實驗設定（只做一次）

在 Orin 2 終端機執行；`cp -n` 不會覆蓋既有 site 檔：

```bash
export REDRHEX_WS=/home/jetson/rinbo_ros_ws
export REDRHEX_SENSOR_V2_CTRL=/home/jetson/redrhex_site/redrhex_policy_sensor_v2_suspended_experimental.yaml
export REDRHEX_SENSOR_V2_BRIDGE=/home/jetson/redrhex_site/lowlevel_bridge_sensor_v2_suspended_experimental.yaml

mkdir -p /home/jetson/redrhex_site
cp -n "$REDRHEX_WS/src/redrhex_rl_controller/config/redrhex_policy_sensor_v2_suspended_experimental.yaml" "$REDRHEX_SENSOR_V2_CTRL"
cp -n "$REDRHEX_WS/src/redrhex_lowlevel_bridge/config/lowlevel_bridge_sensor_v2_suspended_experimental.yaml" "$REDRHEX_SENSOR_V2_BRIDGE"
```

不要修改或覆蓋目前 active 的 `redrhex_policy_full_feedback_rig.yaml` 與 `lowlevel_bridge_full_feedback_rig.yaml`。待 verified ONNX、SHA256、bridge hash、五隻健康腳的 Main／ABAD mapping 與 IMU alignment 都有量測證據後，才在上面兩份 experimental site YAML 填入真值並把 calibration acknowledgement 改成 `true`；`allow_enable` 仍保持 `false`，由啟動命令明確解鎖。

### 2. 每次上電後

先完成原本的 Calibration → Standing → Tripod Check → 回到 Standing。確認 L1 確實斷電、其他五腳方向正確、E-stop 可用，再執行：

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/rinbo_ros_ws/install/setup.bash
export REDRHEX_SENSOR_V2_CTRL=/home/jetson/redrhex_site/redrhex_policy_sensor_v2_suspended_experimental.yaml
export REDRHEX_SENSOR_V2_BRIDGE=/home/jetson/redrhex_site/lowlevel_bridge_sensor_v2_suspended_experimental.yaml

ros2 run redrhex_rl_controller preflight_check \
  --config "$REDRHEX_SENSOR_V2_CTRL" \
  --bridge-config "$REDRHEX_SENSOR_V2_BRIDGE" \
  --disabled-legs L1

ros2 run redrhex_lowlevel_bridge rinbo_power_tool sequence \
  --include-relay --confirm-relay --disabled-leg L1
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status --disabled-leg L1
```

Preflight 必須每一項 `checks[].ok` 都是 `true`；`sequence` 或 `status` 只要不是成功，也立即停止，不要啟動 policy。

### 3. 啟動 3 秒 policy 測試

```bash
ros2 launch redrhex_rl_controller redrhex_policy_bringup.launch.py \
  safety_profile:=custom \
  config:="$REDRHEX_SENSOR_V2_CTRL" \
  bridge_config:="$REDRHEX_SENSOR_V2_BRIDGE" \
  disabled_legs:=L1 \
  bridge_rinbo_allow_enable:=true \
  start_bridge:=true \
  use_fake_sensors:=false
```

另開一個已 source 同一 workspace 的終端機，依序執行：

```bash
ros2 topic echo /redrhex/state_machine_state
# 只有顯示 INIT_STAND 才繼續；Ctrl-C 停止 echo。

ros2 topic pub --once /redrhex/enable_motors std_msgs/msg/Bool "{data: true}"
ros2 topic echo /redrhex/state_machine_state
# 只有顯示 POLICY_READY 才繼續；Ctrl-C 停止 echo。

ros2 param get /redrhex_rl_controller commands.profile
ros2 param get /redrhex_rl_controller commands.fixed_forward_vx
# 必須分別是 fixed_forward 與 0.22。

ros2 topic pub --once /redrhex/enable_policy std_msgs/msg/Bool "{data: true}"
sleep 4
timeout 3s ros2 topic echo /redrhex/state_machine_state --once
timeout 3s ros2 topic echo /redrhex/lowlevel_output_enabled --once
timeout 3s ros2 topic echo /rinbo/motor_output_enabled --once
```

不要送鍵盤速度命令。啟動後最多 3 秒會自動回到 `INIT_STAND`；最後一個 state 必須是 `INIT_STAND`，兩個 output 都必須是 `false`。若未自動停止，立即 E-stop；正常結束再依本文件後面的「正常斷電」流程關閉。

## 你現在先做什麼

### 最新候選：2026-09-01 sensor-v2

Drive 新檔與同資料夾的 sidecar 已放在：

```text
/home/jetson/redrhex_models/incoming/sensor_v2_drive_1mNVkQYh_20260901/
  policy.onnx
  policy.onnx.json
```

| 檢查 | 結果 |
|---|---|
| ONNX SHA256 | `b1754a92cb2ea37623a793d127f56d3247f16ae295185854a8c8565081ff00f9` |
| Sidecar SHA256 | `8ae1e1111088f88c4ea755e278ab95ea6bf69382bfc45292da72f68bc8c7aa54` |
| `onnx.checker`／CPU 推論 | 通過；reference action 有限，CPU p99 約 `0.76 ms` |
| Contract hashes | Contract、feature layout、action、calibration hashes 與 sidecar 相符 |
| I/O | `[1,60,36] + [1,3] → [1,12] + [1,3]` |
| 現有 ROS runtime gate | 失敗：只接受一個 `[1,56]`／`[1,280]` input 與一個 `[1,12]` output |
| Golden evidence | 只有 4 個 random parity samples；recorded samples 是 `0` |
| 可否啟動 policy | **不可以** |

這顆比上一顆好：沒有 `quality_rejected`，而且 sidecar 已提供 36D feature 與 action contract。但 repo 尚無 sensor-v2 runner／60-frame builder／新 action decoder，sidecar 也沒有 filter/reset/L1-disabled 完整規則、同 checkpoint TorchScript、真實 recorded golden 與 training source。Sidecar 宣告 `hardware_ready=true`，卻與目前 site YAML 的五個 calibration gate 都是 `false` 相衝突，因此不能用它自動開啟 acknowledgement。

它必須繼續留在 `incoming/`；active site YAML 保持不變。不能只刪第二個 output、reshape、補零或把 sidecar 改名成 verified report。

### 上一顆 diagnostic

上一顆下載檔是：

```text
/home/jetson/redrhex_models/incoming/
  policy_sensor_v2_run10_seed42_DIAGNOSTIC_ONLY.onnx
```

判定結果：

| 檢查 | 結果 |
|---|---|
| SHA256 | `d89d30cc0faa1a3e0bee20e680f88fa38f68251ea2c8e33fc63a5bd87b40e3bd` |
| Metadata | `diagnostic_only_not_deployable`、`quality_rejected` |
| Inputs | `sensor_history [1,60,36]`、`command [1,3]` |
| Outputs | `actions [1,12]`、`base_velocity_estimate [1,3]` |
| 可否啟動 policy | **不可以** |

這顆檔案留在 `incoming/` 是正確的。不要把它改名成 `policy_verified_*`、不要寫入 site YAML，也不要靠刪掉第二個 output 或補零硬接。

它的 metadata 指向下列 checkpoint；請訓練端從它產生新的 quality-approved deployment bundle：

```text
logs/rsl_rl/redrhex_forward_v2_robust_ppo/
2026-08-23_20-49-36_sensor_v2_overnight_full_20260823_run10_f4_robust_ppo_seed42/
model_600.pt
```

目前 site YAML 也還沒完成：

```text
policy.onnx_path       = /home/jetson/redrhex_models/policy_verified.onnx（檔案不存在）
policy.expected_sha256 = 空白
Main / ABAD / IMU / hardware mapping calibration = false
```

所以現在可以做受限校正，或只測 Calibration／Standing／Tripod；不能做 RL policy 測試。

以後收到另一顆新 ONNX 時，不要覆蓋這顆 diagnostic 檔；直接從下一節建立新的 `<tag>` 資料夾。

## 只要記住這個資料夾規則

```text
/home/jetson/redrhex_models/
├── incoming/                         ← 新收到、尚未驗證的檔案
│   ├── *_DIAGNOSTIC_ONLY.onnx        ← 只留作分析，不能上機
│   └── <本次tag>/                    ← 一套完整的新 policy bundle
│       ├── policy.onnx
│       ├── policy.onnx.json           ← 若 exporter 有提供的 sidecar；不能取代下列 evidence
│       ├── policy.pt                 ← TorchScript export，不是原始 PPO checkpoint
│       ├── redrhex_golden_v2.npz     ← 真實 simulator play 產生
│       ├── training_git_sha.txt      ← 只放 training commit SHA
│       ├── training_env.py
│       ├── training_env_config.py
│       └── training_play.py
├── policy_verified_<本次tag>.onnx    ← 驗證與封裝成功後自動產生
└── policy_verified_<本次tag>.json    ← 比對報告
```

規則只有三條：

1. 新 `.onnx` 一律先放 `incoming/<本次tag>/policy.onnx`。
2. 只有 `.onnx` 或 `.onnx + sidecar` 都不足以上機；TorchScript、golden 與 training evidence 仍要齊全。
3. 只有 `policy_verified_<tag>.onnx` 才能寫入 site YAML。

`<本次tag>` 使用英文字母、數字、底線，不要空格；例如 `run11_seed42_v1`。

## 以後拿到新 ONNX：照這五步做

### 第 1 步：建立本次資料夾

假設新 policy tag 是 `run11_seed42_v1`：

```bash
export REDRHEX_ARTIFACT_TAG=run11_seed42_v1
export REDRHEX_BUNDLE_DIR="/home/jetson/redrhex_models/incoming/${REDRHEX_ARTIFACT_TAG}"
mkdir -p /home/jetson/redrhex_models/incoming
(
  set -e
  test ! -e "$REDRHEX_BUNDLE_DIR" || {
    echo "STOP：這個 tag 已存在，請換一個新 tag"
    exit 1
  }
  mkdir "$REDRHEX_BUNDLE_DIR"
)
```

如果看到 `STOP`，不要沿用舊資料夾；例如改成 `run11_seed42_v2` 後重做。

如果新 ONNX 已在 Jetson，填入它原本的位置，再複製並核對內容：

```bash
export REDRHEX_NEW_ONNX=/新檔案所在位置/新模型.onnx
(
  set -e
  test -f "$REDRHEX_NEW_ONNX"
  test ! -e "$REDRHEX_BUNDLE_DIR/policy.onnx" || {
    echo "STOP：目標已有 policy.onnx，請換新 tag"
    exit 1
  }
  cp -- "$REDRHEX_NEW_ONNX" "$REDRHEX_BUNDLE_DIR/policy.onnx"
  cmp -- "$REDRHEX_NEW_ONNX" "$REDRHEX_BUNDLE_DIR/policy.onnx"
  sha256sum "$REDRHEX_BUNDLE_DIR/policy.onnx"
)
```

如果檔案在 Google Drive，把連結交給 Codex，並說：

```text
請把這套 policy bundle 下載到
/home/jetson/redrhex_models/incoming/run11_seed42_v1/
不要修改 active YAML；先幫我做離線驗證。
```

### 第 2 步：確認 bundle 齊全

```bash
find "$REDRHEX_BUNDLE_DIR" -maxdepth 1 -type f -printf '%f\n' | sort
```

必須看到：

```text
policy.onnx
policy.pt
redrhex_golden_v2.npz
training_env.py
training_env_config.py
training_git_sha.txt
training_play.py
```

少任何一個就停，不要上電，也不要修改 YAML。

`policy.pt` 必須是可由 `torch.jit.load()` 載入的 **TorchScript export**，不是訓練時的 PPO checkpoint；完整比對會再次檢查它。

### 第 3 步：做第一個 ONNX gate

每個新 terminal 先執行：

```bash
export REDRHEX_WS=/home/jetson/rinbo_ros_ws
export REDRHEX_SITE_CTRL=/home/jetson/redrhex_site/redrhex_policy_full_feedback_rig.yaml
export REDRHEX_SITE_BRIDGE=/home/jetson/redrhex_site/lowlevel_bridge_full_feedback_rig.yaml
export REDRHEX_BAD_LEG=L1
source /opt/ros/humble/setup.bash
source "$REDRHEX_WS/install/setup.bash"
export PYTHONPATH="$REDRHEX_WS/src/redrhex_rl_controller${PYTHONPATH:+:$PYTHONPATH}"
```

如果換了 terminal，重新設定本次 tag：

```bash
export REDRHEX_ARTIFACT_TAG=run11_seed42_v1
export REDRHEX_BUNDLE_DIR="/home/jetson/redrhex_models/incoming/${REDRHEX_ARTIFACT_TAG}"
export REDRHEX_SOURCE_ONNX="$REDRHEX_BUNDLE_DIR/policy.onnx"
```

執行：

```bash
python3 "$REDRHEX_WS/src/redrhex_rl_controller/scripts/check_onnx_io.py" \
  "$REDRHEX_SOURCE_ONNX" \
  --expected-obs-dim 56 \
  --expected-action-dim 12 \
  --max-reference-action 1.5
```

必須看到：

```text
ONNX I/O check OK
```

沒有看到就停。不要繼續 compare、package、Preflight 或上電。

### 第 4 步：完整比對與封裝

先確認真機校正已經完成：

```bash
rg -n \
  "main_drive_calibrated|abad_feedback_calibrated|abad_command_calibrated" \
  "$REDRHEX_SITE_BRIDGE"
rg -n \
  "imu_alignment_calibrated|hardware_mapping_calibrated" \
  "$REDRHEX_SITE_CTRL"
```

上面五個值必須全部是 `true`，而且必須來自真機量測。只要有一個是 `false`，先停在第 3 步；依[完整手冊第 0～3 步](redrhex_sim2real_sbrio.md#操作順序)完成校正後再回來。不能先 compare/package，因為 verified artifact 會綁定這兩份校正後的 site YAML。

最省事的方法是把 bundle 位置交給 Codex：

```text
我的完整 policy bundle 在：
/home/jetson/redrhex_models/incoming/run11_seed42_v1/

請用 L1 disabled、現有 site controller/bridge YAML，依完整手冊第 4 步執行
Bridge hash、TorchScript/ONNX/golden compare、package 與 Preflight。
任何 gate 失敗都不要修改 active YAML，也不要上電。
```

要自己操作時，照[完整手冊第 4 步](redrhex_sim2real_sbrio.md#第-4-步離線驗證並封裝-policy)的命令執行。正確順序是：

```text
check_onnx_io
→ 五個真機校正 gate 全為 true
→ bridge_config_hash
→ 把 Bridge hash 寫進 site controller YAML
→ compare_onnx_with_torch
→ package_verified_policy
→ 把 verified path 與 SHA 寫進 site controller YAML
→ preflight_check
```

中間任何一步失敗都停止，不能跳過。

### 第 5 步：確認可以開始 Sim2Real

成功時必須同時存在：

```text
/home/jetson/redrhex_models/policy_verified_run11_seed42_v1.onnx
/home/jetson/redrhex_models/policy_verified_run11_seed42_v1.json
```

而且 site controller YAML 必須指向上面的 verified ONNX 與正確 SHA。最後執行：

```bash
ros2 run redrhex_rl_controller preflight_check \
  --config "$REDRHEX_SITE_CTRL" \
  --bridge-config "$REDRHEX_SITE_BRIDGE" \
  --disabled-legs "$REDRHEX_BAD_LEG"
```

只有所有 `checks[].ok` 都是 `true`，才算模型準備完成。

## 模型準備完成後，如何 Sim2Real

依你的起點選一條，不要混用：

| 你的起點 | 接下來做什麼 |
|---|---|
| R-Slip 已一鍵啟動，尚未執行舊 raw power command | 完整照[R-Slip → Policy 流程](redrhex_sim2real_after_rslip.md) |
| 一般從零啟動 | 完整手冊第 0 → 1 → 3.1 → 3.4 → 5 → 6 → 7 → 8 步 |
| 只測 Calibration／Standing／Tripod | 依[单腿屏蔽操作手册](redrhex_disabled_leg_operation.md)统一 mask，并运行其中的 Cali／Standing／Tripod 章节；不啟動 policy |

實際 Sim2Real 的固定順序是：

```text
Preflight 全通過
→ 唯一 final bridge 與真實 IMU
→ 只讀 launch，確認兩層 output=false
→ 開 relay
→ active launch，但 motors/policy 仍是 false
→ INIT_STAND 時啟用 motors
→ POLICY_READY 時確認 fixed_forward=0.22
→ 啟用 policy
→ 3 秒後自動回 INIT_STAND，兩層 output=false
→ power off
```

不需要鍵盤，也不要發布 `/cmd_vel`；進入 `POLICY_RUN` 時 controller 會自動給 `[0.22,0,0]`。

## 什麼情況一定要停

- 新 ONNX 只有單一檔案，沒有同 checkpoint bundle。
- 檔名或 metadata 有 `DIAGNOSTIC_ONLY`、`not_deployable`、`quality_rejected`，或 status 尚是 `pending_review`／未知值。
- 沒看到 `ONNX I/O check OK`。
- Preflight 任一 `checks[].ok` 是 `false`。
- 真實 IMU 啟動命令、publisher 或 frame 尚未確認。
- Controller、Bridge、FSM、artifact 的 disabled leg 不是同一個 `L1`。
- L1 Main／SL1 沒有實體隔離。
- 任一 output 在不該啟用時是 `true`，或出現過流、欠壓、stale、publisher mismatch、deadline miss。

這些情況不要靠改 YAML acknowledgement、放寬電流／PWM／action limit 或重新命名檔案繞過。

## 如果訓練端問你要匯出什麼

直接把下面文字傳給對方：

```text
請提供 quality-approved RedRHex deployment bundle，不要 DIAGNOSTIC_ONLY 或
quality_rejected artifact。

若 ONNX 有 redrhex_artifact_status / redrhex_quality_status，請分別使用
deployment_approved / quality_approved；不要使用 pending 或自訂未知狀態。

如果沿用現有 ROS runtime：ONNX 必須是 float32，單一 input [1,56] 或 [1,280]，
單一 residual action output [1,12]，並包含完整 RedRHex contract metadata。
請同時提供同 checkpoint 的 TorchScript、redrhex_golden_v2.npz、training Git SHA、
training_env.py、training_env_config.py、training_play.py。

如果正式 policy 必須使用 sensor_history [1,60,36] + command [1,3]，不能只 reshape
或刪除額外 output。請提供 36D feature order、60-frame history/reset、normalizer、
command/action 語意，以及 quality-approved ONNX、TorchScript、golden vectors，
讓部署端另外建立並測試 sensor_v2 runtime contract。
```
