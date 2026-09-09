# RedRHex ROS 2 Policy Controller

這個 package 負責把真機 feedback 組成 policy observation、執行 ONNX、解碼 action，並經安全狀態機發布 `/redrhex/motor_commands`。

## 現場操作文件

目前真機條件是「6 路 main encoder、6 路 ABAD encoder、IMU、架空、L1 已斷電」。操作入口是：

- R-Slip 已完成 Tripod Check：[`docs/redrhex_sim2real_after_rslip.md`](../../docs/redrhex_sim2real_after_rslip.md)
- 從零開始的最短流程：[`docs/redrhex_sim2real_quickstart.md`](../../docs/redrhex_sim2real_quickstart.md)
- 校正與 artifact 細節：[`docs/redrhex_sim2real_sbrio.md`](../../docs/redrhex_sim2real_sbrio.md)
- 查看、更換或清除故障腿 mask：[`docs/redrhex_disabled_leg_operation.md`](../../docs/redrhex_disabled_leg_operation.md)

請勿只靠 ONNX 的 `[1,56] -> [1,12]` shape 判定模型可用，也不要在目前條件下落地測試。舊版 fake-IMU、時間式 INIT_STAND、未驗證 policy 與全六腿測試流程已移除，避免與目前硬體狀態衝突。

## 目前支援的感測模式

| 模式 | Feedback | 用途 |
|---|---|---|
| `full_state` | encoder + 經 policy body frame 校正的 IMU；locomotion 尚需 base velocity estimator | 未來完整部署 |
| `encoder_only_rig` | 真實 encoder；base velocity/gyro 補 0；gravity 固定 `[0,-1,0]` | 只限架空台架驗證 |
| `full_feedback_rig` | 6 main + 6 ABAD encoder + 經校正 IMU；base linear velocity 補 0 | 目前架空 policy 主流程 |

`encoder_only_rig` 必須明確 acknowledgement，且永遠沒有 tilt/fall protection。它不能與 fake sensors 共用，也不能宣稱能驗證落地步態。

## 56 維 observation

順序固定為：

```text
0:3    base linear velocity
3:6    base angular velocity
6:9    projected gravity
9:15   sin(main position)
15:21  cos(main position)
21:27  main velocity / 2π
27:33  ABAD position / 0.61096
33:39  ABAD velocity
39:42  command [vx, vy, wz]
42:44  gait phase [sin, cos]
44:56  action from two policy inferences earlier, training clamp [-1, 1]
```

Policy joint order是 `R1,R2,R3,L1,L2,L3`；Rinbo order 是 `L1,L2,L3,R1,R2,R3`。L1 對應 policy index 3，不是 index 0。

`redrhex_policy_encoder_only_rig.yaml` 與 `redrhex_policy_full_feedback_rig.yaml` 都使用 `commands.profile: fixed_forward`。它只在 `POLICY_RUN` 設成訓練分佈下緣 `[0.22,0,0]`，並套用與 IsaacLab `play_forward_compat` 相同的 `1.0/0.04/0.30` 方程式；其餘 state 都是零。每次 rig `POLICY_RUN` 最多 3 秒。此模式忽略 `/cmd_vel`。

280 維模型的 history 排列固定為：

```text
[current, previous-1, previous-2, previous-3, previous-4]
```

## Policy artifact gate

真機模型必須同時綁定：

- observation/action/input-layout metadata；
- 內嵌 normalizer 與 training action clip；
- ONNX、TorchScript、ordered golden trajectory、controller config 的 SHA256；
- observation/deployment Python source、training env/config source 與 bridge semantic config 的 SHA256；
- training Git SHA；
- Torch/ONNX action parity；
- stateful ROS `ActionDecoder` 完整 command parity；
- 與 raw 56D/280D history 重建一致；fixed-forward bundle 每個 episode 必須精確使用 `[0.22,0,0]`、zero base linear velocity 與 disabled-leg nominal observation；external-command bundle 才要求四種 mode；
- 與 runtime 相同的 disabled-leg mask；
- packaged ONNX SHA256。
- Jetson CPU 重複推論至少 100 次的 p99；rig gate 為 `<=6 ms`，不能用單次推論冒充 benchmark。

`scripts/compare_onnx_with_torch.py` 可產生 machine-readable `redrhex_golden_v2` report；`scripts/package_verified_policy.py` 會重新計算 hash、重新執行 Torch/ONNX 與 decoder trajectory，不能用文字 report 或人工 flag 替 shape-only model 加蓋 stage-5 metadata。

先計算 bridge semantic hash（只排除臨時 `allow_enable` latch），再填入 controller YAML 的 `policy.expected_bridge_config_sha256`：

```bash
ros2 run redrhex_rl_controller bridge_config_hash \
  --bridge-config /path/to/lowlevel_bridge_full_feedback_rig.yaml
```

Canonical bringup launch 會在啟動 node 前重算實際選定的 bridge YAML（包含 effective disabled-leg mask）；不相符會直接拒絕啟動。Launch 不再提供 ABAD calibration/feedback 等 semantic override。

`IsaacLabGoldenRecorder` 是給訓練機 `play.py` 呼叫的 helper；它只接受 live simulator step，不會製造 synthetic golden，並將 training env/config/play source SHA 寫入 NPZ/ONNX。現有上游 `play.py` 尚未自動呼叫它，必須完成整合並錄出真實 episode 後才能執行 compare/package。

整合介面如下；`record_step()` 的 key 必須逐一對應 `golden_policy.VECTOR_FIELDS`（`joint_names` 在 constructor 給定），而 observation、policy action、simulator decoder target 都必須從當下真實 simulator step 讀取：

```python
from redrhex_rl_controller.golden_recorder import IsaacLabGoldenRecorder

recorder = IsaacLabGoldenRecorder(
    output_npz=golden_path, source_onnx=onnx_path, joint_names=joint_names,
    training_git_sha=git_sha, training_env_source=env_py,
    training_env_config_source=env_cfg_py, training_play_source=play_py,
    normalizer_embedded=True, policy_input_dim=policy_obs.shape[-1],
    disabled_legs=["L1"], command_profile="fixed_forward", fixed_forward_vx=0.22,
)
recorder.record_step(**live_simulator_fields)
# 多 episode、跨 warmup、phase coverage 都完成後才呼叫：
recorder.finalize()
```

IMU 校正採樣是唯讀的：

```bash
ros2 run redrhex_rl_controller imu_alignment_tool --duration-s 5 \
  --expected-publisher imu_driver
```

目前可見的 `/home/jetson/RedRhex/policy.onnx` 是舊 decoder contract，raw action 可達數百，因此會被刻意拒絕。

## 安全不變條件

- Controller、manual tool 同時存在時，bridge 拒絕 enabled command。
- Enabled command timestamp 必須單調，且 age 不得超過 100 ms。
- Low-level bridge 再次檢查 exact joint order、profile target limits、PWM、slew、power 與 current。
- `/motor/state`、`/power/state` 必須各只有一個指定 bridge publisher，source stamp 與 sbRIO sequence 都必須前進；duplicate/replay 不刷新 freshness 或 recovery。
- 任何 motor output 中斷後，都必須重新完成 INIT_STAND position/speed/stable dwell。
- 第一次 low-level output acknowledgement 必須在 150 ms 內到達。
- acknowledgement 必須源自 C++ final arbiter 真正接受的 gRPC active output；硬體 backend 不允許改成 Python self-ack。
- controller 與 low-level bridge 會用 runtime mask topic核對 disabled legs，L1 mask 不一致時禁止 enable。
- ROS→gRPC bridge 有 100 ms command watchdog；sbRIO/FPGA 的獨立 watchdog 與 servo mode 0 解除 torque 語意仍須在上電前實測。
- `hardware.disabled_legs: [L1]` 對該已隔離腿使用 nominal observation，並只忽略該腿的 fault/current/temperature/velocity readback；資料 shape 錯或任何健康腿 NaN/超限仍 fail-closed。bus voltage、heartbeat、E-stop 不豁免。
- Rinbo backend 目前不提供 motor temperature telemetry；diagnostics 會明示 `unavailable/inactive`，設定的 55°C 數值不能被當成有效保護。
- L1 main drive 由兩個輸出層強制為 0；SL1 沒有 per-servo software enable，損壞時必須維持實體隔離。
- ABAD command 的 zero/sign/counts-per-radian 未校正並明確 acknowledgement 前，low-level 拒絕 enable；`1000 counts/rad` 不能當成實測值。

## 開發驗證

```bash
cd /home/jetson/rinbo_ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select redrhex_rl_controller redrhex_lowlevel_bridge
colcon test-result --verbose
```

實際啟動、preflight、L1 degraded mode、encoder dry-run、policy 封裝與逐階段 enable 指令，請依 canonical Sim2Real 文件執行。
