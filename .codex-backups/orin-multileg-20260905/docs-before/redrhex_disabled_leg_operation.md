# RedRHex 单腿屏蔽操作手册

这份文件用于在 `Calibration`、`Standing`、`Tripod` 与 ONNX policy 流程之间，统一查看或更换被屏蔽的实体腿。合法腿名只有：

```text
L1  L2  L3  R1  R2  R3
```

真机模式一次最多屏蔽一腿。空 mask 代表严格六腿模式，不代表“忽略所有故障”。

> 屏蔽是启动时硬件 contract，不是运行中热切换。必须先让 motor output 为 `false`、关闭 relay，并退出所有 Cali／Standing／Tripod／RL controller／low-level bridge，再修改配置和重启。不要使用 `ros2 param set ... hardware.disabled_legs ...`；节点内部的安全 mask 是启动时建立的，运行中修改参数显示值不会安全地重建所有控制与保护状态。

## 日常最快操作

停机、确认两层 motor output 都是 `false`、关闭 relay、退出所有相关控制节点，并实体隔离坏腿的 Main Drive 与对应 Servo 后，日常只需记住：

```bash
# 屏蔽 L2
ros2 run redrhex_rl_controller rinbo_leg_mask auto L2

# 解除屏蔽，恢复严格六腿模式
ros2 run redrhex_rl_controller rinbo_leg_mask auto none

# 不输入腿名，打开互动选择菜单
ros2 run redrhex_rl_controller rinbo_leg_mask auto
```

`auto` 会显示“当前 mask → 目标 mask”和实体腿／Main／Servo mapping，再要求现场确认。确认后，它会检查 ROS graph、motor-command publisher 与 relay 状态，一次更新 FSM、Controller、Bridge 三份启动配置，并在写入后做一致性验证；验证失败会回滚。看到下面这类结果才代表配置完成：

```text
SUCCESS：三份启动配置已一致，当前屏蔽腿 = L2
```

`auto` **不会**停止节点、关闭 relay、实体隔离 Servo、启动程序或上电。`auto none` 会让第六腿重新取得输出资格，只能在六腿都已修复、重新校正并完成低功率验证后使用。

## 软件屏蔽能做什么

设置 `hardware.disabled_legs: [L2]` 后：

- Calibration 不等待 L2 的 Hall、位置或完成条件，其余五腿继续校正。
- Standing 与 Tripod 不把 L2 算入完成、position-error 和健康腿 gate。
- FSM 在发布边界强制 L2 Main Drive `enable=false`、PWM 为 0。
- Policy controller 对 L2 使用固定维度的 nominal observation，并把该腿 Main／ABAD action 归零。
- Policy low-level bridge 再次屏蔽该腿 Main Drive；其余五腿的电流、速度、stale telemetry、bus voltage、heartbeat 和 E-stop 保护仍然有效。

软件 mask 不能保证五腿 policy 步态稳定，也不能把六腿训练的 policy 自动变成可靠的五腿 locomotion policy。所有真机试验仍必须架空或可靠支撑，并完成对应 mask 的 golden、封装与 Preflight。

## Servo 必须实体隔离

Main Drive 有逐腿 software enable，但 SL1／SL2／SL3／SR1／SR2／SR3 没有逐颗 servo software enable；`servo_control_mode` 是全局值。坏腿即使已设 mask，对应 servo 仍可能收到 neutral/hold target。

因此，损坏或未验证的腿必须同时实体隔离：

- 对应 Main Drive；
- 对应 ABAD servo（L1 对应 SL1，R2 对应 SR2，以此类推）。

软件 mask、`rinbo_power_tool --disabled-leg` 和 E-stop 都不能替代正确的实体断电、线束隔离或驱动器隔离。

## 第一次准备

每个新 terminal 先载入 ROS 环境：

```bash
export REDRHEX_WS=/home/jetson/rinbo_ros_ws
source /opt/ros/humble/setup.bash
source "$REDRHEX_WS/install/setup.bash"
```

不设路径变量时，工具默认使用 `${REDRHEX_SITE_DIR:-~/redrhex_site}` 下的三份标准文件：

| 配置 | 默认路径 |
|---|---|
| FSM | `~/redrhex_site/rinbo_fsm_disabled_leg.yaml` |
| Controller | `~/redrhex_site/redrhex_policy_full_feedback_rig.yaml` |
| Bridge | `~/redrhex_site/lowlevel_bridge_full_feedback_rig.yaml` |

第一次执行 `auto` 时，若所选路径中只有 FSM 文件缺失，工具会从安装好的 `disabled_leg_template.yaml` 自动建立它，不会覆盖已有 FSM。Controller 与 Bridge 包含现场校正值，工具不会替你猜测或从模板自动建立；缺少任一份都会拒绝写入。请先依[完整 Sim2Real 手册的 site-local YAML 步骤](redrhex_sim2real_sbrio.md#第-2-步建立-site-local-yaml)建立这两份文件，且不要让日常工具修改仓库内模板。

若整套现场配置放在另一目录，只需设一次目录：

```bash
export REDRHEX_SITE_DIR=/path/to/redrhex_site
```

若三个文件不在同一标准目录，才分别设置 `REDRHEX_FSM_CFG`、`REDRHEX_SITE_CTRL` 与 `REDRHEX_SITE_BRIDGE`。逐文件变量优先于 `REDRHEX_SITE_DIR`。

新增或更新工具后，重新 build 并刷新 terminal：

```bash
cd "$REDRHEX_WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  rinbo_fsm redrhex_rl_controller redrhex_lowlevel_bridge
source install/setup.bash
```

## 日常首选与进阶命令

统一入口是：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask {auto|status|verify|commands|set|clear}
```

| 命令 | 用途 | 是否写文件 |
|---|---|---|
| `auto [LEG|none]` | 日常首选；互动或一条命令完成屏蔽／解除屏蔽与最终验证 | 是 |
| `status` | 显示 FSM、Controller、Bridge、Bridge hash 与 ONNX binding 状态 | 否 |
| `verify` | 以退出码检查 FSM 或完整 policy contract | 否 |
| `commands` | 依当前一致的配置打印 Cali／Standing／Tripod／Policy 命令 | 否 |
| `set LEG` | 进阶兼容接口；可 dry-run 或局部操作 | 是 |
| `clear` | 进阶兼容接口；恢复严格六腿模式 | 是 |

需要看完整参数时：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask --help
ros2 run redrhex_rl_controller rinbo_leg_mask auto --help
```

## 查看当前屏蔽腿

只检查三份磁盘配置：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask status
```

同时查看正在运行节点的参数与 relay 状态：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask status --live
```

机器可读输出：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask status --live --json
```

正常结果必须满足：

- `rinbo_cali`、`rinbo_standing`、`rinbo_tripod_rslip` 三段相同；
- Controller 与 Bridge 使用同一腿；
- Controller 内 pin 的 Bridge semantic hash 与实际 Bridge YAML 相同；
- 若节点正在运行，live mask 与磁盘配置相同；
- 若准备启动 policy，ONNX 的 SHA 与 semantic metadata 也必须匹配。

`status` 适合检查与排错，但它不是 policy 上机许可。Policy 仍须通过 `verify --scope policy` 和完整 `preflight_check`。

## 屏蔽一只腿

下面用 L2 举例。

### 1. 先停止并断电

1. 保持机器人架空或可靠支撑，现场人员握住实体 E-stop。
2. 停止 policy 和 motors，确认 `/redrhex/lowlevel_output_enabled` 与 `/rinbo/motor_output_enabled` 都是 `false`。
3. 依正常顺序关闭 relay：

   ```bash
   ros2 run redrhex_lowlevel_bridge rinbo_power_tool off
   ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
   ```

4. 退出 `rinbo_cali`、`rinbo_standing`、`rinbo_tripod_rslip`、`redrhex_rl_controller` 与 `redrhex_lowlevel_bridge`。唯一 final `rinbo_ros2_bridge` 可以保留到确认断电完成。
5. 实体隔离 L2 Main Drive 与 SL2。

`auto` 的确认问题是操作者对上述状态的明确确认，不会替你关闭电源。

### 2. 用 `auto` 一次完成

执行：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask auto L2
```

工具显示 `current -> L2` 与 `L2 = 左中, Main Revolute_23, Servo SL2` 后，确认现场状态才输入 `y`。若输入 `n`、直接 Enter 或按 `Ctrl+C`，不会修改配置。

需要明确的无互动操作时可用：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask auto L2 --yes
```

`--yes` 只跳过输入 `y` 的提示，代表操作者明确证明控制节点已停止、relay 已断电、目标坏腿已实体隔离，而且其余五腿没有已知损坏、可安全进入后续架空／低功率校正；它不会替你执行这些动作，也不会跳过在线安全检查。

旧流程中的 power 命令仍使用 shell 变量。`auto` 完成后会打印应该执行的 `export`；子进程无法替当前 shell 永久设置变量，因此需要在当前 terminal 执行：

```bash
export REDRHEX_BAD_LEG=L2
```

正常模式会：

- 检查是否仍有 mask-consuming node 在运行，有就拒绝；
- 检查 motor-command publisher；若仍有 publisher 就拒绝；
- 要求 `/power/state` 能明确报告 `power=false`；若为 `true` 或无法证明已断电，都拒绝修改；
- 若所选 FSM 文件尚不存在，从 FSM 模板自动建立；Controller 或 Bridge 缺失则拒绝；
- 同时更新 FSM、Controller 与 Bridge site YAML；
- 非空 mask 会把 Controller 的 `disabled_leg_observation_mode` 固定为 `nominal`，避免坏腿或缺失的 readback 进入 policy observation；
- 重算并写入 Bridge semantic SHA256；
- 在 Controller／Bridge mask 真正改变，或原 Bridge hash pin 已失配时，清空旧的 `policy.expected_sha256`，防止误用绑定旧腿或旧硬件语义的 ONNX；
- 同时把 `action.hardware_mapping_calibrated`、`rinbo.main_drive_calibrated`、`rinbo.abad_feedback_calibrated` 与 `rinbo.abad_command_calibrated` 复位为 `false`，强制重新取得腿相关的真机校正证据；
- 将原文件备份到 `~/.local/state/rinbo_leg_mask/backups/<时间戳>/`；
- 在同一个 transaction 中验证写入结果；失败时恢复原文件；
- 只改启动配置，不启动任何节点，也不上电。

如果只是修正 FSM 文件，而 Controller／Bridge 已是目标腿且 Bridge hash pin 正确，工具不会误清 policy 或上述校正确认。重复设置已经一致的同一腿也不会无故解除同一份 policy SHA；仍应运行 `verify` 确认现场状态。

### 3. 查看结果与下一步

`auto` 已经执行最终一致性检查。仍可用下面的独立命令复核并取得下一阶段命令：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask status
ros2 run redrhex_rl_controller rinbo_leg_mask verify --scope fsm
ros2 run redrhex_rl_controller rinbo_leg_mask commands
```

`commands` 只打印命令，不执行、不上电。出现 `MISMATCH`、`NOT MATCHED` 或 nonzero exit 时不要启动任何 FSM 或 policy。

## Calibration、Standing、Tripod

先用当前配置打印命令，避免手动打错路径：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask commands
```

三个 FSM 一次只能运行一个。标准顺序为：

```bash
export REDRHEX_FSM_CFG="${REDRHEX_FSM_CFG:-${REDRHEX_SITE_DIR:-$HOME/redrhex_site}/rinbo_fsm_disabled_leg.yaml}"

ros2 run rinbo_fsm rinbo_cali \
  --ros-args --params-file "$REDRHEX_FSM_CFG"
# 看到 DONE 后 Ctrl+C，等待 process 完全返回

timeout 3s ros2 topic echo /rinbo/motor_output_enabled --once
# 必须是 false

ros2 run rinbo_fsm rinbo_standing \
  --ros-args --params-file "$REDRHEX_FSM_CFG"
# 看到 HEALTHY LEGS STANDING；结束时 Ctrl+C 并等待返回

timeout 3s ros2 topic echo /rinbo/motor_output_enabled --once
# 必须是 false

ros2 run rinbo_fsm rinbo_tripod \
  --ros-args --params-file "$REDRHEX_FSM_CFG"
# 结束时 Ctrl+C，等待 Fully stopped 与 process 返回
```

每个程序启动后都必须看到类似：

```text
DEGRADED MODE: hardware.disabled_legs=[L2]
```

若日志没有目标腿、出现另一只腿或发生任何 `SAFETY STOP`，立即停止，不要接着运行下一阶段。被屏蔽腿不参与完成 gate，但其他五腿的 Hall、position error、3 A、18–30 V、PWM 与 stale telemetry 保护不会被取消。

准备开启 relay 时，power tool 必须使用同一腿名：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool sequence \
  --include-relay --confirm-relay --disabled-leg L2
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status \
  --disabled-leg L2
```

这里的 `--disabled-leg L2` 只告诉 power tool 在 relay/current gate 中排除 L2 的电流通道。它不是 mask setter，不会修改 FSM、Controller、Bridge 或 ONNX contract；日常配置入口是 `rinbo_leg_mask auto L2`。

## ONNX policy

Policy 的 disabled leg 是 artifact contract 的一部分。换腿以后，工具会让旧 policy fail-closed；不能只改 launch 的 `disabled_legs:=L2`，也不能复用绑定 L1 或严格六腿模式的 packaged ONNX。

正确顺序是：

```text
选择并设置 mask
→ 重新量测 Main／ABAD／hardware mapping，并只用真值恢复 calibration acknowledgement
→ 用同一 mask 产生真实 simulator golden
→ bridge_config_hash
→ compare TorchScript / ONNX / golden / ROS decoder
→ package_verified_policy
→ 把新 verified ONNX path 与 SHA 写入 Controller site YAML
→ rinbo_leg_mask verify --scope policy
→ preflight_check
→ 只读 launch
→ 分阶段 enable
```

换 mask 后，`rinbo_leg_mask` 会把 `hardware_mapping_calibrated`、`main_drive_calibrated`、`abad_feedback_calibrated` 与 `abad_command_calibrated` 复位为 `false`。先依真机量测重新完成这些校正；禁止为了继续流程直接手改回 `true`。之后依[完整手册的 policy 验证与封装步骤](redrhex_sim2real_sbrio.md#第-4-步離線驗證並封裝-policy)重新封装。完成新 artifact 和 site YAML 更新后执行：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask verify --scope policy
```

需要同时核对 live nodes 时：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask verify \
  --scope policy --live
```

只有返回 `0`、ONNX binding 为 `MATCH`，并且完整 Preflight 所有 `checks[].ok` 都是 `true`，才可继续。接着可用：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask commands
```

取得使用当前三份 YAML 和当前 mask 的 Preflight 与只读 launch 命令。它不会打印 active unlock，也不会替你开启 motors 或 policy。

未经当前 runtime 支援的 experimental sensor-v2、多输入／多输出或明确标记 `DIAGNOSTIC_ONLY`、`not_deployable`、`quality_rejected` 的 ONNX，不能因为 mask 一致就上机。

## 从一只坏腿切换到另一只

例如 L2 换成 R3：

1. 正常停止并确认两层 output 为 `false`。
2. relay off，并退出全部 mask consumer。
3. 实体隔离 R3 Main 与 SR3；同时确认 L2 是否已修复、校正和安全恢复，不能让未验证的 L2 因为换 mask 而重新带电。
4. 执行：

   ```bash
   ros2 run redrhex_rl_controller rinbo_leg_mask auto R3
   export REDRHEX_BAD_LEG=R3
   ```

   只有看到 `SUCCESS` 才继续。`auto` 会让旧 policy 和腿相关校正确认失效，但无法判断旧的 L2 是否已安全修复；这仍由操作者负责。

5. 重新完成对应阶段的测试；要使用 policy 时重新 compare/package/preflight。

系统只允许一个 disabled leg。若两腿同时损坏、未验证或无法实体隔离，停止实验；不要扩大 `hardware.max_disabled_legs` 绕过保护。

## 恢复严格六腿模式

`auto none` 不是“取消警告”。它会让六腿重新参与输出、完成 gate 和安全检查，所以必须先完成损坏腿的修复、Main／ABAD mapping、Calibration、方向、低功率单腿测试及全部六腿验证。

在 output、relay 和相关节点全部关闭后：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask auto none
```

确认问题会明确询问六腿是否已修复、重新校正并完成低功率验证。只有全部属实才输入 `y`。需要无互动操作时可用 `auto none --yes`；这里的 `--yes` 同时是操作者对六腿已经完成这些验证的明确证明，不是绕过验证的捷径。

完成后可复核：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask status
ros2 run redrhex_rl_controller rinbo_leg_mask verify --scope fsm
```

必须显示 `none`／`disabled_legs: []`。由单腿模式切回六腿同样改变 policy contract，工具会再次清空旧 policy SHA 并复位腿相关校正确认。旧五腿 artifact 会失效；重新取得六腿校正证据后，六腿 policy 必须重新封装并通过 `verify --scope policy` 与 Preflight。

严格六腿模式不应再给 power tool 传 disabled-leg 豁免；当前 terminal 可执行：

```bash
unset REDRHEX_BAD_LEG
```

## 互动菜单、`--yes`、`--offline` 与进阶接口

### 无参数互动菜单

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask auto
```

菜单会显示当前 mask，并列出六腿的位置、Main joint 与对应 Servo；选择 `7`／`none` 恢复严格六腿模式，选择 `q`、输入 `n`、直接 Enter 或按 `Ctrl+C` 都不会修改任何配置。

### `--yes` 的意义

`auto LEG --yes` 只跳过互动确认，适合明确的脚本化操作。对屏蔽腿，它代表操作者证明 output／relay 已关闭、目标腿已实体隔离，且其余五腿没有已知损坏、可安全进入后续架空／低功率校正；对 `auto none --yes`，它代表六腿均已修复、校正并验证。在线 ROS graph、publisher 与 relay 检查仍会执行。

### `--offline` 的意义

正常现场操作不要加 `--offline`。默认 `auto` 必须确认没有 mask consumer、没有 motor-command publisher，而且 `/power/state` 明确为 `false`；无法取得 relay 状态时也会 fail-closed。

只有 ROS graph 明确不可用、机器已实体断电，而且操作者已经从系统层面确认所有 writer 都停止时，才可显式跳过这些在线检查：

```bash
ros2 run redrhex_rl_controller rinbo_leg_mask auto L2 --offline
```

此命令仍会显示确认问题；只有同时加 `--yes` 才完全无互动。`--offline` 不会帮你断电，也不证明 relay 已关闭；它明确跳过 ROS graph、publisher 和 relay 三项在线检查，错误使用会移除重要的现场交叉检查。

### 进阶兼容 `set/clear`

日常使用 `auto`。需要先 dry-run、FSM-only 配置或既有脚本相容时，才直接使用底层接口：

```bash
# 只预览屏蔽 L2
ros2 run redrhex_rl_controller rinbo_leg_mask set L2 \
  --confirm-power-off --dry-run

# 实际屏蔽 L2
ros2 run redrhex_rl_controller rinbo_leg_mask set L2 \
  --confirm-power-off

# 预览恢复严格六腿模式
ros2 run redrhex_rl_controller rinbo_leg_mask clear \
  --confirm-power-off --confirm-all-six-verified --dry-run

# 实际恢复严格六腿模式
ros2 run redrhex_rl_controller rinbo_leg_mask clear \
  --confirm-power-off --confirm-all-six-verified
```

`set/clear` 不提供 `auto` 的互动菜单或自动建立缺失 FSM；确认参数与现场条件必须由调用者完整负责。

`--allow-partial` 只适合刻意准备 FSM-only 配置；使用它时 Controller 与 Bridge 不会被读取或写入。准备接 policy 前仍必须回到三份配置齐全并一致的标准流程。

## 腿部 mapping

Policy 顺序是 `[R1,R2,R3,L1,L2,L3]`，Rinbo 与 power 顺序是 `[L1,L2,L3,R1,R2,R3]`。不要把 policy index 0 误认为 L1。

| 腿 | 位置 | Rinbo index | Power ch | Policy index | Main joint | ABAD joint / servo |
|---|---|---:|---:|---:|---|---|
| L1 | 左前 | 0 | 1 | 3 | `Revolute_18` | `Revolute_17` / SL1 |
| L2 | 左中 | 1 | 2 | 4 | `Revolute_23` | `Revolute_22` / SL2 |
| L3 | 左后 | 2 | 3 | 5 | `Revolute_24` | `Revolute_21` / SL3 |
| R1 | 右前 | 3 | 4 | 0 | `Revolute_15` | `Revolute_14` / SR1 |
| R2 | 右中 | 4 | 5 | 1 | `Revolute_7` | `Revolute_6` / SR2 |
| R3 | 右后 | 5 | 6 | 2 | `Revolute_12` | `Revolute_11` / SR3 |

若实机标签、线束或 channel mapping 与表格不一致，保持所有 output disabled，依低功率逐腿 mapping 流程确认；不要猜。

## 常见错误

| 现象 | 处理 |
|---|---|
| `stop every mask-consuming node` | 仍有 FSM、RL controller 或 low-level bridge；正常停止后重试，不要直接加 `--offline` |
| `/power/state reports relay power=true` | 执行正常 power-off，取得 fresh acknowledgement 后重试 |
| `cannot prove relay power is off` | ROS 无法取得明确的 `power=false`；先排除通讯问题。只有已实体断电且所有 writer 已停止时才用 `--offline` |
| `configuration file does not exist` | 若缺 Controller／Bridge，先建立现场配置；只有缺 FSM 时 `auto` 才会从模板初始化 |
| `disabled-leg mismatch` | 三份 YAML 中至少一处不同；保持断电，用同一次 `auto LEG` 统一 |
| `ONNX mask/config binding: NOT MATCHED` | 旧 artifact 或 site YAML 已变化；重新 compare/package，并更新 path/SHA |
| `policy.expected_sha256 is empty` | mask 改变后工具刻意使旧 policy 失效；完成新封装后填入新 SHA |
| calibration acknowledgement 变回 `false` | mask 或 Bridge semantic pin 已改变；按真机量测重新校正，不可直接手改成 `true` |
| `controller ↔ bridge semantic hash mismatch` | 不可启动 policy；保持断电，检查是否绕过工具手改了 YAML |
| `clear requires --confirm-all-six-verified` | 这是进阶 `clear` 的保护；日常在六腿验证完成后使用 `auto none` |
| `DEGRADED MODE` 显示错误腿 | 立即停止该节点，检查 terminal 环境与参数文件 |

## 紧急停止

任何异音、异味、方向错误、过流、欠压、机架碰撞、stale telemetry、mask mismatch 或 output 状态异常时：

1. 立即按实体 E-stop 或切主电源；实体动作优先。
2. 若 ROS 仍可用，再执行：

   ```bash
   ros2 run redrhex_rl_controller estop_tool assert
   ros2 run redrhex_lowlevel_bridge rinbo_power_tool off
   ```

3. 不要先杀 final bridge；它负责最后的 watchdog、disable 与 power acknowledgement。
4. 等 motor output 与 relay 都确认关闭，再停止其余进程。
5. 排除原因后，从 `rinbo_leg_mask status`、实体隔离、配置一致性和 Preflight 重新开始；不要在同一次异常后直接重启动作阶段。
