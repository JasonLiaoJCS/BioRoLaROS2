# 手動控制技術參考

[回到單腳教學](../manual_leg_control_zh_TW.md)｜[回到入口](../README.md)

平常用操作台即可。本頁供需要自行寫 YAML、追程式或分視窗下指令時查閱。直接執行 `rinbo_manual` 不會代替你建立 sbRIO 通訊、開電源或完成校正。

## 角度與方向定義

六個主馬達依序是 `L1 L2 L3 R1 R2 R3`。L／R 對應現有程式的左／右組；1、2、3 的實際前中後位置要照你機器的接線標籤確認，程式碼本身不能證明實體接線。另有 `SL1…SR3` 六顆伺服，屬於不同的控制通道。

本工具沿用 `rinbo_cali.cpp` 的定義：

- 校正後 Hall 歸零位置為 0°；0° **不一定是腳垂直向下**。
- 360° 是一圈；正負號代表兩個方向。
- 左組：正方向對應原始 encoder 數值減少；右組：正方向對應數值增加。
- 55296 counts = 360°；`normalized_deg = 左右方向符號 × raw_counts × 360 / 55296`。
- `10 deg/s` 是每秒 10°，等於每分鐘約 1.67 圈；`360 deg/s` 才是每秒一圈，但這個測試工具不允許這麼高的設定。

舊 `rinbo_tripod.cpp` 使用 54984.83 counts／圈及不同方向寫法，新工具刻意以校正程式為準。因此第一次一定要觀察 **L2 小角度測試的實際方向與回授**，不要直接套用舊 Tripod 的數字。

`position`／`cycle` 的角度按一圈取模，走距離最短的方向。例如目前 359°，指定 1°，會前進 2°；指定 360° 等同 0°，不是額外轉一圈。恰好相差 180° 時選擇正方向。

## 自己寫動作 YAML

先複製成自己的檔案，避免改到範例：

```bash
cp src/rinbo_fsm/config/manual_single_leg.yaml /home/jetson/manual_legs.yaml
nano /home/jetson/manual_legs.yaml
```

`nano` 用 Ctrl+O、Enter 存檔，Ctrl+X 離開。YAML 用空白縮排，不要用 Tab。你也可以用目前的 VS Code 編輯它。

### 指定某腳的角度

```yaml
duration_s: 3.0
max_pwm: 20.0
max_speed_deg_s: 10.0
acceleration_deg_s2: 10.0
legs:
  L2: {mode: position, angle_deg: 5.0}
  R2: {mode: position, angle_deg: -5.0}
```

兩隻腳一起平順移到指定角度、保持，結束後停用。只想測 L2，就刪除 R2 那行。**不要把不想動的腳設成 `position: 0`；那會要求它主動回零並保持。** 要停用請省略該腳，或寫 `L3: {mode: off}`。

### 各腳不同速度

```yaml
duration_s: 5.0
max_pwm: 20.0
max_speed_deg_s: 30.0
acceleration_deg_s2: 30.0
legs:
  L2: {mode: velocity, speed_deg_s: 10.0}
  R2: {mode: velocity, speed_deg_s: -5.0}
```

L2 從當前位置開始正轉，R2 從當前位置開始反轉。先加速到設定速度，維持 5 秒，再減速與停用。這是「位置積分目標＋速度回授」控制，不是 sbRIO 原生速度模式。速度為 0 仍會主動保持位置；`off` 才是停用。

### 相同速度、不同相位

```yaml
duration_s: 10.0
max_pwm: 20.0
max_speed_deg_s: 30.0
acceleration_deg_s2: 30.0
legs:
  L2: {mode: cycle, speed_deg_s: 20.0, phase_deg: 0.0}
  R2: {mode: cycle, speed_deg_s: 20.0, phase_deg: 180.0}
```

先讓 L2 到 0°、R2 到 180°，完成目標軌跡的共同對齊時間後一起旋轉。對齊交界及最後結束前，程式要求所選腳的角度誤差不超過 2°、速度不超過 5°/s；不符就停止並回報原因。勻速階段的目標形式是 `角度 = 初始相位 + 速度 × 時間`，實際程式還加上速度漸變的積分。兩腳同速就維持 180° 目標相位差；20°/s 對應一圈 18 秒，因此 180° 相差半圈／9 秒。實際角度有回授追蹤誤差，不保證完美同步。

不同腳若設定不同速度，相位差就會隨時間改變。多腳範例可參考 `config/manual_phase.yaml`；**固定角速度與相位的懸空旋轉不等於能行走的步態**，還缺接地段、擺腿段、伺服形狀與承重安排。

每次改好後都先檢查，再決定實機執行：

```bash
ros2 run rinbo_fsm rinbo_manual --plan /home/jetson/manual_legs.yaml
ros2 run rinbo_fsm rinbo_manual --plan /home/jetson/manual_legs.yaml --execute
```

設定啟動時讀一次，執行中不會自動重載，也不支援 `ros2 param set` 改動作。要換相位／速度，先停止、修改、重新檢查再執行。這樣一次動作有明確起點與有限時間。

| 欄位 | 可填範圍／意義 |
|---|---|
| `duration_s` | 必填；1～60 秒，不含對齊／加減速等時間 |
| `max_pwm` | 預設 20，1～80；實際還受現場 Standing 的 `max_pwm` 限制 |
| `max_speed_deg_s` | 預設 30，1～90；同時限制對齊與各腳設定速度 |
| `acceleration_deg_s2` | 預設 30，1～90；限制目標加速度 |
| `angle_deg` / `phase_deg` | −360～360，按一圈取模，需確認實際零點 |
| `speed_deg_s` | 正負皆可，絕對值不得超過 `max_speed_deg_s` |

屏蔽設定仍只有 `/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml` 一個來源。不要在手動 YAML 加 `disabled_legs` 或降低原本保護；拼錯欄位、重複欄位、NaN／Inf、超限速度、選到屏蔽腳都會拒絕。不要為了通過範例檢查而解除故障腳屏蔽。

## 直接執行時的停止與排錯

正常完成會先減速，再停用所有主馬達。**Ctrl+C 是立即要求停用，不會等減速完成**；它與 `/estop` 都不是機械煞車，馬達仍可能慣性轉動。程式退出不會自動關掉電源繼電器。

發生異常時先使用現場實體急停／切斷馬達電源。軟體停止可在另一個已載入環境的 terminal 執行：

```bash
ros2 run redrhex_rl_controller estop_tool assert
ros2 run redrhex_lowlevel_bridge rinbo_power_tool off
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
```

這些軟體命令仍依賴 Jetson／網路／Bridge 活著，不能取代實體停止方式。普通測試結束後也可用 `rinbo_power_tool off` 關閉電源。解除急停、重新上電之前先排除根因，依原有現場流程恢復，不要反覆重啟試圖清除故障。

| 訊息／現象 | 意義與處理 |
|---|---|
| 找不到 `rinbo_manual` | 重新編譯後 `source install/setup.bash`，確認在此工作區 |
| `Calibration ... required` | 尚未真實校正、重開機或設定版本已變；重新校正 |
| `L1 is disabled...` | 設定選中了屏蔽腳；從此次動作 YAML 移除該腳 |
| `Configuration/action is busy` | 其他 FSM 仍持有動作鎖；讓它正常停止並退出 |
| `handshake timeout` | Bridge 不存在、domain 不同、來源不符合或命令 topic 訂閱者不唯一 |
| `missing/stale /motor/state` | 回授中斷或太慢；檢查 sbRIO、感測器電源與網路 |
| `publisher ...`／`epoch ...` | 來源被替換、重複 Bridge 或 Bridge 重啟；檢查正在執行的節點 |
| `power ...`／`current ...`／`voltage ...` | 電源回授不安全；讀取具體原因，檢查電源與機械負載 |
| `tracking error >5000 counts` | 目標與實際相差約 32.6°；可能卡住、方向不對、速度／負載過大 |
| `alignment not reached`／`final pose not reached` | 對齊／收尾時仍超過角度或速度容差；檢查負載、方向、回授與控制增益 |
| `encoder jump/overspeed` | 編碼器突跳／意外重設或所選馬達速度異常 |
| `control loop delayed >100ms` | Jetson 排程延遲；不要一邊實機動作一邊大量編譯或跑重負載 |

狀態觀察請使用 `/rinbo/manual/status` 或 `/motor/state`。**動作中不要 `ros2 topic echo /motor/command`**：既有握手要求命令 topic 只有一個訂閱者（Bridge），增加 echo／錄製器訂閱會觸發保護。

正常手動結束仍可在同次有效校正後接續另一份手動計畫；安全故障會使 Calibration／Standing 紀錄失效。手動動過後不能拿舊 Standing 紀錄直接進 Tripod，需重新完成 Standing；若已有安全故障，先重新 Calibration。

## 控制計算與程式

| 檔案 | 功能 |
|---|---|
| `src/rinbo_fsm/src/rinbo_manual.cpp` | ROS 2 節點、可信回授、握手、PD＋前饋控制、停止 |
| `src/rinbo_fsm/src/manual_motion.hpp` | YAML 檢查、逐腳模式、共同時間軸、平滑對齊／加減速與最終輸出 |
| `src/rinbo_fsm/config/manual_*.yaml` | 單腳角度、獨立速度、相位示例 |
| `src/rinbo_fsm/test/test_manual_*.cpp` | 軌跡、輸出屏蔽、停止與 ROS 節點測試 |

控制迴圈每 10ms 嘗試更新一次，目標與實際位置先換成與校正一致的 counts，再計算：

```text
PWM = 0.35 × 位置誤差(counts)
    + 0.002 × 速度誤差(counts/s)
    + 0.02 × 目標速度(counts/s)
```

接著限制 PWM 幅值及變化速率（100 命令量／秒）。這些是既有校正增益的起始設定，不是每隻腳都已調校好的控制器。目標速度／加速度受限制，不代表馬達實際行為必然相同；真實機械仍需量測。

新入口繼承現場 `rinbo_standing` 區段的電源、回授來源時戳、握手與超時設定，沒有另建一份安全設定。軌跡使用單調時鐘；回授中斷、時鐘倒退、來源異常、電壓電流異常、誤差過大會鎖定停止。Bridge 原有命令超時停用機制保留，但 sbRIO／馬達失聯後的最終硬體行為仍需依現場驗證。

離線測試使用 localhost domain 231；不要在機器人動作時進行編譯與測試：

```bash
ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1 colcon test \
  --packages-select rinbo_fsm --executor sequential
colcon test-result --test-result-base build/rinbo_fsm --verbose
```


## 分視窗操作：不使用操作台時

前提：機身已固定且腳懸空，sbRIO 的 core／FPGA 已按現場流程啟動。視窗 A 保持 Bridge 運行，視窗 B 逐步操作。與中文操作台或 Windows 控制動作擇一使用。任何一步失敗就停在該步。

### 1. 視窗 A：先開 ROS Bridge

在 VS Code 點 **「終端機」→「新增終端機」**，貼上：

```bash
cd /home/jetson/rinbo_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=99

read -r -p '輸入 sbRIO 的 IP，然後按 Enter：' SBRIO_IP
```

輸入 sbRIO IP 並按 Enter 後，再貼下面這段開啟 Bridge：

```bash
export CORE_MASTER_ADDR="${SBRIO_IP}:50051"
export CORE_LOCAL_IP=192.168.30.8
export CORE_IP="$SBRIO_IP"

ros2 run rinbo_ros_bridge rinbo_ros_bridge --ros-args \
  --params-file /home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/config/redrhex_safe.yaml \
  -p core_ip:="$SBRIO_IP"
```

中途會停下來等你輸入 **sbRIO 的 IP**，請填平常連線到 sbRIO 用的位址，再按 Enter。不是填 Jetson 的 IP。

現場預設 Jetson 為 `192.168.30.8`、sbRIO 為 `192.168.30.254`；若網路變更，填實際位址並同步修改 `CORE_LOCAL_IP`。Bridge 有印出 IP 不代表連線成功。

`50051` 是專案原本使用的 sbRIO 通訊埠；若現場改過，要跟著改。99 是目前使用的 ROS 通訊群組。

**Bridge 啟動後，這個視窗保持執行，不要按 Ctrl+C。** 若原本已經開著一份 Bridge，沿用那份，不要再開第二份。

啟動時可能看到這一行，即使前面標示 `[ERROR]`：

```text
MOTOR SAFETY LATCHED: bridge startup requires consecutive fresh all-disabled commands
```

**這一行是程式每次啟動都會設定的馬達保護，不是 Bridge 啟動失敗。** 意思是「先禁止馬達出力，等控制程式完成安全確認」。後面的校正程式會自動送出所需的連續停用命令；不要自己發馬達命令來強制解除。

只看到這一行，還不能證明 sbRIO 已連上。`CORE_IP=192.168.30.254` 也只表示你填了這個地址，不表示連線成功。仍需在下一步確認電源回讀和腳的位置資料。

如果看到 `Rinbo ROS2 Bridge is killed`，表示 Bridge 已退出，需要重新啟動才能繼續。退出時的 `requested ... all-off power packets` 只表示程式已要求關電，是否真的關閉仍需確認。

若連不到 sbRIO，先確認它的實際 IP、實體供電、網路線，以及 sbRIO 原本的通訊／低階程式是否正在跑。不要因為 Bridge 視窗有印字，就繼續嘗試校正。

### 2. 視窗 B：開電源

再新增一個終端機，先貼：

```bash
cd /home/jetson/rinbo_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=99
```

接著，這行會依序開啟控制／感測器供電，最後開啟馬達電源：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool sequence --include-relay --confirm-relay
```

這裡的 `--include-relay` 很重要：**少了它，這個 sequence 指令不會開馬達電源。** `--confirm-relay` 表示你知道這次要開馬達電源。

指令成功結束後，確認電源狀態：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
```

要看到 `power=true`（或 JSON 顯示 `"power": true`）。如果仍是 false，就是馬達電源還沒開成功。

再確認收得到腳的位置資料：

```bash
timeout 5s ros2 topic echo /motor/state --once
```

應該會印出包含 `l1`、`l2`、`r1` 等欄位的資料。沒有資料就先處理 Bridge、sbRIO 或連線問題，不要進行校正。

### 3. 視窗 B：校正，讓程式知道哪裡是零點

**這一步會讓機器人動作，而且可能動到多隻腳和伺服，不只是 L2。** 實際禁用名單以 `rinbo_legs status` 為準；主馬達禁用不代表對應伺服已斷電。

貼上：

```bash
ros2 run rinbo_fsm rinbo_cali
```

等它顯示 **`State: DONE`**，表示校正完成。然後在 **視窗 B** 按 Ctrl+C，讓校正程式退出。

**視窗 A 的 Bridge 不要關，馬達電源也先保持開啟。**

若這次已用原本流程成功校正，設定沒有改過、也沒有發生安全故障，可以沿用完成紀錄。這個單腳工具不需要先執行 Standing 或 Tripod。

### 4. 視窗 B：先檢查單腳設定

先用 L2 範例；若 L2 也被屏蔽，請在自己的動作檔改選可用腿，不要解除故障腿屏蔽：

```bash
ros2 run rinbo_fsm rinbo_manual --plan src/rinbo_fsm/config/manual_single_leg.yaml
```

看到 `CHECK ONLY` 表示設定檢查通過。**這一行只檢查，不會讓腳動。**

範例裡這一段的意思是「讓 L2 移到 5°」：

```yaml
  L2:
    mode: position
    angle_deg: 5.0
```

5° 是相對校正零點的位置，不是每次都再轉 5°。剛校正完、腳還在零點附近，才會是小幅移動。零點不一定是腳垂直向下的位置。

### 5. 視窗 B：讓 L2 動

確認其他動作程式都已退出，不要同時按 Windows 的 Standing／Tripod 等動作按鈕。貼上：

```bash
ros2 run rinbo_fsm rinbo_manual --plan src/rinbo_fsm/config/manual_single_leg.yaml --execute
```

最後的 **`--execute` 就是「真的執行」**。

預期會先顯示 `WAIT`，檢查通過後顯示 `RUN`：L2 慢慢移到 5°，保持一段時間，再停用馬達。沒列在設定裡的主馬達不會被這個程式啟用。

如果程式報錯或脚不動，先看顯示的原因；不要直接加大馬達出力。新工具已做軟體測試，但仍需這一步確認真機的方向與動作。

### 6. 停止與關電

想中途停下，在 **視窗 B** 按 **Ctrl+C**。這會停用驅動，但不會鎖住腳，馬達也可能因慣性再轉一下。發生異常時使用實體急停／切斷馬達電源。

動作程式退出後，在視窗 B 關閉板上電源：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool off
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
```

確認電源關閉後，才到視窗 A 按 Ctrl+C 關掉 Bridge。
