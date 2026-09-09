# Jetson／sbRIO：從零開始控制每一隻腳

這份教學對應本工作區的 `rinbo_manual`。先做「只檢查」，再做懸空單腳 5° 測試，最後才嘗試速度與相位。所有指令都在 **Jetson 的 terminal** 執行。這是主馬達的支撐／懸空測試工具，沒有地面步行、平衡或接觸控制。

## 1. 先認識你現在的架構

```mermaid
flowchart LR
    P[你編輯的 YAML\n每腳角度／速度／相位] --> C[Jetson: rinbo_manual\n產生軌跡及回授控制]
    C -->|ROS 2 /motor/command| B[Jetson: rinbo_ros2_bridge]
    B -->|既有 core / gRPC / Protobuf| R[sbRIO 低階控制]
    R --> M[馬達與編碼器]
    M --> R
    R --> B
    B -->|ROS 2 /motor/state 與 /power/state| C
```

這是依現有 Bridge 的 `NodeHandler.h`、`Motor.pb.h` 與 `core::Publisher` 呼叫確認的程式路徑。**Jetson 上的節點用 ROS 2；到 sbRIO 使用既有 core 通訊橋接**。不需要另寫一套 UDP 協定，也不能假設 sbRIO 本身是 ROS 2 節點。

「節點」是一個做特定工作的程式；「topic」是節點傳遞訊息的通道。ROS 2 的發布／訂閱概念可參考 [Humble 官方入門](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)。本專案的重點如下：

| 名稱 | 用途 |
|---|---|
| `/motor/state` | 讀取編碼器位置、Hall 與伺服回授 |
| `/motor/command` | 發送六顆主馬達的 enable、direction、PWM 等命令 |
| `/power/state` | 讀取電源繼電器、電壓與電流 |
| `/estop` | 軟體緊急停止輸入 |
| `/rinbo/manual/status` | 新工具的各腳角度、速度、PWM 文字狀態 |

`LegCmd.voltage` 在此專案是既有 PWM 命令量，**不是你要填的電池伏特數，也不要把它當作百分比**。新程式替你把角度／速度換算為 PWM。

## 2. 腳的名稱、角度與速度怎麼看

六個主馬達依序是 `L1 L2 L3 R1 R2 R3`。L／R 對應現有程式的左／右組；1、2、3 的實際前中後位置要照你機器的接線標籤確認，程式碼本身不能證明實體接線。另有 `SL1…SR3` 六顆伺服，屬於不同的控制通道。

本工具沿用 `rinbo_cali.cpp` 的定義：

- 校正後 Hall 歸零位置為 0°；0° **不一定是腳垂直向下**。
- 360° 是一圈；正負號代表兩個方向。
- 左組：正方向對應原始 encoder 數值減少；右組：正方向對應數值增加。
- 55296 counts = 360°；`normalized_deg = 左右方向符號 × raw_counts × 360 / 55296`。
- `10 deg/s` 是每秒 10°，等於每分鐘約 1.67 圈；`360 deg/s` 才是每秒一圈，但這個測試工具不允許這麼高的設定。

舊 `rinbo_tripod.cpp` 使用 54984.83 counts／圈及不同方向寫法，新工具刻意以校正程式為準。因此第一次一定要觀察 **L2 小角度測試的實際方向與回授**，不要直接套用舊 Tripod 的數字。

`position`／`cycle` 的角度按一圈取模，走距離最短的方向。例如目前 359°，指定 1°，會前進 2°；指定 360° 等同 0°，不是額外轉一圈。恰好相差 180° 時選擇正方向。

## 3. 先編譯與只檢查，不會讓馬達動

開一個 terminal，每個新 terminal 都先載入環境：

```bash
cd /home/jetson/rinbo_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=99
```

99 是目前此 Jetson 的設定；現場 Bridge 若改用其他 domain，要保持一致。只需在 C++ 程式有改動時重新編譯；**改動你自己的 YAML 不需要重新編譯**：

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
  --packages-select rinbo_fsm --parallel-workers 1
source install/setup.bash
```

先看有效屏蔽設定：

```bash
ros2 run rinbo_fsm rinbo_legs status
```

撰寫本工具時，現場檔 `/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml` 為 revision 2、`disabled_legs: [L1]`。實際以 status 輸出為準；本次沒有改動它。所有範例都沒有啟用 L1。如果某個範例選中的腳也被你屏蔽了，檢查會拒絕，請從該範例刪除那隻腳。

檢查單腳範例：

```bash
ros2 run rinbo_fsm rinbo_manual \
  --plan src/rinbo_fsm/config/manual_single_leg.yaml
```

成功會出現 `CHECK ONLY: no ROS initialized, no motor/power commands sent.`。這只驗證檔案內容與現場屏蔽設定，**不表示已校正或機械測試通過**。

要在 Excel／試算表查看目標軌跡，可輸出 CSV：

```bash
ros2 run rinbo_fsm rinbo_manual \
  --plan src/rinbo_fsm/config/manual_phase.yaml \
  --preview /tmp/manual_phase_preview.csv
```

CSV 假設各腳初始角度都是 0°，不是讀取實機，也不是物理模擬。欄位有時間與各腳的度數／度每秒；沒有選的腳顯示 0，意思是未控制。檔案已存在時請換檔名，工具不覆寫。

## 4. 實機前的準備與校正

先把機身可靠固定／懸空，讓測試腳整圈轉動都不會撞到物品、線材或人，並讓操作者能立即切斷馬達電源。`off` 與程式結束會停用驅動，**不會鎖住關節承重**。

本工具的每腳選擇只控制主馬達。既有 `rinbo_cali` 會定位所有連接的伺服，再校正未屏蔽的主馬達；**即使手動範例只選 L2，前面的 Calibration 仍可能讓其他脚／伺服動作**。目前 L1 被屏蔽，不能因此推斷 SL1 已斷電；已知故障部件需依現場方式實體隔離。現有協定沒有逐顆伺服 disable。

如果 Windows 的既有流程已開啟 Bridge、電源並完成 Calibration，就沿用它，停止校正節點後接到第 5 節；不要重複開另一份 Bridge。手動入口不要求 Standing，且啟動時會使舊 Standing 紀錄失效。

若要從 terminal 從零操作，依序如下。

**Terminal A：只啟動一份現有 Bridge。** 先查實際網路設定：

```bash
ip -br addr
printenv CORE_MASTER_ADDR CORE_LOCAL_IP CORE_IP
```

Bridge 需要 `CORE_MASTER_ADDR`（sbRIO 的 core master 位址與 port）與 `CORE_LOCAL_IP`（Jetson 可被 sbRIO 連到的網卡 IP）。請從現場 launcher／網路設定取得真實值，不能直接假設舊文件的 `192.168.30.x` 仍適用。若環境尚未設定，以下兩行會讓你逐項輸入：

```bash
read -r -p 'sbRIO IP: ' MANUAL_SBRIO_IP
read -r -p 'Jetson 與 sbRIO 連線網卡的 IP: ' MANUAL_JETSON_IP
export CORE_MASTER_ADDR="${MANUAL_SBRIO_IP}:50051"
export CORE_LOCAL_IP="$MANUAL_JETSON_IP"
export CORE_IP="$MANUAL_SBRIO_IP"
ros2 run rinbo_ros_bridge rinbo_ros_bridge --ros-args \
  --params-file /home/jetson/rinbo_ros_ws/src/rinbo_ros_bridge/config/redrhex_safe.yaml \
  -p core_ip:="$MANUAL_SBRIO_IP"
```

50051 沿用現有 core master 設定，若 sbRIO 程式使用不同 port 要同步修改。Bridge terminal 保持開著。

**Terminal B：先開感測器，再確認回授。** 先執行第 3 節的環境載入：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool sequence
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
timeout 5s ros2 topic echo /motor/state --once
timeout 5s ros2 topic echo /power/state --once
```

這裡的 `sequence` 未帶 `--include-relay`，所以只開 digital／signal。若收不到資料，先修正 sbRIO 程式、網路、Bridge 或電源，不要繼續動作。

接下來這一行會開馬達電源繼電器，須在機身已固定、轉動空間已清空時由現場操作者執行：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool relay --confirm-relay
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
```

確認 `power=true` 及電壓／電流符合設定，再做既有校正：

```bash
ros2 run rinbo_fsm rinbo_cali
```

這會產生實際動作。等待 `State: DONE`，再按 Ctrl+C 讓 Calibration 程序退出。若是 `SAFETY STOP` 或 `[FATAL]`，先排除顯示的故障；不要手動建立完成紀錄。手動控制需要相同設定版本、同次開機的真實校正紀錄。

## 5. 第一次只讓 L2 移到 5°

先確認 `rinbo_cali`、`rinbo_standing`、`rinbo_tripod`、RL controller 與其他馬達命令工具都已退出。Bridge 應保持執行；不要在手動控制期間按 Windows 的動作按鈕。

```bash
cd /home/jetson/rinbo_ros_ws
ros2 run rinbo_fsm rinbo_manual \
  --plan src/rinbo_fsm/config/manual_single_leg.yaml \
  --execute
```

只有 **`--execute`** 會初始化控制節點。預期順序：

1. `WAIT`：檢查 Bridge 身分、握手與馬達／電源回授。
2. 以「所選主馬達 enable、PWM=0、servo mode=0」取得 Bridge 的 active ACK；ACK 代表 Bridge 接受命令，不是機械動作完成證明。
3. `RUN`：從當前回授位置平順移動，指定的 L2 到 5°，其他主馬達停用。
4. 每 0.5 秒顯示 L2 角度、速度、PWM；被屏蔽的腳顯示 `MASKED`，沒選的腳顯示 `OFF`。
5. `STOP: plan completed`，連續發送停用命令約 0.3 秒後退出。

5° 是相對 Hall 零點的絕對角度，不是每次執行再加 5°。若剛做完校正且仍在零點附近，這才是一個約 5° 的小動作。若機械曾被手動轉到很遠的地方，先查看回授；不要把這個範例誤認成「只允許移動 5°」。

**`duration_s` 是對齊、加速之後的運轉／保持時間，不是整個程式的總秒數。** 程式額外包含對齊、加減速、1 秒收尾保持與停用發送；開始時會列出預計動作總時間。大相位差配低速度，對齊可能需要數十秒。

如果腳不動但 PWM 已達 20，或位置誤差保護停止，先檢查故障原因、負载、方向與零點。20 是保守的初始上限，不能保證足以克服機械摩擦；不要直接把它提高到 80。增益沿用校正程式，尚未由本工具實機調校。

## 6. 自己寫每隻腳的控制設定

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

不同腳若設定不同速度，相位差就會隨時間改變。六腿版可參考 `config/manual_phase.yaml`；**固定角速度與相位的懸空旋轉不等於能行走的步態**，還缺接地段、擺腿段、伺服形狀與承重安排。

每次改好後都先檢查，再決定實機執行：

```bash
ros2 run rinbo_fsm rinbo_manual --plan /home/jetson/manual_legs.yaml
ros2 run rinbo_fsm rinbo_manual --plan /home/jetson/manual_legs.yaml --execute
```

設定啟動時讀一次，執行中不會自動重載，也不支援 `ros2 param set` 改動作。要換相位／速度，先停止、修改、重新檢查再執行。這樣一次動作有明確起點與有限時間。

| 欄位 | 可填範圍／意義 |
|---|---|
| `duration_s` | 必填；1～60 秒，不含对齊／加減速等時間 |
| `max_pwm` | 預設 20，1～80；實際還受現場 Standing 的 `max_pwm` 限制 |
| `max_speed_deg_s` | 預設 30，1～90；同時限制對齊與各腳設定速度 |
| `acceleration_deg_s2` | 預設 30，1～90；限制目標加速度 |
| `angle_deg` / `phase_deg` | −360～360，按一圈取模，需確認實際零點 |
| `speed_deg_s` | 正負皆可，絕對值不得超過 `max_speed_deg_s` |

屏蔽設定仍只有 `/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml` 一個來源。不要在手動 YAML 加 `disabled_legs` 或降低原本保護；拼错欄位、重複欄位、NaN／Inf、超限速度、選到屏蔽腳都會拒絕。不要為了通過範例檢查而解除故障腳屏蔽。

## 7. 停止與排除問題

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

## 8. 程式放在哪裡、如何修改

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

本次開發僅做編譯、隔離測試及不初始化 ROS 的計畫檢查，未替你上電或執行實機動作。正式使用前，從本頁的懸空單腳小角度測試開始。
