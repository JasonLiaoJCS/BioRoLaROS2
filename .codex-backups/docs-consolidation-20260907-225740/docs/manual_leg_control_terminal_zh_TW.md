# 從開機到讓一隻腳動：完整操作順序

**接好實體電源 → 開 Bridge → 開板上電源 → 校正 → 執行單腳控制。**

前面的步驟不能省略。只有修改 `manual_single_leg.yaml`，機器人不會自己連線或上電。

下面使用 **兩個 Jetson 終端機視窗**：

| 視窗 | 做什麼 | 要不要保持開著 |
|---|---|---|
| A | 開 Bridge，讓 Jetson 能跟 sbRIO 說話 | 要，全程保持執行 |
| B | 開電源、校正、控制腳 | 指令依序執行 |

**先把機身可靠固定、腳懸空，清空轉動範圍，並準備好實體急停。** 確認電池／外接電源已接好，sbRIO 已開機且原本的低階控制程式正在執行。下面的軟體命令不能取代這些實體準備。

任何一步出現 `[FATAL]`、`SAFETY STOP` 或回報失敗，先停在那一步，不要繼續貼下一步指令。

## 1. 視窗 A：先開 ROS Bridge

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

這次實際查到 Jetson 有線網卡的 IP 是 `192.168.30.8`，所以已幫你填在 `CORE_LOCAL_IP`。sbRIO 的 IP 尚未確認，不能直接把舊範例的 `192.168.30.2` 當成實際位址。若不知道 sbRIO IP，先從原本成功連線的設定取得；它是這一步必填的資料。

`50051` 是專案原本使用的 sbRIO 通訊埠；若現場改過，要跟著改。99 是目前使用的 ROS 通訊群組。

**Bridge 啟動後，這個視窗保持執行，不要按 Ctrl+C。** 若原本已經開著一份 Bridge，沿用那份，不要再開第二份。

啟動時可能看到這一行，即使前面標示 `[ERROR]`：

```text
MOTOR SAFETY LATCHED: bridge startup requires consecutive fresh all-disabled commands
```

**這一行是程式每次啟動都會設定的馬達保護，不是 Bridge 啟動失敗。** 意思是「先禁止馬達出力，等控制程式完成安全確認」。後面的校正程式會自動送出所需的連續停用命令；不要自己發馬達命令來強制解除。

只看到這一行，還不能證明 sbRIO 已連上。`CORE_IP=192.168.30.2` 也只表示你填了這個地址，不表示連線成功。仍需在下一步確認電源回讀和腳的位置資料。

如果看到 `Rinbo ROS2 Bridge is killed`，表示 Bridge 已退出，需要重新啟動才能繼續。退出時的 `requested ... all-off power packets` 只表示程式已要求關電，是否真的關閉仍需確認。

若連不到 sbRIO，先確認它的實際 IP、實體供電、網路線，以及 sbRIO 原本的通訊／低階程式是否正在跑。不要因為 Bridge 視窗有印字，就繼續嘗試校正。

## 2. 視窗 B：開電源

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

## 3. 視窗 B：校正，讓程式知道哪裡是零點

**這一步會讓機器人動作，而且可能動到多隻腳和伺服，不只是 L2。** 目前 L1 主馬達被禁用，但這不代表 SL1 伺服已斷電；故障部件仍要按現場方式隔離。

貼上：

```bash
ros2 run rinbo_fsm rinbo_cali
```

等它顯示 **`State: DONE`**，表示校正完成。然後在 **視窗 B** 按 Ctrl+C，讓校正程式退出。

**視窗 A 的 Bridge 不要關，馬達電源也先保持開啟。**

若這次已用原本流程成功校正，設定沒有改過、也沒有發生安全故障，可以沿用完成紀錄。這個單腳工具不需要先執行 Standing 或 Tripod。

## 4. 視窗 B：先檢查單腳設定

第一次先用原本的 L2 範例，不急著改數字：

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

## 5. 視窗 B：讓 L2 動

確認其他動作程式都已退出，不要同時按 Windows 的 Standing／Tripod 等動作按鈕。貼上：

```bash
ros2 run rinbo_fsm rinbo_manual --plan src/rinbo_fsm/config/manual_single_leg.yaml --execute
```

最後的 **`--execute` 就是「真的執行」**。

預期會先顯示 `WAIT`，檢查通過後顯示 `RUN`：L2 慢慢移到 5°，保持一段時間，再停用馬達。沒列在設定裡的主馬達不會被這個程式啟用。

如果程式報錯或脚不動，先看顯示的原因；不要直接加大馬達出力。新工具已做軟體測試，但仍需這一步確認真機的方向與動作。

## 6. 停止與關電

想中途停下，在 **視窗 B** 按 **Ctrl+C**。這會停用驅動，但不會鎖住腳，馬達也可能因慣性再轉一下。發生異常時使用實體急停／切斷馬達電源。

動作程式退出後，在視窗 B 關閉板上電源：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool off
ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
```

確認電源關閉後，才到視窗 A 按 Ctrl+C 關掉 Bridge。

## 成功動過一次後，再學改數字

打開：[單腳設定檔](../src/rinbo_fsm/config/manual_single_leg.yaml)。

| 修改哪裡 | 改了會怎樣 |
|---|---|
| `L2` 改成 `R2` | 改控制 R2 主馬達 |
| `angle_deg: 5.0` 改成 `angle_deg: 10.0` | 目標位置改成 10° |
| `duration_s: 3.0` | 設定保持時間；先維持原值即可 |

其他數字先保持原樣，改完按 **Ctrl+S** 存檔。改這個檔案不用重新編譯。移動、加減速也需要時間，所以總時間會比 `duration_s` 長。

如果 Bridge 和電源仍開著、校正紀錄仍有效，只要重做第 4、5 步。若已關閉整套系統，就從前面的連線、上電流程開始。校正失效或安全故障後，需要重新校正。

---

## 學會單腳角度後，再試「速度」

打開：[速度設定檔](../src/rinbo_fsm/config/manual_velocity.yaml)。

這兩行就是你要看的地方：

```yaml
  L2: {mode: velocity, speed_deg_s: 10.0}
  R2: {mode: velocity, speed_deg_s: -5.0}
```

- L2：每秒轉 10°。
- R2：每秒往反方向轉 5°。
- 想慢一點，把 `10.0` 改成 `5.0`。
- 不想控制 R2，就刪掉 R2 那一整行。

第一次先觀察正負號在實際機器上對應哪個方向。**速度填 0 是保持位置；刪掉該腳才是停用它。**

先檢查：

```bash
ros2 run rinbo_fsm rinbo_manual --plan src/rinbo_fsm/config/manual_velocity.yaml
```

檢查通過且實機準備完成後，在同一行最後加上 `--execute` 再執行。

## 最後再學「相位」

**相位就是：兩隻腳在一圈裡，彼此錯開多少。**

把一圈想成時鐘的圓圈：兩根指針都指 12 點，是同一個位置；一根指 12 點、一根指 6 點，就是錯開半圈，也就是 180°。這是幫助理解的比喻，不是機器人的實際零點方向。

```yaml
  L2: {mode: cycle, speed_deg_s: 20.0, phase_deg: 0.0}
  R2: {mode: cycle, speed_deg_s: 20.0, phase_deg: 180.0}
```

意思是：**先把兩隻腳擺到相差半圈的位置，再讓它們用相同速度旋轉。**

| 兩隻腳的相位設定 | 意思 |
|---|---|
| 0°、0° | 同相位 |
| 0°、90° | 錯開四分之一圈 |
| 0°、180° | 錯開半圈 |

要維持同樣的相位差，兩隻腳的速度要相同。實際動作會有誤差。

可參考：[相位設定檔](../src/rinbo_fsm/config/manual_phase.yaml)。它目前選了五隻腳；第一次只想測兩隻，就只留下你要的兩隻腳設定。沒有通過禁用檢查的腳，請從此次設定移除。

先檢查：

```bash
ros2 run rinbo_fsm rinbo_manual --plan src/rinbo_fsm/config/manual_phase.yaml
```

相位測試會先轉動腳去對齊位置，可能轉很大一段；先確認整圈都有空間。**這是懸空旋轉測試，還不是放在地上走路的程式。**

## 平常操作，只記這個順序

**停止程式 → 修改設定 → Ctrl+S 存檔 → 只檢查 → 加 `--execute` 執行。**

執行中改檔案，不會立刻改變動作。腳名有 `L1 L2 L3 R1 R2 R3`，實體位置看接線標籤。若 L1 顯示禁用，先不要解除，範例已避開它。

這份工具控制的是讓腳旋轉的主馬達，沒有調整另一組伺服。

需要查錯誤訊息、重新編譯、通訊設定或控制原理時，再看[詳細參考資料](manual_leg_control_reference_zh_TW.md)。
