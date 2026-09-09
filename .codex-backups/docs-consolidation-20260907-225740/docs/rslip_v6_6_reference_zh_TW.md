# R-Slip 實驗操作流程

v6.6｜2026-09-07

現場：Orin `192.168.30.8`、sbRIO `192.168.30.254`、ROS domain `99`。
先固定機器人並移開腿部周圍的人員與線材。任何步驟報錯就先處理，不接著執行下一步。

本次 v6.6 將 Windows 局部重試流程擴充到 Standing、Tripod，沿用現場既有 Orin 程式與設定。Windows 不傳入 FSM 設定檔；多腿屏蔽功能以 Orin 實際部署版本為準。早前的 [Orin 改造 prompt](給Orin-GPT-多腿屏蔽改造.md) 留作參考，本次未核對其部署狀態。

## Part A：雙擊執行（平常使用）

### 自動查詢 sbRIO 與 Jetson IP

1. 電腦連上實驗室網路或機器人 Wi-Fi，等 sbRIO、Jetson 完成開機。
2. 雙擊本資料夾的 `查詢 sbRIO Jetson IP.cmd`。
3. 讀取 `SBRIO_IP=...` 與 `JETSON_IP=...`；按 Enter 關閉視窗。

工具使用 Windows 內建 PowerShell 與 OpenSSH，不需要輸入設備密碼。先驗證設定檔常用 IP，再視需要掃描目前啟用介面上的 `192.168.x.0/24` 網段；不支援其他網段或非 /24 網路。Jetson 以 `app\RSlip-SiteConfig.psd1` 的 SSH 指紋辨識；sbRIO 以 SSH 指紋，或已知完整 MAC 加即時 ping 回應辨識。SSH 探測僅交換主機金鑰，不執行遠端命令、不啟動服務、不上電；暫存金鑰檔用完即刪除，不修改個人 SSH 設定。

`NOT_FOUND` 表示本次無法從這台電腦驗證該設備，不代表設備一定關機。請檢查開機、網路線、Wi-Fi 隔離與 SSH 服務；若重灌導致指紋改變，需先核實新身分。多個已驗證位址會全部列出。本工具每次重新查詢，不將舊 IP 當成即時結果，也不自動修改主啟動器設定；目前主啟動器仍限制現場固定 IP。

若其他程式需要取得結果，可在專案根目錄執行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\app\Find-RSlipIps.ps1 -Json
```

JSON 包含查詢時間與每台設備的 IP 陣列、狀態、驗證依據。兩台皆找到時結束碼為 `0`；部分或全部未找到為 `1`；工具執行錯誤為 `2`。命令列加 `-NoPause` 可取消結尾等待。

### 啟動步驟

| 順序 | 執行檔 | 完成後 |
|---|---|---|
| 1 | `1 啟動通訊 R-Slip.cmd` | 通訊就緒、三個 Terminal 開著；不上電 |
| 2 | `2 開啟動力 R-Slip.cmd` | Digital → Signal → Relay；不執行動作 |
| 3 | `3 Calibration R-Slip.cmd` | Calibration DONE，動作程序退出；運行中再按同檔可要求結束 |
| 4 | `4 Standing R-Slip.cmd` | 第一次啟動並保持；第二次雙擊結束 |
| 5 | `5 Tripod R-Slip.cmd` | Standing 結束後啟動；第二次雙擊結束 |
| 完成 | `停止 R-Slip.cmd` | 停止動作、關閉動力，再停止通訊服務 |

第 1 步的三個視窗分別是：`R-Slip sbRIO`（core＋FPGA）、`R-Slip Orin1 Bridge`、`R-Slip Orin2 Manual`。
同一次實驗已開著的視窗不重複開；關掉的視窗可按第 1 步補開。關閉任何視窗不會停止遠端服務或動作。第 4、5 步停止動作後仍有動力，最後仍需按「停止」。

### 重測，不必全部重開

| 想做什麼 | 操作 |
|---|---|
| 補開 Terminal | 按第 1 步；核對通訊後保留現有上電、校正與動作狀態 |
| 已上電又按第 2 步 | 沒有動作正在運行時，已上電只做回讀；尚未上電仍會正常上電 |
| 再做 Calibration | 結束目前 Standing／Tripod 後，按第 3 步 |
| Calibration 失敗後重試 | 等原視窗顯示「可重試」，修正原因後直接按第 3 步 |
| 中途結束 Calibration | 再按第 3 步送出結束要求；等原視窗確認，之後再按才重新校正 |
| 誤關 Calibration 視窗 | 再按第 3 步先恢復並結束殘留校正；確認「可重試」後再按一次 |
| 再做 Standing | Calibration、Standing 或 Tripod 完成後，按第 4 步 |
| 再做 Tripod | Standing 或 Tripod 完成後，按第 5 步 |
| 整套重新開始 | 按「停止」，成功後從第 1 步開始 |

Standing／Tripod 正在運行時，再按**同一個動作檔案**是結束，不是另外啟動一份。重做 Calibration 後需再完成 Standing，才能進 Tripod。第 1 步可在 Standing／Tripod 運行時補開視窗，但不會停止動作。

### Calibration 失敗與取消：保留通訊的重試流程

原先 Calibration 的錯誤處理一律執行完整 rollback：停止動作 → 確認馬達停用 → 關電 → 停 Bridge／FPGA／core。因此失敗後必須從第 1 步重來；清理途中若關掉視窗，還會留下未完成的 Rollback 狀態。這是錯誤處理策略，不是每個 CMD 都清空記憶體；狀態存在 `%APPDATA%\RSlipLauncher\last-session.json`，互斥鎖只負責避免同時操作。

v6.5 對 Orin 明確回報的 Calibration 失敗、未完成就退出、等待超時，以及操作者取消，先嘗試以下恢復：

1. 用本次紀錄的 boot ID、PID、啟動時間與執行檔核對程序，再送 SIGINT；已退出的同一次程序也可處理。
2. 等待既有唯讀探針確認馬達輸出為 false，且 `/motor/command` 發布者為零。
3. 重新確認 core／FPGA／Bridge 身分、連線，以及 Digital／Signal／Relay 開啟、電壓與電流在既有門檻內。
4. 清除先前校正與 Standing／Tripod 的完成結果，進入 `CalibrationRetryReady`。保留 core、FPGA、Bridge 與動力，不重送上電指令；由操作者下一次按第 3 步才重新校正。

**「可重試」代表校正程序已結束，不代表動力已關閉，也不代表故障原因已消失。** 要碰機構、調線或結束實驗，先按「停止」。L1 的 position reset timeout 等故障仍需處理；Windows 不修改 Orin 的校正條件或屏蔽名單。

運行中再按第 3 步，第二個視窗只送出綁定本次程序身分的結束要求，原監控視窗負責清理；舊要求不會取消下一次校正。剛開始、尚未寫入程序身分時可能仍顯示忙碌，待校正進入運行後再按。若原監控已被關閉，下次第 3 步取得鎖後只做恢復，不會直接啟動新動作。若在啟動紀錄完成前中斷，因身分不明會使用完整停止。

任何局部恢復檢查失敗，或監控遇到 SSH／PID 身分等不確定錯誤，仍走原本的完整停止。全系統「停止」要求優先，不會被局部取消清除。`Rollback`、`PowerUnknown` 或既有 `stop.request` 不會被硬改成可重試；先按「停止」完成清理。

正常成功仍是 Calibration DONE → 結束校正程序 → 確認馬達停用 → `CalibrationComplete`。只在真正完成後允許 Standing；重做校正後仍須完成 Standing 才能進 Tripod。第 1、2 步在 `CalibrationRetryReady` 也只核對既有服務／回讀，不重啟通訊或重送上電。

本次 Windows 修改以離線模擬檢查正常完成、失敗重試、取消、視窗中斷、停止搶占與恢復失敗。實機驗收時，在原有固定機器人的條件下，核對重試前後三個通訊 PID 保持一致，且下一次校正仍由 Orin 完成 rearm／active ACK 握手；離線測試不等於已驗證實際機構動作。

### Standing／Tripod 的局部重試（v6.6）

| 狀態／需求 | 下一個操作 | 保留的前置成果 |
|---|---|---|
| Standing 失敗／尚未到位就取消，已恢復為可重試 | 按 4；成功到位並正常結束後才能按 5 | Calibration |
| Tripod 失敗／啟動中取消，已恢復為可重試 | 按 5 | Calibration、已完成 Standing |
| 啟動監控視窗中斷，還沒記錄就緒 | 按原動作先恢復；確認後再按一次重跑 | 該動作先前完成的前置步驟 |
| 已就緒持續運行，想結束 | 再按原動作，保持既有正常停止流程 | 原有前置步驟；正常結束才記錄本次完成 |
| 運行中已故障／自行退出 | 按原動作檢查並恢復；確認可重試後再按 | 前置步驟，不將故障動作算成功 |
| 想重新校正 | 恢復／結束目前動作後按 3 | 重做 Calibration 會清除後續完成結果 |

啟動等待期間再按同檔，第二個視窗只送出綁定本次程序 boot ID／PID／start_ticks 的取消要求，由持鎖的原監控視窗執行清理。剛好已交接為持續運行或正在停止時，再次點擊可能顯示忙碌；此時不會假稱已接受無人處理的取消要求，等目前視窗完成再按即可。不同動作不能互相取消。

明確回報 SAFETY STOP、啟動超時、未就緒退出、運行後異常退出，會嘗試停止本次 exact PID（允許已退出）、檢查 motor-safe、核對通訊與 relay／電壓／電流，再記錄 `StandingRetryReady` 或 `TripodRetryReady`。不重送上電、不停止健康通訊服務、不自動重跑。若啟動時尚未取得可信程序紀錄就中斷，仍使用完整停止。

Standing 重跑前清除原 Standing／Tripod 成果，失敗不放行 Tripod。Tripod 恢復保留已完成的 Standing。Tripod 正常完成仍要求 `=== Fully stopped ===`；缺少該訊息時即使馬達已確認停用，也只能記錄「可重試」。Standing／Tripod 正常結束後亦檢查新日誌，若停止時才出現 SAFETY STOP，不冒充正常完成。

恢復不清除全系統 stop.request。SSH／身分不確定、馬達未停用、通訊／電源回讀異常或全系統停止要求，仍走完整停止。第 1、2 步接受兩種可重試狀態，只補視窗／核對電源，不重置校正。

**局部停止後動力仍在；要碰機構、調線或結束實驗，按「停止」。** 程式維持啟動就緒後交還控制的既有方式，不另建背景監控；運行中的後續異常在下一次按同一動作時檢查。Windows 沒有修改 Orin 的腿部設定、動作條件或保護。

離線驗證包含成功啟動與正常結束、故障／取消後重試、視窗中斷、停止時才出現錯誤、Tripod 缺少完成訊息、全系統停止優先，以及真實兩個本機程序的鎖競爭。實機驗收仍需在原有固定機器人的條件下，確認通訊 PID 保留、原動作正常 rearm，及同一按鈕能停止／重試。

初次使用／換密碼：`tools\更新登入資料.cmd`。
重複的舊啟動入口已封存；主目錄只保留五步驟與停止共六個按鈕。

### 故障腿由 Orin 管理

Windows 不保留 `BAD_LEG`／`ROBOT_CONFIG`、固定 L1／五腿判斷或 degraded log gate；啟動三個 FSM 時不傳 `--params-file` 或任何屏蔽參數。這是「不介入屏蔽」，**不是傳入空名單去覆蓋 Orin**。Bridge 的 `--params-file` 是通訊設定，仍保留。

Orin 改造完成後，三個 FSM 必須自行讀取 **Orin 唯一的有效設定**，由 Orin 決定多腿屏蔽名單、Calibration 是否完成，以及 Standing／Tripod 是否允許。Windows 只負責啟動程序與顯示 Orin 的結果，不替未校正的腿判定成功。

日常改名單的流程：停止目前動作 → 在 Orin 設定本次屏蔽腿（可多隻），或解除特定腿／全部解除 → 確認有效名單 → 重新 Calibration。再依 Orin 允許的條件進 Standing／Tripod。不要在動作中直接解除屏蔽。

多腿管理與自動載入以 Orin 現場實際部署版本提供的指令操作；不需在 Windows 另建設定。自動套用的是使用者明確指定的名單，不能把未知故障或 Calibration 超時自動當成應屏蔽的腿。

Power tool 與 FSM 是不同程式；Windows 不傳 `--disabled-leg`，也不假設 power tool 已讀取 FSM 的設定。Orin 改造時須確認兩者需要的設定整合，不讓 Windows 另建一套名單。

### 重開與復原

- 第 1 步重開：核對遠端實際程序；仍在運行就只補開缺少的視窗，不重啟服務、不重新上電、不清除校正狀態。
- 尚未有上電／動作紀錄，且確認兩台遠端通訊與動作程序都已不存在：停止直接完成；第 1 步也能自動清理這類舊 stop.request，再建立新的通訊 session。這包含先前因 Bridge 不存在而被誤標為 PowerUnknown 的未上電紀錄。
- 停止做到一半：再按「停止」。若同一次執行已留下完整的 all-off 回覆與 Bridge 正常退出紀錄，核對後會接著停止 FPGA／core，不再要求已退出的 Bridge 回覆一次。
- 部分服務仍在、沒有可信停止紀錄或連線失敗：先處理錯誤，不把它當作空白 session。
- 無法確認動力關閉：關閉供應器輸出或拔除動力電源，保留 Orin／sbRIO／網路；執行 `tools\實體斷電後清理 R-Slip.cmd` 並輸入 `OFF`，完成後從第 1 步開始。
- 不需要，也不要手動刪 `stop.request`。

錯誤碼：20＝網路，30＝sbRIO，40＝Bridge，50＝上電前檢查，60＝電源，70＝動作，80／90＝停止或清理。
看視窗最上方第一個錯誤；完整 log 在 `%APPDATA%\RSlipLauncher\logs\`。

v6.3 的本機通訊與上電前檢查改用單一唯讀 ROS 探針：同一個總期限內等端點、參數與資料到齊，不連續啟動多個 ROS CLI。DDS 身分暫未完整會有限等待；錯誤身分、不符參數或持續收不到資料仍會停止。Orin 動作程式若回報 SAFETY STOP，也不會由 Windows 忽略或自動重跑。

## Part B：全部手動開 Terminal

**本 Part 與 Part A 擇一使用。** 全手動啟動不會建立 Windows 五段狀態紀錄，不要接著按第 2～5 步；它們需要第 1 步建立的 session。
先完成前次停止，再開三個 Windows Terminal。每個區塊的指令都在標示的機器內執行。

### Terminal 1：sbRIO，啟動 core 和 FPGA

先登入：

```bash
ssh admin@192.168.30.254
```

在 sbRIO 執行：

```bash
export LD_LIBRARY_PATH=/home/admin/rinbo_sbRIO_ws/install/lib:/home/admin/kilin_sbRIO_ws/install/lib
export PATH=/home/admin/rinbo_sbRIO_ws/install/bin:/home/admin/kilin_sbRIO_ws/install/bin:$PATH
export TERM=xterm
export CORE_LOCAL_IP=192.168.30.254
export CORE_MASTER_ADDR=192.168.30.254:50051
pgrep -af 'grpccore|fpga_driver'
```

有舊的 core／driver 就先停止，不要重複啟動。沒有才執行：

```bash
nohup /home/admin/rinbo_sbRIO_ws/install/bin/grpccore </dev/null >/tmp/rslip-manual-grpccore.log 2>&1 &
sleep 2
netstat -ltn | grep ':50051'
cd /home/admin/rinbo_sbRIO_ws/rinbo_fpga_driver/build
sha256sum NiFpga_FPGA_POWER_RS485_v2.lvbitx
```

50051 應在監聽；bitfile SHA-256 應為
`78975be61bf8b65db6744835626fdb071e41b45c3d8d8cd29065cb0e21e762f7`。
符合才啟動：

```bash
nohup ./fpga_driver </dev/null >/tmp/rslip-manual-fpga_driver.log 2>&1 &
sleep 3
tail -n 40 /tmp/rslip-manual-fpga_driver.log
```

看到 `Session opened (Success)` 才繼續；`Open Failed` 或 `-63101` 不可往下。

### Terminal 2：Orin，啟動 Bridge

先登入：

```bash
ssh jetson@192.168.30.8
```

在 Orin 執行：

```bash
cd /home/jetson/rinbo_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=99
export CORE_MASTER_ADDR=192.168.30.254:50051
export CORE_LOCAL_IP=192.168.30.8
export CORE_IP=192.168.30.254
pgrep -af '/rinbo_ros_bridge'
```

沒有舊 Bridge 才執行，並保留此視窗：

```bash
ros2 run rinbo_ros_bridge rinbo_ros_bridge --ros-args \
  --params-file /home/jetson/rinbo_ros_ws/install/rinbo_ros_bridge/share/rinbo_ros_bridge/config/redrhex_safe.yaml \
  -p core_ip:=192.168.30.254
```

### Terminal 3：Orin，控制電源與動作

先登入：

```bash
ssh jetson@192.168.30.8
```

在 Orin 執行：

```bash
cd /home/jetson/rinbo_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=99
export CORE_MASTER_ADDR=192.168.30.254:50051
export CORE_LOCAL_IP=192.168.30.8
export CORE_IP=192.168.30.254
ros2 node list | grep -Fx '/rinbo_ros2_bridge'
timeout 8 ros2 topic echo --once /motor/state rinbo_msgs/msg/MotorStateStamped
timeout 8 ros2 topic echo --once /power/state rinbo_msgs/msg/PowerStateStamped
```

應只有一個 Bridge，且兩個 state 都有回傳。**到這裡是通訊完成、尚未上電。**

### 上電：仍在 Terminal 3，依序執行

先全部關閉：

```bash
timeout 25 ros2 run redrhex_lowlevel_bridge rinbo_power_tool off --wait-for-subscriber-s 8
timeout 8 ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
```

確認 digital、signal、power 都是 false，再開 Digital、Signal：

```bash
timeout 25 ros2 run redrhex_lowlevel_bridge rinbo_power_tool sequence --wait-for-subscriber-s 8
timeout 8 ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
timeout 6 ros2 topic echo --once /rinbo/motor_output_enabled std_msgs/msg/Bool
timeout 6 ros2 topic info -v /motor/command
```

應為 digital=true、signal=true、power=false；motor output=false，motor command Publisher count=0。
符合才開 Relay：

```bash
timeout 25 ros2 run redrhex_lowlevel_bridge rinbo_power_tool relay --confirm-relay --wait-for-subscriber-s 8
timeout 8 ros2 run redrhex_lowlevel_bridge rinbo_power_tool status
```

三項電源應都是 true；依 power tool 回報檢查電壓與電流。本機門檻為 18–30 V、最大電流小於 3 A、Relay 連續 3 個通過樣本。
任何命令失敗都先停止，不要直接重複上電。**到這裡上電完成、尚未執行動作。**

Relay 這一步只開 Relay，不再重新執行 Digital／Signal 的 sequence。

`--wait-for-subscriber-s 8` 讓真正送指令的 power tool 最多等 8 秒找到 Bridge，找到便立即繼續；不是固定等待，也不重跑上電命令。雙擊入口亦使用相同設定。若上電失敗但 rollback 已成功關電及停止，畫面會說明可從第 1 步重新開始，不再把原始 Off 錯誤當成停止失敗。

### Calibration → Standing → Tripod：仍在 Terminal 3

**只有 Orin 自動載入／多腿屏蔽改造完成後，才使用以下指令。** 先在 Orin 確認本次有效屏蔿名單，每次只執行一個；不從 Windows 傳入設定檔：

```bash
ros2 run rinbo_fsm rinbo_cali
```

等 `State: DONE`；若程序仍留在前景，按 Ctrl+C，等提示字元回來再執行：

```bash
ros2 run rinbo_fsm rinbo_standing
```

等 Orin 回報健康腿已 Standing，觀察完成按 Ctrl+C。程序結束後才執行：

```bash
ros2 run rinbo_fsm rinbo_tripod
```

等 `entering RUNNING`。結束時按 Ctrl+C，等 `Fully stopped` 與提示字元。
若 Orin 回報 SAFETY STOP，不進下一步。

### 實驗結束

回 Windows 雙擊 `停止 R-Slip.cmd`；停止器可尋找手動開啟的已知程序，不要求手動流程事先建立 Windows session。
停止不完整時按 Part A 的實體斷電後清理流程處理，不以關閉視窗代替停止。

## 本機預覽與版本備份

只看指令、不連機器人，可在 Windows Command Prompt 執行：

```bat
"1 啟動通訊 R-Slip.cmd" /DryRun
"2 開啟動力 R-Slip.cmd" /DryRun
"3 Calibration R-Slip.cmd" /DryRun
```

其他入口也接受 `/DryRun`。PowerShell 下在檔名前加 `&`。
完整離線回歸測試，在專案主目錄執行：

```powershell
pwsh -NoProfile -File app\tests\Run-All.ps1
```

離線測試不會實際上電或執行機器人動作；測試通過不等於已完成實機全流程驗證。

主目錄只留常用六個操作按鈕與操作卡。必要程式與測試在 `app`；少用的維護工具與本手冊在 `tools`。
歷史程式、舊測試、修改前的停止紀錄已集中到 `archive/backups/`，重複的舊啟動入口也移出主目錄。這些檔案不參與執行，保留供還原；不要只拿舊設定覆蓋新版程式。本文件中的 Windows 路徑均相對於專案主目錄。
