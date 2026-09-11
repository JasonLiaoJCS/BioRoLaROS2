# 電源发布者交接與冪等開電 v2（2026-09-11）

本文件保留第一階段交付記錄。2026-09-11 已與 [Orin 原生 GUI API](orin_panel_api_v1.md) 一起正式部署；最新部署與測試以該文件為準。只改電源工具與 Bridge 的操作協定，不改 PWM、PID、L3 屏蔽或現場 revision 15 參數。

## 事故根因與證據界線

13:46–13:49 SSH timeout 是更早的網路故障；13:50 Core／Bridge 缺席時沒有可用關電通道，舊紀錄無法證明電源狀態。這不是本次 GID 修正可以修復的網路問題。

使用者提供 13:51:41 SensorOnly、13:51:44 Relay 成功 ACK。既有 guard 在接受 digital/sensors（relay-off sequence）後允許下一個發布者交接；因此第一次兩個 CLI 的 sequence→relay **可以且確實成功**。Relay 開啟後交接許可取消。13:52 新 CLI 換了 GID，sequence 又從 Digital-only 開始，觸發既有拒絕與 fail-closed all-off。

已保存 [Bridge 原始日誌](../diagnostics/power_handoff_20260911/incident-bridge.log)：1789105924.084493704、.135507144、.187522314 三次拒絕字串完全吻合 guard；1789106025.707125757 為正常 shutdown。這是 Orin ROS 時間，不與 sbRIO 牆鐘相減。事故 PID 11680 已退出，無法補取當時 `/proc/11680/exe` 的 SHA；本次磁碟執行檔與來源核對是候選修正依據，不能冒充事故時實際載入的映像雜湊。

`last_guard_violation=None` 是舊 Python 工具檢查電壓／電流的結果。它沒有訂閱 Bridge 的拒絕資訊，所以「沒有電流違規」與「Bridge 拒絕命令」可以同時成立，最後只報三秒 ACK timeout。

Windows 存檔 Access denied、已成功卻誤報失敗及服務停掉後仍按開電的問題依使用者回報已由 v9.4 處理；這裡不重做 Windows session 狀態機。

## 實作選擇

保留短生命週期 CLI，改用 **Bridge 擁有的操作 epoch 與交接協定**，不新增另一個會與既有工具爭用 `/power/command` 的背景發布者。

- 新工具的開電操作先只訂閱。現場已有三筆新的健康回讀、Bridge v2 狀態新鮮且無電源協定故障／software E-stop 時，`ensure-on` 或 `relay` 直接回覆 `already_satisfied`，不建立 power publisher、不送 Digital-only。`sequence` 已達 sensors 或 relay 時也不向下切換。
- 尚未達成時，只補齊缺少的 Digital→Signal→Relay 階段，使用同一個 CLI GID。跨 CLI 的 sensors→relay 仍需原 guard 的交接許可，加上命令之後的實際 relay-off 回讀。
- 電源命令保留原 `PowerCmdStamped`，`header.frame_id` 使用 `P2|<Bridge epoch>|<Off generation>|<request id>`。Bridge 繼續核對唯一發布者、精確節點名稱／namespace、封包 GID、時間與連續序號。公開的 epoch 不是取代上述來源核對的密碼。
- 新 Bridge 的 energizing 指令必須使用 v2；舊格式不會被默默視為安全交接。**Bridge 與 CLI 必須成對更新。** 舊格式 all-off 仍可送出。
- Off 不受設定檔損壞、額外 power publisher、來源已退出或 software E-stop 阻擋。Off 仍需唯一可信的 Bridge 新回讀才能宣稱成功。
- 每次新的 Off 建立新 generation，舊 generation 的開電操作不能繼續。被 Off 搶占的 CLI 不再補送另一套開電或與該 Off 爭用收尾。後續必須由操作者明確開始新操作。
- Power 協定拒絕鎖由受驗證 Off 恢復；既有 software E-stop 維持 process-lifetime sticky，Off 不會解除它。不能拿重啟 Bridge 當日常重按開電的解法。
- Bridge 後端回讀拒絕非相鄰重放、序號倒退及無效／倒退的來源時間。只比較 sbRIO 封包與其前一筆時間，不把 sbRIO 時間當 Orin 時間。後端計數／時鐘重置會導致資料不可認證，必須由維護流程確認來源，不會猜成 ready。

## Windows 確切命令（成對部署完成後）

每次 SSH 遠端 shell 先設定：

```bash
cd /home/jetson/rinbo_ros_ws
source /opt/ros/humble/setup.bash
source /home/jetson/rinbo_ros_ws/install/setup.bash
export ROS_DOMAIN_ID=99
```

唯讀就緒檢查（不建立 power publisher）：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool ready --json --wait-for-subscriber-s 8
```

確保動力已開（取代 Windows 原本兩個 CLI）：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool ensure-on --confirm-relay --json --wait-for-subscriber-s 8
```

關電，包括開電中搶占、前一個 CLI 已退出、重連後：

```bash
ros2 run redrhex_lowlevel_bridge rinbo_power_tool off --json --wait-for-subscriber-s 8
```

可加 `--request-id win-20260911-001`，限 1～80 字的英數字／`_`／`-`；預設自動產生 UUID。每次新操作使用新 id。重查遺失回覆不會回傳舊 cache，而是重新取得新鮮狀態；即使相同 id，也不會用舊 ACK 宣稱成功。

既有 `sequence`、`sequence --include-relay --confirm-relay`、`relay --confirm-relay`、`off`、`digital`、`sensors`、`status` 名稱與選項保留；`relay` 現在會確保完整順序。`digital`／`sensors` 是指定狀態操作，不要用來代替冪等開電；若 ownership 不允許，先明確 Off。需要機器解析時一律加 `--json`，不要再從文字搜尋 `exit_1` 或只解析舊 ACK 片段。

## JSON 與 exit code

`--json` 成功／失敗各輸出一行最終 JSON 到 stdout；ROS 日誌走 stderr。`--dry-run` 保留舊的計畫預覽格式，並不表示操作成功。下列為縮短的 schema 範例，不是實機測試回覆；完整實際 mock 回覆存於驗證目錄的 `mock-results.jsonl`。

```json
{"schema_version":1,"request_id":"win-001","operation":"ensure-on","status":"success","exit_code":0,"command_sent":true,"readiness":"ready","epoch":"bridge-instance","generation":1,"state_source":"/rinbo_ros2_bridge","reason":"","acknowledgement":{"ack_kind":"command_correlated","request_id":"win-001","epoch":"bridge-instance","generation":1,"command_sequence":9,"publisher_gid":"DDS-GID","feedback_sequence":400,"feedback_stamp_ns":1789106000000000000,"digital":true,"signal":true,"power":true,"healthy_samples":3}}
```

```json
{"schema_version":1,"request_id":"win-002","operation":"ensure-on","status":"already_satisfied","exit_code":0,"command_sent":false,"readiness":"ready","epoch":"bridge-instance","generation":1,"state_source":"/rinbo_ros2_bridge","reason":"","acknowledgement":{"ack_kind":"fresh_already_satisfied","request_id":"win-002","feedback_sequence":450,"feedback_stamp_ns":1789106001000000000,"digital":true,"signal":true,"power":true,"healthy_samples":3}}
```

```json
{"schema_version":1,"request_id":"win-003","operation":"ensure-on","status":"rejected","exit_code":20,"command_sent":false,"readiness":"rejected","epoch":"bridge-instance","generation":1,"state_source":"/rinbo_ros2_bridge","reason":"software_estop_latched","acknowledgement":null}
```

```json
{"schema_version":1,"request_id":"win-004","operation":"ensure-on","status":"ack_timeout","exit_code":30,"command_sent":true,"readiness":"backend_unavailable","epoch":"bridge-instance","generation":2,"state_source":"/rinbo_ros2_bridge","reason":"ACK timeout: stage=relay; relay state UNKNOWN; cleanup Off unverified","acknowledgement":null}
```

| exit code | status | Windows 處理 |
|---:|---|---|
| 0 | success | 此操作命令已被 Bridge 接受，且有命令之後的匹配新回讀；Relay 要三筆健康樣本 |
| 0 | already_satisfied | 未送 power 命令，重新取得三筆符合狀態的健康樣本；不是沿用舊 ACK |
| 0 | ready | 唯讀檢查通過；不是新供電或動作授權 |
| 2 | not_sent | CLI 用法／格式錯誤，未送出 |
| 10 | not_sent | 設定／服務／就緒不足，未送出 |
| 20 | rejected | ownership、來源、故障、Off 搶占或健康条件拒絕；讀 reason 與 command_sent |
| 30 | ack_timeout | 已嘗試送出，但缺本次可認證 ACK；狀態未知，不自動重送開電 |
| 31 | state_unknown | 操作途中 epoch／來源變更或驗證中斷；狀態未知 |
| 40 | interrupted | Ctrl+C／SIGINT／SIGTERM／ROS 中斷；若本 CLI 曾送出才盡力送 Off，不能當已關電 |

失敗結果的 `acknowledgement` 固定為 null；`last_verified_state` 只供診斷，可能是前一階段或 cleanup Off，不能拿它覆蓋失敗結果。`bridge_rejection` 提供 Bridge 最後拒絕的 id、GID、序號／原因；它可能包含別次操作的診斷，因此仍以本次 status、request_id 和 reason 為準。

### Timeout 契約

使用上述命令及預設 `repeat=3`、repeat delay 0.05 秒、step delay 0.5 秒、verify timeout 3 秒時，Windows SSH 子程序建議 deadline：**ensure-on 90 秒，off／ready 30 秒**。這包括設定載入、DDS discovery、逐階段等 ACK、失敗時一次有界 Off 收尾及程序結束時間；不是保證網路／OS 排程不會停頓的即時期限。

`--wait-for-subscriber-s` 同時限定初始 v2 就緒等待；每次送命令仍有 subscriber 等待。每階段 graph convergence 最多 1 秒，ACK 最多 `--verify-timeout-s`（0.5～3 秒）。開電最多三階段；失敗最多再做一次 Off，不重送整套開電。若改參數，caller 應增加 deadline：保守計入 `5 + W + N*(W+1+repeat*(0.02+delay)+verify) + (N-1)*step + (W+repeat*(0.02+delay)+verify) + 10` 秒，其中 W 為 subscriber 等待、N≤3，末項含初始化／退出餘量。

## 就緒與停止責任

Bridge 每 50 ms 發布 `/rinbo/power/operation_status`（`std_msgs/msg/String` JSON）：epoch、generation、狀態、來源／接受／拒絕識別，以及 power/motor 資料年齡。CLI 僅採信唯一 `/rinbo_ros2_bridge` 的狀態與 power state，監看期間 GID／epoch 更換即失效。

- `starting`：Bridge 仍未取得兩種後端資料。
- `backend_unavailable`：原有 motor／power 資料已不新鮮，或完全無可用 v2 心跳。
- `ready`：motor ≤250 ms、power ≤350 ms，且無 power 協定故障／software E-stop；開電仍要讀實際 rails 與三筆電源健康樣本。
- `rejected`：power 協定故障或 software E-stop。具體 reason 由 Orin 回報。

因此 Bridge PID 存在、TCP port open 都不夠。Windows 只負責啟動／等待上述就緒、顯示結果、存 session，不複製功率保護狀態機。此 ready 也不表示 Calibration／Standing／馬達 arbiter 已完成；動作握手照舊。

對舊 Bridge 或沒有 v2 心跳時，回 `not_sent`，reason 說明 backend/protocol 不可用，不偷退回舊開電路徑。Off 永遠可以嘗試送出，但缺新協定收據／後端回讀就只能回 ACK timeout，不能猜成功。

## 保留保護与參數

單一 power writer、精確 `/redrhex_rinbo_power_tool`、封包 GID、200 ms 命令 age 硬界與單調序號保留。Relay 確認旗標保留。電壓 18～42 V、健康啟用腿電流 ≤3 A、三筆健康回讀與三筆不健康停止保留；L3 由原生設定載入，Off 不依賴該設定。運動控制的 5 A／25 筆等參數沒有更改；不要把開電健康條件 3 A 與運動中保護 5 A 混為同一項。

## 部署與最小實機驗證

本次先交付候選，不自動啟動 Bridge/Core/FPGA，不上電、不跑動作。成對部署與現場測試要在操作者安排的維護時段進行。候選位置、SHA 與測試結論見 `docs/diagnostics/power_handoff_20260911/delivery.json`。

維護時確認動作與服務正常停止，再以現有建置流程成對安裝：

```bash
cd /home/jetson/rinbo_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --packages-select rinbo_ros_bridge redrhex_lowlevel_bridge --parallel-workers 1
```

此指令只建置／安裝，不啟動服務。Windows 之後依既有啟動流程啟動一次新 Bridge；不是每次 PowerOn 重啟 Bridge。首次應先執行 `ready --json` 驗證 v2。

操作者批准現場供電測試後，最小確認：

1. 起始 Off 新 ACK、ready 通過，記錄 epoch/generation、rails、電壓／電流。
2. ensure-on 一次，確認 Digital→Signal→Relay 與三筆健康回讀。
3. 立即重按，須 `already_satisfied`、command_sent=false，實際 Signal／Relay 不掉電。
4. 模擬只遺失 client 回覆後重查，仍要新回讀；開電途中按 Off，舊操作須被取消且不能自行再開。
5. 每次 Off 都核對新 ACK；遇到 timeout、來源更換或不明狀態就停止測試，不自動重送開電，依現場既有實體斷電流程處理。

本次不以無動作模擬結果保證實際繼電器時序或機械安全，需以上現場回讀驗證；不需要以增大 PWM 或移除保護換取通過。

## 最終離線結果與修改位置

- C++：3 個 suite、26 項通過（含既有 epoch guard、PWM 界限與新增操作協定）。
- 底層 Python：384 項通過；Control Panel：264 項通過。
- 真 DDS、正式 Bridge callback、零 Core 連線：10 個整合情境全部通過（63.69 秒）。涵蓋不同 GID、重按／遺失回覆、部分完成與 ACK timeout、故障鎖、額外發布者、Off 搶占、Bridge 更換與非相鄰舊回讀重放。
- 總計 684 項通過。測試 ROS 限定 domain 232、localhost；原生 candidate 可編譯。未啟動真實供電／動作。
- 參數 revision=15、hash=5866b8ee7bb6a7733c0757fb0d7dd35995a1f78e3a5ec4defe6507c130c13129，基準值差異為零。原 build/installed Bridge SHA 与已安裝 power tool Python SHA 均保持原值。

主要程式：

- `src/rinbo_ros_bridge/src/power_operation_protocol.hpp`：epoch、Off 世代、收據、就緒與重放檢查。
- `src/rinbo_ros_bridge/src/power_command_epoch_guard.hpp`：保留原來源規則，新增唯讀 ownership 查詢。
- `src/rinbo_ros_bridge/src/rinbo_ros_bridge.cpp`：正式 callback 的交接／Off 整合及 50 ms 狀態回報。
- `src/redrhex_lowlevel_bridge/redrhex_lowlevel_bridge/power_operation.py`：冪等規劃、延後建立發布者、來源釘選、本次 ACK、JSON 結果。
- `src/redrhex_lowlevel_bridge/redrhex_lowlevel_bridge/rinbo_power_tool.py`：舊 CLI 名稱、新 ensure-on／ready、JSON 與中斷收尾入口。

已準備候選於 `/home/jetson/.local/state/rinbo-power-handoff/20260911/candidate`，包含 `bin/rinbo_ros_bridge` 與 `python/redrhex_lowlevel_bridge`。可在不初始化 ROS、不送電的情況查看候選 CLI：

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/rinbo_ros_ws/install/setup.bash
PYTHONPATH="/home/jetson/.local/state/rinbo-power-handoff/20260911/candidate/python:$PYTHONPATH" ros2 run redrhex_lowlevel_bridge rinbo_power_tool --help
```

**此為第一階段歷史狀態；現已與原生 GUI API 成對部署到正式入口。** 不要把候選 Python 單独用於舊 Bridge 的開電；先依上方維護建置流程安裝兩個套件，再交 Windows 採用新命令。
