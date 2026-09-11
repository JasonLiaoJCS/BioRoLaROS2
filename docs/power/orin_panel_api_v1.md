# Windows → Orin 原生控制入口 v1（正式部署）

目前更新契約為 [panel-flow-v4](panel_flow_v4_20260911.md)：一般恢復由一次 Communications 完成，真正急停新增 ResetEmergency。磁碟安裝與常駐生效分開記錄，實際版本看 CheckConnection。

2026-09-11。配合 power operation protocol v2。**Windows 只傳請求與顯示結果；不重建機器人的保護、PID、供電或恢復判定。**

2026-09-11 停止修正見 [panel-stop-v2 契約與診斷](panel_stop_recovery_20260911.md)。新版 install 檔案已更新；目前常駐 Runtime 的實際啟用狀態以該次 deployment/activation 紀錄及 CheckConnection 的 `controller_version` 為準。不要因磁碟檔案已更新就推定正在執行的程序已載入。

停止鎖來源、操作可用資訊及原子提交修正見 [panel-latch-v3](panel_latch_recovery_20260911.md)。API 命令及 protocol 不變，執行版本須看實際回覆。

## 已部署的入口

```bash
python3 /home/jetson/rinbo_ros_ws/tools/rslip_panel_client.py --request-base64 '<UTF-8 JSON 的 Base64>'
```

```json
{"protocol":1,"request_id":"0123456789abcdef0123456789abcdef","action":"Communications","sbrio_ip":"192.168.30.254","orin_ip":"192.168.30.8","ros_domain_id":99,"stream":false}
```

請求 ID 是每次明確點擊產生的 32 位小寫 hex。不要每次固定使用上面的示例 ID。不同 GUI 共用同一個 Orin 控制者。只接受表列欄位，不接受密碼、shell 命令、PWM 或 Windows session 授權。

Windows v9.8.1 已由使用者確認走此原生入口。設定參考如下，無須回退舊清理路徑：

```powershell
# app/RSlip-SiteConfig.psd1
ORIN_PANEL_CLIENT = '/home/jetson/rinbo_ros_ws/tools/rslip_panel_client.py'
```

Windows 原始碼不在此工作區；Windows 的現場版本與呼叫方式以使用者提供證據為準。Orin 路徑與 API 已存在並完成正式安裝的唯讀呼叫驗證。

## 回覆與監看契約

入口 10 秒 socket timeout；Communications 首次啟動控制服務額外最多 11 秒。Windows 捕獲期限建議 30 秒。所有耗時動作由持續存在的 Orin worker 執行，不依附 SSH。

**`status=started, exit_code=0` 僅表示 Orin 已持久保存並接受請求，不是校正、站立、上電或關電已完成。** Windows 顯示「處理中」，以 `operation_id` 對應監看中的最終結果。其他操作未完成時 Orin 自己拒絕衝突操作；Windows 不需要複製前置條件。

```json
{"protocol":1,"request_id":"0123456789abcdef0123456789abcdef","action":"Calibration","operation_id":"0123456789abcdef0123456789abcdef","status":"started","reason":"accepted by Orin; completion will be reported in ReadLogs","exit_code":0,"controller_epoch":"..."}
```

動作實際啟動後是 `running`，Calibration 完成並正常交接後為 `completed`；Standing 到位並維持保持為 `holding`；Tripod 持續 `running`，直到明確停止或原生失敗。不得把非零退出、程序提前退出或只有 Popen 成功標成到位完成。

停止修正版：重複停止使用 `coalesced_with` 合併硬體操作，各自保留request/operation ID；Stop可升級StopMotion，EmergencyStop可搶占一般Stop。ACK timeout/缺少硬體證據是 `state_unknown`（終止結果，exit30/31）；搶占保持 `failed`/exit40。完整原始ACK與第一失敗原因不因後續清理覆蓋。

冪等 PowerOn 的最終結果示例（節錄）：

```json
{"protocol":1,"request_id":"...","action":"PowerOn","operation_id":"...","status":"already_satisfied","exit_code":0,"reason":"","data":{"status":"already_satisfied","command_sent":false,"epoch":"...","generation":2,"state_source":"/rinbo_ros2_bridge","acknowledgement":{"ack_kind":"fresh_already_satisfied"}}}
```

失敗示例（節錄）：

```json
{"protocol":1,"request_id":"...","action":"PowerOn","operation_id":"...","status":"failed","exit_code":20,"native_exit_code":20,"reason":"ownership_handoff_required: use verified off before a new enabling operation","data":{"status":"rejected","bridge_rejection":{"request_id":"...","publisher_gid":"...","sequence":1,"reason":"..."}}}
```

錯誤 `2` 格式不正確；`10` 未送出／控制服務不存在；`20` 原生拒絕／busy／停止鎖；`21` 不支援；`30` ACK timeout；`31` 狀態未知／回覆或來源遺失；`40` 被停止搶占。原生程序的其他非零退出碼照實保留（例如 SSH 255）；負的 signal 退出對外為 31，另保留 `native_exit_code`。錯誤原因也寫 stderr。不要把所有非零碼改成泛用 exit_1。

ReadLogs 非串流回傳近期事件與最近 40 個操作（含最終結果）。stream=true 逐行立即 flush，最慢每 2 秒 heartbeat：

```text
RSLIP_ACTION_CONTEXT=Calibration
[原生 Calibration 日誌]
RSLIP_ACTION_STATUS={"kind":"Calibration","request_id":"...","operation_id":"...","status":"completed","exit_code":0,"reason":""}
RSLIP_MONITOR_HEARTBEAT=1
```

GUI 類型也涵蓋 Communications、PowerOn、Stop 與 EmergencyStop。斷線後可以重新開 ReadLogs；近期事件可能重播，操作用 operation_id 合併。重新監看、CheckConnection 絕不啟動通訊、上電或動作。服務尚未開啟的監看明確回報 controller_unavailable，再由 Windows 重連唯讀串流。

同 ID、同內容只讀回記錄，不再執行；同 ID 不同內容拒絕。只對修改請求做去重；唯讀請求永遠讀目前狀態。程序重啟時原本未結束的操作記為 state_unknown 並保留停止鎖，不重播動作、不復活舊 ACK。回覆遺失請查 ReadLogs，不自動重送動作。

## 支援動作

| action | Orin 原生行為 |
|---|---|
| Communications | 一次完成原生服務整理、Off與其後的新ready確認，解除一般停止未知；保留通訊、不上電。真正急停改用ResetEmergency。 |
| PowerOn | v2 ensure-on；新鮮已開電只回報已達成，不發命令、不降回 Digital-only。 |
| Calibration | 停止前一動作並確認 motor output=false；原生全體健康腿校正，沿用現場 L3 屏蔽與所有參數。 |
| Standing | 原生動作交接，原生 MotionSession 核對校正紀錄；到位後維持新版保持控制。 |
| Tripod | 原生動作交接，核對校正／站立紀錄；使用現場設定，持續執行直到停止或原生保護觸發。 |
| StopMotion | 正常 SIGINT，原生 pidfd 身分核對與 8 秒交接期限；新 motor output=false 回讀。正常動作停止保留電源。若正在上電，會取消上電 CLI，其失敗收尾可能關電。 |
| Stop | 立即Off；必要時同一操作恢復backend再確認Off與清理。一般停止狀態原子提交；已結束工作階段重按回明確範圍的already_satisfied。真正急停仍保留。 |
| EmergencyStop | 立即建立持久停止鎖、取消可能上電的 CLI、獨立送 Off 並確認 Bridge 的既有 /estop 鎖，不排在 SSH 初始化後面；再確認動作停止，保留通訊。完成後仍保持真正急停，必須明確 ResetEmergency 完成才能再次上電。 |
| ResetEmergency | 單次明確急停復歸，驗證Off，必要時僅重啟已軟體急停的Bridge，再Off→ready；成功保留通訊與關電。 |
| CheckConnection | 唯讀狀態；回傳原生 power protocol 的 starting / ready / rejected / backend_unavailable、原因與新鮮度。未建立觀察時明確回報未建立通訊，不猜供電。 |
| ReadLogs | 唯讀近期原生日誌、操作結果或串流。 |
| FpgaConsole / PhysicalCleanup | 明確 unsupported、exit 21。不能推定已實體斷電。 |

正常停止保留 Tripod 原有 2 秒減速／5 秒期限，接口外層等待 8 秒，不提高 PWM 或放寬保護。Calibration／Standing 的原生參數及完成紀錄規則不變。

## 唯一所有權、登入與手動路徑

- `rinbo-panel.service` 為 jetson 的 systemd user service。已 enable，已啟用 linger，SSH 關閉與登出不銷毀 Runtime。
- 服務啟動本身是惰性的：不初始化 ROS、不連 sbRIO、不啟動 Core／FPGA／Bridge、不上電。`Restart=no`，崩潰不無條件重建硬體。
- 每個原生子程序沿用 guardian。控制服務真正死亡時，子程序收到 SIGINT；Bridge 保留原有停止／關電語意。
- 使用私有 Unix socket `~/.local/state/rinbo_control/panel.sock`（同 UID 檢查），持久請求與停止鎖在 `panel.sqlite3`，輪替事件記錄在 `panel-events.log`。不要刪資料庫來假裝完成停止。
- 與既有 `./robot.sh` 互動入口共用 `console.lock`，不允許兩個 Runtime 同時接管。若要改回文字控制台：先經 API 完成 Stop，再執行 `systemctl --user stop rinbo-panel.service`，才開 `./robot.sh`。回 GUI 前正常退出文字控制台，Communications 會啟動背景控制者。此版未將 robot.sh 的整套互動選單改成 socket frontend。
- sbRIO 使用既有 Orin 主機金鑰與 `~/.config/rinbo_control/ssh/<IP>.password` 或 SSH key。無密碼檔時使用 BatchMode，不會在背景卡等密碼。認證失敗保留 SSH 退出碼與原始原因；請在 Orin 完成可信主機／登入設定。
- 從 GUI 交給直接 sbRIO 手動 FPGA 路徑：先由 GUI 完成驗證 Stop，再停止 panel service，才依既有 FPGA 單一驅動鎖操作。反向交接須先正常結束手動驅動，再明確 Communications。不可同時啟動第二個驅動；無法確認關電時不得用 PhysicalCleanup 假清除。

## 電源相容性修正

原問題：已開 Relay 後換新的短 CLI publisher GID，沒有經已認證 relay-off 邊界即再次 sequence，被 Bridge 拒絕；CLI 原來只知道本機電壓／電流 guard，因此誤導成 last_guard_violation=None 的 ACK timeout。

新版本由 Bridge 提供 epoch、generation、GID、request ID、ACK、拒絕原因與 backend readiness。保持 sole publisher、指定節點名、來源 GID、時間與序號防重放；不是任意發布者可上電。Off 提升 generation，舊上電請求不能覆蓋。EmergencyStop 同時經 `off --assert-estop` 確認 Bridge 的既有黏著急停鎖，其他直接 CLI 也不能再開電；一般 Off 不清此鎖。明確 Stop 完成關電與 Bridge 清理後，下一次 Communications 才建立新的控制生命週期。

既有 CLI 的 sequence / relay / off 保留；新 Windows 主要操作應走 panel API，不再分兩個 CLI 開電。另補原生 manual 收尾 `sensors`：只允許經完整驗證的明確 111→110 降 Relay；保留兩條感測電源，不做 Off→On 重啟。後續新 publisher 上 Relay 仍須確認新 relay-off ACK。Digital-only 重按不会取得此例外。

細節：[power v2 契約](power_handoff_v2_20260911.md)。該文件的早期「未部署」是第一階段交付狀態；目前兩次修改已一起部署，以此次 [deployment.json](../diagnostics/panel_api_20260911/deployment.json) 為準。

## 驗證與部署

- Python 整包最終回歸：678 passed、1 skipped（含控制台 294、lowlevel 384）；之後新增 stale socket 恢復案例，API／native adapter 最終 32 項通過。
- lowlevel Python：384 passed（共同回歸記錄 python-tests.log）；Bridge C++：27 項、3 個 suite 通過。
- 真實 DDS + 無 Core／FPGA 的 production-callback mock：12 passed，限定 ROS_DOMAIN_ID=232、ROS_LOCALHOST_ONLY=1。
- 涵蓋重按已開電、不同 GID、回覆遺失、relay-off handoff、off 搶占、部分 ACK timeout、epoch／舊 ACK、故障與未授權 publisher；API 涵蓋重複 ID、不同 client、SSH 斷線、controller reboot、原始退出碼、急停搶占與監看不啟動動作。
- 曾有一次 mock 可執行檔尚在連結時提早啟動測試，產生 PermissionError。已保留失敗紀錄，待建置結束後完整重跑通過。急停補測曾找出一處變數放錯迴圈，已修正並重跑完整 12 項 DDS；串流測試亦改成正確的分段讀取，避免假設單次 recv 一定含完整 heartbeat。上述失敗記錄均保留，未當成通過。
- 正式 Bridge 執行檔、lowlevel/control Python 模組、入口與 user service 已部署；未重建或覆蓋另一工作階段的 FSM 執行檔。備份與逐檔 SHA256 見 deployment.json。
- 部署後只執行 CheckConnection、ReadLogs、unsupported 回覆與監看連線測試；未執行實體 Communications、上電、Calibration、Standing 或 Tripod。

## 現場驗證（操作者確認後執行）

1. Windows 寫入 client 路徑，先 CheckConnection、ReadLogs；確認顯示原生狀態、串流 heartbeat 與操作 ID。
2. 明確點 Communications；等待原生 completed 與 ready。認證、位址或 backend 錯誤時先處理原始原因，不繼續開電。
3. 現場確認可供電後點 PowerOn，等本次 ACK 成功；再明確重按一次，應是 already_satisfied／command_sent=false，Relay 不應掉電再開。
4. 確認現場條件後分別執行 Calibration、Standing；以原生 completed／holding 為準。Tripod 若要測試，另由操作者明確啟動。
5. StopMotion：確認正常減速與新 output=false，電源可保持；Stop：確認新 Off ACK 後清理完成。測試 SSH 監看斷線／重连應不影響原生程序。
6. 急停測試由現場明確操作：即使在啟動中也應先 Off、後續不得再開電，停止鎖持續存在。任何新回讀中斷、原生拒絕、ACK timeout 或不符預期運動，立即停止測試並依現場方式斷電，保留第一原因與 operation_id。
