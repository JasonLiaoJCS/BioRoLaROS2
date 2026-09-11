# Orin 操作流程 v4：panel-flow-v4-20260911

本版取代 v2/v3 要求操作者反覆 Communications→Stop→Communications 的一般恢復設計。原生 Python 程式與離線測試已實作；常駐版本以文末部署紀錄及 CheckConnection 為準。Windows 只傳單次請求與呈現 Orin 回覆。

## 正常使用需要幾次點擊

| 情況 | 操作者操作 | 次數與結果 |
|---|---|---|
| 正常開始／結束 | Communications → PowerOn → 動作 → Stop | 4個主要步驟；若動作依序為 Calibration、Standing、Tripod，完整共6次 |
| 正常 Stop 後再開始 | Communications → PowerOn | 開始動作前2次；不需額外 Stop |
| 一般停止未知／殘留一般鎖 | Communications | 1次完成服務整理、新的關電確認、就緒；之後自行 PowerOn |
| 已完成 Stop 又重按 | Stop | 1次回 already_satisfied（結束本控制工作階段）；不建服務、不重複關電、不新增停止鎖 |
| 真正急停後恢復 | ResetEmergency → PowerOn | 1次復歸到通訊就緒且保持關電，再1次明確開電；不需要先 Communications 或 Stop |
| 只停動作 | StopMotion | 1次；正常停止保留通訊與供電。無動作程序時分開回報程序與硬體狀態 |

動作原有 Calibration／Standing 收據、有效回讀與保護仍由原生動作節點執行。不復活過去成功紀錄，不因 reset 就宣稱已校正。

**Communications 現在是「準備到已確認關閉且可開啟動力」的操作**：會停止本機原生舊動作、送本次 Off，並取得其後的新鮮就緒回讀；若原本已開電，這個明確操作會關電。要唯讀查看請用 CheckConnection。重按 PowerOn 仍使用原生 ensure-on，已開啟不降電、不重啟。

## 實際刪除／合併的阻擋

| 原流程／條件 | v4處理 | 保留的責任 |
|---|---|---|
| Communications 前先等不存在 backend 的 motor disabled | 先收尾已辨識的本機動作程序，建立／沿用 backend，再取得本次 Off 與新鮮 ready | 不把「沒有動作程序」當硬體停止證據 |
| Runtime 等 motor/power 新鮮，power tool 又等一次同一就緒事實 | panel 路徑由 Runtime 管 transport/DDS 位址與唯一性；motor/power 新鮮與來源由 power protocol 一次負責 | robot.sh 預設等待行為不變；panel 不能繞過 power protocol |
| stop_motion 的 idle 與 wait_motor_stopped 重複查零發布者 | 一般動作停止使用 wait_motor_stopped 的新 disabled／零發布者條件一次 | 控制者互斥保留 |
| 已有本次 authenticated all-off ACK，Stop 又等 motor Bool 造成假失敗 | 收尾動作並確認無命令發布者，以本次全關 ACK 作停止依據 | motor_output_enabled 可為null，verified_by=current_all_off_ack；不偽造 Bool |
| 通訊成功卻留下普通停止鎖、要求再Stop並拆掉服務 | Communications 本身完成 Off→新ready，原子清除普通阻擋，保留 backend | 若 epoch/generation／供電状态在两阶段間改變，保持unknown |
| timeout 一概顯示 emergency_latched | 分開持久 emergency_context 與一般停止狀態 | 真急停不會被普通 Communications、PowerOn 或 Stop 清除 |
| 已清理 backend 後新ID Stop又嘗試必定無ACK的指令 | 同目標且本機沒有重新出現原生資源時，回覆控制工作階段 already_satisfied | 明列 verified_off=null/current_hardware_confirmed=false；沒有新硬體確認 |
| backend不在時Stop只能要求操作者按三個按鈕 | 立即Off未成功後，由同一個Stop做一次原生通訊恢復、取得新Off並清理 | 不無限重試；恢復失敗保留初次Off與第一個恢復失敗原因 |
| 本工具的Bridge PID活著但DDS找不到，要求使用者退出 | 先等5秒探索恢復；仍缺席才收尾本工具持有的Child，確認退出後重建一次 | 不殺未知程序、不建立第二個Bridge；健康Bridge沿用 |
| Bridge已拒絕且明確command_sent=false，又多加一個panel停止鎖 | 不另加普通停止鎖；呈現Bridge原始拒絕 | 有部分送出／回覆未知仍需Communications收斂；Bridge保護不变 |
| completed、解除鎖、目標紀錄、合併請求結果分開提交 | 延續v3的一次SQLite transaction，增加closed_session与真正急停的持久狀態 | 提交失敗回unknown；不公布未提交完成 |

仍保留：固定ROS domain、不同未收尾目標的所有權、單一控制者鎖、Bridge唯一性／正確位址、未知發布者拒絕、epoch/GID與回讀新鮮度、電流／電壓／PWM、L3與動作保護。跨SSH初始化前後的發布者檢查不能合併成舊快照，因為中間可能出現另一個控制者。

## 真正急停與復歸

`EmergencyStop` 仍走獨立Off通道，可搶占啟動、一般Stop及ResetEmergency。普通Stop可升級StopMotion；重複停止合併一次工作，每筆保留自己的request_id與operation_id。

真正急停用獨立的持久 `emergency_context` 保存來源。一般timeout不冒充使用者急停。載入舊資料時參考已接受的明確EmergencyStop與舊版已驗證Stop邊界；rejected EmergencyStop不算已接受。普通殘留鎖不從歷史ACK直接解鎖，而由Communications取得新的確認。

已知真正急停時，Communications直接指向ResetEmergency，不啟動新Bridge來暗中清除硬體端急停。ResetEmergency：

1. 建立／沿用必要通訊，收尾本機動作，取得本次全關ACK。
2. 若Bridge明確回報software_estop_latched，依現有Bridge「忽略/estop=false、只能重啟清除」的契約，**僅在此明確復歸內**收尾Bridge、重建Bridge；Core/FPGA沿用，recorder/mirror不動。
3. 新Bridge再次Off→新ready；需同一次Off的epoch/generation、其後的新回讀且三路全關。完成後才原子解除真正急停。
4. 其他fault不以重啟掩蓋；復歸中有新Stop／EmergencyStop時，不繼續宣告復歸成功。

沒有每次重啟Bridge的策略。故障的「本機持有但DDS缺席」收尾與「明確軟體急停復歸」是兩個有界情況，均不會自動開電。

## JSON契約與Windows最小修改

protocol仍為1、client路徑不變：

```bash
python3 /home/jetson/rinbo_ros_ws/tools/rslip_panel_client.py --request-base64 '<UTF-8 JSON的Base64>'
```

唯一新增action是 `ResetEmergency`；使用既有按鈕事件產生的新request_id，不加確認模式、前置硬體檢查或自動重送：

```json
{"protocol":1,"request_id":"0123456789abcdef0123456789abcdef","action":"ResetEmergency","sbrio_ip":"192.168.30.254","orin_ip":"192.168.30.8","ros_domain_id":99,"stream":false}
```

Windows最小修改：

- 增加「急停復歸」按鈕，單次送ResetEmergency。其他按鈕、SSH與ReadLogs原樣。
- 顯示原生operator_message（有則優先）與control.recovery_steps，原始reason仍保留在詳細資料。不得自動執行recovery_action。
- 不把所有exit0或already_satisfied一概畫成「硬體已關電」；直接呈現原生completion_scope、verified_off與power_state。這是顯示結果，不是新增Windows保護判斷。

非stream維持快速回started，長時間SSH／服務恢復在原生worker執行。socket回覆期限10秒，Windows捕獲期限沿用30秒；Communications啟動惰性controller最多額外11秒。`started/exit0`只表示接單，最終結果走ReadLogs。單一Stop最多一次內部恢復，不要求Windows重送。ReadLogs串流仍每2秒heartbeat，監看重連不觸發任何動作。

通訊準備成功（欄位節錄，原始ACK保留在data.power與data.native）：

```json
{"protocol":1,"action":"Communications","request_id":"...","operation_id":"...","status":"completed","exit_code":0,"native_exit_code":0,"controller_version":"panel-flow-v4-20260911","emergency_latched":false,"stop_blocked":false,"reason":"通訊已就緒，已取得本次關電確認；可直接開啟動力。","data":{"recovery_verified":true,"backend_retained":true,"control":{"control_state":"ready","stop_state":"clear","emergency_latched":false,"recovery_action":null}}}
```

已完成Stop後重按（**控制生命週期已結束，不是新的硬體ACK**）：

```json
{"protocol":1,"action":"Stop","request_id":"...","operation_id":"...","status":"already_satisfied","exit_code":0,"native_exit_code":0,"emergency_latched":false,"stop_blocked":false,"reason":"本控制工作階段已結束，沒有再次送出命令；目前未連線，未取得新的硬體供電回讀。","data":{"completion_scope":"native_session_closed","verified_off":null,"current_hardware_confirmed":false,"power_state":"unknown","command_sent":false,"prior_session":{"operation_id":"前次已完成Stop","hardware_confirmation":"historical_only"}}}
```

真正急停時PowerOn拒絕：exit20，reason為中文，control.emergency_source包含來源操作，recovery_action=ResetEmergency。一般停止未知則emergency_latched=false、stop_blocked=true、stop_state=unknown，recovery_action=Communications。復歸成功data.emergency_reset_verified=true，仍保持關電。

StopMotion無程序且無backend：already_satisfied，completion_scope=motion_processes、no_motion=true、verified=false、hardware_output=unknown、motor_output_enabled=null、power_action=none。不宣稱硬體已停止／已關電；既有真正急停／未知狀態不會被這種回覆解鎖。

原欄位protocol/request_id/operation_id/exit_code/native_exit_code/reason/ACK/epoch/generation/state_source全部保留。新增stop_blocked專門表示原生阻擋，emergency_latched現在只代表真正急停。既有raw reason不改寫；新增operator_message供中文呈現。物理斷電不由操作者口述或舊ACK推定。

## 已讀現場證據與範圍

保留v2/v3診斷：`1a0f...` completed之後有另外兩筆Stop `83e01...`、`e2905...` ACK timeout重新上鎖，不能只說成功Stop沒有清鎖；這是v3補證的更正。本版直接處理造成不合理操作循環的生命週期與恢復行為。

程式：`panel_server.py`（排程、鎖分類、原子提交、已結束冪等）、`panel_native.py`（一次恢復／復歸／停止語意）、`panel_protocol.py`（新action）、`runtime.py`（panel專用合併等待與自有Bridge恢復，原robot.sh預設不變）。

離線測試使用假的Runtime/backend及隔離ROS domain232 localhost，包含正常按鍵收斂、第一次Stop缺服務、重複Stop、同ID查詢、舊鎖、重載、真正急停復歸、初始化中急停搶占、Off後狀態／epoch/generation變化、未知發布者、不同目標、其他原生backend重新出現與recorder/mirror保留。

安裝工具：`python3 tools/deploy_panel_flow_update.py --apply`，只安裝檔案與備份，不重載服務。常駐Runtime重載由現場操作者明確執行；本次開發不替操作者Stop、重建硬體或結束錄製。部署及版本唯讀查核記錄見 `docs/diagnostics/panel_flow_20260911/`。

## 本次交付結果

- 全套rinbo_control離線／隔離回歸：353 passed，76.60秒。最終安裝路徑90 passed，包含真實Base64 client→本機socket→假backend的ResetEmergency端到端流程；最後另確認Reset無需重啟Bridge時的connection也回ready。
- 四個Python模組已安裝，來源／install SHA256一致；不需重編C++。部署工具僅複製檔案，未重載服務。
- 實際常駐仍為panel-latch-v3-20260911，PID274294；**v4尚未由常駐Runtime載入**。見activation-status.json、live-after-install.json。不能因新版磁碟檔案存在便宣稱生效。
- recorder PID53632、mirror PID53631、start_ticks276275不變，原summary.csv/events.csv仍開啟。開發沒有送Stop、開電、動作或結束錄製。
- 基準檢查前後一致：保留既有來源／執行檔差異（exit2），沒有更改保護參數或基準；現場YAML仍是5866b8ee7bb6a7733c0757fb0d7dd35995a1f78e3a5ec4defe6507c130c13129。

由現場操作者在結束操作、確認可重載的時機執行：

```bash
systemctl --user restart rinbo-panel.service
```

再用原生CheckConnection確認controller_version=panel-flow-v4-20260911，才進行Communications→PowerOn→動作→Stop的現場驗證。若Runtime仍持有backend，重載會觸發原生收尾，因此本次開發沒有代執行這條命令。錄製器不屬於此服務的清理對象。

Windows可直接交接：[最小修改prompt](WINDOWS_PANEL_FLOW_V4_PROMPT.md)。原始測試、SHA與部署／常駐版本證據：docs/diagnostics/panel_flow_20260911/。

檔案備份：`/home/jetson/.local/state/rinbo-deploy-backups/20260911-165057-panel-flow`。
