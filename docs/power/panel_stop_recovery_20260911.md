# Panel 停止／清理修正：panel-stop-v2-20260911

> 操作契約已由 [panel-flow-v4](panel_flow_v4_20260911.md) 更新：一般恢復收進一次 Communications，真正急停使用 ResetEmergency。本文保留歷史診斷與当時部署證據，不再要求日常三步循環。

Windows v9.8.1 維持既有 protocol1 與 `tools/rslip_panel_client.py --request-base64 ...`。不新增 Windows PID、phase、ACK 判斷或參數門檻，不退回舊清理腳本。本次不新增 action；PhysicalCleanup/FpgaConsole 仍 unsupported21。

## 後續更正

後續發現 completed Stop 之後還有兩筆新的 Stop ACK timeout，會重新上鎖；先前「留下 latch/target」不足以證明成功停止沒有清鎖。請以 [停止鎖後續查證與 v3 修正](panel_latch_recovery_20260911.md) 為準。

## 查證結果與原因

證據位於 `docs/diagnostics/panel_stop_20260911/`，包括資料庫唯讀操作副本、原生事件紀錄、原始 power/Bridge/sbRIO 日誌。沒有修改舊 request 結果或 stop latch。

| 請求 | Orin 證據 | 結論 |
|---|---|---|
| Stop `3396238271af4fec990fcb3335dc2385` | Off command_sent=true，但 epoch/generation/ACK 全空，backend_unavailable，exit30 | 曾嘗試送 Off，沒有關電證據。last_guard_violation=None 只表示沒有取得本機 guard 違規，不代表成功或 Bridge 已接受。 |
| StopMotion `821c5f1e32de44419157e1cc32c25db7` | feedback.wait_motor_disabled 等不到本次 disabled 回讀 | 共用上電前 gate 的錯誤文字「不開 Relay」誤用於停止；不是 StopMotion 真的要求開電。原本 generic RuntimeError 又把未知回讀降成 exit20/data=null。 |
| Stop `95ffd...`、`933e...` | controller_busy、no request queued | 排程只允許 EmergencyStop 越過 StopMotion；Stop 越過 StopMotion 及 EmergencyStop 越過 Stop 都被擋。這是 Orin admission 缺陷。 |
| Communications `241cd...` | sbRIO 同一 boot `894a9dac-4c94-4968-8cc2-31db690d71c0`；existing Core/driver 清單空；新啟 Core8764、driver8778；新啟 Bridge140048；兩種 state 新鮮 | 成功前確實重建了 backend，不是 Windows 重畫成功。不能只用後來成功推定之前供電狀態。 |
| Stop `a31e639999124f868a1e6461792ba54d` | 本次 correlated Off ACK、generation1、feedback_seq32176；Bridge正常 shutdown；sbRIO stop exit0 | 此次真的完成已驗證關電與本機原生所有權清理。 |
| Communications `5a4afffe98474cf48769a0ab9a7fe8c8` | 新 Core11318、新 Bridge141530、新 epoch、ready | Stop 之後重新建立通訊是新的 backend 生命週期。 |

Bridge140048 開始的 ROS timestamp 是1789112215.630，正常shutdown是1789112254.122；只用 ROS 同時基準比較。sbRIO 原始 log wall clock 不混用。原日誌未記下第一次 Stop 當下所有 PID／退出碼，仍不能確認更早 backend 缺失的終止原因，更不能直接歸因於 SSH。新結果補上本機 ownership/process snapshot 與事件時間，方便下一次分辨。

## 修正內容

- `feedback.py`：新增停止專用 `wait_motor_stopped`，沿用相同的新 disabled 回讀／零 command publisher 条件及等待時間。回讀不足是 state unknown；資料包含 Bridge數量、發布者、last output/age、power_action=none。上電前 gate 的門檻與語意不變。
- `panel_native.py`：Off 的 publisher 用單一原生 lock 串行；急停可中斷正在等候的一般 Off CLI。清理必須持有**本次 request 的 correlated全關 ACK**、匹配 epoch/generation；舊成功或缺 backend 不可授權清理。停止成功/失敗與電源未知分開記錄。
- `panel_server.py`：相同停止合併、Stop升級StopMotion、EmergencyStop升級一般Stop；取消的是舊操作，不是把新操作丟成busy。Off不排在SSH初始化後。正在停止時暫停一般motion poll，避免把正常停止退出誤記為Tripod異常退出。
- 清理失敗仍保留本次完整 power ACK、原始 code/reason、motion_stop 結果，first failure不被後續錯誤覆蓋。ACK timeout30/unknown31 對外status=state_unknown；搶占仍是既有 failed/exit40。
- feedback 在部分清理後被重建時，原生電源監看會綁到新的 node；舊node callback不能回填新狀態。新Bridge epoch可重新建立觀察，不沿用舊序列。
- 重啟後未完操作保持unknown、不重播，並保留之前結果作明確歷史資料。ReadLogs重播與相同request查詢不執行動作。

沒有修改 Runtime 的上電／動作策略、PWM、電流／電壓、L3、PID、時間或基準。原生 cleanup 仍只匹配已核對的動作/Bridge與本機token管理的sbRIO服務；recorder/mirror不在清理對象。

## Windows 契約（不新增 action）

命令維持：

```bash
python3 /home/jetson/rinbo_ros_ws/tools/rslip_panel_client.py --request-base64 '<UTF-8 JSON Base64>'
```

請求仍為：

```json
{"protocol":1,"request_id":"32位小寫hex","action":"Stop","sbrio_ip":"192.168.30.254","orin_ip":"192.168.30.8","ros_domain_id":99,"stream":false}
```

started只是接單。最新伺服器的 CheckConnection/ReadLogs/操作回覆有 `controller_version=panel-stop-v2-20260911`。Windows不用以版本決定保護，但可顯示維護資訊。

同時重複Stop：各自保留 request_id/operation_id，回started並包含 `coalesced_with=<主停止operation_id>`；最終結果會對每一個ID各輸出一次，data增加 `fulfilled_by_operation_id`。硬體只執行主停止；內層原始 ACK 的 request_id仍是主請求，**不竄改成各個GUI請求**。合併結果清楚標示共同完成來源。

StopMotion期間Stop可接受；EmergencyStop可搶占Stop。較低優先的舊操作回failed40及 `data.superseded_by`；急停成功仍保留鎖。急停之後明確送出的Stop會等急停完成，再完成清理，只有該明確Stop成功可解除鎖。一般修改仍可因busy被拒絕，Windows不得自動重送。

缺backend的Stop最終示例（欄位摘錄）：

```json
{"protocol":1,"request_id":"...","operation_id":"...","action":"Stop","status":"state_unknown","exit_code":30,"native_exit_code":30,"reason":"ACK timeout (3.0s): stage=off; last_guard_violation=None; relay state UNKNOWN","controller_version":"panel-stop-v2-20260911","data":{"operation":"off","command_sent":true,"readiness":"backend_unavailable","epoch":null,"generation":null,"acknowledgement":null,"verified_off":null,"power_state":"unknown","native_owned_services_cleanup":"not_attempted","recovery_action":"Communications","motion_stop":{"status":"unverified","native_exit_code":31}}}
```

StopMotion未知則exit31、power_action=none，processes_stopped只描述本機原生動作程序，不等於硬體已停止／已關電。只有取得新 motor output=false 並確認零發布者才completed。沒有Bridge時即使零發布者，也不能假定最後硬體輸出狀態。

Stop已完成後，用原request_id查詢仍讀回原完成紀錄。**新request_id的Stop是新的硬體確認請求**；若已清理掉backend，沒有新ACK，仍回unknown並指出Communications恢復路徑，不把歷史ACK當作新證據。這不推翻原Stop完成，也不要求Windows保存舊checkpoint來授權操作。

保留protocol、request_id、operation_id、exit_code/native_exit_code、完整data.power或原始Off failure欄位、epoch/generation/state_source/bridge_rejection。`state_unknown`是終止的未知結果，不是繼續等待。沿用ReadLogs追蹤，不解析raw power tool日誌覆蓋panel operation結果。

## 恢復與現場驗證

一般恢復不需要PhysicalCleanup：當停止因backend不可用而unknown，由操作者明確點 **Communications**（原生整理、重建且不上電）取得ready，再點 **Stop** 取得本次Off ACK與清理結果。Communications失敗則保留unknown、原始原因與所有權紀錄；不能靠刪鎖或宣稱實體斷電跳過。若現場實體斷電是必要處置，由操作者處理；程式不將口述斷電當感測證據。

離線測試使用假的backend、程序與ROS localhost隔離domain232。首次全套測試有兩個新test把模擬request99與隔離env232混用，被正確拒絕；已修正test fixture，未放寬domain規則。完整結果與部署狀態見同目錄diagnostics。

維護部署須區分「檔案已安裝」與「常駐Runtime已載入」。本次查核時PID131178仍擁有Bridge141530，recorder53632仍存在。**不得直接為套新版重啟這個Runtime**：`panel_server.main` 的finally與guardian會停止它持有的backend。新版檔案可先放入install；由操作者選擇空檔點Stop完成、確認無其他GUI送入操作後，在Orin維護端重載 `rinbo-panel.service`，再用CheckConnection看到新版controller_version。這是Orin軟體部署步驟，不是要求Windows改回shell清理。

新版生效後由現場操作者點擊：

1. CheckConnection與ReadLogs：確認版本、operation_id、heartbeat、原始reason可見；不啟動硬體。
2. backend已停止時點StopMotion：應state_unknown31且power_action=none，不能顯示「不開Relay」或已關電。點Stop則依本次ACK能力回unknown30/31，不能用舊ACK成功。
3. 明確Communications→ready→Stop：應有本次correlated Off ACK與清理completed；此步不需要開Relay或執行動作。
4. 若要驗證重疊操作，明確StopMotion後立刻Stop，再觀察較高優先請求被接受；重複Stop顯示coalesced_with。急停搶占測試需現場自行選擇適合時機，成功後鎖保留。
5. recorder的PID/檔案仍在；關電後仍可由原錄製接口收尾。以上測試不需要自動PowerOn、Calibration、Standing或Tripod。

## 正式啟用紀錄

2026-09-11 16:18（Orin 當地時間），操作者明確回覆「已按 Stop，顯示 completed，請切換新版」後完成重載。先查證 Stop `1a0f668a55f6445cb8705dbb328003ad` 已 completed/exit0，verified_off=true、本次 correlated ACK 與 native_owned_services_cleanup=completed；Bridge 與動作程序已退出，才重載 `rinbo-panel.service`。

新版 MainPID=213339，controller_epoch=`d345aca5fa38474fa17cb45155af978b`。實際 CheckConnection 與 ReadLogs 都回報 `controller_version=panel-stop-v2-20260911`。目前 backend_unavailable / communications_not_established 是停止後的預期狀態；沒有重建 backend、送電或動作。舊控制者在此次 completed 後仍留下 latch=true 與 target metadata；新版沒有憑歷史 ACK 手動清鎖，現場後續可明確 Communications→ready→Stop 完成新的確認流程。

錄製器 PID53632、start_ticks276275、執行檔 SHA256 與原本 summary.csv/events.csv 開啟路徑均不變，未停止、重建或換成另一輪錄製。啟用證據見 `docs/diagnostics/panel_stop_20260911/activation.json`、`activation-stop-check.json`、`post-activation-check.json`。

驗證：rinbo_control 全套 317 passed；最後增補修正的重點測試 54 passed；安裝路徑驗證及隔離 DDS 測試 55 passed。參數基準檢查保留已知來源／執行檔差異（exit2），未回填基準或變更實驗設定。實機停止搶占及 backend 重建仍由操作者依上方步驟驗證。
