請把 Windows v9.5 控制台切換到已部署的 Orin 原生 API。Orin 端已完成正式 client、持續 Runtime、原生電源交接 v2 與離線驗證。請直接修改 Windows 原始碼並做離線串接測試；實機上電與動作由現場操作者明確點擊。

1. 在 app/RSlip-SiteConfig.psd1 設定：
   ORIN_PANEL_CLIENT = '/home/jetson/rinbo_ros_ws/tools/rslip_panel_client.py'
2. 使用既有 app/RSlip-OrinClient.ps1，透過 SSH 呼叫：
   python3 /home/jetson/rinbo_ros_ws/tools/rslip_panel_client.py --request-base64 '<UTF-8 JSON Base64>'
   JSON 保持 protocol=1、32 位小寫 hex request_id、action、sbrio_ip、orin_ip、ros_domain_id=99、stream。每次明確點擊使用新 ID，不自动重送失敗的修改請求。
3. Communications、PowerOn、Calibration、Standing、Tripod、StopMotion、Stop、EmergencyStop、CheckConnection、ReadLogs 全走此入口；跳過舊 Windows PID/graph/ACK/電壓/PWM/mask/session-phase/stop.request 判定與舊恢復腳本。機器人規則全部由 Orin 負責。
4. 特別處理非同步回覆：exit=0 且 status=started 表示「Orin 已受理，處理中」，絕不是校正完成、站立完成或已關電。後續以 operation_id/request_id 對應 ReadLogs 的 RSLIP_ACTION_STATUS，顯示原生 running/native_done/holding/completed/already_satisfied/stopped/failed/state_unknown 狀態。不要自行推導到位或供電成功。
5. 按鈕提供「停止目前動作」→ StopMotion（正常停止通常保留電源）、「結束實驗」→ Stop、「急停」→ EmergencyStop。急停能搶占開電／啟動；Orin 建立持久停止鎖並確認 Bridge /estop。Windows 不自行清鎖，也不先 Off 再重送 PowerOn。
6. ReadLogs stream=true 每 2 秒內會送 RSLIP_MONITOR_HEARTBEAT=1；關閉／重連監看不停止機器、不啟動通訊。依既有 30 秒無輸出重連唯讀串流。重播事件以操作 ID 合併。單次 SSH 建議 timeout=30 秒；斷線不能推定原生動作已停，要查 ReadLogs/CheckConnection，不重送原動作。
7. 保留 exit_code/native_exit_code、reason、data 內原生 ACK、epoch、generation、state_source 與 bridge_rejection。結果遺失或來源未知要如實顯示，不能只剩 exit_1。通訊、到位、供電成功全部顯示 Orin 結果。
8. FpgaConsole / PhysicalCleanup 目前回 status=unsupported、exit=21，請顯示不支援，不當成功。直接 sbRIO 手動 FPGA 與 Orin 控制者須依交接文件切換，不能猜測實體斷電。
9. 離線測試至少包含：started 不誤報完成、最終原生錯誤不遺失、重按已開電 already_satisfied、監看斷線重連、無 Windows session 仍可 StopMotion/Stop、unsupported、不自動重送。不要在測試中送實體供電／動作。

完整欄位、例子、部署與現場測試清單：
/home/jetson/rinbo_ros_ws/docs/power/orin_panel_api_v1.md

Windows 程式尚未在 Orin 工作區修改；請 Windows 端實作後回報版本、離線測試與 ORIN_PANEL_CLIENT 最終值。
