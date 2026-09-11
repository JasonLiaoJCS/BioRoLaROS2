請將Windows控制台對接Orin的panel-flow-v4-20260911，保持現有單次原生API與ReadLogs，不新增本地機器人檢查或自動恢復序列。

1. 增加「急停復歸」按鈕，只送一次action=ResetEmergency；request_id沿用每次點擊新32位hex。其他欄位與client路徑不變：/home/jetson/rinbo_ros_ws/tools/rslip_panel_client.py --request-base64 <JSON Base64>。
2. 正常仍是Communications→PowerOn→動作→Stop。一般未知由一次Communications在Orin完成恢復；真正急停用ResetEmergency，成功後再由使用者PowerOn。不自動Stop→Communications→PowerOn，不自動送recovery_action，不加確認模式或額外確認框。
3. 優先顯示Orin的operator_message（缺少則reason），顯示control.recovery_steps；raw reason/native_exit_code/ACK保持可查看。
4. 新版emergency_latched只代表真正急停，一般停止未知由stop_blocked/stop_state表達。直接顯示Orin狀態，不推導授權。
5. 已完成Stop後新ID重按可回already_satisfied/exit0，但completion_scope=native_session_closed、verified_off=null、current_hardware_confirmed=false、power_state=unknown：显示「本控制工作階段已結束；未取得新的硬體供電回讀」，不要自行轉成已確認關電。StopMotion同理，motion_processes完成不等於硬體回讀已確認。
6. started仍只代表接單；最終結果從ReadLogs取得，監看重連只重連唯讀串流。PhysicalCleanup/FpgaConsole仍unsupported，不恢復舊shell清理。

新action完整JSON示例（實際每次新ID）：
{"protocol":1,"request_id":"0123456789abcdef0123456789abcdef","action":"ResetEmergency","sbrio_ip":"192.168.30.254","orin_ip":"192.168.30.8","ros_domain_id":99,"stream":false}

詳細契約：docs/power/panel_flow_v4_20260911.md。檔案安裝不等於常駐已載入；由操作者完成維護切換後，再核對CheckConnection.controller_version=panel-flow-v4-20260911。不要用這個維護核對在每次按鈕前建立另一套版本／硬體門檻。
