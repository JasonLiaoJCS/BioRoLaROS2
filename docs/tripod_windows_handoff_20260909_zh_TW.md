後續政策更新：使用者已明確選擇「有限位置誤差只警告」，且已部署至Orin revision 9。請以 [最新政策與部署紀錄](tripod_position_warning_20260909_zh_TW.md) 為準；下文的舊位置自動停止要求及尚未部署描述是先前階段的歷史紀錄。

本交接只優化 Tripod：參數、啟動、停止、錯誤及重試。Orin候選程式已修改，Windows原始碼不在此工作區，尚未代改Windows或部署現場。

**操作介面**

參數頁只需要顯示「啟動秒數」「起始ratio」「目標ratio」，ratio越大越慢；進階欄位可調kp、kd、k_ff、friction_pwm、friction_velocity_counts_s、ratio_step。不需要重編譯或用全模式的 `tune-motion` 改動 Calibration／Standing。

候選版部署後，使用既有 ROS 環境執行：

```bash
# 查目前設定（唯讀）
ros2 run rinbo_fsm rinbo_legs status --json

# 預覽；可在Tripod仍存活時使用，不寫檔、不上電、不啟動ROS節點
ros2 run rinbo_fsm rinbo_legs tune-tripod --dry-run startup_duration=20 start_ratio=40 target_ratio=40

# 儲存；僅在原動作已退出後執行，不會啟動動作
ros2 run rinbo_fsm rinbo_legs tune-tripod startup_duration=20 start_ratio=40 target_ratio=40

# 動作入口不變；只在使用者按啟動時，由現有背景wrapper執行
/home/jetson/rinbo_ros_ws/build/rinbo_fsm/rinbo_tripod
```

20／40／40 是可選量測起點，不是強制最低速度。GUI傳送使用者選值，不要自動套用示例。`--dry-run`只驗證設定格式、範圍及完成紀錄，並回傳候選參數，並不模擬硬體或保證軌跡能追上。模型需求提示在Tripod建構時由 `TRIPOD_REFERENCE_WARNING` / `TRIPOD_REFERENCE_BUDGET` 輸出；警告不阻擋啟動，也不自動修改參數。

`tune-tripod` JSON：`dry_run`、`changed`、`calibration_valid`、`standing_valid`、`parameters`。有效完成紀錄在僅改Tripod參數時保留；缺失、失效、不同boot紀錄不會復活。相同值不改revision。它不接受 `max_pwm`、位置門檻、屏蔽腿或關閉保護等鍵；既有安全設定檔管理方式不變。寫入遇到動作尚在時，只提示「先停止本次Tripod再儲存」，不要自行kill或重做Calibration。

**啟動與監看**

維持現有背景 `nohup`／wrapper、stdin `/dev/null`、獨立日誌、PID＋boot ID＋start_ticks與父程序wait退出碼。新Tripod不需要 `ssh -t`，監看關閉不代表停止。`tail -F` 只看本次日誌；關閉視窗只結束tail／SSH。不要從FPGA console另建驅動。

使用者按一次「啟動Tripod」即可沿用已有、有效的完成紀錄，不加重複確認框。模型超限顯示警告與建議，不變成新的「禁止啟動」狀態。原本缺少通訊、仲裁或完成紀錄仍給出具體缺少項目，不顯示籠統的「安全檢查失敗」。

每次顯示：本次PID／設定revision、STARTUP進度、A/B啟動狀態、當前ratio、L3停用標記、最差腿的帶符號位置誤差與PWM。使用既有 `/rinbo/controller_debug`，訊息定義未變。

**停止、錯誤與重試**

| 使用者操作／事件 | Windows動作 |
|---|---|
| 關閉監看 | 只關監看，不傳signal、不停止Bridge或FPGA。 |
| 按「停止Tripod」 | 核對本次PID的boot/start_ticks/exe後送SIGINT，等父wrapper的wait結果。通常2s減速，截止5s；不改成pkill或立即SIGKILL。 |
| 已經SAFETY_STOP仍存活 | 立即顯示第一原因，不把「PID仍在」誤判為運轉成功；停止按鈕送同一個SIGINT，候選版會退出。 |
| 正常退出0 | 顯示「Tripod已停止」。這只代表主驅動停用流程完成，不能顯示「已關電」。 |
| 故障退出2 | 顯示「已退出；保留原故障」，不要把後續退出結果覆蓋第一原因，也不要再報成未知停止錯誤。其他非零則保留實際碼與日誌。 |
| 正常停止後調參／重試 | 參數只改Tripod可保留有效Calibration；不因調參強迫重新Calibration。需要回站姿時可提供一次操作的「Standing → Tripod」，而非要求使用者分開重做所有步驟。舊Standing完成紀錄本身不是現在姿態的即時量測。 |
| 本次hard position或無效回饋等故障 | 現有故障紀錄失效規則保留。提供清楚的「復歸後重試：Calibration → Standing → Tripod」按鈕，由使用者主動觸發；按一次後依序驗證，不再逐項彈相同確認框；任何一步失敗即停止，不循環重試、不自動上電。 |
| 急停／停止未驗證 | 沿用既有急停及關電驗證，保留未驗證狀態；SIGINT或退出碼不能代替新鮮關電回讀。 |

本次首個錯誤應顯示為「L2（A組）位置誤差18008.93 counts超過硬上限18000」。舊日誌沒有正負號，不能自行補成「落後18008.93」或「反轉」。候選版之後可用 `TRIPOD_FAULT_FRAME` 的 signed_error 精確區分方向。

文字診斷：`TRIPOD_TRACE` 為一般資料，`TRIPOD_PREFAULT` 為首次停止前的緩衝資料，`TRIPOD_FAULT_FRAME` 必須完整保留，不能只取最後一行PWM。第一條 `TRIPOD SAFETY STOP:` 是根本停止原因，後續stale／退出等事件附在其後。依frame_s/source_s/arrival_s判斷資料時間；無可計算幀時會明示 unavailable。

**部署界線**

需部署候選 `rinbo_tripod` 與 `rinbo_legs`；不需要改ROS訊息schema、FPGA、Bridge或Calibration／Standing執行檔。舊背景啟動與舊 `tune-motion` 仍相容，但只調Tripod請改用新入口。部署時不要覆蓋另一工作階段的現場修改，也不要用仍在執行的舊Tripod驗證新功能。此交接未授權或執行任何上電／動作。
