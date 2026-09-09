後續政策更新：使用者已明確選擇「有限位置誤差只警告」，且已部署至Orin revision 9。請以 [最新政策與部署紀錄](tripod_position_warning_20260909_zh_TW.md) 為準；下文的舊位置自動停止要求及尚未部署描述是先前階段的歷史紀錄。

請修改 Windows 桌面 Launcher 的 Tripod 步驟，加入「停止 Tripod（保留供電）」按鈕，並修正啟動成功後顯示「操作已完成」造成的誤解。完成程式與離線測試，不上電、不執行真實動作、不自行部署 Orin。

已確認的事故證據：
- 工作階段 /tmp/rslip-launcher/20260909-125159-092/rinbo_tripod.log。
- 本次 PID 75799，boot b0008f58-5739-453d-be5f-aa3fa7d3bd72，start_ticks 814097；這些只是本次歷史證據，程式不得寫死。
- Windows 日誌在「Tripod 已就緒並保持運行」後印出「操作已完成」，並停止展示後續資料。
- Orin 後續實際原因為：TRIPOD SAFETY STOP: hard position error: L2 counts=18020.730469 hard_limit=18000。
- 停止前 ratio 約7.91，尚未到目標4。不是8秒STARTUP到時自動結束，也不是到達target_ratio後完成。
- 本次仍是舊 Orin 執行檔；候選修正尚未部署。PID存在不代表馬達仍在RUNNING。

一、按鈕與流程
1. 在Tripod步驟下方新增「停止 Tripod（保留供電）」。取得本次PID後，STARTUP、RUNNING和SAFETY_STOP但程序仍存活時都可按；不要被一般Busy旗標鎖住。
2. 日誌已有「S只停動作，0關電與通訊」的操作約定。先檢查並優先共用現有S的動作停止實作，確認它沒有關電副作用。不要呼叫0／全系統關機或一般Stop的關電流程。
3. 使用者按停止後，核對目前管理的PID、boot ID、start_ticks、exe，再送SIGINT。不要pkill整批程序、停止Bridge/FPGA/Core、切relay、重建通訊或立即SIGKILL。
4. 顯示「正在停止Tripod」，非同步等候程序退出與既有馬達輸出停用驗證，UI仍可操作且急停可用。重複點擊只共用同一停止工作，不重複發起流程。
5. 正常減速約2秒，Orin受控停止截止5秒；這些是停止階段的期限，不是Tripod總運轉時間。逾時顯示「停止未確認」並保留急停入口，不宣稱成功。
6. 驗證成功後顯示「Tripod已停止；供電未變更」，解除該動作占用，讓使用者直接再按Calibration、Standing、Tripod。這次流程依序由使用者選擇，不自動上電、不自動重跑任何動作。
7. 若使用者實際斷電、通訊失聯或急停，應忠實顯示讀到的狀態，不能因本按鈕沒有送關電就宣稱供電仍ON。

二、持續運轉與狀態文案
1. Tripod為持續運轉動作，不套用Calibration／Standing的「階段完成」規則。
2. 「Startup done, entering RUNNING」只代表啟動階段完成；顯示「Tripod運轉中」，保持停止按鈕和背景監看。
3. 到target_ratio只是固定該ratio繼續運轉，不當作完成、計時停止、送signal或執行finally清理後端。
4. 啟動工作返回後可以解除啟動工作的UI Busy，但必須保留Tripod動作身分及狀態監看。不要再顯示無條件「操作已完成」。
5. 關閉日誌視窗只關監看視窗，不停止Tripod；主程式中的狀態監看仍需持續。

三、故障與相容性
1. 持續監看本次Orin日誌／既有controller debug與程序身分，不只等待啟動成功文字。
2. 收到首條TRIPOD SAFETY STOP立即顯示「Tripod保護停止」和具體腿名、誤差、門檻，即使PID仍存活。第一原因不得被後續stale、wait結果或「已完成」覆蓋。
3. 舊Orin故障後可能在SIGINT時回傳0；有故障日誌仍必須判為故障。候選版則故障退出2、正常退出0。不要只依退出碼或PID存在判定成功。
4. STOPPING、SAFETY_STOP、程序退出、馬達停用與relay OFF是不同狀態，不可混用。
5. 原有保護、L3屏蔽與完成紀錄規則保留。不要為了重試偽造Calibration／Standing紀錄，也不要降低保護、提高PWM或加上模型警告造成的新禁令。

四、離線測試
覆蓋：STARTUP中停止、RUNNING中停止、SAFETY_STOP仍存活後停止、PID重用、停止重複點擊、停止逾時、啟動成功後持續監看、到target_ratio仍保持RUNNING、舊版exit0但已有故障、關監看不停止後端。
以mock斷言停止Tripod完全沒有呼叫power-off、關Bridge/FPGA/Core或重新初始化的介面。
驗證停止成功後可再選Calibration→Standing→Tripod，但沒有自動執行它們。

請交付修改檔案、按鈕行為、狀態轉換與測試結果。
