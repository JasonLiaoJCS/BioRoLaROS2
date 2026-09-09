# Rinbo 中文操作台

從工作區執行 `./robot.sh`，或載入環境後執行
`ros2 run rinbo_control rinbo_control`。使用者教學見
[Control Panel 簡明教學](../../docs/control_panel_zh_TW.md)。

- `--demo`：純介面演練，無 ROS、socket、硬體子程序與校正紀錄。
- `--check`：檢查安裝與有效現場設定，不初始化 ROS。
- 實機模式要求互動 TTY；Enter 不會自動同意動作確認。
- 預設 sbRIO `192.168.30.254:50051`、Jetson `192.168.30.8`，可在選單修改。

## 結構

| 模組 | 職責 |
|---|---|
| `console.py` | 中文選單、每腳動作編輯、具體動作確認、單一操作台鎖 |
| `presentation.py` | 六腳位置圖、狀態摘要、最近結果與下一步提示；純顯示，不存取硬體 |
| `plans.py` | 純資料驗證、摘要、原子儲存操作偏好 |
| `library.py` | 具名動作收藏的驗證與原子儲存；只保存動作設定 |
| `runtime.py` | 路由／通訊埠檢查、既有程式的生命週期、電源與校正順序 |
| `sbrio.py` | v6.6 core／FPGA 啟動、bitfile SHA-256、SSH 原生登入與遠端程序核對 |
| `feedback.py` | 唯讀 ROS 回讀、來源名稱及時戳檢查；不訂閱 `/motor/command` |
| `guardian.py` | Linux `PR_SET_PDEATHSIG`，在 exec 前安裝父程序死亡通知 |
| `demo.py` | 使用相同選單的離線演練後端 |

此操作台不實作另一個馬達控制迴圈，也不直接發布 motor/power 命令。
實際出力仍經過 `rinbo_manual`／`rinbo_cali`、`rinbo_power_tool` 及既有 Bridge。
Humble 本機 rclpy 不提供每個訂閱回呼的 MessageInfo，因此介面回讀僅作操作前的
輔助檢查；C++ 控制器原本的封包來源 GID、電源與超時保護維持生效。

`rinbo_manual --plan FILE --check-ready` 唯讀檢查真正的校正完成紀錄，
不持有動作 session、不產生或刪除紀錄。實際執行仍由 `MotionSession` 再次驗證。
新增 `relative` 模式使用 `move_deg`（±30°），從開始時的實際回授位置產生小幅軌跡。

## 操作生命週期

執行前先由 C++ 驗證動作檔，再列出主馬達與可能涉及的校正／伺服範圍。
一般模式由操作者確認後才開電源。選單 14 可明確開啟本次現場調機模式；此時
主選單 3 本身就是開始指令，不再詢問第二次。它不存入偏好，關電、執行失敗／取消、
屏蔽管理、網路變更或現場雜湊變更會撤銷；確認後若現場設定雜湊或沿用校正的條件改變，拒絕操作。

選單 13 單獨調整出力，讀取與原生 manual 一致的 `rinbo_standing.max_pwm` 作為
現場上限；支援加減 10、填值與採用該上限。超界增減會保留原值，不默默截斷。
選單 9 的範例沿用目前時間、出力和加速度；每秒 5 度範例僅在速度上限不足時提高至 5。
所有設定修改仍先原子寫入成功才套用，沒有任何設定選項會自行啟動動作。
校正依成功標記及原生完成紀錄雙重檢查；中止／失敗不假造成功。
`rinbo_cali` 自行處理 SIGINT／SIGTERM，訊號只要求主迴圈退出，不直接關閉 ROS context。
單執行緒 executor 停止派發回呼後，送出停用命令，再移除 node 並關閉 ROS。
DONE 後正常交接保留完成紀錄；未完成即中止或真正的故障仍使紀錄失效並以非零碼退出。
控制台仍要求校正子程序沒有 fatal 訊息且正常退出，不能單憑 DONE 標記繼續。
每次操作台啟動後首次執行都要求校正。唯讀背景訂閱持續監看感測器供電：
馬達／電源回讀失效、感測器供電關閉或來源更換都會增加 sensor_epoch。
只有本次操作台已完成校正、epoch 未改變且原生紀錄有效時才允許沿用。

主選單 5 顯示六腳角度與 Hall 回讀。角度換算與 `manual_motion.hpp::encoder_degrees`
一致：每圈 55296 counts、左側反號；沒有回讀時顯示未知，不以設定值替代量測。
Hall 顯示與校正程式一致：原始 0 為觸發、1 為未觸發；仍需停穩及歸零回讀才算校正完成。
執行摘要列出校正／動作範圍、未參與主馬達、校正原因，以及各階段的 PWM 上限。
校正等待上限依現場設定計算：伺服定位＋尋零＋兩次 stop_timeout（停穩及歸零）
＋15 秒啟動餘裕；原生各階段停止條件不變，也不按腳數倍增等待時間。
網路與限制表單逐欄重試；偏好先原子儲存成功，再更新記憶體中的動作／目標。
純查看、重新整理與連線檢查不重寫偏好。動作執行顯示五階段進度。

選單 10 的收藏獨立存於操作偏好旁的 `motions.json`，最多 30 組；載入時重新
驗證目前屏蔿名單，執行前仍交原生程式驗證。選單 11 支援方向／速度與換腳的
純設定調整。選單 12 保留本次程序內最近 20 份不同動作設定，寫入成功才移除
復原項目；收藏與復原都不保存校正狀態、不更改硬體名單、不直接執行動作。

四個 C++ FSM 在建立馬達命令發布者、輸入 guard 與握手計時器前，先在自己的 Node
等待 Bridge 的 motor/state、power/state 端點身分完整（最多 8 秒）。
這個準備階段只查 ROS graph，不接受封包或完成紀錄。既有 2 秒初次輸入、0.25 秒馬達斷訊、
封包來源 GID、來源時戳與執行中發布者變更保護維持原值。
純 FSM 狀態測試使用既有 RINBO_FSM_OFFLINE_TEST fixture；準備函式另以真實本機 DDS 延遲測試覆蓋。

正常動作完成後使用既有 `sensors` 電源模式關閉 relay、保留感測器電源，
避免下一次操作不必要地重啟感測器。進入電源／動作階段後取消或失敗，會嘗試確認 all-off。
其他程式仍在發布馬達命令時，在通訊啟動前拒絕本次動作，執行前再次檢查。
前置檢查因其他控制程式而拒絕，不會因上一次保留感測器供電就觸發關電。
退出會確認關電、終止本操作台啟動的 Bridge，再停止自己啟動的 driver／core。
關電或 Bridge 退出無法確認時，保留遠端通訊與程序紀錄。
不能確認關電時明確回報未知並要求實體停止，不以「已發送命令」代替回讀確認。

日誌每檔 2 MiB，保留兩個輪替檔；每次 session 分開儲存。
UI 行佇列有上限，原始日誌不因 UI 省略訊息而遺失（超過輪替保留量除外）。
本機程序均以參數陣列啟動，未使用 `shell=True`。
sbRIO 使用固定 POSIX shell 腳本，參數經驗證與 shlex.quote。`ssh_auth.py` 可使用工作區外、權限 600 的位址專屬密碼檔，透過 OpenSSH askpass 自動登入；密碼不放進指令列或日誌。
SSH 使用原生主機金鑰檢查，ControlMaster socket 放於私有暫存目錄，退出後清理。

## sbRIO 與 v6.6 的對應

依使用者提供的 Part B 固定 LD_LIBRARY_PATH、PATH、core／driver 路徑、driver 工作目錄及 bitfile 雜湊。
sbRIO 需要 flock、sha256sum、netstat 及可讀的 /proc；缺少就回報錯誤，不略過檢查。
每次選 1 先核對遠端服務、再核對 Bridge 及兩種 fresh state，不上電。
既有服務在執行檔、IP 環境與工作目錄相符時沿用；新 driver 必須有本次專用日誌的成功標記。
新服務記錄 boot ID、PID、start_ticks、exe；停止只對完整核對的本次程序送 SIGINT，未退出時再次核對後送 SIGTERM。
啟動未寫完身分、PID 被重用、設定不符或重複程序均拒絕繼續；不使用 pkill 或自動強制殺除。
遠端 flock 只協調本工具，不能協調 Windows 啟動器，因此兩種入口擇一操作。

首次供電由既有 power tool 執行 off → sequence → relay，等待訂閱者上限 8 秒。
Relay 前與動作結束後確認新 motor_output=false 及零命令發布者。
已上電時以背景唯讀探針確認三筆新資料，沿用 power tool 的電壓／電流判斷，不再發布上電指令。
3 A 邊界依 SOP 改成拒絕（要求嚴格小於 3 A）；禁用腿仍由 Orin 唯一設定決定。

本工具針對逐腳測試；沒有複製 Windows v6.6 的 Standing／Tripod toggle 與保留 Relay 重試狀態機。
失敗／取消保持 all-off 策略，保留通訊供使用者下一次重試，不自動重跑、不偽造完成紀錄。

## 測試

機器人停止動作時，載入 ROS 工作區後執行：

```bash
ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=1 PYTHONPATH="$PWD/src/rinbo_control:$PYTHONPATH" \
  /usr/bin/python3 -m pytest -q src/rinbo_control/test
```

ROS 回讀測試僅在 domain 232、localhost 環境執行，其餘環境會跳過。
測試使用假程序或假 Bridge，不啟動真實硬體 Bridge、電源或校正入口。
sbRIO shell 測試將路徑導向暫存目錄，以獨立名稱的假 ELF core／driver 及 localhost 埠驗證啟停、重用、雜湊、FPGA 失敗及 PID 身分；不使用 SSH。
