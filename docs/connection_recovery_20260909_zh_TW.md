# 第 1 步：自動整理並檢查連線

重新開啟 `./robot.sh`，按 **1** 即可使用。登入電腦、開啟選單及按 Enter 更新畫面，都不會自動停止程序或整理通訊。已開著的舊控制台要重新開啟才會載入新版 Python 程式；原有 **0 關電並離開** 的語意不變。

## 這次卡住的原因

使用者提供的 20:03 日誌顯示 `上次 core 已退出；請先完成停止再重試`。原程式只要舊程序紀錄不再符合，就拒絕再次連線；沒有區分「舊開機留下的紀錄」與「本次開機仍在使用的程序」。錯誤又寫在 stderr，控制台的 JSON 日誌只存 stdout，導致首個失敗原因漏存。

後續唯讀快照證實：

| 項目 | 快照結果 |
|---|---|
| sbRIO 當時 boot ID | `d0c0ba3b-cd9a-4b4f-b5f3-ca85dbfb19d8` |
| 控制台紀錄的 boot ID | `71c59791-68ca-4da7-bcff-4a34f731ecf9` |
| 控制台舊紀錄 | Core PID 3756、FPGA PID 3777 |
| 當時實際程序 | Core PID 25036、FPGA PID 25441 |
| 實際程序輸出日誌 | Windows `/tmp/rslip-launcher/20260909-201210-216/` 下的日誌 |
| 實際程序環境 | `CORE_MASTER_ADDR=192.168.30.254:50051`、`CORE_LOCAL_IP=192.168.30.254` |

因此可以確認舊紀錄會錯誤阻擋恢復；不能把這份快照當成「20:03 時新程序已存在」的證明，也不能據此判定舊 Core 異常崩潰。這次未取得重開機原因。原始快照保存在 [sbrio-readonly.txt](diagnostics/connection_recovery_20260909/sbrio-readonly.txt)。

## 按 1 會做什麼

1. 核對 Jetson 路由位址。用鎖避免兩個此版本控制台同時整理。
2. 找出本使用者、同一 ROS domain、執行檔路徑完全符合此工作區的 Calibration、Standing、Tripod 與逐腳動作。純設定檢查／預覽不算動作。
3. 若有舊動作，核對 PID、啟動時間、執行檔與使用者，透過 Linux pidfd 送 SIGINT 正常停止，最多等待每個程序 8 秒。pidfd 是綁定特定程序的系統介面，可避免 PID 被其他程序重新使用時誤送訊號。接著核對沒有馬達命令發布者，以及新的 `motor output=false` 回讀。動作尚未退出時不強制送 TERM／KILL。
4. 更新本控制台自己的 SSH 連線，保留原有程序所有權紀錄。
5. sbRIO 在既有服務鎖內核對程序，將已證實過期的 `.pid` 紀錄搬入同一工作目錄下的 `recovered.*`，寫入原因；原始 Core／FPGA 日誌不刪除。舊開機的完整 `.starting` 標記也能封存。損壞或尚不能判定的標記保留並回報。
6. 唯一且路徑、位址、工作目錄正確的現有 Core／FPGA 直接沿用。沿用其他入口的程序不會取得其所有權。缺少服務時，只依原有啟動流程嘗試一次；沒有背景重試、無條件重啟或新的 FPGA 初始化機制。
7. 核對唯一 Bridge、目標位址、通訊埠，以及新鮮的 `/motor/state`、`/power/state`。最後再次檢查動作發布者；需要停止舊動作時再次確認輸出停用。通過後才顯示連線檢查完成。

若停止動作時尚無 Bridge，會先正常停止可辨識的動作，再建立通訊，最後核對停止回讀。任何階段都不送上電或新動作命令；停止既有動作本身的收尾行為仍依該控制器實作。

## 自動整理的界線與重試

- 正常 Core／FPGA／Bridge 保留，包括 Windows 背景啟動的服務。連線整理本身不要求 Windows 更改啟動介面，也不接管它啟動的程序。但 Tripod 3300 部署需要 Windows 同步 Bridge 預期參數檢查；20:25／20:26 的 `expected=80.0:actual=3300.0` 是此版本不一致，與舊程序紀錄整理不同，請使用 [Windows 修正交接](windows_bridge_pwm3300_prompt_20260909_zh_TW.md)。
- 不關閉一般 SSH 登入、桌面連線、其他網路服務；不使用 `pkill`、重設網卡或重啟 ROS daemon。
- 多份 Core／FPGA／Bridge、不同路徑或位址、無法核對身分的發布者，以及「driver 還在但 Core 不在」仍會回報具體原因。這些狀態需要個別核對，不能假裝整理成功。
- 第一個遠端錯誤、完整合併輸出及過期紀錄封存位置會保存在本次 `~/.local/state/rinbo_control/logs/.../sbrio-*.json`。本機停止操作另有 `connection-stop-*.json`，含程序身分、SIGINT 與退出／未完成結果。
- 不會因整理連線而清除急停鎖、假造關電證據，或把舊的完成紀錄當成新的安全回讀。
- **按 3** 執行新動作仍先檢查是否有人正在控制馬達；它不會自動停止另一個實驗。要主動整理請按 **1**。

遠端 shell 另有內部 `reconcile` 動作，只核對／封存紀錄，不啟動或停止服務；主要用於離線測試與維護。一般使用者不必自行執行。

## 程式碼位置

| 檔案 | 責任 |
|---|---|
| `src/rinbo_control/rinbo_control/console.py` | 選單 1 與操作說明 |
| `src/rinbo_control/rinbo_control/runtime.py` 的 `reconnect()` | 第 1 步停止、驗證、更新連線與檢查順序 |
| `src/rinbo_control/rinbo_control/connection_recovery.py` | 程序辨識、pidfd 正常停止、整理鎖與稽核日誌 |
| `src/rinbo_control/rinbo_control/sbrio.py` 的 `reconcile_records()` | sbRIO 舊開機／已退出程序紀錄封存；不接管外部服務 |
| `sbrio.py` 的 `_run()`／`refresh_transport()` | 保留首個錯誤、日誌與更新私人 SSH 連線 |
| `src/rinbo_control/test/test_connection_recovery.py`、`test_sbrio.py` | 假程序／假硬體離線回歸 |

## 驗證與現場範圍

共 **254 項測試通過**：完整控制台測試在 domain 231 通過 253 項，1 項需要專用環境的回讀測試另在 domain 232 通過。結果保存在 [diagnostics/connection_recovery_20260909](diagnostics/connection_recovery_20260909/)。測試只啟動本機假的 ELF Core／FPGA／動作程序，以及隔離在 localhost 的 ROS 模擬節點。`./robot.sh --check` 已確認安裝與現場設定可讀。

已核對上一版部署的六個原生執行檔、Bridge 設定及現場 YAML 雜湊全部未變更：Tripod 上限 3300、L3 屏蔽與已驗證的 Calibration／Standing 行為保留。本次未對真實機器人執行停止、啟動、上電或動作，也未提前整理遠端紀錄。實際整理與新回讀驗證由使用者按 **1** 觸發。
