# R-Slip 遠端通訊診斷 — 2026-09-09

本次結論：**目前不新增 FPGA／Bridge 程式修改。已知成功通訊屬於重開機前；後續 Windows 紀錄包含通訊重建被急停鎖阻擋、清理遇到 SSH 逾時及停止流程重疊，v7.4 尚無成功重建通訊紀錄。這是目前有證據支持的恢復中斷流程，不是新版驅動崩潰的證據。** 最新仍只有 Core 與 FPGA，沒有 Bridge，完整通訊尚未恢復。

本次只讀取實機狀態，並在獨立目錄編譯、執行假硬體 console 測試；沒有啟動／停止 FPGA、Core、Bridge，沒有發送電源、編碼器重設或動作命令，沒有修改正式專案程式或部署執行檔。診斷期間另有 Windows 位址的 SSH 工作階段啟動 FPGA，以下分開記錄。

## 1. 程序、開機與版本證據

跨機器時間不能直接排序；連 Orin 本次開機也有系統時鐘校正紀錄。以下優先使用 boot ID、uptime 和 `/proc/PID/stat` 的 start ticks。sbRIO `CLK_TCK=100`。

| 項目 | 證據 |
|---|---|
| sbRIO 本次 boot ID | `64ed8560-4fd6-411a-af52-15dbf953c1ad` |
| Orin 本次 boot ID | `b0008f58-5739-453d-be5f-aa3fa7d3bd72` |
| sbRIO 初次採樣 | uptime `1277.96` 秒，僅找到 Core，未找到 FPGA |
| Core | PID `2762`，start ticks `6123`，即開機後 `61.23` 秒；PPID 1 |
| 後來出現的 FPGA | PID `4190`，start ticks `130639`，即開機後 `1306.39` 秒；PPID `4187` |
| 後續 FPGA | PID `4777`，start ticks `149273`，即開機後 `1492.73` 秒；PPID `4774`，另一個 Windows SSH 工作階段 |
| Windows 補充後配對採樣 | sbRIO uptime `1557.54`，本機時間 `2026-09-08T19:42:01+0000`；Orin uptime `1541.20`，本機時間 `2026-09-09T11:04:18+0800` |
| 此次採樣狀態 | Core 2762、FPGA 4777 存活；FPGA 4190 已退出；Orin 全 `/proc/*/exe` 掃描含 `(deleted)` 無 Bridge，也無名稱符合但無法讀取的候選程序 |

Core `/proc/2762/exe` 是 `/home/admin/rinbo_sbRIO_ws/install/bin/grpccore`，與該磁碟檔 SHA-256 一致：

`20f56da6d967944db2f035144df8ff72aa6254ca8b1eb9f0ec6e15dd77bbdf3c`

Core 的 stdin 是 `/dev/null`，stdout、stderr 指向 `/tmp/grpccore.log` 解析後的 `/var/volatile/tmp/grpccore.log`。環境保留 `SSH_CONNECTION=192.168.30.213 49337 192.168.30.254 22`，忽略 SIGHUP，與 shell history 的 `nohup ... </dev/null >/tmp/grpccore.log 2>&1 &` 相符。可確認由該 SSH 工作階段衍生，不能僅憑這些資料辨認實際操作者或是哪一個 Windows GUI 按鈕。

Core 的 `CORE_LOCAL_IP=192.168.30.254`、`CORE_MASTER_ADDR=192.168.30.254:50051` 正確。`/proc/2762/fd/6` 的 socket inode `19488` 對應 `/proc/net/tcp6` 的 IPv4-mapped `192.168.30.254:50051` LISTEN。BusyBox `netstat -ltn` 沒列出此 socket，**不可據此誤判 Core 未監聽**。

FPGA 4190 的父程序為 `sshd: admin@pts/0`，來源 `192.168.30.213:53093`；cmdline `./fpga_driver`，cwd 是指定的 `rinbo_fpga_driver/build`，stdin `/dev/pts/0`，stdout、stderr 已由 console 重導到 `/tmp/fpga_driver_console.log`。它是附著 SSH 終端的前景啟動。環境 IP、Core 地址與函式庫路徑符合目前設備。

FPGA 4777 由 `sshd: admin@pts/0` PID 4774 啟動，來源 `192.168.30.213:58107`，同樣是 `./fpga_driver`、正確 build cwd、stdin `/dev/pts/0`、stdout/stderr console log，另繼承 `/tmp/rslip-fpga-console.lock` 的 FD 9。環境、執行檔及雜湊與 4190 相同，沒有新增部署證據。

未在已查的 sbRIO init/rc/cron、Orin systemd/cron 設定找到這三個服務的自動啟動項目。這是查核範圍內的結果，不代表已取得 Windows 自動化腳本。

## 2. 暫存修正與實際部署

`/tmp/rslip-console-fix-20260909` **不是本次診斷建立的修正**，開始檢查時已存在且已部署。使用者已提供其來源：另一工作階段「整理 R-Slip實驗操作流程」，thread ID `01a080d7-1b44-7471-b924-bcb709ee1452`。該階段回報約 10:54 修改、10:57 編譯、10:58 模擬測試通過、10:59 備份安裝，11:00:53 回報開啟 Console v2。以上時間來自該工作階段紀錄，不用 sbRIO 的牆鐘時間直接排序。

下列三份程式與正式專案逐檔相同：`console.cpp`、`console.hpp`、`fpga_server.cpp`。以下執行檔 SHA-256 完全相同：

- 暫存 `fpga_driver.candidate`
- 正式 `/home/admin/rinbo_sbRIO_ws/rinbo_fpga_driver/build/fpga_driver`
- 本次分別讀到的 `/proc/4190/exe`、`/proc/4777/exe`

共同 SHA-256：`c847394fa8ff915fc48013783c8b05b7abc4cb48b4b390fce5957550c830c2c8`；ELF Build ID：`98f8eb822ef2320a2ea2ec2861f1e58049685dbc`。

部署備份已存在於 sbRIO `rinbo_fpga_driver/console-backups/20260908-193636/`，包含原始碼、舊執行檔與 README。舊執行檔 SHA-256 為 `27e54e2adfc861eeae150e0ecee9e64e9c6f7446f7da9bec3a005a3b4deb8401`。全部保留。

**建置目錄尚未同步。** 正式 build 內的 `console.cpp.o` 仍含舊 `InputPanel`、`Console::refreshWindow`；已部署執行檔則含新 `Console::run/stop/execute`。暫存 `build.py` 會編譯替代物件再與其餘原物件連結，與現況吻合。因此可證明部署的是候選版，但不能宣稱正式 CMake 建置產物全部與新原始碼一致。今後需要在獨立 build 目錄完成正常建置與版本核對，不能直接重建覆蓋正在使用的執行檔。

sbRIO 工作區 Git HEAD 是 `c7ddce6`，但整個 `rinbo_fpga_driver/` 顯示為未追蹤內容，不能用 HEAD 當成部署版本。版本以以上雜湊及快照辨識。

bitfile SHA-256 仍為 SOP 值 `78975be61bf8b65db6744835626fdb071e41b45c3d8d8cd29065cb0e21e762f7`。

Orin 安裝的 Bridge 是 symlink，指向 `/home/jetson/rinbo_ros_ws/build/rinbo_ros_bridge/rinbo_ros_bridge`。SHA-256 `c9956d9e88b517e5e9ba699d102444411f1ed9563e5bc14abc4e34c1905f7114`，Build ID `0ff86f3681b18b2a22f3735f5b721ce12512a752`；CMake source directory 是本工作區 `src/rinbo_ros_bridge`。目前沒有 Bridge 程序，不能核對執行中的設定，也未做完整可重現建置來證明源碼與執行檔逐位一致。

## 3. Windows 紀錄分組及未啟動／退出判定

Windows 日誌目錄（使用者提供）：`C:\Users\JasonLiao\AppData\Roaming\RSlipLauncher\logs\`。下列是使用者提供的紀錄內容，本次沒有直接存取 Windows 檔案；不能混成同一次、同版本測試。

| 檔名 | 版本及結果 |
|---|---|
| `20260909-102212-868-communications.log` | v7.3.1，重開機前 Core 7322、FPGA 7729、Bridge 18651 ready，STATE=BridgeReady |
| `20260909-102749-284-communications.log`、`20260909-102753-799-communications.log` | v7.3.1，急停鎖仍在，錯誤 90；沒有完成重新建立通訊 |
| `20260909-103211-856-physicalcleanup.log`、`20260909-103226-781-physicalcleanup.log` | v7.3.1，Orin SSH connection timed out，清理失敗 |
| `20260909-103251-451-stop.log` | v7.3.1，等待 sbRIO SSH 到 10:33:18 才返回 |
| `20260909-103308-842-physicalcleanup.log` | v7.3.1，與前述停止重疊，錯誤 90：另一份停止／實體清理正在執行；使用者指出此等待流程已於 v7.4 修改 |
| `20260909-105518-581-stop.log`、`20260909-105542-415-stop.log` | v7.4，Bridge absent confirmed、Core PID 2762／目前 sbRIO boot ID、shutdown_backend_Fpga_count_0，STATE=PowerUnknown；是停止失敗，並非新版啟動測試 |

**已證實的恢復流程障礙**：舊的通訊成功發生在重開機前；之後的建立通訊受急停鎖阻擋，清理遇到 SSH 逾時與流程重疊；目前沒有 v7.4 清理成功後重建通訊的紀錄。v7.4 停止檢查如實指出服務不足以確認關電，不能把錯誤忽略後視為恢復。

10:55–10:56 的缺 FPGA 快照早於另一階段回報的安裝完成。它不能判定新版部署後的 FPGA 不存在，更不能證明新版崩潰。

Orin 另有 10:36 操作台成功啟動 Core 3756、FPGA 3777 的日誌，但遠端 PID receipt boot ID 是 `71c59791-68ca-4da7-bcff-4a34f731ecf9`，不是目前開機。同次 Orin Bridge 日誌沒有 boot ID／退出碼；Orin 時鐘曾跳動，不僅憑牆鐘時間歸屬其退出。

**部署後的獨立程序生命週期**：

- 初次採樣（sbRIO uptime 1277.96）FPGA 缺席；之後在 uptime 1306.39 啟動 4190，已直接核對新版映像並觀察其存活到 uptime 1448.23。
- sbRIO auth.log 對應 SSH 4187／來源 port 53093，記錄 `Received disconnect ... disconnected by user`、session closed。後續 4190、4187 都不存在。
- 再有 SSH 4774／來源 port 58107 啟動 FPGA 4777（uptime 1492.73），正式 exe、雜湊、stdin/TTY 均已核對。
- 掛斷、程序消失與離線 SIGHUP 結果相符，**但沒有 4190 的 wait 退出碼，不能將特定訊號原因定為確證**。也不能把此部署後事件回套到 10:56。
- 最新 console log 是 append 模式，包含初始化後的 loop 訊息，但沒有 PID／boot／uptime 分段；本次搜尋沒有 `Exit Safely`、異常終止等記錄。初始化早期 `Session opened` 在 console 重導前送往終端，不能因 console log 缺該字串就判定初始化失敗。
- 已查的 sbRIO messages、Orin 當次 journal 及 core dump 位置沒有提供本次 FPGA／Bridge 的崩潰證據；缺紀錄不等於排除崩潰。

先前服務究竟在何時、被何個流程／訊號終止，仍缺完整退出紀錄。**「目前沒有完成通訊重建」已有流程證據；「先前 driver 崩潰」未獲證實。**

## 4. 離線 console 測試

在 sbRIO 獨立 `/tmp/rslip-diagnosis-offline-20260909` 編譯 console 與 fake `FpgaHandler`；只連結 ncurses，不包含 NI FPGA 或網路實作。所有訊號只送到測試自行產生的子程序。原有暫存測試、正式 console 日誌均未覆寫。

| 條件 | 觀察 |
|---|---|
| stdin `/dev/null`，stdout pipe，未知 TERM | 存活，沒有 curses 控制碼；測試 SIGINT 後退出 0 |
| stdin pipe EOF | 存活，沒有 curses 控制碼；測試 SIGINT 後退出 0 |
| headless，SIGHUP 預設處理 | 被 SIGHUP 終止，Python returncode `-1` |
| headless，繼承忽略 SIGHUP（模擬 nohup 的訊號設定） | SIGHUP 後存活，SIGINT 後退出 0 |
| TTY 消失，但沒有 SIGHUP | 程序存活；0.6 秒耗用 60 CPU ticks（100 ticks/秒），出現忙迴圈 |
| 控制終端關閉，前景程序 | SIGHUP 終止，returncode `-1` |
| 既有完整 UI 測試重跑 | 在 `test_pty.py:68` 縮小視窗斷言失敗；不能宣稱全通過 |

上表的掛斷測試是本機 PTY／訊號重現，不是切斷實際 FPGA 的 SSH。fake harness 也不等於完整 driver 的初始化或通訊測試。

源碼對應：`Console::init` 在 stdin 或 stdout 非 TTY 時直接略過 UI；`getch()==ERR` 時無區分 timeout 與終端失效，會繼續迴圈。`fpga_server.cpp` 明確安裝 SIGINT handler，沒有安裝 SIGHUP handler。本次讀到的 FPGA 4190、4777 訊號遮罩也未忽略或捕捉 SIGHUP。

因此 headless stdin EOF 不是本次已證實的退出原因；前景 SSH 掛斷有可重現的退出機制，而 TTY 消失忙迴圈是真實的 console 缺陷。**兩者都尚未與 10:56 缺席建立因果連結，本次不先改碼。** UI 縮放測試失敗另列待查，不藉此宣称遠端恢復失敗的主因。

## 5. 交回 Windows 的具體查核／修改要求

目前已取得使用者提供的分組日誌摘要及 GUI 名稱，仍沒有 Windows v7.4 原始碼、實際啟動 shell 命令及完整退出紀錄。以下不是已確認某行 Windows 程式有錯；不要求再次修正已在 v7.4 修改的停止等待流程。

1. 提供本次 run ID 的「啟動通訊／停止／實體清理」日誌，以及實際送出的 sbRIO、Orin shell 命令。需看出清理後是否真的進入啟動步驟，不能以停止錯誤當成啟動結果。
2. 每次啟動寫入 boot ID、uptime、PID、start ticks、`/proc/PID/exe`、SHA-256、cwd、白名單環境變數及 stdout/stderr 路徑；如果啟動後消失，保留父程序 `wait` 結果。停止命令也記錄同一組程序身分及使用的訊號。
3. 若 GUI 承諾「關閉 Terminal 不停止通訊」，需核對是否誤以 `ssh -t ... ./fpga_driver` 維持服務。本次 FPGA 4190、4777 確實是這種前景終端依附狀態，但來源是另一 console 工作階段，尚不能認定 v7.4「01 建立通訊」也用了同一命令。符合該承諾的啟動方式應明確脫離終端，設定正確 cwd、Core 環境、`</dev/null`、stdout/stderr 日誌及 SIGHUP 策略；不能用自動重啟補救退出。是否實際改 Windows 命令，須先核對原命令。
4. Core 需按 socket 身分核對 port 50051，不能只看 BusyBox netstat 的空輸出。FPGA 需核對 session 成功、持續存活；Bridge 需核對本次 PID、ROS node 及 motor/state、power/state 的連續新回讀，才可標記「通訊就緒」。
5. 保留缺服務與關電失敗檢查。實體關電清理仍必須有操作人實際斷電確認，不能把缺 FPGA 視為關電成功。

## 6. GUI 下一步與待確認的最小實機驗證

**現在可按「遠端日誌」保留現況；先不按「01 建立通訊」、停止／清理或任何上電／動作按鈕。** 原因是另有 Console v2 前景 driver 正在執行，且 Windows 的 PowerUnknown 尚未解除；不擅自中止它或重複啟動。

本次沒有查看「連線檢查」按鈕的實作，不能保證它只讀，故不把它當作已核實安全的驗證入口。若「遠端日誌」不是單純檢視／匯出，也應先向 Windows 核對其行為。

待使用者確認「現有實驗可結束、動力已實際斷開、允許清理並只驗證通訊」，最小 GUI 流程是：

1. 按「實體斷電後清理…」，展開恢復區；**確實斷電後**輸入 `OFF`，按「確認斷電，接續清理」。這一步可能停止現有服務，必須等使用者確認，不代為執行。
2. 清理明確成功後，只按「01 建立通訊」。不要按「02 開啟動力」、03／04／05。
3. 收集本次 v7.4 communications log、兩台 boot ID／uptime／PID、exe 雜湊，確認 FPGA session 成功、Bridge node 及 motor/state、power/state 持續有新回讀，才判為通訊恢復。若清理或建立失敗，保留日誌，不自動重試或略過缺失服務。

若當前實驗必須繼續，就停在保留日誌，不做清理；也不要關掉維持 FPGA 前景程序的 SSH 終端。

本次未執行此實機驗證。console 修正已部署且本次已看到其執行，但不代表 v7.4 完整遠端通訊已恢復。

## 附件

- `identity-final.txt`：較早的 sbRIO 程序、雜湊及 socket 證據。
- `recheck-after-windows.txt`、`orin-recheck.json`：Windows 補充後的最新配對採樣。
- `driver-transition.txt`：4190 的 SSH 掛斷紀錄及後續 4777 程序。
- `new-driver.txt`：診斷期間新啟動 FPGA 的父程序、終端及環境。
- `build-provenance.txt`、`object-symbols.txt`：正式原始碼、候選版本及建置物件差異。
- `lifecycle-results.txt`、`lifecycle.sh`：獨立生命週期測試結果及重現腳本。腳本僅可在已準備好的 fake harness 目錄使用。
- `original-test-result.txt`、`console_snapshot/`：原有測試失敗位置與只讀取得的程式快照。
