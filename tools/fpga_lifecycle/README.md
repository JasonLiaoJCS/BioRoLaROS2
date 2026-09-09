# FPGA console／SSH 生命週期修正交接（2026-09-09）

## 交付狀態

本目錄是**尚未部署的修正候選原始碼**。以唯讀取得的 sbRIO 正式 Console v2 為基準，保留另一工作階段的輸入驗證、貼上防護、裝置 mutex 與日誌分流。正式 sbRIO 原始碼、執行檔、bitfile、Orin Bridge 及其他既有工作區修改都未被本次覆寫。

已做 Orin 完整假硬體執行檔測試，以及 sbRIO 隔離目錄的正式 NI／protobuf／gRPC 標頭物件編譯。sbRIO 只執行編譯器，沒有執行候選 driver。沒有啟動／停止真實 Core、FPGA、Bridge，也沒有發送電源或動作命令。正式版完整重新連結、部署與實機驗收仍未執行。

- `src/`、`include/`：供 sbRIO 專案合併的候選程式。
- `baseline/`、`baseline-sha256.json`：本次取得的正式版本及逐檔 SHA-256。
- `lifecycle.patch`：相對正式版的差異；不要將整個舊快照覆蓋回去。
- `prepare_candidate.py`：先核對來源雜湊，再複製至**新的獨立目錄**並套用候選；來源不同就中止，不覆蓋新修改。
- `fpga-service`：供 Windows 安裝後使用的背景啟動、身分核對、只讀監看及明確停止介面。
- `test/`：只使用模擬 NI 與模擬通訊的測試。
- `evidence/`：離線結果、正式標頭編譯結果、唯讀身分與 Bridge 補查。

## 已證實缺陷與事故原因界線

原部署執行檔 SHA-256：`c847394fa8ff915fc48013783c8b05b7abc4cb48b4b390fce5957550c830c2c8`。正式 `console.cpp`：`e2438c64480bb10d0a751ac5625aeb29b0bb7fa6af44054a2c7c7e689a503a66`；`fpga_server.cpp`：`ad008f2a61cf0f0bc4a364c5febf7e17b8c1355d84aec797d0dcace907d9c373`。本次複查仍相同。正式 build 目錄的部分舊物件與已部署候選不同，不能拿該目錄的舊 `console.cpp.o` 判斷執行中版本，也不能只重連舊物件。

| 證據 | 可下的結論 | 不能下的結論 |
|---|---|---|
| Windows 確認 `Open-FpgaConsole.ps1` 用 `ssh -t`、取得 console lock 後 `exec ./fpga_driver`；PID 4190／4777 的 stdin、session、父 sshd 與此相符 | 「監看」實際擁有前景 driver，確實耦合 SSH 終端 | 不能據此推定所有背景啟動入口都如此 |
| 原正式主程式只捕捉 SIGINT；控制 PTY 掛斷測試 returncode=-1 | 未處理 SIGHUP，預設終止可跳過 C++ 收尾 | -1 是 Python 對「由 SIGHUP 終止」的表示，不是應用程式主動返回 -1 |
| 原 `getch()==ERR` 一律 continue；TTY 消失但不送 HUP 時 0.6 秒耗用 60 ticks，CLK_TCK=100 | 終端失效可產生忙迴圈，已重現 | 不能證明事故時曾發生 CPU 滿載 |
| 無 TTY／pipe EOF 原本會略過 console UI | headless EOF 本身不是已證實的 driver 退出原因 | 不應把 EOF 一律解讀為背景服務必須退出 |
| 舊 SIGINT 正常路徑印 `Exit Safely`，FpgaHandler destructor 只 Close／Finalize | 舊訊息缺少明確的停用／關電寫入及回讀驗證 | 關閉 NI session 不能被當成實體電源已斷開的證明 |
| 4190 所屬 sshd 4187 記錄 user disconnect；4777 所屬 sshd 4774 又有 `Timeout, client not responding`／session closed | 兩次都有對應 SSH 終止紀錄，與 SIGHUP 機制吻合 | 沒有兩個 driver 的父程序 wait 狀態，仍不能指定其真正最後退出訊號或排除其他原因 |

原診斷位於 `docs/diagnostics/rslip_20260909/`；本文件補充後續結果，不修改原始歷史快照。跨機器牆鐘不直接排序，使用 boot ID、uptime 與 start ticks。

## 責任與安全退出

**背景 driver** 是唯一 FPGA session 擁有者，接收既有 gRPC motor／power 命令。**監看介面**唯讀日誌或 ROS 狀態，不持有 FPGA session，不啟動、不重啟、不初始化 driver。正常輸入繼續走既有 Windows 控制流程與 Bridge 的仲裁／安全鎖，不從監看視窗直接操控硬體。

保留 `fpga_driver --interactive` 作為明確的維護模式；它本身就是 driver，不是附加到背景 driver 的客戶端。單一 driver 存在時拒絕第二份。關閉這種互動終端的語意是「要求安全停止整個 driver」，Windows 不能把它標成只讀監看。

停止來源包括 SIGINT、SIGTERM、SIGHUP、SIGPIPE、互動 Ctrl+D、TTY HUP／ERR／NVAL、stdio EOF／錯誤、無法初始化 curses 或無法寫出畫面。signal handler 只設停止旗標，不呼叫 NI 或 mutex；主迴圈與 UI 拒絕後續命令、停止並 join UI，接著：

1. 六個馬達全部 `EN=false`、`input_voltage=0`、`state=false`。
2. 電源控制依 Power、Signal、Digital 寫 false。
3. 回讀上述 21 個控制暫存器，確認值均為零／false，並檢查 NI 呼叫狀態。
4. 留下 `shutdown_controls_verified physical_power_unverified`，或 `shutdown_FAILED PowerUnknown`；最後 Close／Finalize。父 wrapper 的 `wait` 保存最終程序退出碼。

有任何先前 NI 錯誤仍逐一嘗試 off 寫入，不因累積錯誤跳過其他關閉動作。不重試啟動、不新增上電／encoder reset／FPGA Run 循環。錯誤退出不代表實體關電成功；即使控制暫存器回讀全零，也不能代替外部電壓／接觸器或既有新鮮回讀確認。

| driver 退出碼 | 意義 |
|---|---|
| 0 | SIGINT／SIGTERM 正常退出，off 控制回讀通過；仍非實體斷電證明 |
| 2 | 終端喪失、HUP／PIPE 或 NI 故障引發的異常退出；已走 off 流程且控制回讀通過 |
| 3 | off 驗證失敗、初始化／例外或獨占鎖拒絕；須讀原因，不可當成關電成功 |
| 64 | 參數錯誤，或要求 interactive 卻沒有輸入／輸出 TTY；在 FPGA 初始化前拒絕 |

`getch` 的正常 timeout 與故障分開處理；立即 ERR 也至少等待至該回合開始後 100ms。輸出設為 nonblocking，避免沒有人讀終端時 UI 阻擋停止。失去終端後 stdout/stderr 留在日誌，不還原到消失的 TTY。

新的 `/tmp/rslip-fpga-driver.lock` 由 driver 在 NI 初始化前取得，至 destructor 完成才釋放；鎖檔不刪除。另掃描既有 `fpga_driver` 拒絕舊版實例。Windows 原 console lock 不是背景 driver 的生命週期鎖，不能用它代替新鎖。切換時各入口必須使用同一新執行檔；不能保證一份之後又被手動啟動、完全不遵守新鎖的舊執行檔不造成競爭。

## Windows 確切介面（候選部署後才可使用）

`start` 先唯讀檢查執行檔包含新版關閉能力標記；舊 driver 不會被這個新入口啟動。不用執行舊版 `--help` 做探測，因為舊版可能忽略參數而初始化硬體。

將本目錄 `fpga-service` 安裝為：
`/home/admin/rinbo_sbRIO_ws/rinbo_fpga_driver/fpga-service`，保持可執行；新 driver 安裝位置沿用 `build/fpga_driver`。以下是交接命令，本次未對真實硬體執行：

```powershell
# 由「建立通訊」流程，在既有 Core/socket、bitfile 雜湊及安全前置檢查通過後明確呼叫。
ssh -T admin@192.168.30.254 /home/admin/rinbo_sbRIO_ws/rinbo_fpga_driver/fpga-service start

# 查詢程序狀態，不啟動服務。
ssh -T admin@192.168.30.254 /home/admin/rinbo_sbRIO_ws/rinbo_fpga_driver/fpga-service status

# Open-FpgaConsole.ps1 改呼叫此命令。關閉視窗/Ctrl+C 只結束 tail/SSH。
ssh -T admin@192.168.30.254 /home/admin/rinbo_sbRIO_ws/rinbo_fpga_driver/fpga-service watch

# 由明確的停止流程呼叫；身分核對後送 SIGINT，不自動升級成 SIGKILL。
ssh -T admin@192.168.30.254 /home/admin/rinbo_sbRIO_ws/rinbo_fpga_driver/fpga-service stop
```

- `start` 不啟動 Core、不建立 Bridge、不發上電命令；Windows 原本的前置檢查仍是必要步驟。內部只執行一次 `nohup setsid ... _run`，stdin=/dev/null、stdout/stderr=每次專屬日誌，driver 使用 `--headless`。STARTED 僅指程序身分已取得，**不是通訊就緒**；後續仍核對本次 Session opened、持續存活、唯一 Bridge、連續新 motor/state 與 power/state。
- 同一管理器的存活實例可 idempotent 返回；既有未管理 driver 不採用、不重啟。沒有上一份退出 receipt 時拒絕再次 start。status／watch／stop 都沒有隱含啟動。
- `stop` 核對 boot ID、PID、start ticks、exe，再送 SIGINT，等 `.exitcode` 至多約 10 秒。非零、逾時、receipt 不完整、driver 已缺席皆失敗；不把「找不到程序」當成新一次關電成功。
- 每次資料保存在 `/home/admin/.local/state/rslip-fpga-service/run.*/`：`identity`、`driver.log`、`exitcode`。`current` 只記目前 run 名稱。日誌包含 wrapper PID、driver 雜湊、driver boot/PID/monotonic 時間、退出原因與 wrapper wait 結果。
- 正常整機停止順序仍由 Windows 管理：停止動作來源、確認馬達停用及新鮮全關電回讀，再停止 Bridge、driver，最後依所有權停止 Core。急停／回讀不足保持既有安全鎖與 PowerUnknown。只有 driver 而無 Bridge 時，明確的 driver stop 可嘗試本機 off 寫入，但其返回不能冒充 ROS 新鮮回讀或實體關電確認。

**監看與輸入分開：**背景模式不讀 stdin，不接受將 `:P`／`:M` 字串送進 SSH 作為指令。背景驅動的命令介面仍是 gRPC `power/command`、`motor/command`，Windows 經 Orin Bridge 的 ROS `/power/command`、`/motor/command` 與現有仲裁流程送出。狀態使用既有 `/power/state`、`/motor/state`。需要原始 console 指令時必須進入明確維護流程、確認沒有背景 driver，再手動啟動 `fpga_driver --interactive`；不能由「監看」自動切換或自動停止背景服務。

### 與既有背景方式相容

`rinbo_control/sbrio.py` 的 `nohup "$driver" </dev/null >"$driver_log" 2>&1 &` 與 Windows 類似背景啟動仍相容：無參數保留 auto-TTY，無 TTY 不建立 UI，EOF 不停止背景服務；新執行檔仍捕捉 SIGINT，SIGTERM 也走同一 off 流程。

不必為更換監看入口而重啟現有背景 driver。由舊管理器啟動的 driver，監看應直接 `tail -n 80 -f <該次已核對的日誌路徑>`，停止仍使用原管理器的 PID receipt；新 `fpga-service` 不自動採用它。後續明確的一次新建立通訊才選擇切換管理器，不能讓兩個管理器各啟動一份。

對新版本，不能再依賴 `nohup` 的忽略 SIGHUP 設定保命：driver 主動將 HUP 納入 off 流程；`setsid` 與全 stdio 重導才是背景服務脫離 SSH TTY 的方法。舊背景啟動可加上 `setsid`，並採用上述 wait wrapper／身分記錄；不要移除既有安全前置檢查或加入自動重啟。

## Bridge：獨立結論及後續新事件

Windows 提供的最近一次歷史成功啟動是 v7.3.1、`20260909-102212-868-communications.log`，Orin boot `b2960697-92e9-47ec-b589-865440847537`、Bridge 18651、start ticks 77195、wrapper 18649，屬於上一次開機。其 `.exitcode` 暫存檔已不存在，未取得最終 wait 狀態；`REMOTE_EXIT=0 GATE=start unique Bridge` 只是啟動腳本成功。另一個 11882 的 exit=0 不可套用。

本次 boot 是 `b0008f58-5739-453d-be5f-aa3fa7d3bd72`。依 Windows 提供的檢索結果，截至 12:06 現存日誌無此 boot 的 Bridge 啟動記錄；三份 10:55:18、10:55:42、12:06:10 Stop 都先三次確認 Bridge absent，再在 `shutdown_backend_Fpga_count_0` 中止，沒有進入恢復或停止 Bridge 的階段。12:06:36 急停因 `/power/command` 無 subscriber 失敗，沒有成功關電回覆。

所以目前有證據支持的缺席原因是**此前重開機後，已記錄的通訊恢復流程尚未完成 Bridge 建立**。11:04 是缺席的觀測時間，不是已確認的死亡時間。缺席、Bridge 崩潰、console 導致 Bridge 退出是三個不同命題；後兩者未獲證实。現有 Orin journal 無對應 Bridge crash／OOM 記錄，也沒有歷史父程序 wait，故不排除其他工作階段曾啟動又退出。

本機 `rinbo_control.runtime.Child` 是另一種生命週期：透過 `guardian.py` 設定 parent-death SIGINT；操作台消失可要求其 Bridge 子程序停止。這是另一個可行退出機制，沒有本次 PID／parent 證據就不能指定為歷史原因；本次未修改該安全保護。

**診斷進行中的新事件：**12:21:07 唯讀採樣（Orin uptime 6149.89）已出現 Bridge **32424**、start ticks **599694**、wrapper **32422**，日誌 `/tmp/rslip-launcher/20260909-121824-216/rinbo_ros_bridge.log`，stdin `/dev/null`，無 controlling TTY。exe SHA-256 仍為 `c9956d9e88b517e5e9ba699d102444411f1ed9563e5bc14abc4e34c1905f7114`。父 bash 命令直接證實其 wait／寫 `.exitcode` 機制，目前仍在等子程序，不存在退出碼屬正常。

同一輪 sbRIO 採樣（uptime 6167.03，boot 未變）已是 Core **7589**、FPGA **7996**，不再是 Core 2762；FPGA 仍是原 c847… 版本、無 controlling TTY。這些都是**其他工作階段的新啟動**，本次沒有介入，也沒有把它們的存在宣稱為已通過完整實機安全驗收。新事件不能回填為 10:55–12:06 期間已成功啟動的證據。

## 離線重現與限制

```sh
sh tools/fpga_lifecycle/test/build.sh /tmp/rslip-lifecycle-tests-20260909
python3 tools/fpga_lifecycle/test/test_lifecycle.py /tmp/rslip-lifecycle-tests-20260909
python3 tools/fpga_lifecycle/test/test_service.py /tmp/rslip-lifecycle-tests-20260909
g++ -std=c++14 -Itools/fpga_lifecycle/include tools/fpga_lifecycle/test/endpoint_test.cpp -o /tmp/rslip-lifecycle-tests-20260909/endpoint_test
/tmp/rslip-lifecycle-tests-20260909/endpoint_test
```

`build.sh` 將三份候選正式 .cpp 一起編譯，NI 呼叫與 gRPC NodeHandler／訊息皆替換成 fake；不連結 `NiFpga.c`、NI runtime 或 gRPC，`fake-linkage.txt` 可檢查。測試只對自行產生的 fake PID 發訊號。`vendor/NiFpga.h` 只在測試副本加上 aarch64 可編譯宣告，不能拿去部署；正式標頭相容性另外用 sbRIO 編譯驗證。

本次 25 項生命週期案例通過。兩種 PTY 喪失測試約 0.003 秒退出，斷線後 CPU 約 0.59–0.65 tick；立即 ERR 的 0.6 秒觀測為 9 ticks，與同一 fake 主迴圈無終端的 8–10 ticks 相當，沒有舊版 60 ticks 的滿載。數字僅是本次 Orin 模擬測量，不是實機停止時限保證。

完整結果見 `evidence/offline-results.json`：包含無 TTY／EOF、各訊號、PTY 關閉、ERR CPU 上限、故障注入、重複驅動、正常停止。service 測試驗證實際腳本的背景 stdio/TTY、單一初始化、重複 start 沿用、關閉 watch 不影響 driver、stop receipt；endpoint 單元測試覆蓋 pipe EOF/HUP 與已關 FD/POLLNVAL。

這些證據確認軟體處理與 NI 呼叫順序，不能證明真實 NI 關閉寫入或 readback 一定成功。實機驗收需另安排已知安全狀態；本次依指示不執行。新舊日誌與證據不合併為同一次事故。
