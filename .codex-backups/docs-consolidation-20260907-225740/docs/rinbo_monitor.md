# Rinbo 即時監控面板

Orin 執行網頁服務，Windows 瀏覽器查看六腿命令及感測器。面板只有讀取功能，可以先開著，再依原有 Windows 流程執行 Calibration。

## 開啟

本次已在此工作區編譯 `rinbo_ros_bridge`、`rinbo_fsm`、`rinbo_monitor`。新版 Bridge 必須重新啟動才會提供命令鏡像；請在原有動作流程已停止後，以既有操作介面重新啟動 Bridge。

在 **Orin 檔案管理員**開啟 `/home/jetson/rinbo_ros_ws`，雙擊 `Rinbo-Monitor.desktop`。第一次若出現信任提示，選擇「允許啟動／信任並啟動」。它會開啟 terminal 並執行同目錄的 `start_rinbo_monitor.sh`，自動載入 ROS 環境並啟動面板。

也可以對 `start_rinbo_monitor.sh` 選「執行／在終端機執行」。若雙擊 `.sh` 只開啟文字編輯器，使用上面的 `.desktop` 啟動檔，或在 terminal 執行：

```bash
bash /home/jetson/rinbo_ros_ws/start_rinbo_monitor.sh
```

這兩個啟動檔在 Orin 執行；Windows 仍使用瀏覽器查看。啟動檔預設使用 ROS domain 99；若 terminal 已設定 `ROS_DOMAIN_ID`，則沿用該值。可以在命令後加上 `--port 8089` 換埠。若面板已開著，請使用原來的 terminal 與連結；啟動檔不會關閉既有程序。

手動逐步啟動的等效命令：

```bash
cd /home/jetson/rinbo_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=99
ros2 run rinbo_monitor rinbo_monitor --host 0.0.0.0 --port 8088
```

`ROS_DOMAIN_ID` 是 ROS 通訊群組，必須與 Bridge 相同；目前現場流程使用 99。保留 terminal 開啟，按 `Ctrl+C` 只會關閉監控面板。

terminal 會印出帶有 `?token=...` 的存取連結。Windows 開啟該連結，把 `0.0.0.0` 換成 Windows 原本連接 Orin 的 IP，保留完整 token。若不確定 IP，在 Orin 執行 `hostname -I` 查看。Orin 本機可把主機位址換成 `127.0.0.1`。

每次啟動都產生新的隨機金鑰，網頁和資料 API 都要求金鑰驗證；驗證後以 HttpOnly、SameSite=Strict cookie 保留存取狀態，網址會自動移除金鑰。重新啟動後請使用新的存取連結。此 HTTP 連線沒有加密，請只在可信任網路使用。預設不帶 `--host` 時僅本機可看；`--host 0.0.0.0` 則允許持有金鑰的 Windows 瀏覽器連線。

若需重新編譯：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rinbo_ros_bridge rinbo_fsm rinbo_monitor
source install/setup.bash
```

## 畫面怎麼看

每條腿的表格有「收到」和「轉送」兩欄：

| 資料 | 意義 |
|---|---|
| 收到 | Bridge 收到控制器的原始命令，包含稍後可能被安全檢查拒絕的命令 |
| 轉送 | Bridge 已將命令交給 gRPC 傳輸層；包含 Bridge 自行產生的停止命令。**不代表 sbRIO 收到或執行成功** |
| enable / direction / state | 原始主馬達控制欄位；direction 為原始布林值，左右腿不能直接當成相同轉向 |
| PWM 幅度 | 控制程式的輸出幅度。協定欄位名為 `voltage`，此處不是實測電壓 |
| reset | 當下歸零請求；只出現一個封包也會累計並保留事件。請求數不等於完成數 |
| 主馬達 counts / tick_count | 原始編碼器位置與 tick_count 回讀；counts 是編碼器刻度，未轉成角度 |
| Hall | 原始感測器值；依現有校正程式，0 表示原點觸發、1 表示未觸發 |
| 伺服目標／回讀／誤差 | 編碼器刻度；誤差是轉送目標減回讀。mode=0 或資料過期時不計算誤差 |
| 電流／Bus | L1…R3 對應電源 ch1…ch6；Bus 顯示 ch7，沿用目前站點設定 |
| seq | 保留原命令來源序號，可比較收到／轉送。Bridge 自行停止命令可能是 0；不是 sbRIO 確認序號 |

上方顯示每路資料距離接收的時間、總樣本數，以及存在時的 ROS 來源時戳年齡。預設超過 0.5 秒即標為過期；這是**畫面提示**，不更動控制器較嚴格的安全門檻。過期的最後數值仍保留供排錯，曲線留白，不表示現在仍有效。`/motor/state` 的 ROS 時戳是 Bridge 的接收時間，不是 sbRIO 的硬體時間。

## 「即時」有多快

目前是近即時監控，沒有零延遲或固定最長延遲保證：

- **資料接收：** ROS callback 收到一筆資料就更新後端，不必等待下一次網頁刷新；實際接收頻率由 Bridge 發布頻率、通訊與電腦負載決定。
- **畫面刷新：** 每次 HTTP 回應與繪圖完成後等 200 毫秒，再請求下一筆，所以最高約 5 Hz（每秒 5 次），實際會稍慢。瀏覽器分頁在背景、網路慢或電腦忙碌時，延遲可能更長。
- **曲線取樣：** 後端每 100 毫秒記錄一次最新值，目標為 10 Hz；多個曲線點可能在一次畫面刷新時一起顯示。
- **短暫 reset：** 在資料接收時另存事件，不要求它剛好出現在 200 毫秒的畫面刷新時刻；但 best effort 傳輸仍可能漏掉封包。

尚未量測「實際機械變化 → sbRIO → Bridge → Windows 畫面」的完整延遲。200 毫秒是網頁請求間隔，不是完整延遲的實測結果。這個面板適合看校正進度、停在哪條腿與命令／回讀差異；毫秒級尖峰與封包完整性需另外使用高速紀錄分析。

曲線保留約 60 秒、每秒 10 點，可切換腿查看主馬達位置／PWM 與伺服目標／回讀。主馬達位置與 PWM 各用一張獨立圖，PWM 從 0 起算；時間範圍可選最近 10／30／60 秒。位置保留原始 counts，不取 `2π` 餘數，也不自動展開角度。紅色直線標示該腿的 reset 請求接收事件，並中斷跨越該事件的主馬達位置連線；它不是硬體歸零確認。

「暫停畫面」只凍結即時數值與曲線，後端繼續收集／錄製，錄製按鈕及狀態仍更新。「下載最近 60 秒快照」下載後端當下的資料、曲線與最新 300 筆事件；這份即時緩衝在面板啟動時自動開始收集，會循環覆蓋。它與下方的手動錄製是兩份不同資料。

## 自己選擇開始／停止錄製

1. 用捷徑啟動面板並開啟網頁。若更新前的面板仍開著，只重開監控面板程序並使用新 token 連結；本次不需要重新編譯或重啟 Bridge。
2. 在準備執行 Calibration 前，按 **開始錄製**，確認狀態變成「錄製中」。按鈕不會啟動馬達。
3. 依原有流程操作機器。完成或發生異常後，按 **停止錄製**。這只停止存檔，不會停止機器。
4. 等待狀態變成「錄製已停止」，按 **下載本次錄製 JSON**。

錄製從伺服器接受開始請求時起算，立即存一份最新狀態，之後以 10 Hz 保存命令與感測器快照；收到的 reset、Hall 變化及 ROS 日誌另外保存。停止請求之後不再加入新資料，背景寫入完成後才開放下載。快照保留各來源的接收年齡、來源時間戳、過期標記與完整訊息欄位，包括主馬達 enable／direction／PWM／reset、伺服、Hall 及各路電壓電流。過期資料仍帶著過期標記保存，不當成新回讀。

手動錄製不會因超過 60 秒而覆蓋前面的資料。預設儲存在 `/home/jetson/rinbo_ros_ws/log/monitor_recordings/`（從工作區啟動時），每次有獨立檔名；也可以在啟動指令加上 `--record-dir /其他目錄`。開始下一次不會刪除舊檔，舊檔可在 Orin 的該目錄取得；網頁下載按鈕指向目前這一次。多個瀏覽器共用同一個錄製狀態。

每次最多錄製 30 分鐘或約 64 MiB，達限制自動停止並顯示原因。寫入佇列有上限；若磁碟太慢會停止錄製、標示缺漏，而不阻塞 ROS callback。檔案写入期間副檔名為 `.json.partial`，正常停止後才改為 `.json`；程式正常退出會嘗試完成存檔，斷電或強制殺程序可能留下不完整的 `.partial`。

手動錄製 JSON 使用 `schema_version: 2`：

- `started_unix_s`：開始錄製的電腦時間。
- `entries[]`：依接收／取樣順序保存；`kind` 為 `sample` 或 `event`。
- `t_recording_s`：從本次按下開始錄製起算的秒數。
- sample 的 `data.t_monitor_s`、event 的 `data.t`：從監控程序啟動起算的秒數，可與原本即時曲線比對。
- `summary`：錄製長度、實際保存的樣本／事件數、停止原因與寫入缺漏數。缺漏數只計本機寫入佇列溢位，不能計算傳輸途中遺失的 ROS 封包。

畫面自動收集與手動錄製都不會補讀過去資料。請在需要觀察的動作之前按開始。

## 校正的已知證據

最新下載的 JSON：`elapsed_s = 479.768`，代表面板已開啟約 8 分鐘；600 個曲線點僅涵蓋 `+419.859` 到 `+479.758` 秒。截圖則顯示 `+695` 到 `+755` 秒，不是這份檔案的同一時間段。

這份 JSON 顯示 L2、R1、R2、R3 已完成，失敗原因是 `CALIBRATION SAFETY STOP: hall search timeout: L3`（約 `+436.323s`）。L3 在校正期間的轉送 PWM 約為 80，但位置回讀始終為 1，Hall 未觸發。使用者已確認 L3 主馬達確實轉動，因此可確認位置回讀未追蹤實際運動；應優先檢查 L3 編碼器、接線與 sbRIO 回傳通道對應。僅凭這份監控檔仍不能判定是哪個硬體元件故障。

面板及 Bridge 沒有對顯示位置做角度取餘數，這份資料也沒有顯示每圈 55296 counts 的折返。截圖的跳變仍需對應時間段的資料確認；歸零、回讀跳變或缺資料都不能直接當成 `2π` 問題。此次沒有加入會隱藏跳變的自動角度展開。

先前日誌：

2026-09-07 21:09 的 `rinbo_cali_51374_1788786581036.log` 顯示站點屏蔽 L1、五顆健康伺服定位成功；R1、R2、R3 得到歸零回讀。L2 已經 Hall 觸發並進入 RESETTING，但未記錄 DONE；L3 尚未記錄 Hall 觸發。程序後來收到 SIGINT。校正的安全原因原本只寫 stderr，這次加上 ROS error log，讓之後的停止原因能在面板出現。

程式每腿只送一個封包的 `reset_position=true`，下一個回讀即改回 false；這是需要量測的可疑點，尚不能斷言是遺失封包。這次未提高 PWM、延長超時、改寫校正完成條件或自動屏蔽 L3。

下次校正失敗後，先保留／下載面板紀錄，再看：

1. L2 的「收到 reset」和「轉送 reset」是否都增加？主馬達位置之後是否接近 0？
2. L3 是否持續有 enable 與 PWM？位置是否變化、Hall 是否曾從 1 變 0？
3. 是否先出現電壓下降、電流異常、資料過期或 Bridge 安全停止？

這些觀察能縮小問題範圍，但面板沒有 sbRIO 命令確認資料，無法單獨證明硬體收到了 reset。

## 通訊與驗證

面板訂閱 `/rinbo/monitor/motor_requested`、`/rinbo/monitor/motor_forwarded`、`/motor/state`、`/power/state`、`/rinbo/motor_output_enabled`、`/rinbo/motor_arbiter_ready` 及 `/rosout`。只保留名稱以 `rinbo_` 或 `redrhex_` 開頭的節點日誌。

**不要讓面板、`ros2 topic echo` 或錄包工具直接訂閱 `/motor/command`。** 現有 FSM 要求該 topic 只有 Bridge 一個接收者；新增接收者會使校正無法啟動或安全停止。監控鏡像不參與這個控制通道，也不放寬其限制。

鏡像採用 best effort（允許丟失監控資料），避免監控接收端要求控制通道等待重送。reset 事件保存的是面板確實接收到的封包；未觀察到不能證明沒有發出。來源資料各自更新，不保證同一取樣時刻。面板使用 Python 標準 HTTP server，沒有額外網頁框架或外部 CDN。

離線驗證命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=1 \
  python3 -m pytest src/rinbo_monitor/test -q
ctest --test-dir build/rinbo_fsm --output-on-failure \
  -R 'test_motor_arbiter_handshake|test_cali_multileg|test_latest_state_safety'
ctest --test-dir build/rinbo_ros_bridge --output-on-failure
node src/rinbo_monitor/test/test_panel.js
```

ROS 整合測試只在 localhost domain 232 運作，不啟動硬體 Bridge；測試確認面板不增加 `/motor/command` 接收者、不發布 motor/power 命令，並接收模擬 reset／位置／Hall。實機校正問題仍待新一輪回讀證據確認。

2026-09-07 驗證結果：三個套件編譯成功；9 項 Python 測試、85 項 C++ 回歸測試通過。JavaScript 的六腿渲染、事件、暫停、過期與無資料狀態檢查通過。本機 HTTP 實際驗證未帶金鑰回傳 401、登入後網頁與 API 回傳 200。Orin 未安裝可執行的 Chromium，因此尚未做真實瀏覽器截圖驗證，也未啟動實機校正。面板啟動於 domain 99 時尚未收到 Bridge 資料，需依原流程啟動新版 Bridge。

錄製功能更新驗證：16 項 Python 測試通過，面板套件編譯成功；新增起訖範圍、完整快照／短事件、超過 60 秒保留、舊檔保留、存檔完成才下載、過期標記、時間／大小上限及寫入佇列溢位測試；前端檢查包含錄製按鈕、獨立位置／PWM 圖、原始 counts 與 reset 標記。
