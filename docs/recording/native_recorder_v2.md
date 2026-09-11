# 原生記錄器 v2 與動作 observer 相容性

2026-09-11。只修改觀察、檔案與介面；電流／電壓門檻、PID、PWM、disabled_legs 與動作策略不變。

## 已查證問題

正式 `ros2 run` 透過工作區 install overlay 啟動的 `rinbo_cali`、`rinbo_standing`、`rinbo_tripod` 原先都指向 9 月 9 日 build 二進位檔，沒有之後 header 加入的被動 recorder 例外。`rinbo_tripod_rslip` 是 ROS 節點名，不是此工作區的 executable 名稱。

原始路徑、symlink、SHA256、mtime 與 AMENT_PREFIX_PATH 已保存於 `docs/diagnostics/recorder_native_20260911/before-executables.json`。不能用 header 的修改時間代替執行檔驗證。

本次開始時 recorder PID 53632 的實際映像 SHA256 為 `bfeae6d0caf24066158480b21f638a47b348fecddbb85cf66ccd8bf7e0ff87d7`，帶著 Windows adapter remap，仍開啟原 summary/events 檔案。最近一列各 age=-1，表示尚未收到相應 topic，並非有效量測正在增加。這是該列的證據，不推定機器人供電或動作狀態。沒有關閉、停止或接管現有錄製。

## 正式 GUI 命令

不需 Windows 先 source ROS、發布名稱、trigger 或檢查機器人參數：

```bash
python3 /home/jetson/rinbo_ros_ws/tools/rinbo_recorder_client.py version --json
python3 /home/jetson/rinbo_ros_ws/tools/rinbo_recorder_client.py start --name my_test --json
python3 /home/jetson/rinbo_ros_ws/tools/rinbo_recorder_client.py status --json
python3 /home/jetson/rinbo_ros_ws/tools/rinbo_recorder_client.py stop --json
python3 /home/jetson/rinbo_ros_ws/tools/rinbo_recorder_client.py export --json
```

- 預設 domain99；隔離測試使用 `--domain 232 --no-start`。支援 `--request-id <字串>`（最多80字元）、`--timeout 8`。
- start 在 recorder 原生單次回呼中同時套用名稱、建立新一輪並開檔，回覆才是名稱 ACK。名稱會清理成安全檔名，回覆的 `name` 為實際採用值。
- 已錄製時再 start（即使換名稱）回覆 already_recording、原本的 name/run_dir，不切檔也不重命名；先明確 stop，再 start 才是新一輪。
- stop 是關檔操作，不是刪檔，也不停止 Bridge、FPGA、Core 或動作。再次 stop 仍回覆實際關檔狀態。磁碟失敗不能偽裝成功。
- Windows／SSH 退出不等於 stop。正式 service 不依附 SSH，沒有無條件 restart，也不因 recorder 啟動而建立機器人通訊。
- status、export 不自動啟動任何 recorder。start 只有在沒有既有 recorder 時可啟動 `rinbo-recorder.service`；服務初始 auto_start=false，再由本次原子請求開始。
- 若舊 recorder 還存在，回 `legacy_recorder_active`、exit21；保留其 adapter 及原控制路徑，不殺舊程序、不啟動第二個。新版程序存在但 API 不通則 `api_unavailable`、exit31，recording=null，不猜已停止／已關檔。
- Windows 建議 start timeout45秒、status/stop30秒、export180秒。ROS ACK 等待預設8秒；export 尚需本機檔案快照壓縮時間。超时不得推定已停止，可再查 status。

## JSON 契約

```json
{
  "protocol": 1,
  "schema_version": 2,
  "version": "native-recorder-v2-20260911",
  "request_id": "...",
  "action": "start",
  "status": "recording",
  "exit_code": 0,
  "recording": true,
  "name": "my_test",
  "run_dir": "/home/jetson/rinbo_logs/20260911_..._my_test_...",
  "files_closed": false,
  "disk_error": null,
  "command_source": "diagnostic",
  "rows": {"summary": 0, "events": 0, "commands": 0, "power_samples": 0, "controller_samples": 0},
  "topics": {"power": {"state": "unavailable", "age_s": null}},
  "files": [{"name": "summary.csv", "path": "/home/jetson/rinbo_logs/.../summary.csv", "bytes": 1234}]
}
```

status 另外列出 motor、power、requested、forwarded、controller、safety、safety_detail。沒有接收過為 unavailable；已收到則回 age_s、last_received_steady_s，具 header 的資料另有 source_stamp_ns/source_seq/source_age_s。超過1秒標為 stale，**這只是記錄器資料提示，不是新增機器人保護門檻**。安全事件本來就不是週期 heartbeat，unavailable 不代表已有故障。

rows 是已成功寫入並 flush 的資料列計數，不含標頭。磁碟故障時保留最後確認的計數及 disk_error，不能把未完成列當成功；flush 不等於已具備突然斷電的檔案系統持久性保證。

exit0 表示此次原生操作完成；2=CLI輸入不正確；10=服務不存在；20=原生拒絕；21=舊 recorder 尚未交接；31=API/ACK不可確認；50=磁碟／檔案失敗。非零保留 reason 與 stderr。

export 會先由 recorder 回呼 flush 所有檔案與各檔長度，再由 client 只複製這些長度，產生一致截斷點的 tar.gz。原檔可繼續錄製。回覆增加：

```json
{"export":{"path":"/home/jetson/rinbo_logs/exports/...tar.gz","bytes":12345,"sha256":"...","snapshot":true}}
```

Windows 用既有 SSH/SFTP 下載此 path，下載後核對 bytes/SHA256。不要刪除原始 run_dir。此接口不自動上傳到外部平台。

底層原生服務 `/rinbo/recorder/control` 使用既有 ROS 型別 `rcl_interfaces/srv/SetParametersAtomically`，string parameters 為 action、name（start）、request_id；`result.successful` 與 `result.reason` 中的 JSON 是 ACK。不修改 rinbo_msgs ABI，也不讓 GUI 分兩個 topic 設定名稱與開始。一般 Windows 只需呼叫上述 Python client。

## 資料來源與欄位

| 檔案 | 內容 |
|---|---|
| metadata.yaml | schema/version、實际 topic、QoS、來源語意與 run_name |
| summary.csv | 保留舊欄位，再附加 requested_* / forwarded_*、seq、age；100Hz 摘要 |
| commands.csv | 每一筆收到的 requested/forwarded 命令，含 source、訊息時間與seq |
| power_samples.csv | 每一筆收到的 PowerState，各通道電流／電壓、訊息時間與seq，不經100Hz摘要抽樣 |
| controller_samples.csv | 每一筆原生 ControllerDebug，含時間／seq與既有全部控制欄位 |
| events.csv | 舊 SafetyEvent 欄位保留，附加版本、event_kind、事件時間/seq與detail_json |

診斷模式直接 best_effort depth100 訂閱 `/rinbo/monitor/motor_requested` 與 `/rinbo/monitor/motor_forwarded`，不訂閱 `/motor/command` 或 `/power/command`。motor/state、power/state 是 reliable depth50。舊 `cmd_*` 欄位只作 requested 的相容別名，**不是 applied**；forwarded 也只是 Bridge 向後端送出的副本，不等於硬體執行 ACK。

`command_source:=legacy` 保留舊控制 topic 觀察方式，供已更新的動作節點相容使用。新 GUI 使用 diagnostic，不需要它。兩種模式都沒有硬體命令 publisher，也沒有 sbRIO SSH。

rosbag launch 預設改成監看 topics，明確拒絕把 `/motor/command`、`/power/command` 加回，並使用 `diagnostic_bag_qos.yaml` 指定 mirror best_effort。不能先開 bag，卻因尚未发现發布者而默默使用不相容 reliable。

## 原生過流事件的完整路徑

三個動作節點只在既有 first safety stop 路徑增加被動明細發布。命令停止仍先執行，記錄失敗不能阻止停止或清除第一原因。原有過流計數、25筆／5A、10A硬上限與L3排除規則未修改。

已驗證的 `/power/state` → 原生 power guard/policy → first safety stop → `/rinbo/safety_detail`（reliable）→ recorder callback → events.csv 直接寫入並 flush。detail_json 包含 quantity、channel、measured、threshold、power_seq、power_stamp_ns、event_seq/event_stamp_ns、controller_state、tau/ratio/cycle、全8通道值及disabled mask。舊 `/rinbo/safety_event` message 不變；Tripod原事件也保留。Cali/Standing新增相容事件，沒有可用的位置誤差填NaN，由CSV留空，不捏造零誤差。

量測值是觸發時原生最後驗證的 power packet。記錄器另保留所有收到的 power packets；開始前最多2秒／10000筆緩衝，開始後持續到明確stop，能保留觸發前後資料。無法看見硬體沒有發布的瞬間峰值，也不保證 best-effort 沒掉包；可用seq與原生日誌核對。100Hz summary 不是峰值證據。

## 所有權與交接

- 動作 observer 規則仍要求恰好一個指定 Bridge、正確型別、Bridge GID與握手。只排除已審核、沒有發布 motor/power command 的 `/rinbo_data_recorder`。不把總數改成2；recorder獨自存在不能滿足Bridge條件；多Bridge、冒名publisher與換GID仍拒絕。
- 原生動作／通訊整理的匹配清單不含 recorder 或 Windows mirror。機器人關電後，recorder仍可繼續接收停止事件，再由錄製stop完成關檔。
- 此次新 recorder 使用版本化正式 binary；不刪除舊程序映像或更動其CSV。現有錄製不會因安裝而升級，舊PID仍是舊程式。
- **目前不要移除 Windows QoS adapter。** 現場選擇本輪結束後，用既有錄製控制路徑stop並確認檔案刷新，再正常結束該舊 recorder 與 mirror；不要用機器人全停腳本代替錄製停止。之後由新client start取得v2 recording ACK，確認command_source=diagnostic、兩個mirror收到資料，再讓Windows移除adapter/remap/跨topic名稱與trigger流程。

## 驗證與版本證據

隔離 domain232 / localhost 的 29 項測試通過：原子名稱 ACK、重複 start/stop、requested/forwarded 區分、正確 QoS、真實 rosbag 存檔、初始及錄製中磁碟故障、資料 stale/恢復、程序重建保留歷史、CLI export 與原生整理不碰 recorder。測試記錄 `staged-tests-final.log`。其中 `isolated-trigger-detail.json` 保留原生 power guard + 真實 recorder 的事件範例：L2/ch2=6.25A、threshold=5A、power_seq=129、25筆，故意給屏蔽L3的99A未誤認為觸發來源；全部是隔離模擬資料。

FSM CTest 12 組通過，包含201個 GTest案例與1個停用程式檢查；涵蓋 observer 加入／退出、缺少Bridge、多Bridge、非法publisher、GID/ACK，以及既有Cali/Standing/Tripod/Manual回歸。四個動作部署前後的唯讀有效設定輸出相同，見 `fsm-config-comparison.json`。基準檢查仍會列出已審查的來源／執行檔差異，沒有參數值差異，未更寫固定基準或現場revision。

初輪測試曾抓到 client DDS discovery 等待不足、install 尚缺 QoS YAML，以及部署預檢抓到 Manual 編譯產物未含新版本旗標；已修正並重驗證，保留原失敗日誌。source時間或單次建置成功不作為部署完成的唯一證據。

測試與部署逐檔SHA、symlink、正在錄製之PID前後映像證據，集中於 `docs/diagnostics/recorder_native_20260911/`。真正正式 executable 請以 deployment.json 與 after-executables.json 為準，不以來源日期推定。

正式部署完成於2026-09-11 15:17；回復備份：`/home/jetson/.local/state/rinbo-deploy-backups/20260911-151727-recorder/`。正式 install 入口再次跑29項隔離測試全部通過（`installed-tests-final.log`）。Cali、Standing、Tripod、Manual 的 `ros2 run ... --version` 均已回覆 native-observer-v2；前三者包含 safety-detail-v1。Tripod executable 為 `rinbo_tripod`。

recorder 正式 install symlink 現在指向 `build/rinbo_data_recorder/releases/native-recorder-v2-20260911/rinbo_data_recorder`，SHA256 `a6fb03d68b7bab32f334789cc9b005c139674efe01376183415dcc007039bfb3`。舊 `build/rinbo_data_recorder/rinbo_data_recorder` 刻意保留，因 PID53632 仍執行該映像；這是交接期間的版本隔離，不可誤將該舊檔拿來啟動下一輪。

部署後 PID53632 的 start_ticks、映像SHA、開啟CSV路徑完全相同。新 client 在 domain99 正確回 exit21 legacy_recorder_active，未接管該舊程序。`rinbo-recorder.service` 已安裝、inactive/disabled，等明確start時才啟動；既有 `rinbo-panel.service` 仍active、PID131178，未重啟。只更新已測試檔案，沒有發出任何硬體命令或程序停止訊號。

無硬體驗證方式：

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/rinbo_ros_ws/install/setup.bash
ros2 run rinbo_fsm rinbo_cali --version
ros2 run rinbo_fsm rinbo_standing --version
ros2 run rinbo_fsm rinbo_tripod --version
ros2 run rinbo_data_recorder rinbo_data_recorder --version
```

新FSM回 native-observer-v2-20260911；新recorder回 native-recorder-v2-20260911 schema=2 control_api=1。上述新版本旗標在ROS初始化前返回；不要對舊版recorder猜用--version，舊程式不一定支援。

現場後續只需在本輪自行收尾並切換後，先開recording、核對status資料age與實際列數，再由操作者明確執行原有實驗；發生原生保護時核對events.csv的detail_json與power_samples.csv相同seq，最後stop確認files_closed=true、export下載核對雜湊。此工作没有自動上電或執行真實動作。
