# 給 Windows Codex：接上 Orin 原生 recorder v2

請修改錄製的啟動、狀態顯示、停止和下載；機器人動作、參數、保護仍全部由 Orin 管理。不要改 PWM、電流／電壓、L3 屏蔽或機器人停止策略。

實際入口已提供：

```text
/home/jetson/rinbo_ros_ws/tools/rinbo_recorder_client.py
```

通过既有 SSH 執行（名稱必須作為單一安全引用的 argv，不可直接拼接 shell）：

```bash
python3 /home/jetson/rinbo_ros_ws/tools/rinbo_recorder_client.py version --json
python3 /home/jetson/rinbo_ros_ws/tools/rinbo_recorder_client.py start --name my_test --request-id <本次ID> --json
python3 /home/jetson/rinbo_ros_ws/tools/rinbo_recorder_client.py status --json
python3 /home/jetson/rinbo_ros_ws/tools/rinbo_recorder_client.py stop --request-id <本次ID> --json
python3 /home/jetson/rinbo_ros_ws/tools/rinbo_recorder_client.py export --json
```

預設 ROS domain99，client 自行 source 正確 Orin overlay，不需 Windows 尋找 recorder executable 或 ROS service。client version 只代表入口版本；必須同時用 status/start 的 **伺服器回覆** 確認 `version=native-recorder-v2-20260911`、`schema_version=2`、`command_source=diagnostic`。

1. **先保留現有錄製與 adapter。** Orin 本次部署沒有停止舊 PID 53632。若回 `legacy_recorder_active` / exit21，顯示「舊版仍在使用，請先正常結束本輪」，保留原 adapter、remap、stop 路徑，不啟動第二個 recorder、不刪 session 或 CSV。操作者明確結束本輪、確認關檔，再正常結束此錄製的舊 recorder 與 mirror；不可用機器人全停或廣泛 kill 代替。
2. v2 start 單一操作已包含名稱 ACK 與開始；移除該模式的 `/output_filename`、`/trigger`、`/set_recording` 跨 topic 三步流程。新 request_id 每次點擊產生即可。重複 start 回 already_recording，沿用回覆 name/run_dir，不誤報新建。新一輪需明確 stop 後 start。
3. 確認 v2 recording ACK 後，才移除該模式的 QoS adapter/remap。v2 直接訂閱 requested/forwarded，兩者要分開顯示；requested 不是 applied，forwarded 也不是硬體執行 ACK。
4. 用 status 顯示 recording、run_dir、rows、files_closed、disk_error 與 topics age。沒有發布的 topic 是 unavailable，過期是 stale。資料中斷或 SSH timeout 不代表錄製已停止；顯示「狀態未知」並讓使用者查詢，不自動重送 start/stop。
5. stop 的 exit0 + status=stopped + files_closed=true 才是原生完成關檔。不是機器人停止或關電。允許先完成機器人關電，再錄製 stop 收尾，不要在機器人清理時殺 recorder。
6. export 回傳 `export.path/bytes/sha256`。用 SFTP 下載該 tar.gz，核對大小與 SHA256；保留原資料。錄製中 export 是已 flush 長度的快照，不停止錄製。下載是 Windows 的責任，不要求 Windows 管理 ROS QoS 或硬體安全邏輯。

JSON 成功範例（欄位摘錄，實際還含 files/topics）：

```json
{"protocol":1,"schema_version":2,"version":"native-recorder-v2-20260911","request_id":"abc","action":"start","status":"recording","exit_code":0,"recording":true,"name":"my_test","run_dir":"/home/jetson/rinbo_logs/...","files_closed":false,"disk_error":null,"command_source":"diagnostic","rows":{"summary":0,"events":0,"commands":0,"power_samples":0,"controller_samples":0}}
```

舊版交接未完成範例：

```json
{"protocol":1,"version":"native-recorder-v2-20260911","action":"status","status":"legacy_recorder_active","exit_code":21,"recording":null,"reason":"Current recorder has no native v2 API; keep its adapter and finish it with its existing control path. No process stopped."}
```

穩定 exit：0完成/已達成；2輸入錯誤；10原生服務不可用；20原生拒絕；21舊版仍存在；31 API/ACK狀態不可確認；50磁碟/匯出失敗。保留 JSON reason 與原始 stderr。start SSH 捕獲期限至少45秒，status/stop30秒，export180秒（大檔可能需更久）；ROS ACK 預設8秒，可 `--timeout` 設1～30秒並相應延長SSH期限。

先做 Windows 假 SSH/JSON 離線測試：成功、already_recording、legacy21、unknown31、disk50、下載大小/雜湊錯誤。實機只在操作者結束現有錄製後切換，核對 v2 ACK、topics age、實際列數和安全事件 seq；不得自動上電或執行 Calibration/Standing/Tripod。

完整契約與部署證據：`docs/recording/native_recorder_v2.md`、`docs/diagnostics/recorder_native_20260911/deployment.json`。
