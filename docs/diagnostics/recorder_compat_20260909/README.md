# Orin 記錄器相容性與校正失敗診斷

事件日期：2026-09-09；修復／離線驗證日期：2026-09-10。工作區 `/home/jetson/rinbo_ros_ws`。

## 已證實根因

1. `motor_arbiter_handshake.hpp` 原先同時要求 ROS publisher 回報的「全部 command subscription 數量 = 1」與 graph 中「唯一 subscription 是 Bridge」。`/rinbo_data_recorder` 正常訂閱 `/motor/command` 後，兩個條件都把合法觀察者算進去了。
2. 本輪唯讀 ROS graph 再次觀察到 `/motor/command`：publisher `/rinbo_cali`，subscribers `/rinbo_ros2_bridge`、`/rinbo_data_recorder`，兩者 namespace `/`、型別 `rinbo_msgs/msg/MotorCmdStamped`。`/power/command` 當時沒有 publisher，subscribers 同樣是 Bridge 與 recorder，型別 `rinbo_msgs/msg/PowerCmdStamped`。完整 GID 在 `live_graph.json`。
3. 記錄器 `src/rinbo_data_recorder/src/rinbo_data_recorder.cpp` 沒有 `create_publisher` 或命令發布：motor／power callbacks 僅快取收到的樣本，timer 寫 CSV；trigger／filename／set_recording 只控制記錄檔。metadata 用的外部命令是讀取 Git 資訊。保留現有兩個 command subscriptions，沒有停掉現場記錄器。
4. 現場記錄器程序啟動時間為 23:45:05，Standing 在 23:45:07 因 subscription 總數 2 停止；時間吻合。沒有「兩個 Bridge」的證據。

## 實際修正

- `motor_arbiter_handshake.hpp`：逐一檢查 command subscriptions 的 namespace／node／topic type，必須有且只有一個 `/rinbo_ros2_bridge`；額外只接受 `/rinbo_data_recorder` 的正確型別 subscription。GID pin 綁定找到的 Bridge，與 recorder 排序、加入或離開無關。
- 同一份 graph 判定供初始握手、disabled-command rearm、armed 監控、輸出 gate、等待訊息與 timeout 使用。四個 FSM 不再傳入全部訂閱者數量。
- command publishers 不套用觀察者例外：motor 必須只有目前控制器，且 namespace／型別正確；power 額外 publisher、錯誤型別與 recorder 發布 power command 都拒絕。Bridge 原有 publisher arbitration 完全保留。
- ready／heartbeat／epoch／rearm ACK／active ACK 的唯一 Bridge 來源、callback GID、epoch、session frame、seq 對應和 heartbeat/ACK 時限均保留。graph 仍在既有 heartbeat 回呼檢查；沒有每筆 motor sample 額外執行同步 graph 查詢。
- `rinbo_ros_bridge.cpp`：只更新原本「只能有一個 subscription」的過時註解，Bridge 執行邏輯、參數與正式執行檔未變。
- `rinbo_cali.cpp`、`rinbo_standing.cpp`、`rinbo_tripod.cpp`、`rinbo_manual.cpp`：只更換 handshake 呼叫介面，沒有更改參考軌跡、方向、PD／PWM 計算、停止命令或腿位屏蔽。

## 其他 power／stop／diagnostic 搜尋

搜尋 `src/`、`tools/` 內 `get_subscription_count`、`count_subscribers`、`get_subscriptions_info_by_topic`、`endpoint_count` 及命令 topic 使用處：

- Orin `rinbo_power_tool.py` 的 subscription 等待條件為 `> 0`，不是 `= 1`；啟用時另檢查唯一 command publisher、唯一可信 power-state publisher，成功還需新鮮且相符的狀態回讀。記錄器加入不會觸發 subscription 數量拒絕。
- `rinbo_bringup_check.py` 顯示 power subscriber 清單與數量，沒有要求總數 1；`biorola_fault_diag.py` 只把數量放入診斷資料。其他 backend、motor command、estop 的 subscriber 等待也不是總數 1 gate。
- Bridge 的 motor／power **publisher** 唯一性檢查是另一種保護，沒有把它放寬。
- 這些工具的 `> 0` 本身不代表 Bridge 身分已驗證；本輪未將該條件宣稱為安全關電證據。仍以各工具的可信狀態回讀／上層 stop probe 判定結果。
- 額外發現：`record_experiment.launch.py` 的預設 bag topic 清單還包含兩個 command topics。它另外啟動的 `rosbag2_recorder` 並非本輪已審核的 `/rinbo_data_recorder`；不能因為名稱也含 recorder 就自動放行。若開啟 raw bag，請用下述觀測 topic 清單覆寫 `bag_topics`，避免新增未知 command subscriber。現場既有 recorder 的 CSV command 收集保留。
- Windows `rslip_probe.py` 的修復由使用者提供狀態；本工作區沒有該 Windows 原始碼，本輪沒有替換或驗證它。Windows 仍負責啟動與顯示。

## calibration.json 生命週期與本次失效原因

權威實作在 `src/rinbo_fsm/src/robot_config.cpp`：

1. 正式入口只讀固定 `/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml`，receipt 路徑直接附加 `.calibration.json` 或 `.standing.json`。CLI 不允許替代設定、remap 或額外參數覆寫。替代 path 只用於離線測試 API。
2. `MotionSession` 先取得設定共用鎖與動作獨佔鎖，再讀設定／檢查 receipt。正常設定寫入取得設定獨佔鎖；平行動作或管理寫入會被拒絕。
3. 開始新的 Calibration 會使 Calibration 和 Standing 兩份舊結果失效。Standing／Manual 開始時只刪除 Standing，並要求 Calibration 覆蓋所有本次使用的腿。Tripod 要求兩份結果都有效。
4. 所有啟用腿完成伺服定位、尋 Hall、停穩、reset 後，收到真實零位置回讀，Calibration 才呼叫 `complete()`；不是送出 reset 就算成功。receipt 記錄 boot_id、config hash、revision、stage 與校正腿清單。
5. `complete()` 重新讀取設定，revision 或 hash 不符就拒絕並使結果失效。寫入使用同目錄 `mkstemp`、完整 write、`fsync`、`rename`、目錄 `fsync`；讀者不應看見半份 JSON。
6. 真正安全停止會呼叫 `invalidate()`，刪除兩份結果。正常 Calibration 完成後交接會保留；未完成中斷不會建立成功結果。
7. 設定修改使結果失效；部分既有調參 API 只在先前 prerequisite receipt 真正有效且 prerequisite 階段不受變動影響時，於獨佔鎖內搬移該 prerequisite 到新 revision/hash。缺失／錯誤結果不會憑空建立。本輪未呼叫或改動這些 API。

日誌證據：23:43 的 Calibration 已記錄 L2 零回讀完成、`State: DONE` 與 `CALIBRATION HANDOFF READY ... receipt preserved`；Standing 其後到位。23:45:07 Standing 的 recorder 誤停走到既有 `invalidate()`，足以解釋 23:46:02／11 Tripod 找不到 calibration.json。所有這些日誌仍是 revision 15、相同 SHA256。沒有發現本次事件的並行設定寫入或路徑不同證據。

因此修正誤判的來源，保留真正故障後的失效規則與原子寫入。沒有改 `robot_config.cpp`，沒有恢复歷史成功紀錄。本輪開始時兩份 receipt 都不存在。

## L2：觀測、推論與證據缺口

- 原始 log 見 `event-evidence.json`，保留檔名與行號。日誌內 ROS 時戳與 Windows 顯示時間存在約 1 秒差距；以下相對時間用 Orin ROS 時戳計算，不混用 Windows 顯示秒數。
- 較早成功 Calibration：L2 從 raw=31 向負值移動；TRACE 包含 raw=-1484／-4289、dir=0、正的 normalized PWM，最後收到零回讀。這不支持「整晚都是固定反接」的直接結論，也不能排除間歇接觸、後續硬體變化或負載回推。
- Standing 停止前 L2 raw=-27663、velocity≈0、normalized hold PWM=10、dir=0。約 73 秒後的新 Calibration TRACE 仍為相同 raw、速度≈0；伺服等待約 0.5 秒後進入 DC_SPINNING。這些資料降低「上一個 Standing 一直滑行到新校正」的可能性，但不是高頻殘留速度量測。
- 新 Calibration 的 Bridge boot id 未改，latch generation 2→3 發生在尚未提交的 rearm。Bridge 收到五筆 disabled commands、發布 rearm ACK seq=5，FSM 收到 post-ACK ready，接著 active hold ACK seq=8。不是直接沿用上一個 Standing 的 active ACK。
- 尋零基準建立於 ROS time `1788968780.443388889` 附近；反向停止於 `1788968780.624420376`，約 0.181 秒後。L2 從 -27663 到 -27131，反向位移 +532 counts。程式算出 normalized forward travel=-532，低於既有 -500 保護門檻；負數起點本身不會讓檢查反號。
- Calibration 左腿正向 normalized PWM 對應 `direction=false`（dir=0），voltage 為 PWM 絕對值；reference 要求 raw counts 減少，velocity 使用 raw 速度反號。Bridge 原樣轉送 direction／voltage 與回傳 raw position／tick_count，沒有在這兩处額外反轉。
- 當時 TRACE 每 0.5 秒一筆，反向事件在第一筆帶出力的 TRACE 前已發生。因此沒有足夠證據證明該 0.181 秒內 L2 實際送出的逐筆 direction／PWM，更不能由理想 reference 推定馬達已按相同方向出力。
- 現有 `RosInputGuard` 驗證 Bridge 來源、來源時間、seq/stamp 遞增，拒絕重播／超齡資料；但此次 log 沒保存每筆 motor sample seq/stamp，也無法由此排除上游硬體／傳輸鏈對位置快取或符號的問題。搜尋記錄目錄沒有找到涵蓋這次事件的原始 bag／CSV；現場 recorder 啟動命令帶 `auto_start:=false`，程序存在本身不代表当時已開啟寫檔。
- 已加入 production Calibration handler 的負起點回歸測試：+500 不觸發、+532 仍停止；正向 reference／命令方向仍正確。這只證明程式判斷，**不代表 L2 實體問題已修好**。

下次經操作者確認支撐、安全停機及實驗授權後，保留既有 `/rinbo_data_recorder` 對兩個 command topics 的 CSV 收集；額外 raw bag 使用以下 `bag_topics` 清單：

```text
/motor/state,/power/state,/rinbo/monitor/motor_requested,/rinbo/monitor/motor_forwarded,/rinbo/motor_output_enabled,/rinbo/motor_arbiter_ready,/rinbo/motor_arbiter_heartbeat,/rinbo/motor_arbiter_epoch,/rinbo/motor_rearm_ack,/rinbo/motor_active_ack,/rosout
```

Bridge 的 `motor_requested`／`motor_forwarded` 是命令鏡像，正常轉送保留來源 header／seq／stamp，可與 motor state 對齊。`forwarded` 只代表已交給傳輸端，不是 sbRIO 或馬達實際執行的確認；Bridge 自行產生的 disabled 命令也可能沒有原始控制器 header。獨立 rosbag 不應再直接訂閱 `/motor/command` 或 `/power/command`，否則會引入未審核的 command subscriber。現有 CSV 是 timer 採樣摘要，且沒有完整 sample seq/stamp，不能取代 raw bag 對齊。優先檢查 L2 支撐／負載、接線和 encoder 符號，再由受控單腿測試比較 requested、forwarded、回讀，不直接反轉方向或放寬保護。

## 驗證、參數與部署

最終驗證：FSM 的 12/12 組 CTest 全部通過，包含 201 個 GTest 案例及 1 個舊 sine-sweep 入口退役檢查；實際 recorder CSV 整合測試 1/1、power tool 157/157、Bridge power epoch guard 16/16 通過。測試只使用 localhost ROS domain 231／232；完整 FSM 測試另在隔離 PID namespace 執行，使設定管理測試只看見自己的程序，保留正式程序偵測保護。

新增／更新測試與建置檔案：

| 檔案 | 用途 |
|---|---|
| `src/rinbo_fsm/test/test_motor_recorder_compat.cpp` | graph 身分／type／publisher 規則與實際 ROS 握手、recorder 進出、ACK／epoch 回歸 |
| `src/rinbo_fsm/test/test_motor_arbiter_handshake.cpp` | 舊 state 測試改用已驗證 graph 狀態，保留 GID／時限測試 |
| `src/rinbo_fsm/test/test_robot_config.cpp` | 原子 receipt 寫入、revision/hash 一致性與故障失效 |
| `src/rinbo_fsm/test/test_cali_multileg.cpp` | L2 負起點 +500／+532 counts 與方向映射 |
| `src/rinbo_data_recorder/test/test_command_recording.py` | 真實 recorder 執行檔可寫入兩種命令 CSV，且沒有命令 publisher |
| `src/rinbo_fsm/CMakeLists.txt` | 註冊相容性測試與獨佔測試執行，避免測試 domain 相互干擾 |

測試與 build 的最終結果記錄於 `verification.json` 及同目錄 logs/XML。獨立編譯位置：`build/recorder_compat/rinbo_fsm`，隔離安裝位置：`install/recorder_compat/rinbo_fsm`。這不會讓 Windows 固定的 `build/rinbo_fsm` 路徑自動更新。

原版與新版設定工具讀出的完整有效參數完全相同，現場 YAML 的 SHA256 仍為 `5866b8ee7bb6a7733c0757fb0d7dd35995a1f78e3a5ec4defe6507c130c13129`，revision 15，L3 屏蔽不變。兩份 receipt 仍不存在；五個正式 FSM／設定工具執行檔的 hash 仍與基準相同。

本輪未開電、未啟動實際動作、未停止／重啟 Bridge 或現場 recorder。唯讀程序檢查仍有舊 Calibration 在運行；替換其 executable 可能改變 `/proc/PID/exe` 與既有 launcher 身分核對。因此本輪不覆蓋正在使用的正式執行檔，保留可供部署的已測產物。

操作者下一步：用已修正的 Windows「結束實驗／關電」完成可信關電驗證與舊動作程序清理。清理完成後，於 Orin 對正式目錄執行下列編譯／安裝（只建置，不會開電或啟動節點），再檢查參數；Bridge 與 recorder 不需要為此修復重啟。

```bash
cd /home/jetson/rinbo_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
MAKEFLAGS='-j2 -l2' colcon build --base-paths src/rinbo_fsm --packages-select rinbo_fsm --allow-overriding rinbo_fsm --executor sequential
python3 tools/parameter_baseline.py check
```

原始參數與基準副本均保留；檢查列出的本輪來源 hash 差異屬於上述 handshake／呼叫介面變更，不代表增益或安全數字漂移。`motion_limits.py` 是本輪開始前既有差異。不要為了讓 check 回傳 0 而更新固定基準或還原其他修復。

部署後仍需完成真正有效的 Calibration，再做 Standing；L2 若再次反向停止，應保留資料並處理實體原因，不能直接進 Tripod。`POWER STATE UNKNOWN` 仍不能當作已關電。
