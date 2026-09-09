# 到位／追蹤限制調整（2026-09-09）

目前值以[重要參數基準](IMPORTANT_PARAMETER_BASELINE.md)及操作台現場讀回為準：Calibration／Standing／Manual 上限 500、Tripod 3300；Standing `settle_time_s=0`，Tripod 額外 slew 關閉。下方事故與部署紀錄描述 19:53 當時版本，不代表目前值。

已於 19:53 部署；現場設定 revision **11**。沒有上電或啟動實機動作。

## 這次 Standing 為什麼停止

使用者提供的首個原因是 `standing position timeout: R2`：目標 27648、實際 27866、誤差 218 counts，等待 20.000786 秒。
舊版 Standing 到位要求誤差 **小於 200 counts**、原始速度小於 500 counts/s，且連續符合 0.3 秒；轉動等待逾時為 20 秒。
218 counts 依現行 Calibration／Standing 的 55296 counts/rev 換算是 **1.4193°**，200 counts 是 1.3021°。
因此這次是到位等待逾時；不是 Tripod 的 18000-count 硬位置誤差停止，也不是上述等待 arbiter 的 WARN。

部署前五支 FSM 執行檔的 SHA256 仍與先前已部署版本一致；現行來源與這筆條件相符。
本次 Orin `/tmp` 已沒有 19:27 那份完整原始日誌，不能倒推當時實際速度、持續到位時間或每個 PWM，也不能把目前磁碟檔案當作當時 `/proc/PID/exe` 的直接證據。
218 counts 為何剩下來，可能涉及控制增益、摩擦等；這段資料不能確定物理根因。

## 實際操作

```bash
cd /home/jetson/rinbo_ros_ws
./robot.sh
```

主選單 **15 到位／追蹤限制** → 選 **1 Calibration、2 Standing、3 Tripod、4 sim-to-real**。

- 選欄位編號：填新數字，畫面有單位、範圍與作用說明。
- **p**：套用本頁較寬調機設定，先列出新舊值，Enter 儲存；q 取消。
- **r**：恢復本頁 `standard_20260909_r15` 標準值；先預覽再儲存。Tripod 保持只警告、PWM 3300、slew 關閉；Standing 停穩時間為 0。
- **0**：返回。儲存不會上電、送運動命令或自動重試。
- 動作程序仍在執行時可以查看／預覽，儲存會請你先停止該程序。包括停在 SAFETY_STOP 但尚未退出的程序。
- 儲存檢查版本、使用既有鎖與原子寫入，再讀回 hash；不能用過時的畫面覆寫新版本。
- 改 Calibration 限制後需重做校正；只改 Standing 限制保留仍有效的 Calibration，重做 Standing；只改 Tripod 保留仍有效的前置結果。已失效的結果不重建。

## 本次套用的 FSM 現場值

| 階段／設定 | 原值 | 新值 |
|---|---:|---:|
| Calibration：伺服定位最多等待 | 20 s | 60 s |
| Calibration：尋零最多等待 | 30 s | 60 s |
| Calibration：停穩／歸零各階段等待 | 5 s | 15 s |
| Standing：到位容許誤差 | 200 counts | 1000 counts（約 6.51°） |
| Standing：轉到站姿最多等待 | 20 s | 60 s |
| Standing：尋零最多等待 | 30 s | 60 s |
| Tripod：位置誤差處理 | 只警告 | 保持只警告 |

這是現場調機起始設定，不是馬達或機構規格認證。較寬誤差讓 Standing 可以在仍有小幅誤差時交接，沒有改目標位置、方向、零點或控制增益。
Standing 仍需速度低於 500 counts/s 且連續符合 0.3 秒；到位後偏離停止門檻仍為 12000 counts。
Calibration 仍必須找到 Hall、停穩、收到歸零回讀；不能把延長等待解讀成略過校正。
Tripod 的 9000／18000 counts 在只警告模式不觸發有限位置誤差停機；切回停止模式後恢復軟門檻及固定 18000 硬上限。

## 可以填的數字

原生驗證的到位等待時間是 `(0,600]` 秒；控制台以 0.1 秒為最小顯示輸入。
Standing 到位容許 `(0,12000]` counts、停穩速度 `(0,5000]` counts/s、穩定時間 `[0,10]` 秒（0 表示不要求低速與持續停穩）、到位後偏離門檻 `(0,55296]` counts；偏離門檻不得小於到位容許。
Tripod 軟門檻 `(0,12000]` counts、超限時間 `(0,2]` 秒、次數整數 `[1,10]`，模式為 `0=警告、1=停止`。
這些是程式輸入範圍，不表示所有組合都適合機構。文字介面到位 counts 至少 1；穩定時間可以填 0。

## 保留的保護

- L3 屏蔽、supported_leg_test、單一動作程序／驅動限制及方向錯誤偵測。
- 無效或非有限回饋、失聯、過期或不可信的來源、arbiter heartbeat／ACK。
- 每腿電流 5 A，25 次；電壓 18～42 V，5 次；電源資料過期 0.5 秒、馬達資料過期 0.25 秒。
- Relay 回讀、手動停止與原有 SIGINT 行為；PWM 上限 80、Tripod slew 250/s；不新增上電或重啟機制。
- 既有 bus-current guard 仍為 false，未宣稱這項原本未啟用的功能受到啟用保護。

## sim-to-real 的界線

選單 15 → 4 可明確選擇 **啟動命令真正使用的 YAML**：列出 install、src 與 redrhex_site 下的候選，或自行貼上絕對路徑。
這些位置可能是不同檔案，修改 src 不表示 install 自動更新。
可以調整 `state_machine.init_stand_timeout_s`（最高 60 秒）、`init_stand_position_tolerance_rad`（最高 0.12 rad ≈ 6.88°）、`init_stand_velocity_tolerance_rad_s`（最高 0.25）。p 延長等待到 60 秒；其原本到位容許就比 Standing 舊值寬。
本次沒有已確認的實際 sim-to-real 啟動 profile，因此沒有批次改寫這些檔案或擅自選用模型。
設定保存原檔備份及其餘註解／數值，不更改模型 metadata。其模型封裝會核對設定 hash；修改後需重新封裝／preflight，畫面會明確提示，不能沿用舊驗證。
現有 `rinbo_ros_backend.py` 仍以目標速度乘比例換算 PWM，不能宣稱它已有完整位置 PID，或認為任何誤差都會自動補償。

## 真正的設定與程式位置

- **日常設定檔**：`/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml`，各 `parameters.<節點>.safety` 欄位。
- **Standing 讀值與停止／到位判定**：`src/rinbo_fsm/src/rinbo_standing.cpp`，搜尋 `position_tolerance_counts`、`ROTATE_180`、`standing position timeout`、`hold_error_counts`。
- **Calibration 逾時與歸零判定**：`src/rinbo_fsm/src/rinbo_cali.cpp`，搜尋 `servo_homing_timeout_s`、`hall_search_timeout_s`、`stop_timeout_s`、`RESETTING`。
- **Tripod**：`src/rinbo_fsm/src/rinbo_tripod.cpp` 的 `check_position_error_safety`；固定硬門檻在 `src/rinbo_fsm/src/tripod_reference.hpp` 的 `kHardPositionError`。
- **預設值、允許欄位、範圍、原子存檔與結果失效規則**：`src/rinbo_fsm/src/robot_config.cpp` 的 `defaults`、`validate_parameters`、`tune_limits`。
- **共用供電／通訊等範圍**：`src/rinbo_fsm/src/safety_invariants.hpp`；電源判定在 `rinbo_power_guard.hpp`。
- **文字介面**：`src/rinbo_control/rinbo_control/motion_limits.py`；sim-to-real 設定頁在 `sim_motion_limits.py`。
- **sim-to-real 驗證與控制**：`src/redrhex_rl_controller/redrhex_rl_controller/rl_controller_node.py`、`preflight_check.py`；低層 PWM 在 `src/redrhex_lowlevel_bridge/redrhex_lowlevel_bridge/rinbo_ros_backend.py`。

要日常改限制，使用選單 15 即可；手改 YAML 必須同時管理 revision／完成結果，不建議繞過工具。改 C++ 的預設／上限還需要重新編譯。

## CLI 與 Windows

Windows 原來的啟動指令可繼續使用同一份 Orin 現場設定，無需加 ROS `-p` 覆寫。
若 Windows 也要提供設定頁，呼叫以下原生介面；不能直接在 Windows 複製另一份 YAML 當有效設定。

```bash
# 純讀取
/home/jetson/rinbo_ros_ws/build/rinbo_fsm/rinbo_legs status --json
# N 換成上一行回傳的 revision；預覽
/home/jetson/rinbo_ros_ws/build/rinbo_fsm/rinbo_legs tune-limits standing --expect-revision N --dry-run position_tolerance_counts=1000 rotate_timeout_s=60
# 儲存：相同命令移除 --dry-run；再 status --json 核對
```

`ERROR 90` 表示 Windows 無法核對遠端程序／關電紀錄，與位置到位門檻無關。這次未修改 Windows 程式，也未偽造關電證據或清除該鎖；僅憑提供的摘要不能判定連線中斷、PID 更換或哪筆證據失配。

## 驗證

控制台測試 **232 項通過、1 項原有測試跳過**；新選單可由實際主選單進入，取消／預覽／儲存／還原與 sim-to-real 備份讀回已驗證。
FSM 離線測試 **117 項全部通過**（Standing 10、Calibration 15、Tripod 28、設定 22、arbiter／保護 42）。`./robot.sh --check` 通過，沒有初始化 ROS 或連線設備。
離線結果及部署 hash 記錄於 `docs/diagnostics/motion_limits_20260909/`。
測試涵蓋 R2=27866 重現、到位穩定時間、持續偏離、新參數／版本衝突、前置結果管理、L3 屏蔽、既有 Calibration／Tripod／供電與通訊回歸，以及控制台預覽、取消、自訂、還原與無硬體操作。
沒有送電或執行真實動作。新到位範圍是否足以讓實機平順交接，仍需要由你啟動後觀察實際速度與誤差；離線完成不能代替這項實測。
