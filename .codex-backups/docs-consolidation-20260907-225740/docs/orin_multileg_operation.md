# Orin R-Slip 多腿屏蔽操作

校正時需要對照命令與感測器，請使用 [即時監控面板](rinbo_monitor.md)；面板可從 Windows 瀏覽器查看。

本頁取代舊文件中 R-Slip 的 `rinbo_leg_mask auto`、`--params-file`、`--disabled-leg` 與腿名環境變數流程。Windows v6.4 按鈕與 Bridge／電源／停止流程維持原樣。

唯一有效設定為 `/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml`。沿用現場站點路徑，schema version 1；三個 FSM 自動載入同一份屏蔿名單與各自的安全參數，不依賴工作目錄、Windows 或 shell 環境。`colcon build` 不安裝或覆寫此站點檔。

2026-09-05 遷移時，站點檔為 `[]`，舊 `l1_degraded_test.yaml` 為 `[L1]`。使用者明確選擇採用站點 `[]`、解除全部屏蔽；因此六腿現在都會參與命令、回讀與完成判斷。這不表示已知故障的 L1 已修復。L3 沒有被自動加入屏蔽。

## 最短操作卡

先依原有流程停止動作並等 FSM 程序退出，再於已載入 ROS 工作區環境的 Orin terminal 執行：

```bash
ros2 run rinbo_fsm rinbo_legs status
ros2 run rinbo_fsm rinbo_legs set L1 L3
ros2 run rinbo_fsm rinbo_legs disable R2
ros2 run rinbo_fsm rinbo_legs enable L3
ros2 run rinbo_fsm rinbo_legs enable-all
```

以上 `set L1 L3` 是示例，本次部署沒有執行。`set` 取代整份名單；`disable` 加入，`enable` 移除；`enable-all` 明確儲存空名單。大小寫與前後空白會正規化；未知腿名、同次重複腿名、空 `set` 都拒絕整筆操作。對已屏蔽腿再 `disable`、已啟用腿再 `enable` 是不改版本的冪等操作。

每次寫入顯示絕對路徑、格式版本、修訂版本、雜湊、屏蔽與啟用腿，以及下一次啟動使用的設定。不用重新編譯。名單有變更後，重新走 **Calibration → Standing → Tripod**；舊設定的完成紀錄不能通過後續入口。

三個 Windows 既有 build 執行檔與以下手動入口都直接讀站點設定：

```text
ros2 run rinbo_fsm rinbo_cali
ros2 run rinbo_fsm rinbo_standing
ros2 run rinbo_fsm rinbo_tripod
```

上述是實際動作入口；本次部署沒有執行。查看入口設定而不初始化 ROS、不發布命令，使用：

```bash
ros2 run rinbo_fsm rinbo_cali --check-config
ros2 run rinbo_fsm rinbo_standing --check-config
ros2 run rinbo_fsm rinbo_tripod --check-config
```

`--check-config` 只驗證並列出設定；不認證機械狀態，也不建立 Calibration／Standing 完成紀錄。機器可讀管理狀態為 `rinbo_legs status --json`。

## 設定與完成紀錄

站點 YAML 只有一個頂層 `disabled_legs`，其他欄位為 `schema_version`、`revision`、`test_mode: supported_leg_test` 與 `parameters`。`parameters` 下的節點名稱是 `rinbo_cali`、`rinbo_standing`、`rinbo_tripod_rslip`。電流／電壓、PWM、超时、來源時戳與步態參數由舊站點檔保留；過時的 `hardware.max_disabled_legs: 1` 已移除。

設定缺失、損壞、無法讀取、版本不支援、重複 YAML key 或外部 ROS 參數覆蓋會明確失敗，沒有全腿啟用的後備設定。不要再把舊 degraded YAML 或 `-p hardware.disabled_legs` 傳給 FSM。舊範例 YAML 留作歷史參考，不是有效來源。

工具用跨程序鎖與原子替換避免半份設定。FSM 啟動時固定讀一份快照並持鎖到退出；執行期間不能熱解除屏蔽。已知動作程序仍在執行、檢查程序狀態失敗或鎖被占用時，管理工具拒絕變更，不會殺程序。單純 Bridge／ROS daemon 不阻擋變更。

Calibration 和 Standing 只在真實完成條件成立後寫入與設定雜湊、修訂版本綁定的紀錄；檔案放在站點設定旁。重新校正、設定變更或安全停止會使相應成果失效。不要手工建立或還原完成紀錄。重開機也需要重新校正。

六腿全部屏蔽可以儲存為全停設定；三個動作入口會在 ROS 初始化前回報 `[FATAL] 没有可測試腿`，不會印出 DONE／STANDING／RUNNING。普通腿故障依然會報 `SAFETY STOP` 與具體腿名，不會自動改名單。

## 支撐測試與控制邊界

`test_mode: supported_leg_test` 是有可靠外部支撐或懸空時的功能測試。剩餘腿沿既有個別腿相位與軌跡運作；名單可以使某個 Tripod 相位組完全沒有腿。這些測試不以地面承重或穩定步行為完成條件，不保證少腿時能在地面行走。

主驅動的最後發布關卡逐腿強制 `enable=false`、`direction=false`、`voltage=0`、`state=0`、`reset_position=false`。這涵蓋校正、保持、過渡與停止分支。屏蔽腿顯示 SKIPPED，不算成已校正。健康腿校正需依序滿足伺服定位、Hall、停止與歸零回讀；超時仍失敗。

協定的 `ServoCmd` 只有 `position_encoder`；`MotorCmdStamped` 只有一個全域 `servo_control_mode`，Bridge 原樣轉送。無逐顆 servo disable，因此屏蔽腿的伺服仍可能收到 neutral/hold 目標。需要整腿不動／斷電時，必須按現場硬體方式實體隔離相應 Main Drive 與 Servo；軟體主驅動 mask 不代表整隻腳已斷電。

Power tool 會在 ROS 初始化前，以同一個 `rinbo_legs status --json` loader 取得並鎖定名單，自動排除指定腿的逐腿電流檢查。全域電源、bus voltage、通訊、停止與健康腿保護保留。舊 `--disabled-leg` 會明確拒絕，不能覆盖站點。緊急／普通 `off` 路徑在設定損壞時仍保留。

ONNX／RL controller 的獨立 policy contract 本次沒有改成多腿模式；本頁只適用 R-Slip 三個 FSM 與既有 power tool。不要將 R-Slip 的多腿測試結果當作 RL policy 驗證。

## Windows 日誌標記

- 校正成功：`State: DONE`
- Standing 完成：`ALL <N> HEALTHY LEGS STANDING`
- Tripod 啟動：`Startup done, entering RUNNING`
- Tripod 正常停止：`=== Fully stopped ===`
- 失敗：`SAFETY STOP` 或 `[FATAL]`，附具體原因

`N` 是實際啟用腿數，失敗分支不產生成功標記。主執行檔名稱與 build/install 入口保持原樣。

## 本次變更與驗證

原始修改備份在 `/home/jetson/rinbo_ros_ws/.codex-backups/orin-multileg-20260905/`，含變更前套件、站點檔、文件與既有 git diff/status。本次保留 `ros_input_guard.hpp` 初始 UNKNOWN／過期封包等待及六個 callback 早返回修正。

已完成兩個套件的乾淨編譯：`rinbo_fsm`、`redrhex_lowlevel_bridge`。測試通過：87 項 C++、舊 sine-sweep 入口停用檢查、248 項 Python、16 項 Bridge 回歸，以及 18 項 FSM 入口檢查；各 FSM 與命令 mask 均涵蓋全部 64 種組合。完整 [測試結果](../.codex-backups/orin-multileg-20260905/validation-results.txt) 與 [變更檔案清單](../.codex-backups/orin-multileg-20260905/changed-files.txt) 已保存。測試使用 localhost mock／隔離 domain，未啟動實機 Calibration／Standing／Tripod、未发布 motor/power 控制到機器人 domain 99、未上電、未重啟硬體程序。歸零回讀時序、伺服實體隔離、懸空機械運動與 Windows 五按鈕端到端流程仍需現場確認。
