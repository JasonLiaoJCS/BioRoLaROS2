# Calibration／Standing／Tripod 方向與到位檢查（2026-09-08）

## 結論

單腳動作的速度、角度、相位和出力設定沒有改寫 Calibration／Standing 的控制參數。L1 這次 Standing 的問題是「目標往負方向，回讀卻跑到正方向」，不是正常到位後繼續下一個動作。

已修正能由程式與離線測試確認的缺陷。**L1 的實體馬達方向／編碼器極性仍未確認，不能宣稱實機已修好。** 本次沒有上電或試轉，也沒有猜測並反轉 L1 的方向設定。

## 單腳設定會影響什麼？

| 項目 | 實際行為 |
|---|---|
| 選單 2、7、9、11、13 的動作參數 | 存到 `~/.local/state/rinbo_control/settings.json`；執行時另產生 `logs/…/plan-….yaml`，不改共用硬體 YAML |
| 單腳動作呼叫 Calibration | `--plan` 只決定校正哪些腳；校正速度、出力仍讀自己的現場設定 |
| 選單 8 屏蔽腳 | 明確修改共用屏蔿名單，Calibration／Standing／Tripod 都會使用 |
| 實際做過單腳動作 | 取消舊 Standing 完成紀錄，因為實體姿勢已可能改變；不改 Standing 目標或轉速 |
| 同時啟動兩支控制程式 | 共用動作鎖及 Bridge 單一命令來源檢查會阻擋，不把兩份命令混合 |

共用設定是 `/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml`。動作程式的完成紀錄是旁邊的 `.calibration.json`、`.standing.json`，不是控制參數檔。

## L1 的實際證據

### 最新一輪為什麼停止？（補查 21:32～21:33）

目前 Orin 上最新的實機 Standing 日誌是 `rinbo_standing_69566_1788874367720.log`。以下時間均為 2026-09-08、UTC+8；離線測試日誌不算實機執行。

| 時間 | 已記錄的事實 |
|---|---|
| 21:32:47.928 | L1 在 Hall 零點，Standing 目標 −27648 counts |
| 21:32:52.930 | L2、R1、R2 完成；L1、R3 尚未完成 |
| 21:33:04.455 | 仍為 3／5 隻完成，L1 沒有 DONE |
| 21:33:04.565 | Standing 收到 `signal_handler(signum=2)`，即 SIGINT；可能來自 Ctrl+C 或啟動器停止，日誌無法辨認誰送出 |
| 21:33:04.666 | Bridge 記錄 `active motor command stream exceeded 100ms`，進入停用馬達流程 |

**這輪的停止順序是停止訊號 → 命令中斷 → Bridge 停用，不是 L1 站立完成，也不是新加入的反方向檢查觸發。** 這輪在修正版本編譯前執行，不能拿它當作修正後的實測。較早 21:27 那輪另有位置逾時問題，見下方，不應把兩次事件混成一次。

已保存本次查到的 Standing／Bridge 日誌副本到工作區 `log/standing-audit-20260908-2133/`，避免啟動器覆寫後遺失。

### 有沒有這輪的完整命令與 encoder 錄製？

已檢查監控錄製目錄、`~/rinbo_logs`、工作區紀錄、使用者資料／下載目錄及家目錄中的 rosbag 檔案。目前找到的監控快照是 9 月 7 日，機器人的 rosbag／CSV 是 6 月 16 日；**未找到 9 月 8 日 21:32～21:33 這輪的完整錄製**。當時的每筆 PWM、direction、enable 與 encoder 不能事後由文字日誌還原。若檔案下載到 Windows 或存於其他位置，仍須取得該檔比對。

程式檢查確認 Standing 按左右腳規則產生方向與 PWM；Bridge 將 L1 的 enable、direction、PWM、state、reset 逐欄轉到 gRPC 的 L1，回讀也是 L1 對 L1，沒有查到交叉送至別隻腳或額外反號。這只能確認程式映射，不能宣稱該輪所有封包和實體馬達輸出都正確。

### 補齊記錄工具

`rinbo_data_recorder` 的舊版預設 rosbag 只有控制器的 `/motor/command`，缺少 Bridge 收到／轉送的鏡像。現在預設另錄：

- `/rinbo/monitor/motor_requested`：Bridge 收到的命令。
- `/rinbo/monitor/motor_forwarded`：Bridge 交給傳輸層的命令，包括它自行產生的停止命令。
- `/rinbo/motor_output_enabled`、`/rinbo/motor_arbiter_ready`：Bridge 的輸出／握手狀態。

原有 `/motor/state`、電源與 `/rosout` 仍保留；自訂 `bag_topics` 時，metadata 也會記錄實際傳給 rosbag 的清單。這些新欄位在 **raw_bag** 中，`summary.csv` 的 motor command 欄仍是控制器命令。新增錄製不改馬達控制指令。

「轉送」只能證明 Bridge 交給傳輸層，**不是 sbRIO 或 FPGA 的執行確認**。現有馬達回讀只有 position、tick_count、Hall，沒有逐筆命令執行回覆；也不能把命令中的 `voltage` 當作量到的馬達電壓。

記錄工具已重新編譯安裝，新增 2 項測試通過：自訂話題與 metadata 一致，以及在本機隔離 ROS 群組 232 啟動真正的 rosbag，錄下模擬的「要求轉動／轉送停止」兩份不同命令與 encoder／Hall 後，直接讀取 bag 核對欄位。測試只發布診斷鏡像與假回讀，沒有建立馬達或電源命令發布者，也沒有啟動硬體 Bridge。

### 較早 21:27 那輪的位置異常

`~/.ros/log/rinbo_standing_55782_1788874042462.log` 記錄：

- 啟動位置 L1＝0，Hall 已觸發，Standing 指定 L1 目標＝−27648。
- L2、R1、R2 先後完成；L1 沒有出現 DONE。
- 本次檢查讀到啟動器的完整輸出：`standing position timeout: L1 pos=145953 target=-27648 error=173601 elapsed=20.000592`。
- 約 18 秒時，L1 Hall 又觸發一次，與它持續轉過零點的現象相符，但 Hall 本身無法判斷方向。

啟動器 `/tmp/rslip-launcher/20260908-212337-159/rinbo_standing.log` 會被後續啟動覆寫；上面的逾時數值是在覆寫前讀取並摘錄的。最近另一輪 Standing 也沒有 L1 DONE，但操作者先停止了它。

Calibration／Standing 的約定是：正向動作時左腳 raw counts 減少、右腳增加；正 PWM 命令的左側 direction＝0，右側 direction＝1。Bridge 逐腳轉送 direction、enable、voltage、state、reset_position，沒有另外把 L1 反號。

因此，L1 的回讀與預期運動方向不一致。可能是驅動方向、編碼器極性、接線、低階映射，或負載／外力將腳推往反方向；只有位置與 Hall 日誌，還不能判定應反轉哪一端。

也比對了 Git 原始版本：Calibration／Standing 的左右方向公式與原版本一致。歷史版本曾使用出力上限 500，現在現場限制為 80；這項先前的共用限制變更可能影響跟隨能力，不能說所有歷史控制行為都完全一樣，但本次單腳設定 20 並沒有覆蓋 Standing 的 80。本次未把上限直接恢復成歷史值。

另以唯讀 SSH 檢查 sbRIO 的 `/home/admin/rinbo_sbRIO_ws/rinbo_fpga_driver/src/fpga_server.cpp`、`fpga_handler.cpp`：六腳依同一順序寫入各自 EN／DIR／PWM 寄存器；L1 沒有獨立反號，位置讀取為 signed I32。命令處理和接收使用同一 mutex，未發現這兩處同時改寫同一命令造成混包。這是現存原始碼檢查，尚未證明運行二進位與該源碼逐位一致，也沒有驗證 FPGA 內部接線。

低階程式另有一個接口差異：它把 `state` 寄存器固定寫成 true，而沒有使用傳入的 `cmd.state`；`enable` 仍按命令逐腳轉送。此行為對三種控制器相同，不能單獨解釋 L1 反方向。`state` 在 FPGA 的具體含義尚未確認，本次沒有遠端改寫或部署驅動程式。

## 已修正與新增紀錄

1. **Calibration／Standing：辨識反方向回讀。** 相對本次起點，回讀往預期相反方向超過 500 counts（約 3.26°）就停止並指出腳名及原始值。Calibration 在接受 Hall 前也檢查，避免往錯方向轉卻被當作校正成功。這不是禁止所有短暫反向修正命令。
2. **Standing：到位必須確認停穩。** 位置誤差小於 200 counts、速度小於 500 counts/s，連續保持 0.3 秒才 DONE，避免高速經過目標的一筆資料被當作完成。
3. **Standing：保持階段加上速度阻尼。** 原本只有位置誤差回授，現在也用原有 `kd` 抑制到位後的速度；偏離保持目標超過現有硬限制 12000 counts 時停止並使紀錄失效。
4. **Calibration／Standing：每 0.5 秒輸出 TRACE。** 列出各腳的 raw position、target_raw、換算速度、帶正負號的 PWM 命令、enable 和 dir，供實際命令／回讀比對。PWM 是命令，不是量到的電壓或力矩。
5. **Tripod：修正負相位圈數。** 舊程式先 `floor` 計算圈數，負相位又減一圈，會讓目標多退 360°。目前正常執行路徑的活動腳通常使用非負相位，不能把這個邊界錯誤當成本次 Standing 的原因。
6. **Tripod：修正高終端速度的啟動反向軌跡。** 例如啟動 8 秒、`start_ratio=4`，原三次曲線會先產生負速度。此情況改用單調遞增曲線，保持起終點、總時間與指定終端速度。預設 ratio＝8 的原啟動曲線沒有這個反向問題。
7. **尋零與站立參考增加加減速。** 尋零用 0.5 秒加速到原本的最高速度；Standing 半圈移動用加速、勻速、減速，約需 5.5 秒，再確認停穩。最高速度仍為原本每秒約 36°，出力上限不變，終點不再把參考速度從勻速直接切成零。

## 小幅反轉還需要確認什麼？

Calibration／Standing 使用位置與速度誤差控制。若腳超前目標，公式本來就可能輸出負 PWM 來修正；負命令也可能是在煞住正向速度，不等於馬上實際反轉。舊線性軌跡的速度切換已改善；實體慣性、回讀時間變動和增益仍可能影響幅度，不能用現有日誌斷言只由其中一項造成。

另外，Tripod 使用 54984.83 counts／圈，Calibration／Standing 使用 55296；差約 0.56%。這個比例差不會單獨解釋 L1 朝相反方向轉，但要做精確相位控制，仍需依實際編碼器和傳動比統一確認。本次未任意更改歷史硬體比例。

接下來應先核對 L1 的方向鏈路，再做受控的單腳測試並保存 TRACE。若新程式回報 `opposite encoder travel: L1`，表示問題被辨識並停止，**不是機構方向已被自動修好**。不要靠提高出力或關閉這項檢查繼續 Standing／Tripod。

## 本次驗證結果

- Calibration、Standing、Tripod 已重新編譯；控制台已重新安裝。
- C++ 離線測試共 32 項通過：Calibration 14、Standing 8、Tripod 10。包含單腳參數隔離、屏蔽腳、反方向回讀、停穩／保持失效，以及 Tripod 啟動與相位邊界。
- 控制台相關 Python 測試 61 項通過；最後調整錯誤說明文字後，相關 24 項再次通過（包含在前述範圍，不重複計數）。
- `./robot.sh --check` 通過，確認目前屏蔽 L3；`git diff --check` 通過。
- 測試使用隔離設定與本機 ROS 群組，沒有對機器人上電或發送試轉命令。以上結果證明列出的程式行為通過測試，不代表 L1 實體故障已排除。
