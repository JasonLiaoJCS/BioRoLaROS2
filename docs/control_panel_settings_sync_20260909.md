# Control Panel 參數同步與連線核對（2026-09-09）

本次只修改 `./robot.sh` 使用的 Python 介面及說明，沒有更改現場 YAML、C++ 控制器、執行檔、服務、電源或動作。入口透過 PYTHONPATH 載入 `src/rinbo_control`；下次啟動 `./robot.sh` 即使用修正，不需要為本次介面修改重建原生控制器。

## 已確認與修正

1. 選單 15 的 r 原本恢復過時預設：Tripod PWM 80、位置誤差停止、slew 開啟，Standing 容差 200、等待 20 秒、停穩 0.3 秒。現在明確標為「恢復本頁標準值（20260909 r15）」，與固定基準逐欄核對；只有使用者選 r 並確認儲存才套用。Tripod 為 3300、位置誤差只警告、slew 關閉；Standing 容差 1000、等待 60 秒、停穩時間 0；Calibration 為 60／60／15 秒。
2. 主畫面增加原生有效參數的版本與 Calibration／Standing／Manual／Tripod 的 KP、KD、K_FF、PWM。逐腳動作顯示計畫及 Manual 現場上限取較小值的有效 PWM，避免把儲存計畫 80 誤讀成 Tripod 或 Manual 現場上限。
3. 選單 7 進入時重新讀取現場 Manual 上限；仍允許保存原生支援範圍內的計畫，真正輸出由控制器取計畫與現場上限的較小值，不偷偷改寫計畫或現場參數。
4. 修正停穩速度說明：只有 settle_time_s 大於 0 時才用它判定到位。舊操作文件中的 80／20 ms／Tripod slew 說明標為歷史，並連到現行基準。

## 後續調參規則

- 操作台使用原生 `rinbo_legs status --json` 讀取有效值；選單 15 以原生 `tune-limits --dry-run` 預覽、帶 revision 保存並讀回驗證。進入頁面、連線或執行不會比較現場參數是否等於固定標準。
- 原生支援範圍內的數值可更改，PID/PWM 不要求等於 80 或固定基準。Calibration 的外層等待時間已隨現場各階段 timeout 計算，測試涵蓋最高 600 秒的階段設定。
- 原生硬範圍與格式仍保留，例如 Manual 計畫 PWM 1～500、Tripod 1～3300，以及有限數字、有效腿位與參考一致性。超出原生支援範圍屬於修改控制器契約，不能只刪 UI 檢查便保證能運作。
- 同時被另一工作階段改設定時需重新讀取確認；舊 Calibration／Standing 紀錄不符時需重做相應步驟，不會因此要求將數值改回舊版。
- 供電／通訊、唯一控制來源、手動停止與 L3 屏蔽維持。步驟 1 既有自動整理流程維持，沒有加入廣泛 kill、無條件重啟或上電。
- Windows 的 `wrong_parameter:motor_command_max_pwm:expected=80:actual=3300` 是另一入口的固定預期值，本工作區沒有該 Windows App 原始碼；本次沒有宣稱修復那個 App。

## 驗證證據

- [現場基準核對](diagnostics/control_panel_settings_20260909/baseline_check.json)：revision 15、SHA 5866b8ee7bb6a7733c0757fb0d7dd35995a1f78e3a5ec4defe6507c130c13129，參數差異為空。固定基準與原生執行檔未改。
- `parameter_baseline.py check` 會列出 motion_limits.py 的來源差異，因此 exit 2；這是本次修復舊還原介面的已審查差異，並未重寫基準雜湊使它消失。這個手動工具不是操作台啟動門檻。
- [介面差異](diagnostics/control_panel_settings_20260909/interface.patch)，修改前副本亦保存在同一目錄。
- `./robot.sh --check` 通過，L3 保持屏蔽；此命令不初始化 ROS、不連線設備。
- [完整離線測試報告](diagnostics/control_panel_settings_20260909/pytest.xml)：**264 項通過（80.93 秒）**，包含介面、儲存／取消／版本變更、連線復用、終端停止、整理流程及 ROS 模擬回讀。ROS 模擬限定 domain 232、localhost，沒有發布真實機器人命令。首次測試因 PYTHONPATH 漏帶 ROS 路徑，263 通過、1 匯入失敗；保留原紀錄，修正環境後重跑。
- [23:01 唯讀連線快照](diagnostics/control_panel_settings_20260909/live_connection.json)：Jetson 路由來源 192.168.30.8，sbRIO 192.168.30.254:50051 TCP open，唯一 Bridge 指向同一 IP，motor/state 與 power/state 新鮮，無馬達命令發布者。回讀馬達 Relay 原已開啟、約 23.72 V；觀察程式沒有改變電源。

唯讀觀察只關閉自己的 ROS 訂閱節點，不呼叫會執行關電的 Runtime.close。沒有執行真實步驟 1 接管／整理，也沒有啟動、停止 Bridge/Core/FPGA 或動作。離線通過及連線快照不能代替實機動作、急停與機械追蹤測試，也不能保證日後網路永不斷線。
