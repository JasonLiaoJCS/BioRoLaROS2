# Windows 修正交接：Bridge 預期出力上限仍固定為 80

將以下內容交給能存取 Windows RSlipLauncher 原始碼的桌面版開發代理。

---

請直接修正 Windows RSlipLauncher 的 Bridge 參數相容性檢查，完成離線測試及可執行版本的建置。保留目前其他工作階段的修改，不要自動執行建立通訊、上電或機器人動作。

已確認的事故日誌：

```text
2026-09-09 20:25:19 與 20:26:21
RSLIP_FATAL=ProbeFault:wrong_parameter:motor_command_max_pwm:expected=80.0:actual=3300.0
後續：bridge_present_during_pre_power_recovery:1
ERROR [40]: Bridge graph/params/live state 失敗（remote exit=49）
```

Orin 已依使用者要求部署 Tripod Max PWM 3300，Bridge 必須允許傳輸至 3300。這是原始命令值的上限，不是百分比，也不是要求固定輸出 3300。Windows 的固定預期值 80 已與目前部署不一致。

請完成以下修改：

1. 搜尋 `motor_command_max_pwm`、`wrong_parameter`、`ProbeFault`，找出建立通訊的參數白名單／預期值表、產生遠端探測腳本的程式、啟動參數覆寫及相關測試。修正實際被建置／執行的來源與內嵌資源，不只修改說明文件。不要全面取代程式中的所有 80。
2. 本次部署的 Bridge 預期 `motor_command_max_pwm` 改為數值 **3300.0**。保留原有型別、有限值及其他參數檢查。集中定義這個預期值，讓探測腳本與測試使用同一份來源；不要直接把 actual 複製為 expected，也不要無條件接受任意上限。
3. 檢查啟動命令是否仍覆寫 `motor_command_max_pwm:=80`；若有，移除這個過期覆寫，使用部署設定，或明確同步為 3300.0。Orin 的部署設定是：

   ```text
   /home/jetson/rinbo_ros_ws/install/rinbo_ros_bridge/share/rinbo_ros_bridge/config/redrhex_safe.yaml
   rinbo_ros2_bridge.ros__parameters.motor_command_max_pwm = 3300.0
   ```

   該 YAML 的 `core_ip` 預設為 `192.168.30.2`；啟動時必須保留目前設定目標的覆寫。本次使用者的 sbRIO 是 `192.168.30.254`，不要因改出力參數而遺失既有 `core_ip`／CORE_MASTER_ADDR／CORE_LOCAL_IP 設定。
4. Calibration、Standing、逐腳動作各自的 PWM 上限仍為 **80**；只有 Tripod 目前為 **3300**，Bridge 的共用傳輸上限為 **3300**。後續 ratio 5.9 診斷已證實 Tripod 的 250 PWM/s 額外限制拖延控制修正，新部署使用 `safety.enable_pwm_slew_limit=false`。同步檢查 Windows 是否強制要求此參數為 true，或啟動／設定時覆寫回 true；若有應移除，允許本次已選擇的 false。不要因保留舊檢查而重新開啟它。保留 L3 屏蔽、過流、供電／通訊保護、來源與新鮮度核對及手動停止；每次操作重新讀取 Orin 的 revision／hash，不固定使用舊版本 12。不要改 Orin 的出力上限回 80 來遷就舊 Windows 檢查。
5. 若唯一 Bridge 已存在且位址、參數、來源與實際新回讀都通過，沿用它。不能因本次參數檢查錯誤而直接再啟動一份或強制殺掉現有 Bridge。
6. 保留第一個停止／失敗原因。將此種不相容顯示為「Windows 預期參數與 Orin 部署不一致」，列出 expected、actual 及處理方式。`bridge_present_during_pre_power_recovery:1` 只是後續復原檢查觀察到一個 Bridge，不能取代首因，也不能被描述成存在多份 Bridge。
7. 保留完整 all-off、停止與急停路徑、原有新鮮關電回讀與程序身分核對，不假造 Bridge absence、成功關電或清除舊停止紀錄。檢查是否有停止路徑錯用啟動參數預期值而阻擋關電要求；如有，將「啟動相容性檢查」與「可信停止／關電所需檢查」分開，保留後者，不因 PWM 版本不一致而略過整個停止嘗試。此項需以程式與測試核實，現有日誌並未證明完整關電成功或失敗的最終結果。
8. 加入離線測試：
   - 新版預期 3300.0、實際 3300.0：此參數檢查通過；其他 graph／位址／新回讀檢查仍執行。
   - 新版預期 3300.0、實際 80.0：清楚回報部署不一致，不自動改參數、上電或重啟。
   - 實際值缺少、型別錯誤、NaN／Inf、超過 3300：拒絕啟動通過。
   - 唯一且正確的既有 Bridge：沿用，不產生第二個 Bridge。
   - 參數不符後的復原：首因保留，既有完整 all-off 路徑仍執行；沒有新鮮關電證據時不得宣告成功。
   - Calibration／Standing／逐腳動作的 80、Tripod 的 3300、L3 屏蔽與既有停止語意不受影響。
9. 完成建置，回報修改檔案與行號、測試結果、產物位置及可辨識的新版本號。啟動日誌需能辨認 App 版本與本次預期的 Bridge 上限。不要自動操作真實機器人。

Windows CLI／GUI 的日常按鈕可以保持原樣；本次關鍵是更新內部部署參數契約與錯誤說明。

Orin 對照來源：

```text
src/rinbo_ros_bridge/config/redrhex_safe.yaml:12
    motor_command_max_pwm: 3300.0
src/rinbo_ros_bridge/src/motor_output_limits.hpp:10
    inline constexpr double kMaxRawMotorCommand = 3300.0;
src/rinbo_ros_bridge/src/rinbo_ros_bridge.cpp:1019–1020
    declare_parameter<double>("motor_command_max_pwm", rinbo_bridge::kMaxRawMotorCommand)
```
