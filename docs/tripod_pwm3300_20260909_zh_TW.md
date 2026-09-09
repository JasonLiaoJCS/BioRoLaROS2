# Tripod Max PWM = 3300（2026-09-09）

後續更新：ratio 約 5.9 的實跑證實 250 PWM/s 額外限制會拖延修正，已新增關閉此限制的修正與設定入口；請以 [GitHub 比對與最新修正](tripod_gait_compare_20260909_zh_TW.md) 為準。下文「slew 250/s 不變」描述的是 20:16 部署當時。

已於 20:16 部署並讀回確認，現場設定 revision **12**。相關離線測試 **152 項全部通過**；控制台安裝檢查通過。部署前未觀察到正在執行的 Bridge，沒有自動啟動或重啟。

使用者指定 Tripod 的最大出力為 **3300 原始命令值**。這是上限，不是固定命令，也不代表百分比、伏特或實測扭力。

## 必須一起生效的層次

1. 現場 YAML：`parameters.rinbo_tripod_rslip.max_pwm: 3300.0`。
2. Tripod：`rinbo_tripod.cpp` 的 `max_pwm_` 讀取預設 3300；STARTUP、RUNNING、STOPPING 均以 `std::clamp(..., -max_pwm_, max_pwm_)` 限幅。
3. 共用設定讀取：`robot_config.cpp` 對 Tripod 呼叫 `validate_tripod_pwm`，接受 `(0,3300]`；Calibration／Standing／Manual 仍走 80 上限。
4. Bridge：`motor_command_max_pwm: 3300.0`，其傳輸上限與預設同為 3300；不再於 80 擋住 Tripod 命令。這是共用傳輸範圍，其他控制器仍保留各自的 80 上限。

Bridge 以 `grpc_legs[index]->set_voltage(ros_legs[index]->voltage)` 直接複製原始命令。
存檔 FPGA driver 的 `processMotorCommands` 將這個數字限制在 0～4096，轉為 uint16_t，透過 `write_iv_`／`NiFpga_WriteU16` 寫入暫存器。
本次沒有更動 FPGA driver，也沒有量測數值對實際電壓／扭力的關係。

## 設定入口

`./robot.sh` → **15 到位／追蹤限制 → 3 Tripod → 5 出力上限 Max PWM**，可填 1～3300。
p 較寬調機設定使用 3300 且位置誤差只警告；r 舊版數值會恢復 80 並恢復位置誤差停止，均先預覽再儲存。
原生 CLI（N 為 status 回傳的目前版本）：

```bash
/home/jetson/rinbo_ros_ws/build/rinbo_fsm/rinbo_legs status --json
/home/jetson/rinbo_ros_ws/build/rinbo_fsm/rinbo_legs tune-limits tripod --expect-revision N --dry-run max_pwm=3300
# 儲存：相同命令移除 --dry-run
```

## 保留的行為

- Calibration／Standing／逐腳動作 max_pwm 仍為 80；之前 Standing 的 1000-count 到位範圍與 60 秒期限保留。
- Tripod 的增益、軌跡速度、原始 slew 250/s、關閉／減速時間不變。從零爬升到 3300 受 slew 限制，至少需要 13.2 秒；因此上限提高不等於立即輸出 3300。
- L3 屏蔽、單一動作與單一發布者、來源／時間戳檢查、過流／供電／失聯／非有限資料、急停與 SIGINT 停止保留。
- 只改 Tripod 上限時，仍有效的 Calibration／Standing 完成結果可沿用；不重建失效結果。

## Windows 配合

啟動命令不變，但需使用新版 Bridge 與 `redrhex_safe.yaml`；若 Windows 額外覆寫 `motor_command_max_pwm:=80`，需移除或改成 3300。
Windows 的 Bridge 參數探測／預期值表也必須同步為 `motor_command_max_pwm=3300.0`。20:25／20:26 日誌已確認舊檢查仍要求 80，因而拒絕實際為 3300 的 Bridge。不能只改啟動參數而漏改檢查；[可直接交接的 Windows 修改 prompt](windows_bridge_pwm3300_prompt_20260909_zh_TW.md) 包含修改範圍與離線驗收條件。
啟動日誌應顯示 Bridge `max_pwm=3300.0`、Tripod `max_pwm=3300`。
只改 Tripod YAML、保留舊 Bridge 的 80 上限，會導致超過 80 的命令被拒絕。本次不會自動啟動或重啟 Bridge。

## 測試與部署證據

`docs/diagnostics/tripod_pwm3300_20260909/` 保存測試結果、部署 hash 與現場設定讀回。
離線測試包括：Tripod 實際控制迴圈能達到 3300 且不超過、slew 仍有效、L3 零輸出、正常停止；設定只改 Tripod、非法數值拒絕；Bridge 接受 3300、拒絕 3300.01／負值／NaN／Inf，明確設為 80 時仍拒絕超過 80。
沒有執行真實 Tripod、上電或送任何運動命令。
