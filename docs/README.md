# 我要做什麼，就看哪一份

**查標準參數或恢復誤改設定：先開 [重要參數基準](IMPORTANT_PARAMETER_BASELINE.md)。** 這份是固定標準，請勿隨一般程式維修更新。

**日常操作先看 [Control Panel 簡明教學](control_panel_zh_TW.md)：開控制台、屏蔽腳、調動作、測單腳與停止。** 更多細節再查下列教學。

| 我要做的事 | 打開這份 | 從哪裡開始 |
|---|---|---|
| **讓一隻腳動，調角度、速度或相位** | [單腳測試](manual_leg_control_zh_TW.md) | 用 `./robot.sh` 開操作台，依序選 1 → r（入門範例）→ 3 |
| **把模擬訓練的模型放到真機測試** | [Sim2Real](redrhex_sim2real_sbrio.md) | 先看「先判斷你在哪一步」，確認模型格式與缺少的資料 |
| **禁止某幾隻腳參與動作，或恢復它們** | [腿部啟用／屏蔽](orin_multileg_operation.md) | `./robot.sh` 選 **8**，用編號查看、屏蔽、解除屏蔽 |

**只選一隻腳做這次動作，與把其他腳從校正等流程中屏蔽，是兩件事。** 前者看單腳測試；後者看腿部啟用／屏蔽。

需要看即時角度回讀、校正錯誤或保存紀錄時，開 [監控與錄製](rinbo_monitor.md)。

## 常用檔案在哪裡

| 找什麼 | 位置 |
|---|---|
| 中文操作台 | [robot.sh](../robot.sh) |
| 單腳／速度／相位範例 | [manual_single_leg.yaml](../src/rinbo_fsm/config/manual_single_leg.yaml)、[manual_velocity.yaml](../src/rinbo_fsm/config/manual_velocity.yaml)、[manual_phase.yaml](../src/rinbo_fsm/config/manual_phase.yaml) |
| 現場啟用／屏蔿名單 | [/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml) |
| Sim2Real 現場設定 | `/home/jetson/redrhex_site/`；以啟動時指定的 controller／bridge YAML 為準 |
| 新模型與驗證完成的模型 | `/home/jetson/redrhex_models/incoming/<tag>/`、`/home/jetson/redrhex_models/policy_verified_<tag>.onnx` |
| 監控錄製檔 | 從工作區啟動時預設在 `log/monitor_recordings/` |

## 只有查細節時才需要

- [手動控制技術參考](reference/manual_control.md)：自己寫 YAML、角度定義、控制計算、分視窗指令與排錯。
- [Windows R-Slip v6.6 參考](reference/rslip_v6_6.md)：原有 Windows 啟動器的操作流程；其中 Windows 檔案不在這個 ROS 工作區。
- [操作台程式結構](../src/rinbo_control/README.md)、[RL 程式規格](../src/redrhex_rl_controller/README.md)：修改程式時查閱。
