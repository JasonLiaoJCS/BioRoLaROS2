# 腿部啟用／屏蔽：決定哪些腳可以參與動作

[回到入口](README.md)｜[單腳測試](manual_leg_control_zh_TW.md)｜[Sim2Real](redrhex_sim2real_sbrio.md)

**想讓校正、站立、Tripod 與手動控制排除某幾隻腳，就在這裡設定。** 名單保存在 Orin；修改後不用重新編譯。

**日常不用打下面的 ROS 指令：開啟 `./robot.sh`，選 8「腳的屏蔽管理」即可。** 選 1 屏蔽、選 2 解除，再填腳的編號；可一次填多個。主畫面會顯示目前名單。完整步驟見 [Control Panel 簡明教學](control_panel_zh_TW.md)。以下指令保留作為手動操作與排查參考，修改的是同一份共用設定。

只是這一次想讓 L2 動，直接在[單腳操作台](manual_leg_control_zh_TW.md)選 L2 即可。操作台會只校正與控制本次選中的腳，不修改共用屏蔿名單。不帶 `--plan` 直接執行 `rinbo_cali`，則仍校正全部啟用腳，供 Standing／Tripod 流程使用。

## 1. 先看目前名單

在 Jetson／Orin 終端機執行：

```bash
cd /home/jetson/rinbo_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rinbo_fsm rinbo_legs status
```

它只顯示設定，不會上電。以這次輸出的「屏蔽腿／啟用腿」為準，不要把教學中的例子當成機器現在的狀態。

| 腿名 | 位置 |
|---|---|
| L1、L2、L3 | 左前、左中、左後 |
| R1、R2、R3 | 右前、右中、右後 |

實體接線仍須對照機器標籤。`disabled_legs` 是屏蔿名單，沒列在裡面的腿就是啟用腿。

## 2. 停止動作，再選一個變更

先用目前的操作介面停止動作，等待 Calibration／Standing／Tripod／手動控制／RL 等控制程序退出並確認馬達停用。若要碰機構、改接線或恢復故障腿，先完成關電與現場檢查。

**下列各列是不同用途的選項，不是要從上到下全部執行。**

| 我要做什麼 | 指令 | 結果 |
|---|---|---|
| 指定完整屏蔿名單 | `ros2 run rinbo_fsm rinbo_legs set L1 L3` | 只屏蔽 L1、L3；原本其他屏蔽項目會被移除 |
| 多屏蔽一隻腳 | `ros2 run rinbo_fsm rinbo_legs disable R2` | 保留原名單，再加入 R2 |
| 重新允許某隻腳 | `ros2 run rinbo_fsm rinbo_legs enable L3` | 只移除 L3 的屏蔽，其他項目保留 |
| 恢復全部六腿 | `ros2 run rinbo_fsm rinbo_legs enable-all` | 清空屏蔿名單；僅在所有腿都已確認可用時使用 |

`disable`、`enable` 也能接多個腿名，例如 `disable L1 L3`。不確定是否要替換整份名單時，用 `disable`／`enable` 增減指定腿。

「啟用」只是允許後續控制，不會立刻上電或轉動。`enable-all` 也不代表故障已修好。六腿全部屏蔽時可以保存設定，但動作程式會拒絕開始。

## 3. 核對結果，再重新校正

```bash
ros2 run rinbo_fsm rinbo_legs status
ros2 run rinbo_fsm rinbo_cali --check-config
```

兩行都是檢查。要看到正確腿名，且沒有 `[FATAL]`。名單有變更就不能沿用舊校正／站立完成紀錄；不要手工修改完成紀錄。

| 接下來要做的事 | 下一步 |
|---|---|
| 單腳測試 | 回[單腳操作台](manual_leg_control_zh_TW.md)，讓它按流程重新校正後執行 |
| R-Slip 站立／Tripod | 用原有通訊與電源流程，重新依序完成 Calibration → Standing → Tripod |
| RL 模型測試 | 回 [Sim2Real](redrhex_sim2real_sbrio.md)，核對模型與兩份 RL YAML 的腿部規格；FSM 改好不代表模型也改好 |

通訊、電源及機身支撐都已就緒時，R-Slip 的實際動作入口如下。每個程序成功後先按 Ctrl+C 等待退出、確認馬達輸出為 false，才進下一個：

| 階段 | 指令 | 成功標記 |
|---|---|---|
| 校正 | `ros2 run rinbo_fsm rinbo_cali` | `State: DONE` |
| 站立 | `ros2 run rinbo_fsm rinbo_standing` | `ALL <N> HEALTHY LEGS STANDING` |
| Tripod | `ros2 run rinbo_fsm rinbo_tripod` | `Startup done, entering RUNNING` |

`N` 是實際啟用腿數。Tripod 正常停止會顯示 `=== Fully stopped ===`。出現 `SAFETY STOP` 或 `[FATAL]` 就處理原因，不接下一步。Windows 啟動方式見[原流程參考](reference/rslip_v6_6.md)。

## 屏蔽的範圍

- 主馬達在最後發布前強制停用，輸出為 0；屏蔽腿顯示 `SKIPPED`，不算校正成功。
- 現有協定沒有逐顆伺服斷電功能。屏蔽主馬達不代表同腿伺服已斷電；要整腿隔離，依現場硬體方式處理 Main Drive 與 Servo。
- 電源工具自動讀相同名單，排除禁用腿的逐腿電流檢查；全域電源、匯流排電壓、通訊與健康腿的保護仍有效。
- 多腿模式是 `supported_leg_test`，適用可靠支撐／懸空測試；不保證少腿時能在地面站穩或行走。
- **ONNX／RL 是獨立規格。** 現有架空範本及封裝流程有單腿限制，不能直接套用這裡的任意多腿組合。

## 設定與程式在哪裡

| 位置 | 用途 |
|---|---|
| [現場 YAML](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml) | 唯一 FSM 名單來源；頂層含 `schema_version`、`revision`、`disabled_legs`、`test_mode`、`parameters` |
| [rinbo_legs.cpp](../src/rinbo_fsm/src/rinbo_legs.cpp) | 查看、加入、移除屏蔽的指令入口 |
| [robot_config.cpp](../src/rinbo_fsm/src/robot_config.cpp)／[robot_config.hpp](../src/rinbo_fsm/src/robot_config.hpp) | 設定讀寫、版本、鎖定、校正與站立完成紀錄 |
| [disabled_legs.hpp](../src/rinbo_fsm/src/disabled_legs.hpp) | 腿名對應、屏蔽判斷與輸出限制 |
| [rinbo_cali.cpp](../src/rinbo_fsm/src/rinbo_cali.cpp)／[rinbo_standing.cpp](../src/rinbo_fsm/src/rinbo_standing.cpp)／[rinbo_tripod.cpp](../src/rinbo_fsm/src/rinbo_tripod.cpp) | 各動作如何跳過禁用腿與判斷完成 |
| [rinbo_power_tool.py](../src/redrhex_lowlevel_bridge/redrhex_lowlevel_bridge/rinbo_power_tool.py) | 電源流程載入同一份名單 |
| [FSM test/](../src/rinbo_fsm/test) | 設定管理與多腿測試程式 |

設定檔在工作區外，編譯不會覆寫它。用 `rinbo_legs` 管理名單，不要把舊 degraded YAML、`--params-file` 或 `-p hardware.disabled_legs` 傳給 FSM，也不要用 `ros2 param set` 在動作中切換。

## 常見問題

| 狀況 | 怎麼處理 |
|---|---|
| `Configuration/action is busy` 或鎖被占用 | 讓現有動作程序正常停止並退出，再操作；不要刪鎖檔繞過 |
| 名單沒變、版本也沒增加 | 對已禁用腿再禁用，或對已啟用腿再啟用，本來就不會修改版本 |
| 設定缺失、損壞、格式不支援 | 先修復設定；程式不會自動退回全腿啟用 |
| `沒有可測試腿` | 六腿都被屏蔽；只在確認某腿可用後解除它的屏蔽 |
| 動作中某腿故障 | 程式停止並指出腿名，不會自動幫你改名單；處理後再決定是否屏蔽 |
