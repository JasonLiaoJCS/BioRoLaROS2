# 三大工作入口：單腳測試、Sim2Real、腿部啟用／屏蔽

以後找教學、改設定或讀程式，先從本頁開始。以下路徑以本工作區為準；現場設定與模型另存在工作區外。

| 你要做的事 | 先讀這份教學 | 主要程式／設定入口 |
|---|---|---|
| 測試一隻腳，設定角度、速度、相位 | [單視窗操作教學](manual_leg_control_zh_TW.md) | [robot.sh](../robot.sh)、[單腳設定](../src/rinbo_fsm/config/manual_single_leg.yaml)、[rinbo_manual.cpp](../src/rinbo_fsm/src/rinbo_manual.cpp) |
| 把模擬訓練的模型接到真機（Sim2Real） | [完整操作手冊](redrhex_sim2real_sbrio.md)；新模型先看[最簡說明書](redrhex_sim2real_quickstart.md) | [RL 控制器套件](../src/redrhex_rl_controller/README.md)、[啟動檔](../src/redrhex_rl_controller/launch/redrhex_policy_bringup.launch.py) |
| 屏蔽／重新啟用某幾隻腳 | [Orin 多腿屏蔽操作](orin_multileg_operation.md) | [rinbo_legs.cpp](../src/rinbo_fsm/src/rinbo_legs.cpp)、[現場屏蔽設定](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml) |

## 1. 單腳測試：教學、設定、程式

**平常使用先看單視窗教學；想逐條理解指令，再看分視窗教學。**

| 位置 | 什麼時候找它 |
|---|---|
| [manual_leg_control_zh_TW.md](manual_leg_control_zh_TW.md) | 日常主入口：用 `./robot.sh` 開操作台，依選單完成通訊、設定動作、校正與執行 |
| [manual_leg_control_terminal_zh_TW.md](manual_leg_control_terminal_zh_TW.md) | 從開機開始，分視窗依序啟動 Bridge、上電、校正、檢查單腳設定、動作與關電 |
| [manual_leg_control_reference_zh_TW.md](manual_leg_control_reference_zh_TW.md) | 查架構、角度定義、YAML 寫法、控制計算與排錯；第 8 節是程式說明 |
| [manual_single_leg.yaml](../src/rinbo_fsm/config/manual_single_leg.yaml) | 直接用 `rinbo_manual --plan` 時的單腳範例：L2 移到相對校正零點的 5° |
| [manual_velocity.yaml](../src/rinbo_fsm/config/manual_velocity.yaml) | 各腳速度設定範例 |
| [manual_phase.yaml](../src/rinbo_fsm/config/manual_phase.yaml) | 各腳相位設定範例；相位表示旋轉週期中的位置 |
| [robot.sh](../robot.sh) | 工作區最外層的操作台啟動腳本 |
| [console.py](../src/rinbo_control/rinbo_control/console.py) | 中文選單、選哪些腳、編輯動作、執行確認 |
| [runtime.py](../src/rinbo_control/rinbo_control/runtime.py)／[sbrio.py](../src/rinbo_control/rinbo_control/sbrio.py) | 通訊、上電、校正順序，以及 sbRIO core／FPGA 啟動 |
| [rinbo_manual.cpp](../src/rinbo_fsm/src/rinbo_manual.cpp) | 實際手動控制節點：讀回授、控制主馬達、處理停止 |
| [manual_motion.hpp](../src/rinbo_fsm/src/manual_motion.hpp) | 解析動作 YAML、產生角度／速度／相位軌跡、限制輸出 |
| [rinbo_cali.cpp](../src/rinbo_fsm/src/rinbo_cali.cpp) | 動作前的校正流程，包括伺服定位、Hall 偵測與主馬達歸零 |

操作台裡的「小轉一下 +5°」是從目前位置再轉 5°；`manual_single_leg.yaml` 的 `angle_deg: 5.0` 是移到校正零點後的 5°，兩者目標不同。

單腳動作只選一隻腳，不代表前面的校正只動那隻腳；首次校正會涉及全部未禁用主馬達及相關伺服。詳細範圍以主教學與操作台確認畫面為準。

相關驗證程式：[手動軌跡測試](../src/rinbo_fsm/test/test_manual_motion.cpp)、[控制器測試](../src/rinbo_fsm/test/test_manual_controller.cpp)、[操作台測試目錄](../src/rinbo_control/test)。

## 2. Sim2Real：詳細流程與實作位置

Sim2Real 指把模擬中訓練的控制模型接到真機。這裡的 policy 是控制模型，ONNX 是模型檔案格式。

| 教學位置 | 內容／閱讀時機 |
|---|---|
| [redrhex_sim2real_quickstart.md](redrhex_sim2real_quickstart.md) | 新模型放哪裡、需要哪些配套檔案、模型檢查、完整比對與封裝、上機前檢查 |
| [redrhex_sim2real_sbrio.md](redrhex_sim2real_sbrio.md) | 詳細主手冊：第 2 步現場 YAML；第 3 步 encoder／IMU 校正；第 4 步模型驗證封裝；第 5～8 步上機前檢查、只讀驗收、架空測試與停止 |
| [redrhex_sim2real_after_rslip.md](redrhex_sim2real_after_rslip.md) | 已走 R-Slip 啟動流程後，如何銜接 Tripod 檢查與 policy 接管 |
| [redrhex_rl_controller/README.md](../src/redrhex_rl_controller/README.md) | 模型輸入與輸出規格、感測模式、驗證要求及套件測試 |

| 程式／設定位置 | 負責什麼 |
|---|---|
| [redrhex_policy_bringup.launch.py](../src/redrhex_rl_controller/launch/redrhex_policy_bringup.launch.py) | 啟動 policy 與橋接節點，核對設定 |
| [rl_controller_node.py](../src/redrhex_rl_controller/redrhex_rl_controller/rl_controller_node.py) | 真機 RL 控制主流程 |
| [observation_builder.py](../src/redrhex_rl_controller/redrhex_rl_controller/observation_builder.py) | 將感測資料組成模型輸入 |
| [policy_onnx_runner.py](../src/redrhex_rl_controller/redrhex_rl_controller/policy_onnx_runner.py) | 載入並執行 ONNX 模型 |
| [action_decoder.py](../src/redrhex_rl_controller/redrhex_rl_controller/action_decoder.py) | 將模型輸出轉成馬達目標 |
| [state_machine.py](../src/redrhex_rl_controller/redrhex_rl_controller/state_machine.py)／[safety_filter.py](../src/redrhex_rl_controller/redrhex_rl_controller/safety_filter.py) | 控制啟用階段與安全限制 |
| [scripts/](../src/redrhex_rl_controller/scripts) | `check_onnx_io.py` 檢查模型輸入輸出；`compare_onnx_with_torch.py` 比對；`package_verified_policy.py` 封裝 |
| [golden_recorder.py](../src/redrhex_rl_controller/redrhex_rl_controller/golden_recorder.py) | 提供訓練端錄製真實模擬軌跡的工具，供模型比對使用 |
| [preflight_check.py](../src/redrhex_rl_controller/redrhex_rl_controller/preflight_check.py) | 上機前檢查 |
| [RL config/](../src/redrhex_rl_controller/config)／[low-level bridge config/](../src/redrhex_lowlevel_bridge/config) | 模型控制與硬體橋接的設定範本；依教學選用對應模式 |
| [lowlevel_bridge_node.py](../src/redrhex_lowlevel_bridge/redrhex_lowlevel_bridge/lowlevel_bridge_node.py)／[rinbo_ros_backend.py](../src/redrhex_lowlevel_bridge/redrhex_lowlevel_bridge/rinbo_ros_backend.py) | 將 policy 指令與真機 Rinbo 訊息對接 |
| [rinbo_ros_bridge.cpp](../src/rinbo_ros_bridge/src/rinbo_ros_bridge.cpp) | ROS 與 sbRIO gRPC 通訊的最終橋接程式 |

工作區外另有兩個重要位置：

- `/home/jetson/redrhex_site/`：現場實際使用的 YAML；要核對啟動時指定的是哪一份。
- `/home/jetson/redrhex_models/`：模型目錄；教學規定新模型放 `incoming/<tag>/`，完成驗證與封裝後才產生 `policy_verified_<tag>.onnx`。

**版本適用範圍：** 上述 Sim2Real 文件開頭已註明，舊 R-Slip `--params-file`、`--disabled-leg` 與腿名環境變數流程已被取代。涉及 Calibration／Standing／Tripod 和電源工具時，以 [Orin 多腿操作](orin_multileg_operation.md) 為準。ONNX 模型仍有獨立的腿部設定與驗證要求，不能把 R-Slip 的多腿設定直接當成模型已支援。

這組教學的實機流程以架空測試為範圍。模型能否執行，需依當次模型與現場設定完成驗證；找到教學或模型檔不表示已通過。

## 3. 屏蔽／開啟某幾隻腳：目前的主要入口

**R-Slip 與手動測試的腿部名單，先找 [Orin 多腿屏蔽操作](orin_multileg_operation.md)。**

| 位置 | 用途 |
|---|---|
| [orin_multileg_operation.md](orin_multileg_operation.md) | 查看名單、一次屏蔽多腿、加入屏蔽、解除部分屏蔽、全部解除，以及變更後的重新校正流程 |
| [現場 rinbo_fsm_disabled_leg.yaml](/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml) | 唯一有效的 FSM 現場設定；頂層 `disabled_legs` 是屏蔿名單，未列出的腿是啟用腿 |
| [rinbo_legs.cpp](../src/rinbo_fsm/src/rinbo_legs.cpp) | 管理指令入口：`status`、`set`、`disable`、`enable`、`enable-all` |
| [robot_config.hpp](../src/rinbo_fsm/src/robot_config.hpp)／[robot_config.cpp](../src/rinbo_fsm/src/robot_config.cpp) | 固定設定路徑、讀寫檢查、版本、鎖定，以及校正／站立完成紀錄管理 |
| [disabled_legs.hpp](../src/rinbo_fsm/src/disabled_legs.hpp) | 腿名對應、逐腿屏蔽與輸出限制 |
| [rinbo_cali.cpp](../src/rinbo_fsm/src/rinbo_cali.cpp)／[rinbo_standing.cpp](../src/rinbo_fsm/src/rinbo_standing.cpp)／[rinbo_tripod.cpp](../src/rinbo_fsm/src/rinbo_tripod.cpp) | 校正、站立、Tripod 如何使用名單、跳過禁用腿與判斷完成 |
| [rinbo_power_tool.py](../src/redrhex_lowlevel_bridge/redrhex_lowlevel_bridge/rinbo_power_tool.py) | 電源工具如何載入相同名單並檢查電流 |

教學中的指令意義如下；變更前先依主教學停止動作，並等待控制程序退出。

| 指令 | 意義 |
|---|---|
| `ros2 run rinbo_fsm rinbo_legs status` | 只查看目前屏蔽／啟用名單 |
| `ros2 run rinbo_fsm rinbo_legs set L1 L3` | 將整份屏蔿名單替換為 L1、L3 |
| `ros2 run rinbo_fsm rinbo_legs disable R2` | 在原名單中加入 R2 |
| `ros2 run rinbo_fsm rinbo_legs enable L3` | 從屏蔿名單移除 L3，讓它可參與後續動作 |
| `ros2 run rinbo_fsm rinbo_legs enable-all` | 清空屏蔿名單 |

這裡「啟用」是允許該腿參與後續控制，並不會立刻上電或讓腳轉動。只想指定這次手動動作的腳，使用第 1 節的操作台／動作 YAML；要讓校正等流程也排除某腳，才使用本節的屏蔿名單。名單變更後需重新校正；Standing／Tripod 依主教學重新完成前置階段。

主馬達的軟體屏蔽不代表同腿伺服已斷電；整腿隔離的限制見主教學「支撐測試與控制邊界」。

**舊文件與 RL 相關位置：** [redrhex_disabled_leg_operation.md](redrhex_disabled_leg_operation.md)、[rinbo_leg_mask.py](../src/redrhex_rl_controller/redrhex_rl_controller/rinbo_leg_mask.py)、[degraded_mode.py](../src/redrhex_rl_controller/redrhex_rl_controller/degraded_mode.py)。其中舊跨 FSM 的 `rinbo_leg_mask auto` 流程不能當成目前多腿管理入口；RL 的模型綁定規則仍須另外核對。[l1_degraded_test.yaml](../src/rinbo_fsm/config/l1_degraded_test.yaml) 與 [disabled_leg_template.yaml](../src/rinbo_fsm/config/disabled_leg_template.yaml) 是舊範例，不是目前 FSM 名單來源。

相關驗證程式：[設定管理測試](../src/rinbo_fsm/test/test_robot_config.cpp)、[校正多腿測試](../src/rinbo_fsm/test/test_cali_multileg.cpp)、[站立多腿測試](../src/rinbo_fsm/test/test_standing_multileg.cpp)、[Tripod 多腿測試](../src/rinbo_fsm/test/test_tripod_multileg.cpp)。

## 共用輔助文件

- [rinbo_monitor.md](rinbo_monitor.md)：操作與校正時查看即時命令、感測器和日誌。
- [rslip_v6_6_reference_zh_TW.md](rslip_v6_6_reference_zh_TW.md)：R-Slip v6.6 操作流程參考。
- [rinbo_control/README.md](../src/rinbo_control/README.md)：中文操作台的模組結構與驗證方式。
