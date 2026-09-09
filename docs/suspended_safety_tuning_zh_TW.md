# 吊掛測試保護調整（2026-09-07）

適用於目前腳完全懸空的現場設定：電源供應器為 24 V、整機上限 15 A，ROS 的 Bus 電壓是升壓後 36 V。以下是軟體測試門檻，不是馬達、驅動器或升壓器的額定規格。

| 項目 | 原設定 | 本次設定 |
| --- | --- | --- |
| FSM 與吊掛 Policy Bridge 的母線過壓 | 30 V | 42 V |
| 單腳一般電流門檻 | 3 A | 5 A |
| FSM 連續過流筆數 | 10 | 25 筆不同的電源回授 |
| 吊掛 Policy Bridge／Policy 連續過流筆數 | 3／5 | 10／10 |
| 單腳大電流 | 一般計數判斷 | 超過 10 A 立即停止；Policy 使用一般門檻的兩倍，目前也是 10 A |
| Calibration 舵機回原位 | 10 秒 | 20 秒 |
| Calibration／Standing 尋找 Hall | 12 秒 | 30 秒 |
| Calibration 等待停止／歸零回覆 | 各 2 秒 | 各 5 秒 |
| Standing 轉半圈到位 | 7 秒 | 20 秒 |
| Tripod 位置誤差 | 5,000 counts，連續 10 筆 | 9,000 counts，至少 10 筆且連續超限 0.5 秒 |
| Tripod 大幅位置偏離 | 共用原誤差門檻 | 超過 18,000 counts 立即停止 |
| 吊掛 Policy 起始姿勢到位 | 12 秒 | 30 秒 |
| 吊掛 Policy 每次啟動時長 | 固定 3 秒 | 預設 60 秒，可設定大於 0、最多 300 秒 |
| 吊掛 Policy 實測速度停機門檻 | 2 rad/s | 4 rad/s |
| 吊掛 Policy 控制週期間隔／推論耗時 | 12／8 ms | 30／16 ms，仍需連續 3 次超限才停 |

Tripod 一圈約 54,985 counts；9,000 counts 約 59°，18,000 counts 約 118°。一般位置超限恢復後計數與持續時間重新起算。啟動軌跡到達終點時，不再單憑當下的一筆一般超限就停止；仍要回到誤差範圍內才能進入 RUNNING。

電流的筆數只由新回授累積，不能把「25 筆」當成固定秒數。36 V、5 A 的短暫電流峰值不會再被另一個未套用計數的輸出允許檢查立即切掉；持續超限、大電流、急停、失去回授、繼電器斷電與無效資料仍會阻止輸出。首次啟動仍需一筆正常電源資料。

15 A 是電源輸入端整機上限，不能解讀為每腳 15 A。24 V × 15 A = 360 W；36 V 端的理想總輸出只有 10 A，實際可用輸出還要扣除升壓損耗。本次 5 A 是每腳軟體過流門檻，不代表可以讓六腳各自持續使用 5 A，也不代表已驗證整機總功率。原有選配 Bus 電流保護的開關與映射保持現場設定。

Tripod 的 `start_ratio: 8.0`、`target_ratio: 4.0` 保留；它們是軌跡時間倍率，數值越大轉得越慢，並不是額外的停機門檻。PWM 上限、目標轉速、動作方向與校正完成條件也保留。Calibration 現有的歸零重試會在停住且馬達禁能時執行，仍須收到實際零位回授才能完成。

## 實際套用位置

- `/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml`：Calibration、Standing、Tripod 實際使用的設定；保留禁用腳名單，更新 revision，舊完成紀錄失效。
- `/home/jetson/redrhex_site/{lowlevel_bridge,redrhex_policy}_{full_feedback_rig,sensor_v2_suspended_experimental}.yaml`：兩組現場吊掛 Policy 設定。
- 工作區中 `encoder_only_rig`、`full_feedback_rig`、`sensor_v2_suspended_experimental` 範本與對應 Bridge 設定同步更新，包含 Bridge 設定雜湊。
- 修改前備份：`.codex-backups/suspended-tuning-20260907/`。

完成編譯後，重新啟動控制程序，依序執行 Calibration → Standing → Tripod。Policy 仍需符合原本的模型、感測器與校正要求；修改了部署設定或程式後，舊 ONNX 的部署驗證資訊可能需要重新驗證及封裝。本次沒有偽造模型驗證或校正完成紀錄。

## 驗證範圍

使用本機隔離的 ROS 測試網域，以合成回授驗證短暫超限恢復、持續超限停止、大幅超限立即停止、延後到位、歸零實際回覆及禁用腳行為。這些驗證沒有讓實體機器人通電或轉動，不能取代吊掛實測。

已完成驗證：3 個套件編譯成功、9 組 CTest（118 項 Google Test 與 1 項停用工具檢查）通過、573 項 Python 測試通過；現場 revision 4 的三個 FSM 設定檢查通過。現場兩組 Policy／Bridge 的設定雜湊也已核對一致。
