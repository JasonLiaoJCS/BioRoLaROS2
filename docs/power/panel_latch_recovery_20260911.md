# Panel 停止鎖恢復修正：panel-latch-v3-20260911

> 操作契約已由 [panel-flow-v4](panel_flow_v4_20260911.md) 更新：一般恢復收進一次 Communications，真正急停使用 ResetEmergency。本文保留歷史診斷與当時部署證據，不再要求日常三步循環。

Windows 維持 protocol1 與原本單次 API。修正全部在 Orin；不增加自動 Stop→Communications→PowerOn、不刪鎖、不把歷史 ACK 當新硬體回讀。

## 現場原因：更正先前診斷

先前文件將「Stop completed 後看到 latch=true/target」描述為殘留鎖，證據不完整。此次唯讀資料庫與原生事件紀錄補出以下順序（同一舊 controller epoch）：

| 原生事件序號 | operation_id | 結果 |
|---|---|---|
| 77 | `1a0f668a55f6445cb8705dbb328003ad` | Stop completed，verified_off=true，cleanup=completed |
| 81 | `83e01a1cfba548f593c0517ece8ac127` | **另一筆 Stop**，Off ACK timeout30，epoch/generation/ACK 無法取得 |
| 86 | `e2905b64119143ad890c9c332298ac97` | **再一筆 Stop**，Off ACK timeout30 |
| 新版91 | `b165dfea2cbd4d49a5a94ad2d642531b` | StopMotion state_unknown31，缺 Bridge／新 disabled 回讀 |
| 新版111 | `cc1f1ed756894d0ead6bb23991a7db70` | Communications completed；沒有解鎖授權 |

兩個額外 Stop 確實存在於 Orin 的操作表與事件日誌，並非 Windows 顯示把 raw tool 結果誤當主操作。舊程式每接受一個新的停止請求便設 latch=true、保存 target；失敗不解除。這解釋了成功 Stop 後仍看到鎖與 target、重載後持續鎖定。記錄不足以認定這兩筆來自哪個 GUI／操作者，不能推定 Windows 自動重送，更不能歸因於 SSH 不通。

另有**已確認的程式缺陷，但沒有證據顯示它就是上述現場鎖定的直接成因**：舊版先 commit completed，再分別 commit latch=false 與 DELETE target。程序若在中間崩潰／重載，便可能留下不一致狀態；重複 Stop 的合併結果也曾分開提交。此次修正這些窗口。

查核期間新增 Stop `573675fd94e740b2984cbbfb4cd46c6f` 已完成，本次 verified_off=true、cleanup=completed；唯讀查核確認 latch=false、backend 已停止。因此**目前不需要再按一次 Stop**。這筆由現場送入，開發測試沒有送出 Stop。

## 程式修正

- `panel_server.py`：停止請求與其鎖來源一起持久保存，成功 Stop 的結果、latch=false、移除 target、歷史提交標記及合併請求結果，在同一 SQLite transaction 提交；提交成功後才發布完成事件。
- 清鎖核對本次 Stop 的原生 verified_off/cleanup 結果與鎖擁有者，不只依賴待處理數量。較舊 Stop 不可清掉新 EmergencyStop 的鎖。
- 資料庫提交失敗回 state_unknown31，保留原始硬體結果與 `state_commit=failed`，不假報 completed。入列失敗回滾，未持久保存前不送硬體指令。
- 持久記錄 `latch_context`：operation_id、action、reason、phase、controller_epoch、時間。舊版本沒有來源資料時標成 legacy_metadata / legacy_stop_latch_source_unrecorded，operation_id=null，不用猜測補值。
- CheckConnection、操作最終結果及 stop_latched 拒絕提供 `control`；ReadLogs 直接提供持久 latch 來源與操作結果，不依賴 backend 狀態查詢。Communications 完成但仍鎖定時，成功 reason 也直接寫出恢復步驟。
- 不自動修補既有資料庫的鎖；不掃歷史 completed 來解鎖。控制者重載不重播硬體請求。

## GUI 契約：原命令不變

```bash
python3 /home/jetson/rinbo_ros_ws/tools/rslip_panel_client.py --request-base64 '<UTF-8 JSON Base64>'
```

```json
{"protocol":1,"request_id":"0123456789abcdef0123456789abcdef","action":"CheckConnection","sbrio_ip":"192.168.30.254","orin_ip":"192.168.30.8","ros_domain_id":99,"stream":false}
```

修改請求每次明確點擊使用新 ID；查詢已完成的同一請求可沿用原 ID／內容或使用 ReadLogs。**新的 Stop 是新的硬體確認，不能把它當成唯讀查詢。**

回覆新增欄位如下，舊有 protocol/request_id/operation_id/status/exit_code/native_exit_code/ACK 均保留：

```json
{
  "controller_version":"panel-latch-v3-20260911",
  "emergency_latched":true,
  "control":{
    "control_state":"stop_latched",
    "emergency_latched":true,
    "latch":{
      "source":"native_controller",
      "operation_id":"失敗停止的32位ID",
      "action":"Stop",
      "reason":"ACK timeout ...",
      "phase":"state_unknown",
      "controller_epoch":"...",
      "updated_at_ns":1789110000000000000
    },
    "recovery_action":"Stop",
    "recovery_steps":[
      "通訊已就緒，但停止鎖仍在。請明確按 Stop，取得本次關電確認並完成清理。",
      "Stop completed 後若要繼續，按 Communications 重建通訊；確認原生停止鎖已解除，再自行按 PowerOn。"
    ],
    "action_availability":{"PowerOn":"blocked_stop_latch","Communications":"available","Stop":"available"},
    "availability_scope":"controller_admission_only; native hardware checks still apply"
  }
}
```

`control` 同時放在最終結果的 data.control 及頂層；CheckConnection 原 data.readiness 不變。PowerOn 拒絕還提供 data.recovery_action / data.recovery_steps，reason 是可直接顯示的文字。Windows 只顯示这些原生資料，無須推導或執行恢復序列。

- readiness=ready 只表示通訊端就緒，**不等於 PowerOn 已允許**。
- control_state：operation_in_progress、stop_latched、connection_required、ready。
- action_availability 只描述控制者是否接受操作；subject_to_native_checks 表示仍須由 Orin 執行原生供電／動作檢查，不保證硬體動作必定成功。
- recovery_action=ReadLogs：等待當前操作；Communications：缺通訊或已正常停止；Stop：通訊ready但仍有停止鎖；null：此層沒有恢復要求。
- 歷史結果中的 control 是該結果提交時的快照。現況請讀 CheckConnection；不要用歷史結果推定當前已關電或鎖已解除。

## 操作者驗證

若目前確實 latch=true 且 ready：明確按 Stop，等 completed 且 control.emergency_latched=false；要繼續再按 Communications，結果應 ready 且不重新上鎖。由操作者自行決定是否 PowerOn；測試不需要自動上電。

若已看到 Stop completed 且 latch=false/backend停止（此次最新狀態），直接 Communications 即可。**不要再送新的 Stop 來驗證上一個 Stop。**

缺 backend 時的新 Stop 仍可能 unknown；這不推翻原來那次完成，也不能由軟體猜成新確認成功。原生會指出 Communications→Stop→Communications 的人工恢復步驟，每一步由操作者明確點擊。

部署與測試證據位於 `docs/diagnostics/panel_latch_20260911/`；實際常駐版本以 CheckConnection.controller_version 為準。PWM、電流／電壓、L3、動作參數與 recorder/mirror 均未修改。

## 測試與正式部署結果

- rinbo_control 全套：329 passed（74.11秒）。先前一輪327 passed/1 failed是既有 guardian 測試在假子程序退出後讀取 `/proc` 遇到 ProcessLookupError；測試改為將檔案／程序已消失視為已退出，沒有改正式 guardian 或 Runtime。
- 最終安裝路徑：66 passed，包含新增停止鎖／SQLite 回滾／合併結果原子提交／ReadLogs 不依賴 backend 的測試。假 backend 與 domain232 localhost 隔離，不使用實體硬體。
- 2026-09-11 16:30正式重載，MainPID274294，controller_epoch=`b8ea7d987db441c1a2be210b7fcb449f`。CheckConnection 與 ReadLogs 均實際回報 `panel-latch-v3-20260911`。
- 重載前再次確認現場 Stop `573675fd94e740b2984cbbfb4cd46c6f` completed、有本次全關 ACK、清理完成、latch=false、target metadata不存在，且沒有 Bridge／原生動作程序，才重載閒置 panel。
- 重載後唯讀結果：emergency_latched=false、control_state=connection_required、recovery_action=Communications。不需要額外 Stop，不會自動 Communications 或 PowerOn。
- recorder PID53632、mirror PID53631、start_ticks276275均保留，summary.csv/events.csv仍開啟，未改錄製輪次或刪資料。
- 正式程式只替換 `panel_server.py`；備份位於 `/home/jetson/.local/state/rinbo-deploy-backups/20260911-162956-panel-latch/`。來源與 install SHA256一致，部署與啟用JSON保留於 diagnostics。
- 參數基準前後輸出相同（既有來源／執行檔差異使check回exit2）；現場YAML SHA256仍為`5866b8ee7bb6a7733c0757fb0d7dd35995a1f78e3a5ec4defe6507c130c13129`。沒有為了讓check通過而改基準。

Windows 不需要新增 action 或前置檢查；若要呈現更清楚的提示，只顯示 Orin 的 reason/control/recovery_steps。實體通訊重建、供電與動作驗證仍由現場操作者明確點擊。
