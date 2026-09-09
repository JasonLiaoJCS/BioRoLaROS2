已部署到現場 Orin，設定 revision 9。127項離線測試全部通過；Calibration、Standing、Tripod 的部署後唯讀設定檢查均成功。舊的已保護停止 Tripod PID75799已退出；沒有啟動動作或發送電源命令。

本次依使用者明確選擇：「位置誤差只記錄警告，不自動停機，供我在旁邊調校」。這份政策取代先前交接中「9000／18000 counts必須自動停止」的要求；其他保護沒有因此關閉。

**新的行為**

現場設定使用既有鍵：

```yaml
parameters:
  rinbo_tripod_rslip:
    safety:
      stop_on_position_error: false
```

`false`現在的明確意思是「有限數值的位置追蹤誤差只警告」。不是忽略NaN，也不是關閉所有安全檢查。

- 有限誤差超過9000，包含超過舊18000上限：記錄 `TRIPOD_POSITION_WARNING ... mode=warn_only action=continue`，不進入SAFETY_STOP，不讓Calibration／Standing完成紀錄失效。
- 位置誤差持續存在：持續運轉；警告最多約每秒一筆，完整數值仍保存在controller debug和TRIPOD_TRACE。
- STARTUP時間到時，有限位置誤差不構成隱藏的對準等待；目標位置仍連續、多圈位置仍保留，不用重設零點把誤差抹掉。
- 健康腿的NaN／Inf、無效軌跡／速度、非正回饋時間間隔：仍停止。L3依原屏蔽規則排除。
- 過流、電源異常、通訊失聯、發布者／仲裁異常、手動停止／急停：原行為保留。
- PWM80、slew250/s、L3屏蔽、8秒STARTUP與ratio8→4原現場設定保留，沒有自動加大輸出或改速度。
- 到target_ratio仍持續RUNNING；正常SIGINT停止不切relay、不關Bridge／FPGA／Core。

9000與18000仍作診斷參考；這次不是將硬上限換成另一個任意大數字。`TRIPOD_POSITION_POLICY mode=warn_only ...` 在啟動日誌明確顯示所選政策。`TRIPOD_POSITION_WARNING`不可被GUI當成`TRIPOD SAFETY STOP`。

**設定介面與相容性**

新增只修改這個政策的管理命令：

```bash
ros2 run rinbo_fsm rinbo_legs tripod-position-policy warn --dry-run
ros2 run rinbo_fsm rinbo_legs tripod-position-policy warn
# 日後若使用者要恢復自動位置停止：
ros2 run rinbo_fsm rinbo_legs tripod-position-policy stop
```

變更仍使用動作／設定鎖、原子寫入及revision/hash；既有有效的Calibration／Standing紀錄可以保留，既有失效紀錄不能被偽造恢復。當動作正在執行時不偷偷改變它的政策。

舊版共用設定讀取器會拒絕`stop_on_position_error:false`，因此部署包含Tripod、rinbo_legs，以及連結該讀取器的Calibration／Standing／Manual。後三者的控制原始碼未改，更新的是設定相容性。單獨換Tripod而留著舊讀取器會讓下一次Calibration啟動失敗，故本次一併處理。

Windows現有Tripod啟動命令不需要改參數；Orin讀取現場設定。若GUI另有硬編碼要求此鍵必須true，需移除該項額外限制，顯示「位置誤差：只警告」。其他保護狀態照實呈現；不要把位置警告當成停機或自動呼叫急停。

**驗證與部署紀錄**

實際結果以 [deployment.json](diagnostics/tripod_position_warning_20260909/deployment.json)、[測試結果](diagnostics/tripod_position_warning_20260909/test-results.json) 為準。原檔備份位於 `.codex-backups/tripod_position_warning_20260909/`。

驗證重點：有限誤差超過18000及持續誤差不停止；STARTUP不被有限誤差卡住；NaN／Inf、L3屏蔽與急停仍正確；警告模式能接受手動停止；切回stop模式保留原門檻行為；政策預覽不寫入、變更可復原且不改其他控制器參數；Calibration／Standing及其餘核心保護回歸。

部署不會啟動任何馬達或上電。已知舊Tripod PID75799先前因L2誤差停止；只有核對其boot/start_ticks/exe與故障日誌一致時，才以SIGINT使該已停止的程序退出並釋放設定鎖。若有不同身分或其他正在執行的動作，部署不會任意終止它。
