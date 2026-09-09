# Orin 工作區完整程式備份：2026-09-09

目的地：`JasonLiaoJCS/BioRoLaROS2`，備份分支 `backup/20260909-workspace`。

本次包含工作區所有 Git 未忽略的原始碼、設定、文件、測試、診斷及 `.codex-backups` 歷史副本。保留其他工作階段的修改，沒有執行機器人控制或重建原生執行檔。

依既有 `.gitignore` 排除 build、install、colcon log、編譯中間檔、Python 快取。這是可還原程式及設定的 Git 備份，不是整顆 Orin 磁碟映像；作業系統、ROS 安裝、SSH 憑證及工作區外的設備執行狀態不在其中。

另外附上工作區外兩份目前有效的操作設定：

- `site-current.yaml`：當時 `/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml`，revision 15。正式恢復優先使用 `tools/parameter_baseline.py restore` 預覽及 `restore --apply`；不要直接退回 revision 或復活舊完成紀錄。
- `control-panel-settings.json`：只包含網路位址、通訊埠與儲存動作計畫，不含 SSH 密碼或認證資料。原位置 `/home/jetson/.local/state/rinbo_control/settings.json`。

重要參數說明見 `docs/IMPORTANT_PARAMETER_BASELINE.md`。操作台修正驗證 264 項測試通過，詳見 `docs/control_panel_settings_sync_20260909.md`。本次未重新執行真實硬體測試。

下載到另一個目錄可用：

```bash
git clone --branch backup/20260909-workspace https://github.com/JasonLiaoJCS/BioRoLaROS2.git rinbo-backup
```

備份分支只在成功推送後可從 GitHub 取得；本機備份是否已上傳，需用 `git ls-remote origin refs/heads/backup/20260909-workspace` 與本機該分支 SHA 核對，不能只憑這份說明認定成功。
