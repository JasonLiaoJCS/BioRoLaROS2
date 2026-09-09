import json

import pytest

from redrhex_rl_controller import estop_tool


def test_clear_requires_confirmation_and_mentions_bridge_restart():
    with pytest.raises(SystemExit) as exc_info:
        estop_tool.main(["clear"])

    assert "final bridge must still be restarted" in str(exc_info.value)


def test_clear_dry_run_exposes_sticky_final_bridge(capsys):
    estop_tool.main(["clear", "--dry-run"])
    payload = json.loads(capsys.readouterr().out)

    assert payload["estop"] is False
    assert payload["final_bridge_restart_required_after_clear"] is True


def test_assert_dry_run_does_not_require_bridge_restart(capsys):
    estop_tool.main(["assert", "--dry-run"])
    payload = json.loads(capsys.readouterr().out)

    assert payload["estop"] is True
    assert payload["final_bridge_restart_required_after_clear"] is False


def test_interrupted_estop_publish_exits_nonzero_and_reports_unknown(monkeypatch):
    def interrupted(_args):
        raise estop_tool.EstopDeliveryUnknown(
            "E-stop publication was interrupted; delivery is UNKNOWN."
        )

    monkeypatch.setattr(estop_tool, "_run_ros", interrupted)

    with pytest.raises(SystemExit) as exc_info:
        estop_tool.main(["assert"])

    assert exc_info.value.code != 0
    assert "delivery is UNKNOWN" in str(exc_info.value)
