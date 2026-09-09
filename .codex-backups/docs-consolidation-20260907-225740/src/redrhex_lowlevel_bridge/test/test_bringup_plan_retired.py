import pytest

from redrhex_lowlevel_bridge import biorola_bringup_plan


def test_legacy_bringup_plan_fails_closed(capsys):
    with pytest.raises(SystemExit) as exc_info:
        biorola_bringup_plan.main([])

    assert exc_info.value.code == 2
    output = capsys.readouterr().out
    assert "is retired" in output
    assert "redrhex_sim2real_after_rslip.md" in output
    assert "allow_enable=true" not in output
    assert "single-main-velocity" not in output


def test_legacy_bringup_plan_can_only_show_manual_locations(capsys):
    biorola_bringup_plan.main(["--show-manuals"])
    output = capsys.readouterr().out
    assert "redrhex_sim2real_quickstart.md" in output
