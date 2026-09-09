import pytest

from redrhex_lowlevel_bridge import biorola_fault_diag


def test_power_tuple_match_requires_exact_state() -> None:
    summary = {
        "received": True,
        "digital": False,
        "signal": False,
        "power": False,
    }
    assert biorola_fault_diag.power_tuple_matches(
        summary, biorola_fault_diag.POWER_STATES["off"]
    )
    assert not biorola_fault_diag.power_tuple_matches(
        {**summary, "power": True}, biorola_fault_diag.POWER_STATES["off"]
    )
    assert not biorola_fault_diag.power_tuple_matches(
        {**summary, "received": False}, biorola_fault_diag.POWER_STATES["off"]
    )


@pytest.mark.parametrize("mode", ["power-sweep", "command-path"])
def test_retired_write_mode_fails_before_ros_or_node_construction(
    monkeypatch, mode: str
) -> None:
    calls = {"init": 0, "node": 0}

    def init() -> None:
        calls["init"] += 1

    def construct(_args):
        calls["node"] += 1
        raise AssertionError("retired mode must not construct a ROS node")

    monkeypatch.setattr(biorola_fault_diag.rclpy, "init", init)
    monkeypatch.setattr(biorola_fault_diag, "BioRoLaFaultDiag", construct)

    with pytest.raises(SystemExit) as exc_info:
        biorola_fault_diag.main([mode])

    assert exc_info.value.code != 0
    assert "retired" in str(exc_info.value)
    assert "read-only" in str(exc_info.value)
    assert calls == {"init": 0, "node": 0}


def test_interrupted_read_only_diagnostic_exits_nonzero(monkeypatch) -> None:
    class Report:
        def finalize(self, _summary, _verdict):
            return None

    class InterruptedDiag:
        def __init__(self, _args):
            self.errors = []
            self.report = Report()

        def run(self):
            raise KeyboardInterrupt

        def topic_summary(self):
            return {}

        def build_summary(self, mode, extra=None):
            return {"mode": mode, "verdict": list(self.errors), **(extra or {})}

        def destroy_node(self):
            return None

    monkeypatch.setattr(biorola_fault_diag, "BioRoLaFaultDiag", InterruptedDiag)
    monkeypatch.setattr(biorola_fault_diag.rclpy, "init", lambda: None)
    monkeypatch.setattr(biorola_fault_diag.rclpy, "ok", lambda: True)
    monkeypatch.setattr(biorola_fault_diag.rclpy, "shutdown", lambda: None)

    with pytest.raises(SystemExit) as exc_info:
        biorola_fault_diag.main(["snapshot"])

    assert exc_info.value.code != 0
    assert "incomplete" in str(exc_info.value)
