import pytest

from redrhex_rl_controller import motor_command_tool


@pytest.mark.parametrize("mode", sorted(motor_command_tool.RETIRED_MOTION_MODES))
def test_retired_motion_modes_fail_before_ros_initialization(monkeypatch, mode):
    initialized = False

    def unexpected_init():
        nonlocal initialized
        initialized = True
        raise AssertionError("rclpy.init must not run for a retired motion mode")

    monkeypatch.setattr(motor_command_tool.rclpy, "init", unexpected_init)

    with pytest.raises(SystemExit) as exc_info:
        motor_command_tool.main([mode])

    assert exc_info.value.code != 0
    assert "no ROS publisher was created" in str(exc_info.value)
    assert "mapping CLI is not implemented" in str(exc_info.value)
    assert initialized is False


def test_enable_flag_always_fails_before_ros_initialization(monkeypatch):
    initialized = False

    def unexpected_init():
        nonlocal initialized
        initialized = True
        raise AssertionError("rclpy.init must not run when --enable is rejected")

    monkeypatch.setattr(motor_command_tool.rclpy, "init", unexpected_init)

    with pytest.raises(SystemExit) as exc_info:
        motor_command_tool.main(["disable", "--enable", "--confirm-risk"])

    assert exc_info.value.code != 0
    assert "Motion/enable modes are retired" not in str(exc_info.value)
    assert "Manual motor enable/motion modes are retired" in str(exc_info.value)
    assert initialized is False


def test_list_joints_remains_read_only_and_does_not_initialize_ros(monkeypatch, capsys):
    monkeypatch.setattr(
        motor_command_tool.rclpy,
        "init",
        lambda: (_ for _ in ()).throw(
            AssertionError("list-joints must not initialize ROS")
        ),
    )

    motor_command_tool.main(["list-joints"])

    output = capsys.readouterr().out
    assert "Revolute_" in output
    assert "no motor command is sent" in output


def test_interrupted_disabled_stream_exits_nonzero_and_reports_unknown(monkeypatch):
    class InterruptedTool:
        def __init__(self, _args):
            pass

        def run(self):
            raise KeyboardInterrupt

        def destroy_node(self):
            return None

    monkeypatch.setattr(motor_command_tool, "MotorCommandTool", InterruptedTool)
    monkeypatch.setattr(motor_command_tool.rclpy, "init", lambda: None)
    monkeypatch.setattr(motor_command_tool.rclpy, "ok", lambda: True)
    monkeypatch.setattr(motor_command_tool.rclpy, "shutdown", lambda: None)

    with pytest.raises(SystemExit) as exc_info:
        motor_command_tool.main(["disable"])

    assert exc_info.value.code != 0
    assert "output state is UNKNOWN" in str(exc_info.value)
