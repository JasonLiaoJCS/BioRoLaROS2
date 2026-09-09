from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from redrhex_rl_controller import redrhex_contract as C
from redrhex_rl_controller.observation_builder import ObservationBuilder, SensorV2HistoryBuffer
from redrhex_rl_controller.policy_onnx_runner import PolicyONNXRunner


def _sensor_v2_model(path) -> None:
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    action = numpy_helper.from_array(np.arange(12, dtype=np.float32).reshape(1, 12), name="a")
    velocity = numpy_helper.from_array(np.ones((1, 3), dtype=np.float32), name="v")
    graph = helper.make_graph(
        [
            helper.make_node("Constant", [], ["actions"], value=action),
            helper.make_node("Constant", [], ["base_velocity_estimate"], value=velocity),
        ],
        "sensor_v2",
        [
            helper.make_tensor_value_info("sensor_history", TensorProto.FLOAT, [1, 60, 36]),
            helper.make_tensor_value_info("command", TensorProto.FLOAT, [1, 3]),
        ],
        [
            helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, 12]),
            helper.make_tensor_value_info("base_velocity_estimate", TensorProto.FLOAT, [1, 3]),
        ],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    meta = model.metadata_props.add()
    meta.key, meta.value = "contract_id", "redrhex.student-observation.v2"
    onnx.checker.check_model(model)
    onnx.save(model, path)


def test_sensor_v2_runner_uses_actions_and_validates_both_inputs(tmp_path) -> None:
    path = tmp_path / "sensor_v2.onnx"
    _sensor_v2_model(path)
    runner = PolicyONNXRunner(str(path))
    assert runner.is_sensor_v2
    assert runner.io_info.interface_kind == "sensor_v2"
    action = runner.run(np.zeros((60, 36), np.float32), np.array([0.22, 0.0, 0.0], np.float32))
    assert np.array_equal(action, np.arange(12, dtype=np.float32))
    with pytest.raises(ValueError, match="sensor_history"):
        runner.run(np.zeros((59, 36), np.float32), np.zeros(3, np.float32))
    with pytest.raises(ValueError, match="command"):
        runner.run(np.zeros((60, 36), np.float32), np.zeros(2, np.float32))


def test_sensor_v2_history_is_60hz_oldest_to_newest_and_repeat_initialized() -> None:
    history = SensorV2HistoryBuffer()
    first = history.update(np.zeros(36, np.float32), 1.0)
    assert first.shape == (60, 36)
    assert np.all(first == 0.0)
    unchanged = history.update(np.ones(36, np.float32), 1.005)
    assert np.all(unchanged == 0.0)
    advanced = history.update(np.ones(36, np.float32), 1.0 + 1.0 / 60.0)
    assert np.all(advanced[:-1] == 0.0)
    assert np.all(advanced[-1] == 1.0)
    history.reset()
    reset = history.update(np.full(36, 2.0, np.float32), 2.0)
    assert np.all(reset == 2.0)


def test_sensor_v2_l1_nominal_observation_and_fixed_forward_command() -> None:
    builder = ObservationBuilder(
        {
            "sensor_profile": "encoder_only_rig",
            "encoder_only_rig_acknowledged": True,
            "base_lin_vel_source": "zero",
            "abad_feedback_source": "commanded",
            "command_profile": "fixed_forward",
            "fixed_forward_vx": 0.22,
            "disabled_leg_indices": [3],
            "disabled_leg_observation_mode": "nominal",
        }
    )
    message = SimpleNamespace(
        name=list(C.MAIN_DRIVE_JOINT_NAMES),
        position=[0.1, 0.2, 0.3, 9.9, 0.5, 0.6],
        velocity=[1.0, 2.0, 3.0, 99.0, 5.0, 6.0],
    )
    builder.update_joint_state(message, now_s=1.0)
    frame = builder.build_sensor_v2_frame(1.01)
    assert frame.shape == (36,)
    assert frame[6 + 3] == pytest.approx(np.sin(C.INIT_MAIN_DRIVE_POS[3]))
    assert frame[12 + 3] == pytest.approx(np.cos(C.INIT_MAIN_DRIVE_POS[3]))
    assert frame[18 + 3] == 0.0
    assert frame[24 + 3] == C.INIT_ABAD_POS[3]
    assert frame[30 + 3] == 0.0
    assert np.array_equal(builder.sensor_v2_command(), np.array([0.22, 0.0, 0.0], np.float32))
