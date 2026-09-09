"""Explicit deployment state machine for RedRhex."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class RedRhexState(str, Enum):
    BOOT = "BOOT"
    SENSOR_CHECK = "SENSOR_CHECK"
    MOTOR_IDLE = "MOTOR_IDLE"
    INIT_STAND = "INIT_STAND"
    WARMUP = "WARMUP"
    POLICY_READY = "POLICY_READY"
    POLICY_RUN = "POLICY_RUN"
    PROTECTIVE_STOP = "PROTECTIVE_STOP"
    FALL_DETECTED = "FALL_DETECTED"
    RECOVER = "RECOVER"


@dataclass
class StateMachineInputs:
    policy_loaded: bool = False
    sensors_ready: bool = False
    motor_feedback_ready: bool = False
    lowlevel_alive: bool = False
    estop: bool = False
    safety_ok: bool = True
    fall_detected: bool = False
    init_stand_done: bool = False
    warmup_done: bool = False
    enable_policy: bool = False
    recover_requested: bool = False
    reasons: list[str] = field(default_factory=list)


class RedRhexStateMachine:
    def __init__(self, require_motor_feedback: bool = False, require_lowlevel_heartbeat: bool = False) -> None:
        self.state = RedRhexState.BOOT
        self.previous_state = RedRhexState.BOOT
        self.require_motor_feedback = bool(require_motor_feedback)
        self.require_lowlevel_heartbeat = bool(require_lowlevel_heartbeat)
        self.last_transition_reason = "node startup"

    def transition(self, new_state: RedRhexState, reason: str) -> None:
        if new_state != self.state:
            self.previous_state = self.state
            self.state = new_state
            self.last_transition_reason = reason

    def update(self, inputs: StateMachineInputs) -> RedRhexState:
        if inputs.estop or not inputs.safety_ok:
            if inputs.fall_detected:
                self.transition(RedRhexState.FALL_DETECTED, "fall detected")
            else:
                detail = "; ".join(inputs.reasons) if inputs.reasons else "safety violation"
                self.transition(RedRhexState.PROTECTIVE_STOP, detail)
            return self.state

        if self.state == RedRhexState.BOOT:
            if inputs.policy_loaded:
                self.transition(RedRhexState.SENSOR_CHECK, "policy loaded")
            return self.state

        if self.state == RedRhexState.SENSOR_CHECK:
            motor_ok = inputs.motor_feedback_ready or not self.require_motor_feedback
            bridge_ok = inputs.lowlevel_alive or not self.require_lowlevel_heartbeat
            if inputs.sensors_ready and motor_ok and bridge_ok:
                self.transition(RedRhexState.MOTOR_IDLE, "sensors and bridge ready")
            return self.state

        if self.state == RedRhexState.MOTOR_IDLE:
            self.transition(RedRhexState.INIT_STAND, "enter init stand")
            return self.state

        if self.state == RedRhexState.INIT_STAND:
            if inputs.init_stand_done:
                self.transition(RedRhexState.WARMUP, "init stand complete")
            return self.state

        if self.state == RedRhexState.WARMUP:
            if inputs.warmup_done:
                self.transition(RedRhexState.POLICY_READY, "warmup complete")
            return self.state

        if self.state == RedRhexState.POLICY_READY:
            if inputs.enable_policy:
                self.transition(RedRhexState.POLICY_RUN, "policy enabled")
            return self.state

        if self.state == RedRhexState.POLICY_RUN:
            if not inputs.enable_policy:
                self.transition(RedRhexState.POLICY_READY, "policy disabled")
            return self.state

        if self.state in (RedRhexState.PROTECTIVE_STOP, RedRhexState.FALL_DETECTED):
            if inputs.recover_requested and inputs.safety_ok and not inputs.estop:
                self.transition(RedRhexState.RECOVER, "manual recover requested")
            return self.state

        if self.state == RedRhexState.RECOVER:
            self.transition(RedRhexState.INIT_STAND, "recover requires init stand")
            return self.state

        return self.state
