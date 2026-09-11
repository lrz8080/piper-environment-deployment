#!/usr/bin/env python3
"""Control a Piper arm with the BETAFPV/TITA remote through ROS 2 Joy.

The mode layout follows DDTRobot/airbot_joy:

  right switch low    + trigger off -> end-effector XYZ
  right switch low    + trigger on  -> toggle gripper open/closed
  right switch middle + trigger off -> joints 1-3
  right switch middle + trigger on  -> joints 4-6
  right switch upper  + trigger off -> end-effector RPY
  right switch upper  + trigger on  -> zero pose (hold to confirm)

All internal Cartesian values use metres/radians. Conversion to Piper SDK
units is performed only when a command is sent.
"""

import argparse
import math
import os
import threading
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


class ControlMode(Enum):
    CARTESIAN_XYZ = "cartesian_xyz"
    GRIPPER = "gripper"
    JOINT_123 = "joint_123"
    JOINT_456 = "joint_456"
    CARTESIAN_RPY = "cartesian_rpy"
    ZERO = "zero"


@dataclass(frozen=True)
class JoyCommand:
    mode: ControlMode
    axes: Tuple[float, float, float, float]


@dataclass(frozen=True)
class JoySample:
    axes: Tuple[float, ...]
    buttons: Tuple[int, ...]
    received_at: float


class JoyMapper:
    """Translate raw BETAFPV Joy or airbot_joy output into one command."""

    def __init__(self, input_format: str = "raw", deadzone: float = 0.08):
        self.input_format = input_format
        self.deadzone = deadzone

    def _axis(self, value: float) -> float:
        value = max(-1.0, min(1.0, float(value)))
        magnitude = abs(value)
        if magnitude <= self.deadzone:
            return 0.0
        scaled = (magnitude - self.deadzone) / (1.0 - self.deadzone)
        return math.copysign(scaled, value)

    @staticmethod
    def _button(buttons: Sequence[int], index: int) -> bool:
        return index < len(buttons) and bool(buttons[index])

    def parse(self, sample: JoySample) -> Optional[JoyCommand]:
        if self.input_format == "raw":
            return self._parse_raw(sample.axes)
        return self._parse_airbot(sample.axes, sample.buttons)

    def _parse_raw(self, axes: Sequence[float]) -> Optional[JoyCommand]:
        if len(axes) < 8:
            return None

        # Exact axis permutation used by DDTRobot/airbot_joy.
        mapped = (axes[3], -axes[2], axes[0], -axes[1])
        mapped = tuple(self._axis(value) for value in mapped)

        switch = axes[7]
        trigger_on = axes[6] < -0.5
        if switch > 0.5:
            mode = ControlMode.ZERO if trigger_on else ControlMode.CARTESIAN_RPY
        elif switch < -0.5:
            mode = ControlMode.GRIPPER if trigger_on else ControlMode.CARTESIAN_XYZ
        else:
            mode = ControlMode.JOINT_456 if trigger_on else ControlMode.JOINT_123
        return JoyCommand(mode, mapped)

    def _parse_airbot(
        self, axes: Sequence[float], buttons: Sequence[int]
    ) -> Optional[JoyCommand]:
        if len(axes) < 4:
            return None
        mapped = tuple(self._axis(value) for value in axes[:4])

        # Decode the buttons emitted by airbot_joy_node.cpp.
        if self._button(buttons, 0):
            mode = ControlMode.ZERO
        elif self._button(buttons, 5):
            mode = ControlMode.GRIPPER
        elif self._button(buttons, 4) and self._button(buttons, 6):
            mode = ControlMode.JOINT_456
        elif self._button(buttons, 4):
            mode = ControlMode.JOINT_123
        elif self._button(buttons, 6):
            mode = ControlMode.CARTESIAN_RPY
        else:
            mode = ControlMode.CARTESIAN_XYZ
        return JoyCommand(mode, mapped)


class PiperRemoteController:
    JOINT_LIMITS = tuple(
        (math.radians(low), math.radians(high))
        for low, high in (
            (-150, 150),
            (0, 180),
            (-170, 0),
            (-100, 100),
            (-70, 70),
            (-120, 120),
        )
    )
    WORKSPACE_LIMITS = ((-0.10, 0.50), (-0.30, 0.30), (0.02, 0.50))
    ORIENTATION_LIMITS = tuple(
        (math.radians(low), math.radians(high))
        for low, high in ((-180, 180), (-100, 112), (-75, 75))
    )

    def __init__(
        self,
        can_channel: str,
        speed_percent: int,
        linear_speed: float,
        angular_speed: float,
        joint_speed: float,
        gripper_speed: float,
        reset_hold: float,
        enable_timeout: float,
        dry_run: bool = False,
    ):
        self.speed_percent = max(1, min(100, speed_percent))
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.joint_speed = joint_speed
        self.gripper_speed = gripper_speed
        self.reset_hold = reset_hold
        self.dry_run = dry_run
        self.piper = None

        self.joints = [0.0] * 6
        self.pose = [0.057, 0.0, 0.215, 0.0, math.radians(85.0), 0.0]
        self.gripper_mm = 0.0
        self.mode = None
        self.mode_started_at = 0.0
        self.reset_sent = False
        self.gripper_toggle_armed = True
        self.last_step_at = time.monotonic()
        self.last_dry_print = 0.0

        if not dry_run:
            try:
                from piper_sdk import C_PiperInterface_V2
            except ImportError as exc:
                raise RuntimeError(
                    "找不到 piper_sdk，请先执行: pip3 install piper_sdk"
                ) from exc

            self.piper = C_PiperInterface_V2(can_channel)
            self.piper.ConnectPort()
            deadline = time.monotonic() + enable_timeout
            while not self.piper.EnablePiper():
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"Piper 在 {enable_timeout:.1f}s 内未能使能")
                time.sleep(0.02)
            time.sleep(0.1)
            self.sync_feedback()

    @staticmethod
    def _clamp(value: float, limits: Tuple[float, float]) -> float:
        return max(limits[0], min(limits[1], value))

    def sync_feedback(self) -> None:
        if self.piper is None:
            return
        joint_msg = self.piper.GetArmJointMsgs().joint_state
        pose_msg = self.piper.GetArmEndPoseMsgs().end_pose
        gripper_msg = self.piper.GetArmGripperMsgs().gripper_state

        raw_joints = (
            joint_msg.joint_1,
            joint_msg.joint_2,
            joint_msg.joint_3,
            joint_msg.joint_4,
            joint_msg.joint_5,
            joint_msg.joint_6,
        )
        self.joints = [value * math.pi / 180000.0 for value in raw_joints]
        self.pose = [
            pose_msg.X_axis / 1_000_000.0,
            pose_msg.Y_axis / 1_000_000.0,
            pose_msg.Z_axis / 1_000_000.0,
            pose_msg.RX_axis * math.pi / 180000.0,
            pose_msg.RY_axis * math.pi / 180000.0,
            pose_msg.RZ_axis * math.pi / 180000.0,
        ]
        self.gripper_mm = gripper_msg.grippers_angle / 1000.0

    def _send_joints(self) -> None:
        values = [round(value * 180000.0 / math.pi) for value in self.joints]
        if self.piper is not None:
            self.piper.MotionCtrl_2(0x01, 0x01, self.speed_percent, 0x00)
            self.piper.JointCtrl(*values)
        self._dry_status("joint")

    def _send_pose(self) -> None:
        xyz = [round(value * 1_000_000.0) for value in self.pose[:3]]
        rpy = [round(value * 180000.0 / math.pi) for value in self.pose[3:]]
        if self.piper is not None:
            self.piper.MotionCtrl_2(0x01, 0x00, self.speed_percent, 0x00)
            self.piper.EndPoseCtrl(*(xyz + rpy))
        self._dry_status("pose")

    def _send_gripper(self) -> None:
        if self.piper is not None:
            self.piper.GripperCtrl(round(self.gripper_mm * 1000.0), 1000, 0x01, 0)
        self._dry_status("gripper")

    def _dry_status(self, command_type: str) -> None:
        if not self.dry_run or time.monotonic() - self.last_dry_print < 0.2:
            return
        self.last_dry_print = time.monotonic()
        joint_deg = [round(math.degrees(value), 1) for value in self.joints]
        pose = [round(value, 3) for value in self.pose]
        print(
            f"[DRY-RUN] {command_type}: joint={joint_deg}, "
            f"pose={pose}, gripper={self.gripper_mm:.1f}mm"
        )

    def step(self, command: JoyCommand, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        dt = max(0.0, min(now - self.last_step_at, 0.05))
        self.last_step_at = now

        if command.mode != self.mode:
            self.sync_feedback()
            self.mode = command.mode
            self.mode_started_at = now
            self.reset_sent = False
            print(f"[MODE] {command.mode.value}")
            if command.mode == ControlMode.GRIPPER:
                if self.gripper_toggle_armed:
                    self.gripper_toggle_armed = False
                    self.gripper_mm = 0.0 if self.gripper_mm > 35.0 else 70.0
                    self._send_gripper()
                    state = "打开" if self.gripper_mm > 35.0 else "闭合"
                    print(f"[GRIPPER] {state} -> {self.gripper_mm:.0f}mm")
                    return True
                return False
            self.gripper_toggle_armed = True
            return False

        axes = command.axes
        if command.mode == ControlMode.ZERO:
            if not self.reset_sent and now - self.mode_started_at >= self.reset_hold:
                self.joints = [0.0] * 6
                self._send_joints()
                self.reset_sent = True
                print("[RESET] 已发送零位指令")
                return True
            return False

        if command.mode == ControlMode.GRIPPER:
            delta = axes[3] * self.gripper_speed * dt
            if abs(delta) < 1e-9:
                return False
            self.gripper_mm = self._clamp(self.gripper_mm + delta, (0.0, 70.0))
            self._send_gripper()
            return True

        if command.mode in (ControlMode.JOINT_123, ControlMode.JOINT_456):
            if not any(axes[:3]):
                return False
            offset = 0 if command.mode == ControlMode.JOINT_123 else 3
            for axis_index in range(3):
                joint_index = offset + axis_index
                value = self.joints[joint_index] + axes[axis_index] * self.joint_speed * dt
                self.joints[joint_index] = self._clamp(
                    value, self.JOINT_LIMITS[joint_index]
                )
            self._send_joints()
            return True

        if command.mode == ControlMode.CARTESIAN_XYZ:
            if not any(axes[:3]):
                return False
            for index in range(3):
                value = self.pose[index] + axes[index] * self.linear_speed * dt
                self.pose[index] = self._clamp(value, self.WORKSPACE_LIMITS[index])
            self._send_pose()
            return True

        if command.mode == ControlMode.CARTESIAN_RPY:
            if not any(axes[:3]):
                return False
            for axis_index in range(3):
                pose_index = axis_index + 3
                value = self.pose[pose_index] + axes[axis_index] * self.angular_speed * dt
                self.pose[pose_index] = self._clamp(
                    value, self.ORIENTATION_LIMITS[axis_index]
                )
            self._send_pose()
            return True

        return False

    def pause(self) -> None:
        """Drop the active mode so control resumes from fresh feedback."""
        self.mode = None
        self.reset_sent = False
        self.last_step_at = time.monotonic()

    def disable(self) -> None:
        if self.piper is not None and hasattr(self.piper, "DisableArm"):
            self.piper.DisableArm(7)


def detect_robot_namespace(
    unique_id_file: str = "/proc/device-tree/serial-number",
) -> str:
    configured_namespace = os.environ.get("ROBOT_NS", "").strip().strip("/")
    if configured_namespace:
        return configured_namespace

    try:
        serial_number = Path(unique_id_file).read_bytes().replace(b"\0", b"")
        unique_id = serial_number.decode("utf-8")[6:].strip()
    except (OSError, UnicodeError):
        unique_id = ""
    return f"d1{unique_id}" if unique_id else "d1"


def default_joy_topic() -> str:
    return f"/{detect_robot_namespace()}/joy"


def default_sdk_mode_node() -> str:
    return f"/{detect_robot_namespace()}/teleop_command"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="BETAFPV/TITA 遥控器控制 Piper 机械臂"
    )
    parser.add_argument("--can", default="can1", help="Piper CAN 接口，默认 can1")
    parser.add_argument(
        "--joy-topic",
        default=default_joy_topic(),
        help=(
            "ROS 2 Joy topic；默认优先读取 ROBOT_NS，否则根据设备序列号生成"
        ),
    )
    parser.add_argument(
        "--input-format",
        choices=("raw", "airbot"),
        default="raw",
        help="raw=BETAFPV原始Joy；airbot=/airbot_play/joy 重映射消息",
    )
    parser.add_argument(
        "--sdk-mode-node",
        default=default_sdk_mode_node(),
        help="提供 use_sdk 参数的 ROS 2 节点",
    )
    parser.add_argument("--rate", type=float, default=50.0, help="控制频率 Hz")
    parser.add_argument("--deadzone", type=float, default=0.08, help="摇杆死区")
    parser.add_argument("--linear-speed", type=float, default=0.10, help="XYZ最大速度 m/s")
    parser.add_argument("--angular-speed", type=float, default=0.50, help="RPY最大速度 rad/s")
    parser.add_argument("--joint-speed", type=float, default=0.50, help="关节最大速度 rad/s")
    parser.add_argument("--gripper-speed", type=float, default=35.0, help="夹爪最大速度 mm/s")
    parser.add_argument("--arm-speed", type=int, default=30, help="Piper SDK速度百分比")
    parser.add_argument("--reset-hold", type=float, default=0.8, help="零位组合保持时间")
    parser.add_argument("--enable-timeout", type=float, default=8.0, help="使能超时秒数")
    parser.add_argument("--dry-run", action="store_true", help="不连接机械臂，只打印目标")
    parser.add_argument("--disable-on-exit", action="store_true", help="退出时失能机械臂")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    positive_values = (
        args.rate,
        args.linear_speed,
        args.angular_speed,
        args.joint_speed,
        args.gripper_speed,
        args.reset_hold,
        args.enable_timeout,
    )
    if any(value <= 0 for value in positive_values) or not 0 <= args.deadzone < 1:
        raise SystemExit("速度和超时参数必须为正数，deadzone 必须位于 [0, 1)")

    try:
        import rclpy
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters
        from sensor_msgs.msg import Joy
    except ImportError as exc:
        raise SystemExit(
            "找不到 ROS 2 Python 环境，请先 source /opt/ros/humble/setup.bash"
        ) from exc

    mapper = JoyMapper(args.input_format, args.deadzone)
    lock = threading.Lock()
    latest: List[Optional[JoySample]] = [None]

    def joy_callback(message: Joy) -> None:
        sample = JoySample(
            tuple(message.axes), tuple(message.buttons), time.monotonic()
        )
        with lock:
            latest[0] = sample

    rclpy.init()
    node = rclpy.create_node("betafpv_piper_teleop")
    # Keep the subscription alive for the full process lifetime.
    joy_subscription = node.create_subscription(
        Joy, args.joy_topic, joy_callback, 10
    )
    sdk_mode_service = f"{args.sdk_mode_node.rstrip('/')}/get_parameters"
    sdk_mode_client = node.create_client(GetParameters, sdk_mode_service)
    sdk_state = {
        "known": False,
        "enabled": False,
        "future": None,
        "next_query_at": 0.0,
    }

    def poll_sdk_mode() -> None:
        now = time.monotonic()
        future = sdk_state["future"]
        if future is not None and future.done():
            try:
                result = future.result()
                value = result.values[0]
                if value.type != ParameterType.PARAMETER_BOOL:
                    raise RuntimeError("use_sdk 不是 bool 参数")
                enabled = bool(value.bool_value)
                if not sdk_state["known"] or enabled != sdk_state["enabled"]:
                    state = "true，允许控制" if enabled else "false，禁止控制"
                    print(f"[SDK MODE] {state}")
                sdk_state["known"] = True
                sdk_state["enabled"] = enabled
            except Exception as exc:
                sdk_state["known"] = False
                sdk_state["enabled"] = False
                print(f"[SDK MODE] 读取失败: {exc}")
            sdk_state["future"] = None

        if (
            sdk_state["future"] is None
            and now >= sdk_state["next_query_at"]
            and sdk_mode_client.service_is_ready()
        ):
            request = GetParameters.Request()
            request.names = ["use_sdk"]
            sdk_state["future"] = sdk_mode_client.call_async(request)
            sdk_state["next_query_at"] = now + 0.2

    print(
        f"持续等待 {args.joy_topic} ({args.input_format}) 的有效 Joy 消息，且 "
        f"{args.sdk_mode_node}.use_sdk=true；Ctrl+C 退出..."
    )
    controller = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            poll_sdk_mode()
            with lock:
                sample = latest[0]
            if (
                sdk_state["known"]
                and sdk_state["enabled"]
                and sample is not None
                and mapper.parse(sample) is not None
            ):
                break
        if not rclpy.ok():
            return 0

        # Do not connect or enable the physical arm until Joy input is valid.
        controller = PiperRemoteController(
            can_channel=args.can,
            speed_percent=args.arm_speed,
            linear_speed=args.linear_speed,
            angular_speed=args.angular_speed,
            joint_speed=args.joint_speed,
            gripper_speed=args.gripper_speed,
            reset_hold=args.reset_hold,
            enable_timeout=args.enable_timeout,
            dry_run=args.dry_run,
        )
        print(
            f"监听 {args.joy_topic} ({args.input_format}), Piper={args.can}, "
            f"{'DRY-RUN' if args.dry_run else 'LIVE'}"
        )
        print("请先在遥控器菜单切换到 use-sdk mode；Ctrl+C 退出")

        period = 1.0 / args.rate
        # Ignore the sample used for startup validation. Commands are driven
        # only by Joy messages received after the arm has been enabled.
        last_processed_at = sample.received_at
        sdk_state["known"] = False
        sdk_state["future"] = None
        sdk_state["next_query_at"] = 0.0
        was_sdk_enabled = False
        while rclpy.ok():
            started = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            poll_sdk_mode()
            with lock:
                sample = latest[0]
            sdk_enabled = sdk_state["known"] and sdk_state["enabled"]
            if not sdk_enabled:
                if was_sdk_enabled:
                    controller.pause()
                    print("[SDK MODE] 控制已暂停")
                was_sdk_enabled = False
                if sample is not None:
                    last_processed_at = sample.received_at
            elif not was_sdk_enabled:
                controller.pause()
                was_sdk_enabled = True
                if sample is not None:
                    last_processed_at = sample.received_at
                print("[SDK MODE] 等待下一条 Joy 消息后恢复控制")
            elif sample is not None and sample.received_at > last_processed_at:
                last_processed_at = sample.received_at
                command = mapper.parse(sample)
                if command is not None:
                    controller.step(command, time.monotonic())
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print("\n退出遥控器控制")
    finally:
        del joy_subscription
        node.destroy_client(sdk_mode_client)
        if controller is not None and args.disable_on_exit:
            controller.disable()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
