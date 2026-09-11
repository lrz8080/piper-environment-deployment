import importlib.util
import math
import pathlib
import unittest
from unittest import mock


SCRIPT = (
    pathlib.Path(__file__).parents[1]
    / "example"
    / "teleop"
    / "betafpv_piper_teleop.py"
)
SPEC = importlib.util.spec_from_file_location("betafpv_piper_teleop", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def raw_sample(switch, trigger_on=False, sticks=(0.0, 0.0, 0.0, 0.0)):
    # Inverse of airbot_joy: mapped=(raw[3], -raw[2], raw[0], -raw[1]).
    axes = [sticks[2], -sticks[3], -sticks[1], sticks[0], 0.0, 0.0,
            -1.0 if trigger_on else 1.0, switch]
    return MODULE.JoySample(tuple(axes), (), 0.0)


class ParserTest(unittest.TestCase):
    def test_default_joy_topic_uses_robot_namespace(self):
        with mock.patch.dict("os.environ", {"ROBOT_NS": "d15047105"}):
            self.assertEqual(MODULE.build_parser().parse_args([]).joy_topic,
                             "/d15047105/joy")

    def test_default_joy_topic_uses_device_serial_number(self):
        with mock.patch.dict("os.environ", {}, clear=True), mock.patch.object(
            MODULE.Path,
            "read_bytes",
            return_value=b"ABCDEF5047105\0",
        ):
            self.assertEqual(MODULE.build_parser().parse_args([]).joy_topic,
                             "/d15047105/joy")

    def test_default_joy_topic_falls_back_to_d1(self):
        with mock.patch.dict("os.environ", {}, clear=True), mock.patch.object(
            MODULE.Path,
            "read_bytes",
            side_effect=OSError,
        ):
            self.assertEqual(MODULE.build_parser().parse_args([]).joy_topic,
                             "/d1/joy")

    def test_default_sdk_mode_node_uses_robot_namespace(self):
        with mock.patch.dict("os.environ", {"ROBOT_NS": "d15047105"}):
            self.assertEqual(
                MODULE.build_parser().parse_args([]).sdk_mode_node,
                "/d15047105/teleop_command",
            )


class JoyMapperTest(unittest.TestCase):
    def test_raw_modes_match_airbot_joy_readme(self):
        mapper = MODULE.JoyMapper("raw", deadzone=0.0)
        cases = (
            (-1.0, False, MODULE.ControlMode.CARTESIAN_XYZ),
            (-1.0, True, MODULE.ControlMode.GRIPPER),
            (0.0, False, MODULE.ControlMode.JOINT_123),
            (0.0, True, MODULE.ControlMode.JOINT_456),
            (1.0, False, MODULE.ControlMode.CARTESIAN_RPY),
            (1.0, True, MODULE.ControlMode.ZERO),
        )
        for switch, trigger, expected in cases:
            with self.subTest(expected=expected):
                self.assertEqual(
                    mapper.parse(raw_sample(switch, trigger)).mode, expected
                )

    def test_raw_axis_permutation(self):
        mapper = MODULE.JoyMapper("raw", deadzone=0.0)
        command = mapper.parse(raw_sample(0.0, sticks=(0.1, -0.2, 0.3, -0.4)))
        for actual, expected in zip(command.axes, (0.1, -0.2, 0.3, -0.4)):
            self.assertAlmostEqual(actual, expected)

    def test_airbot_encoded_modes(self):
        mapper = MODULE.JoyMapper("airbot", deadzone=0.0)
        cases = (
            ([0] * 12, MODULE.ControlMode.CARTESIAN_XYZ),
            ([0, 0, 0, 0, 0, 1, 0], MODULE.ControlMode.GRIPPER),
            ([0, 0, 0, 0, 1, 0, 0], MODULE.ControlMode.JOINT_123),
            ([0, 0, 0, 0, 1, 0, 1], MODULE.ControlMode.JOINT_456),
            ([0, 0, 0, 0, 0, 0, 1], MODULE.ControlMode.CARTESIAN_RPY),
            ([1], MODULE.ControlMode.ZERO),
        )
        for buttons, expected in cases:
            sample = MODULE.JoySample((0.0,) * 4, tuple(buttons), 0.0)
            with self.subTest(expected=expected):
                self.assertEqual(mapper.parse(sample).mode, expected)

    def test_deadzone_is_rescaled(self):
        mapper = MODULE.JoyMapper("raw", deadzone=0.1)
        self.assertEqual(mapper.parse(raw_sample(0.0, sticks=(0.05, 0, 0, 0))).axes[0], 0)
        self.assertAlmostEqual(
            mapper.parse(raw_sample(0.0, sticks=(0.55, 0, 0, 0))).axes[0], 0.5
        )


class ControllerTest(unittest.TestCase):
    def controller(self):
        controller = MODULE.PiperRemoteController(
            can_channel="can1",
            speed_percent=30,
            linear_speed=0.1,
            angular_speed=0.5,
            joint_speed=0.5,
            gripper_speed=35.0,
            reset_hold=0.8,
            enable_timeout=1.0,
            dry_run=True,
        )
        controller.last_step_at = 1.0
        return controller

    def test_joint_limits_match_piper_sdk_defaults(self):
        expected_degrees = (
            (-150.0, 150.0),
            (0.0, 180.0),
            (-170.0, 0.0),
            (-100.0, 100.0),
            (-70.0, 70.0),
            (-120.0, 120.0),
        )
        actual_degrees = tuple(
            tuple(math.degrees(value) for value in limits)
            for limits in MODULE.PiperRemoteController.JOINT_LIMITS
        )
        for actual, expected in zip(actual_degrees, expected_degrees):
            self.assertAlmostEqual(actual[0], expected[0])
            self.assertAlmostEqual(actual[1], expected[1])

    def test_joint_velocity_and_mode_transition(self):
        controller = self.controller()
        command = MODULE.JoyCommand(
            MODULE.ControlMode.JOINT_123, (1.0, 0.0, 0.0, 0.0)
        )
        self.assertFalse(controller.step(command, now=1.0))
        self.assertTrue(controller.step(command, now=1.02))
        self.assertAlmostEqual(controller.joints[0], 0.01)

    def test_cartesian_xyz_updates_position(self):
        controller = self.controller()
        controller.pose = [0.1, 0.0, 0.2, 0.0, 0.0, math.pi / 2]
        command = MODULE.JoyCommand(
            MODULE.ControlMode.CARTESIAN_XYZ, (1.0, 0.0, 0.0, 0.0)
        )
        controller.step(command, now=1.0)
        controller.step(command, now=1.02)
        self.assertAlmostEqual(controller.pose[0], 0.102)
        self.assertAlmostEqual(controller.pose[1], 0.0)

    def test_cartesian_rpy_updates_orientation(self):
        controller = self.controller()
        command = MODULE.JoyCommand(
            MODULE.ControlMode.CARTESIAN_RPY, (1.0, -0.5, 0.25, 0.0)
        )
        controller.step(command, now=1.0)
        controller.step(command, now=1.02)
        self.assertAlmostEqual(controller.pose[3], 0.01)
        self.assertAlmostEqual(controller.pose[4], math.radians(85.0) - 0.005)
        self.assertAlmostEqual(controller.pose[5], 0.0025)

    def test_zero_requires_hold(self):
        controller = self.controller()
        controller.joints = [0.1] * 6
        command = MODULE.JoyCommand(MODULE.ControlMode.ZERO, (0.0,) * 4)
        controller.step(command, now=1.0)
        self.assertFalse(controller.step(command, now=1.79))
        self.assertEqual(controller.joints, [0.1] * 6)
        self.assertTrue(controller.step(command, now=1.8))
        self.assertEqual(controller.joints, [0.0] * 6)

    def test_gripper_trigger_toggles_once_per_mode_entry(self):
        controller = self.controller()
        gripper = MODULE.JoyCommand(MODULE.ControlMode.GRIPPER, (0.0,) * 4)
        xyz = MODULE.JoyCommand(MODULE.ControlMode.CARTESIAN_XYZ, (0.0,) * 4)

        self.assertTrue(controller.step(gripper, now=1.0))
        self.assertEqual(controller.gripper_mm, 70.0)
        self.assertFalse(controller.step(gripper, now=1.02))
        self.assertEqual(controller.gripper_mm, 70.0)

        controller.step(xyz, now=1.04)
        self.assertTrue(controller.step(gripper, now=1.06))
        self.assertEqual(controller.gripper_mm, 0.0)

    def test_pause_requires_a_fresh_mode_transition(self):
        controller = self.controller()
        command = MODULE.JoyCommand(
            MODULE.ControlMode.JOINT_123, (1.0, 0.0, 0.0, 0.0)
        )
        controller.step(command, now=1.0)
        controller.step(command, now=1.02)
        controller.pause()
        self.assertIsNone(controller.mode)
        self.assertFalse(controller.step(command, now=1.04))

if __name__ == "__main__":
    unittest.main()
