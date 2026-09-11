"""
Pico 4 VR 手柄 MQTT 传感器
通过 ao2car.ddt.dev/vr/ 平台的 MQTT 协议读取 Pico 4 手柄数据
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import json
import numpy as np
import threading
import time
from scipy.spatial.transform import Rotation as R

import paho.mqtt.client as mqtt

from sensor.teleoperation_sensor import TeleoperationSensor


MQTT_BROKER = "120.79.156.21"
MQTT_PORT = 8083
MQTT_PATH = "/mqtt"
MQTT_USERNAME = "arm_server"
MQTT_PASSWORD = "serverarm"


class PicoMqttSensor(TeleoperationSensor):
    """通过 MQTT 读取 Pico 4 VR 手柄位姿"""

    def __init__(self, name="pico_mqtt", control_side="right"):
        """
        Args:
            name: 传感器名称
            control_side: 用哪个手柄控制 Piper — "left" 或 "right"
        """
        super().__init__()
        self.name = name
        self.control_side = control_side

        # 当前手柄数据
        self._latest_controller = None  # arm0001/controller
        self._latest_buttons = None     # arm0001/controller_data
        self._lock = threading.Lock()

        # 上一位姿（用于计算 delta）
        self._prev_position = None
        self._prev_orientation = None

        # 夹爪状态
        self._gripper_value = 0.0

        self._client = None
        self._running = False

    # ─── MQTT 回调 ────────────────────────────────────
    def _on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            print(f"[PicoMqtt] MQTT 连接成功")
            client.subscribe("arm0001/controller", qos=0)
            client.subscribe("arm0001/controller_data", qos=0)
        else:
            print(f"[PicoMqtt] MQTT 连接失败, rc={rc}")

    def _on_message(self, client, userdata, msg):
        if msg.topic == "arm0001/controller":
            try:
                data = json.loads(msg.payload.decode())
                with self._lock:
                    self._latest_controller = data
            except Exception as e:
                print(f"[PicoMqtt] controller 解析失败: {e}")

        elif msg.topic == "arm0001/controller_data":
            try:
                data = json.loads(msg.payload.decode())
                with self._lock:
                    self._latest_buttons = data
            except Exception as e:
                print(f"[PicoMqtt] controller_data 解析失败: {e}")

    # ─── 生命周期 ─────────────────────────────────────
    def set_up(self):
        self._client = mqtt.Client(
            client_id=f"piper_teleop_{int(time.time())}",
            protocol=mqtt.MQTTv311,
            transport="websockets",
        )
        self._client.ws_set_options(path=MQTT_PATH)
        self._client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message

        self._client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        self._client.loop_start()
        self._running = True
        print(f"[PicoMqtt] 已连接到 {MQTT_BROKER}:{MQTT_PORT}{MQTT_PATH}")
        print(f"[PicoMqtt] 请在 Pico 4 浏览器打开 https://ao2car.ddt.dev/vr/ 并完成标定")

    def cleanup(self):
        self._running = False
        if self._client:
            self._client.loop_stop()
            self._client.disconnect()
        print("[PicoMqtt] 已断开 MQTT")

    # ─── 数据解析 ─────────────────────────────────────
    def _parse_controller(self):
        """
        解析 arm0001/controller → 返回 (local_position_delta, global_orientation_euler, gripper, grip_pressed)

        controller 格式:
        [x, y, z, w, dx, dy, dz, isMove, clamp,   ← 左臂 (0-8)
         x, y, z, w, dx, dy, dz, isMove, clamp]   ← 右臂 (9-17)
        """
        with self._lock:
            data = self._latest_controller

        if data is None or len(data) < 18:
            return None

        if self.control_side == "right":
            offset = 9
        else:
            offset = 0

        x  = data[offset + 0]
        y  = data[offset + 1]
        z  = data[offset + 2]
        w  = data[offset + 3]
        dx = data[offset + 4]
        dy = data[offset + 5]
        dz = data[offset + 6]
        is_move = data[offset + 7]
        clamp_val = data[offset + 8]

        position = np.array([x, y, z], dtype=np.float64)
        # 四元数格式: w, dx, dy, dz
        quat = np.array([dx, dy, dz, w], dtype=np.float64)  # xyzw

        return position, quat, clamp_val, bool(is_move)

    def _parse_buttons(self):
        """解析 arm0001/controller_data → 返回扳机/握把状态"""
        with self._lock:
            data = self._latest_buttons
        return data or {}

    # ─── 计算位姿 delta ──────────────────────────────
    def _compute_delta(self, position, quat):
        """计算相对于上一帧的位姿变化 (position delta 和 euler delta)"""
        if self._prev_position is None:
            self._prev_position = position
            self._prev_orientation = quat
            return np.zeros(3), np.zeros(3)

        pos_delta = position - self._prev_position

        # 计算旋转变化
        prev_rot = R.from_quat(self._prev_orientation)
        curr_rot = R.from_quat(quat)
        delta_rot = curr_rot * prev_rot.inv()
        euler_delta = delta_rot.as_euler('xyz', degrees=False)

        self._prev_position = position
        self._prev_orientation = quat

        return pos_delta, euler_delta

    # ─── 外部接口 ─────────────────────────────────────
    def get_state(self):
        parsed = self._parse_controller()
        if parsed is None:
            return {"end_pose": np.zeros(12), "gripper": 0.0, "grip_pressed": False}

        position, quat, clamp_val, is_move = parsed
        pos_delta, euler_delta = self._compute_delta(position, quat)

        try:
            euler = R.from_quat(quat).as_euler('xyz', degrees=False)
        except Exception:
            euler = np.zeros(3)

        end_pose = np.concatenate([pos_delta, euler_delta, position, euler])
        gripper = float(np.clip(clamp_val, 0.0, 1.0))

        # 优先用 controller 里的 isMove，其次用 controller_data 里的 grip
        grip_pressed = bool(is_move)
        if not grip_pressed:
            buttons = self._parse_buttons()
            if buttons:
                if self.control_side == "right":
                    grip = buttons.get("right_grip", 0.0)
                else:
                    grip = buttons.get("left_grip", 0.0)
                grip_pressed = grip > 0.5

        return {
            "end_pose": end_pose,
            "gripper": gripper,
            "grip_pressed": grip_pressed,
        }

    def reset(self, *args, **kwargs):
        self._prev_position = None
        self._prev_orientation = None
        print(f"[PicoMqtt] 位姿 delta 已重置")


# ─── 简单测试 ────────────────────────────────────────
if __name__ == "__main__":
    sensor = PicoMqttSensor(name="test_pico", control_side="right")
    sensor.set_up()
    sensor.set_collect_info(["end_pose", "gripper"])

    print("等待 VR 数据... (Pico 4 浏览器打开 https://ao2car.ddt.dev/vr/ 并完成标定)")
    print("按 Ctrl+C 退出")

    try:
        while True:
            state = sensor.get_state()
            if state["grip_pressed"]:
                delta_pos = state["end_pose"][:3]
                delta_eul = state["end_pose"][3:6]
                gripper = state["gripper"]
                print(f"\r 位移: [{delta_pos[0]:.4f}, {delta_pos[1]:.4f}, {delta_pos[2]:.4f}]  "
                      f"旋转: [{delta_eul[0]:.3f}, {delta_eul[1]:.3f}, {delta_eul[2]:.3f}]  "
                      f"夹爪: {gripper:.2f}  [ACTIVE]  ",
                      end="")
            else:
                print(f"\r 等待握把按下... 夹爪: {state['gripper']:.2f}  [IDLE]  ", end="")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n退出")
        sensor.cleanup()
