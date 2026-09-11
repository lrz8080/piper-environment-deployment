#!/usr/bin/env python3
import sys
sys.path.append("./")
import os
import time
import json
import math
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R
import paho.mqtt.client as mqtt
from typing import Tuple, List, Optional
from my_robot.agilex_piper_single_base import PiperSingle, condition
from data.collect_any import CollectAny
from utils.data_handler import is_enter_pressed
MQTT_BROKER = "120.79.156.21"
MQTT_PORT = 8083
MQTT_PATH = "/mqtt"
MQTT_USERNAME = "arm_server"
MQTT_PASSWORD = "serverarm"
VR_TO_ROBOT_POS = np.array([
    [ 0, 1, 0],
    [-1, 0, 0],
    [ 0, 0, 1],
])
VR_TO_ROBOT_ROT = np.array([
    [ 1, 0, 0],
    [ 0, -1, 0],
    [ 0, 0, -1],
])
POSITION_SCALE = 1.2
ROTATION_SCALE = 1.2
MAX_LIN_VEL = 4.0
MAX_ANG_VEL = 8.0
FILTER_TAU_POS = 0.02
FILTER_TAU_ROT = 0.03
DEAD_ZONE_POS = 0.002
DEAD_ZONE_ROT = 0.02
DEFAULT_FREQ = 100
WORKSPACE_LIMITS = np.array([
    [-0.2,  0.60],
    [-0.40,  0.40],
    [ 0.05,  0.50],
])
ARM_ROT_UNIT = 1000.0 * math.pi / 180.0
RAD_TO_QPOS = 180.0 / (1000.0 * math.pi)
class PicoPiperIKTeleop:
    def __init__(self, freq: int = DEFAULT_FREQ):
        self.freq = freq
        self.dt = 1.0 / freq
        self.robot = PiperSingle()
        self.robot.set_up()
        self.robot.reset_position()
        time.sleep(2)
        self._lock = threading.Lock()
        self._latest_ctrl = None
        self._latest_btns = None
        self._client = mqtt.Client(transport="websockets")
        self._client.ws_set_options(path=MQTT_PATH)
        self._client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        self._client.loop_start()
        print(f"[MQTT] 已连接 {MQTT_BROKER}:{MQTT_PORT}{MQTT_PATH}")
        self._ref_vr_pos: Optional[np.ndarray] = None
        self._ref_vr_rot: Optional[np.ndarray] = None
        self._ref_arm_qpos: Optional[np.ndarray] = None
        self._dirty = True
        self._filter_alpha_pos = self.dt / (FILTER_TAU_POS + self.dt)
        self._filter_alpha_rot = self.dt / (FILTER_TAU_ROT + self.dt)
        self._filtered_pos: Optional[np.ndarray] = None
        self._last_sent_pos: Optional[np.ndarray] = None
        self._filtered_rpy: Optional[np.ndarray] = None
        self._last_sent_rpy: Optional[np.ndarray] = None
        self._max_step_pos = MAX_LIN_VEL * self.dt
        self._max_step_rot = MAX_ANG_VEL * self.dt
        print("\n" + "=" * 60)
        print("Pico 4 VR 遥操作 Piper 已就绪 (笛卡尔映射 + 内置 IK)")
        print(f"  映射: target_pose = arm_qpos + VR_TO_ROBOT @ vr_delta × scale")
        print(f"  VR→Robot Pos:\n{np.array2string(VR_TO_ROBOT_POS, prefix='    ')}")
        print(f"  VR→Robot Rot:\n{np.array2string(VR_TO_ROBOT_ROT, prefix='    ')}")
        print(f"  位置缩放: {POSITION_SCALE}  姿态缩放: {ROTATION_SCALE}")
        print(f"  频率: {self.freq}Hz  死区: {DEAD_ZONE_POS*1000:.0f}mm/{DEAD_ZONE_ROT*1000:.0f}mrad")
        print(f"  限速: {MAX_LIN_VEL}m/s, {MAX_ANG_VEL}rad/s")
        print(f"  握把按下→捕获参考→移动手柄→机械臂跟随")
        print("=" * 60 + "\n")
    def _on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            client.subscribe("arm0001/controller", qos=0)
            client.subscribe("arm0001/controller_data", qos=0)
            print("[MQTT] 订阅成功")
        else:
            print(f"[MQTT] 连接失败 rc={rc}")
    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            with self._lock:
                if msg.topic == "arm0001/controller":
                    self._latest_ctrl = data
                elif msg.topic == "arm0001/controller_data":
                    self._latest_btns = data
        except Exception:
            pass
    def _read_vr(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], bool, float]:
        with self._lock:
            ctrl = self._latest_ctrl
            btns = self._latest_btns
        if ctrl is None or len(ctrl) < 18:
            return None, None, False, 0.0
        pos = np.array(ctrl[9:12], dtype=np.float64)
        rot = np.array(ctrl[12:15], dtype=np.float64)
        raw_is_move = ctrl[16]
        clamp_val = float(ctrl[17])
        grip = bool(raw_is_move)
        if not grip and btns is not None:
            grip = btns.get("right_grip", 0.0) > 0.5
        return pos, rot, grip, clamp_val
    def teleop_step(self) -> bool:
        vr_pos, vr_rot, grip, clamp = self._read_vr()
        if vr_pos is None or not grip:
            if self._dirty is False and grip is False and self._ref_vr_pos is not None:
                print("  [GRIP] 握把松开, 机械臂保持")
            self._dirty = True
            self._filtered_pos = None
            self._filtered_rpy = None
            self._last_sent_pos = None
            self._last_sent_rpy = None
            return False
        if self._dirty or self._ref_vr_pos is None:
            self._ref_vr_pos = vr_pos.copy()
            self._ref_vr_rot = vr_rot.copy()
            data = self.robot.get()
            qpos = np.array(data[0]["left_arm"]["qpos"], dtype=np.float64)
            self._ref_arm_qpos = qpos.copy()
            self._filtered_pos = qpos[:3].copy()
            self._last_sent_pos = qpos[:3].copy()
            self._filtered_rpy = qpos[3:].copy()
            self._last_sent_rpy = qpos[3:].copy()
            self._dirty = False
            joints = np.array(data[0]["left_arm"]["joint"], dtype=np.float64)
            qpos_rot_rad = qpos[3:] * ARM_ROT_UNIT
            print(f"\n  [REF] 参考帧捕获 (握把按下):")
            print(f"    VR:     pos={np.round(vr_pos, 3)}m, rot={np.round(vr_rot, 2)}rad")
            print(f"    机械臂: pos={np.round(qpos[:3], 3)}m, rpy={np.round(qpos_rot_rad, 2)}rad")
            print(f"    关节角: {np.round(joints, 2)}rad")
            print(f"    机械臂不动, 移动手柄开始跟随...\n")
            return False
        vr_delta_pos = VR_TO_ROBOT_POS @ (vr_pos - self._ref_vr_pos)
        vr_delta_pos[np.abs(vr_delta_pos) < DEAD_ZONE_POS] = 0.0
        raw_target_pos = self._ref_arm_qpos[:3] + vr_delta_pos * POSITION_SCALE
        self._filtered_pos = (self._filter_alpha_pos * raw_target_pos +
            (1.0 - self._filter_alpha_pos) * self._filtered_pos)
        pos_delta = self._filtered_pos - self._last_sent_pos
        pos_delta_norm = float(np.linalg.norm(pos_delta))
        if pos_delta_norm > self._max_step_pos:
            pos_delta *= self._max_step_pos / pos_delta_norm
        target_pos = self._last_sent_pos + pos_delta
        target_pos = np.clip(target_pos,
                             WORKSPACE_LIMITS[:, 0],
                             WORKSPACE_LIMITS[:, 1])
        R_ref = R.from_euler('xyz', self._ref_vr_rot).as_matrix()
        R_curr = R.from_euler('xyz', vr_rot).as_matrix()
        R_delta = R_curr @ R_ref.T
        trace = float(np.trace(R_delta))
        cos_angle = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
        angle = math.acos(cos_angle)
        if abs(angle) < 1e-8:
            omega = np.zeros(3)
        else:
            axis = np.array([
                R_delta[2, 1] - R_delta[1, 2],
                R_delta[0, 2] - R_delta[2, 0],
                R_delta[1, 0] - R_delta[0, 1],
            ]) / (2.0 * math.sin(angle))
            omega = axis * angle
        omega_mapped = VR_TO_ROBOT_ROT @ omega * ROTATION_SCALE
        omega_mapped[np.abs(omega_mapped) < DEAD_ZONE_ROT] = 0.0
        rpy_delta_qpos = omega_mapped * RAD_TO_QPOS
        raw_target_rpy = self._ref_arm_qpos[3:] + rpy_delta_qpos
        self._filtered_rpy = (self._filter_alpha_rot * raw_target_rpy +
            (1.0 - self._filter_alpha_rot) * self._filtered_rpy)
        rot_delta_qpos = self._filtered_rpy - self._last_sent_rpy
        rot_delta_rad = rot_delta_qpos * ARM_ROT_UNIT
        rot_norm = float(np.linalg.norm(rot_delta_rad))
        if rot_norm > self._max_step_rot:
            rot_delta_rad *= self._max_step_rot / rot_norm
            rot_delta_qpos = rot_delta_rad / ARM_ROT_UNIT
        target_rpy = self._last_sent_rpy + rot_delta_qpos
        target_qpos = np.concatenate([target_pos, target_rpy])
        pos_changed = np.linalg.norm(pos_delta) > 0.0002
        rot_changed = np.linalg.norm(rot_delta_rad) > 0.0003
        if not pos_changed and not rot_changed:
            return False
        if not hasattr(self, '_dbg_cnt'):
            self._dbg_cnt = 0
        self._dbg_cnt += 1
        if self._dbg_cnt % 50 == 0:
            vr_d = VR_TO_ROBOT_POS @ (vr_pos - self._ref_vr_pos)
            print(f"  [DBG] vr_delta_pos={np.round(vr_d, 3)}m, "
                  f"omega={np.round(omega_mapped, 3)}rad, "
                  f"target_pos={np.round(target_pos, 3)}m, "
                  f"target_rpy_qpos={np.round(target_rpy, 4)}")
        try:
            self.robot.move_modeP(target_qpos, clamp)
            self._last_sent_pos = target_pos.copy()
            self._last_sent_rpy = target_rpy.copy()
            return True
        except Exception as e:
            print(f"\n[ERROR] move_modeP 失败: {e}")
            return False
    def reset_ref(self):
        self._ref_vr_pos = None
        self._ref_vr_rot = None
        self._ref_arm_qpos = None
        self._filtered_pos = None
        self._filtered_rpy = None
        self._last_sent_pos = None
        self._last_sent_rpy = None
        self._dirty = True
    def cleanup(self):
        self._client.loop_stop()
        self._client.disconnect()
        self.robot.reset()
        print("[MQTT] 已断开")
def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Pico VR → Piper 笛卡尔遥操作 (内置IK)")
    parser.add_argument("--episodes", type=int, default=50,
                        help="采集轨迹数")
    parser.add_argument("--freq", type=int, default=DEFAULT_FREQ,
                        help=f"控制频率 Hz (默认 {DEFAULT_FREQ})")
    args = parser.parse_args()
    os.environ["INFO_LEVEL"] = "INFO"
    condition["save_freq"] = args.freq
    teleop = PicoPiperIKTeleop(freq=args.freq)
    for episode_id in range(args.episodes):
        print(f"\n{'─'*40}")
        print(f"第 {episode_id + 1}/{args.episodes} 条轨迹")
        print("按 Enter 开始采集...")
        print(f"{'─'*40}")
        while True:
            if is_enter_pressed():
                break
            time.sleep(0.1)
        teleop.reset_ref()
        collection = CollectAny(condition=condition, start_episode=episode_id,
                                move_check=False, resume=True)
        print("开始! 握把按下→移动/旋转手柄, 松开保持, 按 Enter 结束\n")
        step = 0
        moved_count = 0
        try:
            while True:
                moved = teleop.teleop_step()
                if moved:
                    moved_count += 1
                data = teleop.robot.get()
                collection.collect(data[0], data[1])
                step += 1
                time.sleep(1.0 / args.freq)
                if is_enter_pressed():
                    break
        except KeyboardInterrupt:
            print("\n中断")
            break
        if len(collection.episode) > 0:
            collection.write()
        print(f"  轨迹 {episode_id + 1} 已保存 ({step} 帧, {moved_count} 帧控制)")
        if moved_count == 0:
            print("  [WARN] 没有移动帧! 确认:")
            print("    1. Pico 4 进入3D场景并完成标定")
            print("    2. 按下握把(grip)并移动手柄")
    teleop.cleanup()
    print("全部完成!")
if __name__ == "__main__":
    main()
