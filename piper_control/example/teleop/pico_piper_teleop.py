import sys
sys.path.append("./")
import os
import time
import json
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R
import paho.mqtt.client as mqtt
from my_robot.agilex_piper_single_base import PiperSingle, condition
from data.collect_any import CollectAny
from utils.data_handler import is_enter_pressed
MQTT_BROKER = "120.79.156.21"
MQTT_PORT = 8083
MQTT_PATH = "/mqtt"
MQTT_USERNAME = "arm_server"
MQTT_PASSWORD = "serverarm"
POSITION_SCALE = 1.0
ROTATION_SCALE = 1.0
MAX_POSITION_VELOCITY = 0.5
MAX_ANGULAR_VELOCITY = 1.0
FILTER_TAU = 0.08
FILTER_TAU_ROT = 0.12
DEAD_ZONE_POS = 0.001
DEAD_ZONE_ROT = 0.015
DEFAULT_FREQ = 100
class PicoPiperTeleop:
    def __init__(self, freq: int = DEFAULT_FREQ):
        self.freq = freq
        self.dt = 1.0 / freq
        self.max_step_delta_pos = MAX_POSITION_VELOCITY * self.dt
        self.filter_alpha_pos = self.dt / (FILTER_TAU + self.dt)
        self.dead_zone_pos = DEAD_ZONE_POS
        self.max_step_delta_rot = MAX_ANGULAR_VELOCITY * self.dt
        self.filter_alpha_rot = self.dt / (FILTER_TAU_ROT + self.dt)
        self.dead_zone_rot = DEAD_ZONE_ROT
        self.robot = PiperSingle()
        self.robot.set_up()
        self.robot.reset()
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
        self._ref_vr_pos = None
        self._ref_vr_rot = None
        self._ref_arm_pos = None
        self._ref_arm_rpy = None
        self._grip_was_pressed = False
        self._dirty = True
        self._filtered_target_pos = None
        self._last_sent_target_pos = None
        self._filtered_target_rpy = None
        self._last_sent_target_rpy = None
        print("\n" + "=" * 60)
        print("Pico 4 VR 6-DOF 遥操作 Piper 已就绪 (绝对位姿映射)")
        print(f"  位置: target_pos = ref_arm_pos + (vr_pos - ref_vr) × {POSITION_SCALE}")
        print(f"  姿态: target_rpy = ref_arm_rpy + (vr_rot - ref_vr) × {ROTATION_SCALE}")
        print(f"  控制频率:  {self.freq} Hz  (每帧 {self.dt*1000:.1f} ms)")
        print(f"  位置 EMA τ={FILTER_TAU:.2f}s 死区={self.dead_zone_pos*1000:.0f}mm 限速={MAX_POSITION_VELOCITY}m/s")
        print(f"  姿态 EMA τ={FILTER_TAU_ROT:.2f}s 死区={self.dead_zone_rot*1000:.0f}mrad 限速={MAX_ANGULAR_VELOCITY}rad/s")
        print(f"  握把按下 → 捕获参考帧 → 机械臂 6-DOF 跟随")
        print(f"  握把松开 → 机械臂保持不动")
        print(f"  重新握把 → 重新捕获参考帧 (当前位姿 = 新原点)")
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
    def _read_vr(self):
        with self._lock:
            ctrl = self._latest_ctrl
            btns = self._latest_btns
        if ctrl is None or len(ctrl) < 18:
            return None, None, False, 0.0
        x, y, z = ctrl[9], ctrl[10], ctrl[11]
        rx, ry, rz = ctrl[12], ctrl[13], ctrl[14]
        is_move = bool(ctrl[16])
        clamp_val = float(ctrl[17])
        grip_pressed = is_move
        if not grip_pressed and btns:
            grip_pressed = btns.get("right_grip", 0.0) > 0.5
        return (np.array([x, y, z], dtype=np.float64),
                np.array([rx, ry, rz], dtype=np.float64),
                grip_pressed, clamp_val)
    def teleop_step(self) -> bool:
        pos, rot, grip, clamp = self._read_vr()
        if pos is None or not grip:
            self._dirty = True
            self._grip_was_pressed = False
            self._filtered_target_pos = None
            self._last_sent_target_pos = None
            self._filtered_target_rpy = None
            self._last_sent_target_rpy = None
            return False
        if self._dirty or self._ref_vr_pos is None:
            self._ref_vr_pos = pos.copy()
            self._ref_vr_rot = rot.copy()
            data = self.robot.get()
            current_qpos = np.array(data[0]["left_arm"]["qpos"], dtype=np.float64)
            self._ref_arm_pos = current_qpos[:3].copy()
            self._ref_arm_rpy = current_qpos[3:].copy()
            self._dirty = False
            self._grip_was_pressed = True
            self._filtered_target_pos = self._ref_arm_pos.copy()
            self._last_sent_target_pos = self._ref_arm_pos.copy()
            self._filtered_target_rpy = self._ref_arm_rpy.copy()
            self._last_sent_target_rpy = self._ref_arm_rpy.copy()
            print(f"  [REF] 6-DOF 参考帧: vr_pos={np.round(self._ref_vr_pos, 3)}, "
                  f"vr_rot={np.round(self._ref_vr_rot, 3)}, "
                  f"arm_pos={np.round(self._ref_arm_pos, 3)}, "
                  f"arm_rpy={np.round(self._ref_arm_rpy, 3)}")
            return False
        vr_offset_pos = (pos - self._ref_vr_pos) * POSITION_SCALE
        vr_offset_pos[np.abs(vr_offset_pos) < self.dead_zone_pos] = 0.0
        raw_target_pos = self._ref_arm_pos + vr_offset_pos
        self._filtered_target_pos = (
            self.filter_alpha_pos * raw_target_pos +
            (1.0 - self.filter_alpha_pos) * self._filtered_target_pos)
        pos_delta = self._filtered_target_pos - self._last_sent_target_pos
        pos_delta = np.clip(pos_delta, -self.max_step_delta_pos, self.max_step_delta_pos)
        target_pos = self._last_sent_target_pos + pos_delta
        target_pos = np.clip(target_pos, [-0.05, -0.25, 0.05], [0.35, 0.25, 0.45])
        vr_offset_rot = (rot - self._ref_vr_rot) * ROTATION_SCALE
        vr_offset_rot = np.arctan2(np.sin(vr_offset_rot), np.cos(vr_offset_rot))
        vr_offset_rot[np.abs(vr_offset_rot) < self.dead_zone_rot] = 0.0
        raw_target_rpy = self._ref_arm_rpy + vr_offset_rot
        self._filtered_target_rpy = (
            self.filter_alpha_rot * raw_target_rpy +
            (1.0 - self.filter_alpha_rot) * self._filtered_target_rpy)
        rot_delta = self._filtered_target_rpy - self._last_sent_target_rpy
        rot_delta = np.arctan2(np.sin(rot_delta), np.cos(rot_delta))
        rot_delta = np.clip(rot_delta, -self.max_step_delta_rot, self.max_step_delta_rot)
        target_rpy = self._last_sent_target_rpy + rot_delta
        target_qpos = np.concatenate([target_pos, target_rpy])
        pos_changed = not np.allclose(pos_delta, 0, atol=0.0002)
        rot_changed = not np.allclose(rot_delta, 0, atol=0.0003)
        if not pos_changed and not rot_changed:
            return False
        if not hasattr(self, '_dbg_cnt'):
            self._dbg_cnt = 0
        self._dbg_cnt += 1
        if self._dbg_cnt % 30 == 0:
            print(f"  [DBG] vr_pos={np.round(pos, 3)}, vr_rot={np.round(rot, 2)}, "
                  f"target_pos={np.round(target_pos, 3)}, "
                  f"target_rpy={np.round(target_rpy, 2)}")
        try:
            self.robot.move_modeP(target_qpos, clamp)
            self._last_sent_target_pos = target_pos.copy()
            self._last_sent_target_rpy = target_rpy.copy()
            return True
        except Exception as e:
            print(f"\n移动失败: {e}")
            return False
    def reset_ref(self):
        self._ref_vr_pos = None
        self._ref_vr_rot = None
        self._ref_arm_pos = None
        self._ref_arm_rpy = None
        self._filtered_target_pos = None
        self._last_sent_target_pos = None
        self._filtered_target_rpy = None
        self._last_sent_target_rpy = None
        self._grip_was_pressed = False
        self._dirty = True
    def cleanup(self):
        self._client.loop_stop()
        self._client.disconnect()
        self.robot.reset()
        print("[MQTT] 已断开")
def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--freq", type=int, default=DEFAULT_FREQ,
                        help=f"控制频率 Hz (默认 {DEFAULT_FREQ}，AgileX 官方 moveP 示例为 100Hz)")
    args = parser.parse_args()
    os.environ["INFO_LEVEL"] = "INFO"
    condition["save_freq"] = args.freq
    teleop = PicoPiperTeleop(freq=args.freq)
    for episode_id in range(args.episodes):
        print(f"\n{'─'*40}")
        print(f"第 {episode_id + 1}/{args.episodes} 条轨迹")
        print("按 Enter 开始采集...")
        print(f"{'─'*40}")
        while True:
            if is_enter_pressed():
                break
            time.sleep(0.1)
        teleop.robot.reset()
        time.sleep(1)
        teleop.reset_ref()
        collection = CollectAny(condition=condition, start_episode=episode_id,
                                move_check=False, resume=True)
        print("开始! 握把按下→移动手柄, 松开握把→机械臂保持, 按 Enter 结束\n")
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
