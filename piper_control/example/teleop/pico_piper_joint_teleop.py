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
JOINT_SCALE = {
    "x":  -20.0,
    "y": 20.0,
    "z":  -20.0,
}
ROTATION_SCALE = {
    "rx": 20.0,
    "ry":  -10.0,
    "rz":  10.0,
}
DEAD_ZONE_JOINT = 0.080
DEFAULT_FREQ = 200
class PicoPiperJointTeleop:
    def __init__(self, freq: int = DEFAULT_FREQ):
        self.freq = freq
        self.dt = 1.0 / freq
        self.dead_zone = DEAD_ZONE_JOINT
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
        self._prev_vr = None
        self._dirty = True
        self._joint_limits = np.array([
            [-2.618,  2.618],
            [ 0.0,    3.142],
            [-2.967,  0.0],
            [-1.745,  1.745],
            [-1.222,  1.222],
            [-2.094,  2.094],
        ])
        print("\n" + "=" * 60)
        print("Pico 4 VR 关节空间遥操作已就绪 (直驱模式)")
        print(f"  控制频率: {self.freq} Hz  (每帧 {self.dt*1000:.1f} ms)")
        print(f"  死区:     {self.dead_zone*1000:.1f} mrad")
        print(f"  握把按下 + 移动手柄 → 控制关节 1/2/3")
        print(f"  握把按下 + 旋转手柄 → 控制关节 4/5/6")
        print(f"  扳机 → 控制夹爪")
        print(f"  松开握把 → 机械臂保持")
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
        return np.array([x, y, z], dtype=np.float64), np.array([rx, ry, rz], dtype=np.float64), grip_pressed, clamp_val
    def teleop_step(self) -> bool:
        pos, rot, grip, clamp = self._read_vr()
        if pos is None or not grip:
            self._dirty = True
            return False
        if self._dirty or self._prev_vr is None:
            self._prev_vr = np.concatenate([pos, rot])
            self._dirty = False
            print(f"  [REF] 关节追踪起点: pos={np.round(pos, 3)}, rot={np.round(rot, 3)}")
            return False
        vr_current = np.concatenate([pos, rot])
        vr_delta = vr_current - self._prev_vr
        dj1 = vr_delta[0] * JOINT_SCALE["x"]
        dj2 = vr_delta[1] * JOINT_SCALE["y"]
        dj3 = vr_delta[2] * JOINT_SCALE["z"]
        dj4 = vr_delta[3] * ROTATION_SCALE["rx"]
        dj5 = vr_delta[4] * ROTATION_SCALE["ry"]
        dj6 = vr_delta[5] * ROTATION_SCALE["rz"]
        raw_delta = np.array([dj1, dj2, dj3, dj4, dj5, dj6])
        raw_delta[np.abs(raw_delta) < self.dead_zone] = 0.0
        if np.allclose(raw_delta, 0, atol=0.0002):
            return False
        data = self.robot.get()
        current_joints = np.array(data[0]["left_arm"]["joint"], dtype=np.float64)
        target_joints = current_joints + raw_delta
        for i in range(6):
            target_joints[i] = np.clip(target_joints[i],
                                       self._joint_limits[i][0],
                                       self._joint_limits[i][1])
        self._prev_vr = vr_current
        if not hasattr(self, '_dbg_cnt'):
            self._dbg_cnt = 0
        self._dbg_cnt += 1
        if self._dbg_cnt % 30 == 0:
            print(f"  [DBG] vr_delta={np.round(vr_delta, 4)}, "
                  f"joint_delta={np.round(raw_delta, 4)}, "
                  f"target={np.round(target_joints, 3)}")
        try:
            self.robot.controllers["arm"]["left_arm"].set_joint(target_joints)
            self.robot.controllers["arm"]["left_arm"].set_gripper(clamp)
            return True
        except Exception as e:
            print(f"\n移动失败: {e}")
            return False
    def reset_ref(self):
        self._prev_vr = None
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
                        help=f"控制频率 Hz (默认 {DEFAULT_FREQ}，AgileX 官方 moveJ 示例为 200Hz)")
    args = parser.parse_args()
    os.environ["INFO_LEVEL"] = "INFO"
    condition["save_freq"] = args.freq
    teleop = PicoPiperJointTeleop(freq=args.freq)
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
        print("开始! 握把→移动/旋转手柄, 按 Enter 结束\n")
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
