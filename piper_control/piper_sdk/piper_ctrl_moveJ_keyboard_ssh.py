#!/usr/bin/env python3
# -*-coding:utf8-*-
# 键盘控制Piper机械臂关节角度 - SSH远程版
# 通过SSH终端直接读取按键，按住即连续运动（利用终端自动重复）
# 需要安装: pip install piper_sdk

import time
import sys
import select
import termios
import tty
from piper_sdk import *

class PiperJointKeyboardControllerSSH:
    def __init__(self, can_channel="can1"):
        self.piper = C_PiperInterface_V2(can_channel)
        self.piper.ConnectPort()

        print("正在使能机械臂...")
        while not self.piper.EnablePiper():
            time.sleep(0.01)
        print("机械臂已使能")

        self.piper.GripperCtrl(0, 1000, 0x01, 0)

        self.factor = 57295.7795
        self.step_deg = 0.5
        self.step_gripper_mm = 3.0

        self.limits_deg = [
            (-150, 150), (0, 180), (-170, 0),
            (-100, 100), (-70, 70), (-120, 120),
        ]

        self.poses_deg = {
            '1': ('归零', [0, 0, 0, 0, 0, 0]),
            '2': ('Home', [0, 15, -15, 20, -15, 30]),
            '3': ('抓取', [30, 20, -30, 50, -20, 15]),
        }

        self.key_map = {
            'q': (0, +1), 'a': (0, -1),
            'w': (1, +1), 's': (1, -1),
            'e': (2, +1), 'd': (2, -1),
            'r': (3, +1), 'f': (3, -1),
            't': (4, +1), 'g': (4, -1),
            'y': (5, +1), 'h': (5, -1),
        }

        self.print_help()
        self._init_joints()

    def _init_joints(self):
        try:
            jm = self.piper.GetArmJointMsgs()
            self.joints = [
                jm.joint_state.joint_1, jm.joint_state.joint_2,
                jm.joint_state.joint_3, jm.joint_state.joint_4,
                jm.joint_state.joint_5, jm.joint_state.joint_6,
            ]
            self.joints = [j * 0.001 / 180 * 3.1415926 for j in self.joints]
            self.gripper_mm = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle * 0.001
            print("已读取当前关节角度")
        except:
            self.joints = [0.0] * 6
            self.gripper_mm = 0.0

    def print_help(self):
        print("\n" + "=" * 60)
        print("  机械臂关节键盘控制 (SSH远程 | 按住连续运动)")
        print("=" * 60)
        print("  Q/A - 关节0±    W/S - 关节1±    E/D - 关节2±")
        print("  R/F - 关节3±    T/G - 关节4±    Y/H - 关节5±")
        print("  Z/X - 夹爪±    空格 - 夹爪开/关")
        print("  1/2/3 - 归零/Home/抓取    0 - 紧急归零")
        print("  Ctrl+C - 退出")
        print("-" * 60)
        print(" 提示: 按住按键不放 → 连续运动, 松开即停")
        print("       步进: %.1f°/次  夹爪: %.0fmm/次" % (self.step_deg, self.step_gripper_mm))
        print("=" * 60 + "\n")

    def deg_to_rad(self, deg): return deg * 0.01745329252
    def rad_to_deg(self, rad): return rad * 57.295779513
    def clamp(self, v, lo, hi): return max(lo, min(hi, v))

    def move_joint(self, idx, direction):
        cur_deg = self.rad_to_deg(self.joints[idx])
        new_deg = self.clamp(cur_deg + direction * self.step_deg,
                             self.limits_deg[idx][0], self.limits_deg[idx][1])
        self.joints[idx] = self.deg_to_rad(new_deg)

    def move_gripper(self, delta_mm):
        self.gripper_mm = self.clamp(self.gripper_mm + delta_mm, 0, 70)

    def send(self):
        try:
            cmds = [round(j * self.factor) for j in self.joints]
            g = round(self.gripper_mm * 1000)
            self.piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
            self.piper.JointCtrl(*cmds)
            self.piper.GripperCtrl(g, 1000, 0x01, 0)
        except Exception as e:
            sys.stderr.write(f"\n[CMD ERR: {e}]\n")

    def status_line(self, key=""):
        degs = [f"{self.rad_to_deg(j):6.1f}" for j in self.joints]
        sys.stderr.write(
            f"\rJ0:{degs[0]} J1:{degs[1]} J2:{degs[2]} "
            f"J3:{degs[3]} J4:{degs[4]} J5:{degs[5]} "
            f"G:{self.gripper_mm:5.0f}mm  key:{key:3s}   \r"
        )

    def handle_key(self, ch):
        if ch in self.key_map:
            j, d = self.key_map[ch]
            self.move_joint(j, d)
            self.send()
            self.status_line(ch)
        elif ch == 'z':
            self.move_gripper(self.step_gripper_mm)
            self.send()
            self.status_line('G+')
        elif ch == 'x':
            self.move_gripper(-self.step_gripper_mm)
            self.send()
            self.status_line('G-')
        elif ch == ' ':
            self.gripper_mm = 0.0 if self.gripper_mm > 35 else 70.0
            self.send()
            sys.stderr.write(f"\n[夹爪: {'开' if self.gripper_mm > 35 else '关'}]\n")
        elif ch in self.poses_deg:
            name, degs = self.poses_deg[ch]
            self.joints = [self.deg_to_rad(d) for d in degs]
            self.send()
            sys.stderr.write(f"\n[位姿: {name}]\n")
        elif ch == '0':
            self.joints = [0.0] * 6
            self.gripper_mm = 0.0
            self.send()
            sys.stderr.write("\n[紧急归零]\n")

    def run(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            sys.stderr.write("\033[?25l")  # hide cursor
            self.running = True
            self.status_line()
            while self.running:
                r, _, _ = select.select([sys.stdin], [], [], 0.02)
                if r:
                    ch = sys.stdin.read(1)
                    if ch == '\x03':  # Ctrl+C
                        self.running = False
                        break
                    self.handle_key(ch.lower())
                else:
                    # No key pressed — show idle status
                    pass
            # Final status on exit
            sys.stderr.write("\n")
            degs = [f"{self.rad_to_deg(j):6.1f}" for j in self.joints]
            sys.stderr.write(f"最终: {''.join(degs)}  G:{self.gripper_mm:.0f}mm\n")
        finally:
            sys.stderr.write("\033[?25h")  # show cursor
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Piper 关节键盘控制 (SSH远程版)")
    parser.add_argument("--can", default="can1", help="CAN接口")
    parser.add_argument("--step", type=float, default=0.5, help="步进度数(默认0.5°)")
    args = parser.parse_args()
    try:
        ctrl = PiperJointKeyboardControllerSSH(args.can)
        ctrl.step_deg = args.step
        ctrl.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback; traceback.print_exc()
