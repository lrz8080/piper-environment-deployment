import sys
sys.path.append("./")

import numpy as np

from my_robot.base_robot import Robot

from controller.Piper_controller import PiperController
try:
    from sensor.Realsense_sensor import RealsenseSensor
except ImportError:
    RealsenseSensor = None

from data.collect_any import CollectAny

# CAMERA_SERIALS = {
#     'head': '023422071967',  # Replace with actual serial number
#     'wrist': '152122077595',   # Replace with actual serial number
# }
# "327322061566"
# '319522064878'
CAMERA_SERIALS = {
    'head': '141722070883',  # Replace with actual serial number
    'wrist': '207522074706',   # Replace with actual serial number
}
START_POSITION_POS_LEFT_ARM = [
                0.057, 
                0.0, 
                0.115, 
                0.0, 
                0.085, 
                0.0, 
]
# Define start position (in radians)
START_POSITION_ANGLE_LEFT_ARM = [
    0.0,   # Joint 1
    0.0,   # Joint 2
    0.0,   # Joint 3
    0.0,   # Joint 4
    0.0,   # Joint 5
    0.0,   # Joint 6
]
# Define start position (in degrees)
START_POSITION_ANGLE_RIGHT_ARM = [
    0,   # Joint 1
    0,    # Joint 2
    0,  # Joint 3
    0,   # Joint 4
    0,  # Joint 5
    0,    # Joint 6
]

condition = {
    "robot":"piper_single",
    "save_path": "./datasets/", 
    "task_name": "piper_trash_pickup", 
    "save_format": "hdf5", 
    "save_freq": 15, 
}


class PiperSingle(Robot):
    def __init__(self, condition=condition, move_check=True, start_episode=0):
        super().__init__(condition=condition, move_check=move_check, start_episode=start_episode)

        self.condition = condition
        self.controllers = {
            "arm":{
                "left_arm": PiperController("left_arm"),
            },
        }
        self.sensors = {
            "image":{
                "cam_head": RealsenseSensor("cam_head"),
                "cam_wrist": RealsenseSensor("cam_wrist"),
            },
        }
    def set_can_name(self,can_name:str = "slave"):
        print("Test:Setting can name to:",can_name)
        self.can_name = can_name
    # ============== init ==============
    def reset(self):
       self.controllers["arm"]["left_arm"].reset(np.array(START_POSITION_ANGLE_LEFT_ARM))
    def reset_position(self):
         self.controllers["arm"]["left_arm"].reset_position(np.array(START_POSITION_POS_LEFT_ARM))
    def set_up(self):
        super().set_up()
        # can_name = self.can_name
        can_name = "can0"
        print("Test:Slave can name:",can_name)
        self.controllers["arm"]["left_arm"].set_up(can_name)
        if "image" in self.sensors:
            self.sensors["image"]["cam_head"].set_up(CAMERA_SERIALS["head"])
            self.sensors["image"]["cam_wrist"].set_up(CAMERA_SERIALS["wrist"])

        # 只对实际存在的控制器/传感器类型设置 collect_type
        collect_types = {"arm": ["joint","qpos","gripper"]}
        if "image" in self.sensors:
            collect_types["image"] = ["color"]
        self.set_collect_type(collect_types)
        
        print("set up success!")
    def move_modeP(self,position,gripper):
        self.controllers["arm"]["left_arm"].move_modeP(position,gripper)
    
    
    def show_camera(self):
        """打开相机实时预览窗口"""
        from sensor.SensorVisualizer import SensorVisualizer
        import time

        vis = SensorVisualizer(figsize=(12, 5))
        self._cam_running = True
        print("相机实时预览中，关闭窗口即可退出...")
        while self._cam_running:
            if not hasattr(self, '_cam_running'):
                break
            data = self.get()
            sensor_data = data[1]
            vis.visualize(sensor_data)
            time.sleep(0.05)

if __name__=="__main__":
    import time
    robot = PiperSingle()
    robot.set_up()
    # collection test
    robot.reset()
    data_list = []
    for i in range(100):
        print(i)
        data = robot.get()
        robot.collect(data)
        time.sleep(0.1)
    robot.finish()

    # moving test
    move_data = {
        "arm":{
            "left_arm":{
            "qpos":[0.057, 0.0, 0.216, 0.0, 0.085, 0.0],
            "gripper":0.2,
            },
        },
    }
    robot.move(move_data)
    time.sleep(1)
    move_data = {
        "arm":{
            "left_arm":{
            "joint":[0.00, 0.0, 0.0, 0.0, 0.0, 0.0],
            "gripper":0.2,
            },
        },
    }
    robot.move(move_data)