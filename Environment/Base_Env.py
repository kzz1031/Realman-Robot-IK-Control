from isaacsim.simulation_app import SimulationApp
# simulation_app = SimulationApp({"headless": False})
import os
import numpy as np
from isaacsim.core.api.objects import GroundPlane
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
import omni.replicator.core as rep
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.api import SimulationContext
from isaacsim.core.api.robots import Robot
from isaacsim.sensors.camera import Camera
import matplotlib.pyplot as plt
from Utils.Code_Tools import get_unique_filename
from termcolor import cprint

class BaseEnv:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add(GroundPlane(prim_path="/World/Ground", name="Ground", color=np.array([0.2, 0.2, 0.2])))
        self.demo_light = rep.create.light(position=[0, 0, 0], light_type="dome")
        self.record_flag = False
        self.context = SimulationContext()
        # self.realman_usd_path = "/home/kzzwang/Documents/code/DigitTwin/Assets/overseas_75_b_v_description/overseas_75_b_v_description_lockplatform2.usd"
        self.realman_usd_path = "/home/kzzwang/Documents/code/DigitTwin/Assets/overseas_75_b_v_description/realman_with_pads_cameras.usd"
        self.saving_data = {
                "joint_positions": [],
                "head_images": []
            }
        add_reference_to_stage(
            usd_path=self.realman_usd_path,
            prim_path="/World/Realman"
        )

        self.head_camera = Camera(
            prim_path="/World/Realman/realman_with_pads_camera/realman_with_pads/overseas_75_b_v_description_lockplatform2/overseas_75_b_v_description/head_link2/camera_link/rsd455/RSD455/Camera_OmniVision_OV9782_Color",
            name="Head_Camera"
        )

        self.right_wrist_camera = Camera(
            prim_path="/World/Realman/realman_with_pads_camera/realman_with_pads/overseas_75_b_v_description_lockplatform2/overseas_75_b_v_description/ClampingPad_Right/rsd455/RSD455/Camera_OmniVision_OV9782_Color",
            name="Right_Wrist_Camera"
        )
        self.left_wrist_camera = Camera(
            prim_path="/World/Realman/realman_with_pads_camera/realman_with_pads/overseas_75_b_v_description_lockplatform2/overseas_75_b_v_description/ClampingPad_Left/rsd455/RSD455/Camera_OmniVision_OV9782_Color",
            name="Left_Wrist_Camera"
        )

        self.table = SingleXFormPrim(
            prim_path="/World/Table",
            name="Table",
            position=np.array([-0.1, -0.85, 0.12]),
            scale=np.array([1.0, 1.0, 1.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )

        add_reference_to_stage(
            usd_path="/home/kzzwang/Documents/code/DigitTwin/Assets/assembly.usd",
            prim_path="/World/Table"
        )

    def set_robot(self, robot: Robot):
        self.robot = robot    

    def reset(self):
        self.world.reset()

    def step(self):
        self.world.step(render=True)

    def start_record(self, task_name: str):
        if self.record_flag == False:
            self.record_flag = True
            self.step_num = 0
            self.task_name = task_name
            self.saving_data = {
                "joint_positions": [],
                "images": []
            }

            if not os.path.exists(f"Data/{task_name}/train_data/"):
                os.makedirs(f"Data/{task_name}/train_data/")
            if not os.path.exists(f"Data/{task_name}/head_camera/"):
                os.makedirs(f"Data/{task_name}/head_camera/")
            if not os.path.exists(f"Data/{task_name}/left_camera/"):
                os.makedirs(f"Data/{task_name}/left_camera/")
            if not os.path.exists(f"Data/{task_name}/right_camera/"):
                os.makedirs(f"Data/{task_name}/right_camera/")

            self.context.add_physics_callback("record_callback", self.record_callback)
    
    def stop_record(self):
        if self.record_flag == True:
            self.record_flag = False
            self.context.remove_physics_callback("record_callback")
            record_file_name = get_unique_filename(f"Data/{self.task_name}/train_data/record_data", ".npz")
            np.savez(record_file_name, **self.saving_data)
            cprint(f"Record data saved to {record_file_name}", "green")


    def record_callback(self, step_size: float):
        if self.step_num % 5 == 0:
            joint_positions = self.robot.get_joint_positions()
            rgba = self.head_camera.get_rgba()
            plt.imsave(f'Data/{self.task_name}/head_camera/head_camera_{self.step_num}.png', rgba[:, :, :3])
            rgba = self.left_wrist_camera.get_rgba()
            plt.imsave(f'Data/{self.task_name}/left_camera/left_camera_{self.step_num}.png', rgba[:, :, :3])
            rgba = self.right_wrist_camera.get_rgba()
            plt.imsave(f'Data/{self.task_name}/right_camera/right_camera_{self.step_num}.png', rgba[:, :, :3])
            self.saving_data["joint_positions"].append(joint_positions)
            self.saving_data["head_images"].append(rgba)
            
        self.step_num += 1

# if __name__ == "__main__":
#     env = BaseEnv()
#     while True:
#         env.step()

# simulation_app.close()