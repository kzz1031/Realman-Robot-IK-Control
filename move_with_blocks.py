from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api.objects import GroundPlane
from isaacsim.core.api.robots import Robot
import omni.replicator.core as rep
import omni.usd
import numpy as np
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation, Articulation
from isaacsim.core.api.controllers import ArticulationController
from isaacsim.robot_motion.motion_generation import KinematicsSolver
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from termcolor import cprint
from Kinematics.Kinematics_Solver import RealmanKinematicsSolver
from Environment.Base_Env import BaseEnv
from Robot.realman_robot import Realman
import matplotlib.pyplot as plt

base_env = BaseEnv()

realman = Realman(
    prim_path="/World/Realman",
    name="realman",
    world=base_env.world
)
base_env.set_robot(realman)
realman_articulation = SingleArticulation("/World/Realman")

base_env.reset()

realman.set_world_pose(position=(0, 0, 0.2), orientation=(1, 0, 0, 0) )
realman.set_joint_positions(positions=np.array([-1]), joint_indices=np.array([14]))

realman.initialize()
realman_articulation.initialize()
base_env.head_camera.initialize()
base_env.left_wrist_camera.initialize()
base_env.right_wrist_camera.initialize()

base_env.head_camera.add_motion_vectors_to_frame()
for _ in range(5):
    base_env.step()

cprint(f"realman base position: {realman.get_world_pose()[0]} orientation: {realman.get_world_pose()[1]}", "blue")
cprint(f"_kinematics_solver.get_all_frame_names(): {realman.ki_solver.get_kinematics_solver().get_all_frame_names()}", "green")
cprint(f"realman.num_dof: {realman.num_dof}", "blue")
cprint(f"realman.dof_names: {realman.dof_names}", "blue")
# ====== 主仿真循环 ======
# cprint("Starting recording...", "yellow")
# base_env.start_record(task_name="realman_demo")
# stage1: hold the box
step = 0
last_left_target_pos = None 
last_left_target_ori = None
last_right_target_pos = None
last_right_target_ori = None
left_target_changed = True
right_target_changed = True
current_left_target_pos, current_left_target_ori = realman._left_target.get_world_pose()
current_right_target_pos, current_right_target_ori = realman._right_target.get_world_pose()

realman.dense_step_action()

# realman._left_target.set_world_pose(position=np.array([0.356, -0.55, 0.73]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))
# realman._right_target.set_world_pose(position=np.array([-0.181, -0.573, 0.73]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))

# realman.dense_step_action()

# realman._left_target.set_world_pose(position=np.array([0.306, -0.55, 0.73]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))
# realman._right_target.set_world_pose(position=np.array([-0.121, -0.573, 0.73]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))

# realman.dense_step_action()

# realman._left_target.set_world_pose(position=np.array([0.306, -0.55, 1.0]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))
# realman._right_target.set_world_pose(position=np.array([-0.121, -0.573, 1.0]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))

# realman.dense_step_action()

# cprint("Stop recording...", "yellow")
# base_env.stop_record()

# while simulation_app.is_running():
#     base_env.step()

# while True:
#     base_env.step()
#     if left_target_changed or right_target_changed:
#         print(f"Left target changed: {left_target_changed}, Right target changed: {right_target_changed}")
#         realman.dense_step_action()

#         last_left_target_pos = current_left_target_pos
#         last_left_target_ori = current_left_target_ori
#         last_right_target_pos = current_right_target_pos
#         last_right_target_ori = current_right_target_ori
        
#     current_left_target_pos, current_left_target_ori = realman._left_target.get_world_pose()
#     current_right_target_pos, current_right_target_ori = realman._right_target.get_world_pose()

#     left_target_changed = (
#         not np.allclose(current_left_target_pos, last_left_target_pos) or
#         not np.allclose(current_left_target_ori, last_left_target_ori)
#     )

#     right_target_changed = (
#         not np.allclose(current_right_target_pos, last_right_target_pos) or
#         not np.allclose(current_right_target_ori, last_right_target_ori)
#     )


# simulation_app.close()
