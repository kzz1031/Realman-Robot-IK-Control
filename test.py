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


base_env = BaseEnv()

realman = Robot(
    prim_path="/World/Realman",
    name="realman",
)
base_env.world.scene.add(realman)
realman_articulation = SingleArticulation("/World/Realman")

base_env.reset()
realman.set_world_pose(position=(0, 0, 0.2), orientation=(1, 0, 0, 0) )
realman.initialize()
realman_articulation.initialize()
# cprint(f"Articulation is initialized: {realman_articulation.is_initialized}", "blue")
cprint(f"realman base position: {realman.get_world_pose()[0]} orientation: {realman.get_world_pose()[1]}", "blue")

realman_kinematics_solver = RealmanKinematicsSolver(
    realman_description_path=base_env.realman_discription_path,
    urdf_path=base_env.realman_urdf_path,
    realman_articulation=realman_articulation
)

cprint(f"_kinematics_solver.get_all_frame_names(): {realman_kinematics_solver.get_kinematics_solver().get_all_frame_names()}", "green")

cprint(f"realman.num_dof: {realman.num_dof}", "blue")
cprint(f"realman.dof_names: {realman.dof_names}", "blue")

# cprint(f"Realman Joints Positions: {realman.get_joint_positions()}", "yellow")
# cprint(f"Realman Articulation Joints Positions: {realman_articulation.get_joint_positions()}", "green")
# ====== 主仿真循环 ======

step = 0
while True:
    # print(ur10.get_joint_positions())
    # step += 1
    base_env.step()
    # if step % 10000 == 0:
    #     realman_kinematics_solver.update(realman.get_world_pose()[0], realman.get_world_pose()[1])
        
simulation_app.close()
