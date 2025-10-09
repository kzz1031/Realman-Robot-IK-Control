from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api.objects import GroundPlane
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
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


world = World(stage_units_in_meters=1.0)
rep.create.light(position=[0, 0, 0], light_type="dome")
# 4. 添加地面
world.scene.add(GroundPlane(prim_path="/World/Ground", name="Ground", color=np.array([0, 0, 0])))

# 5. 导入 Realman 机械臂 USD 文件
#realman_usd_path = "/home/kzzwang/Documents/code/DigitTwin/Assets/realman/rmc_aida_l_ros1-develop2/rmc_aida_l_ros1-develop/src/dual_arm_control/dual_arm_description/overseas_75_b_v_description/urdf/overseas_75_b_v_description/overseas_75_b_v_description.usd"
realman_usd_path = "/home/kzzwang/Documents/code/DigitTwin/Assets/overseas_75_b_v_description/overseas_75_b_v_description_lockplatform.usd"
realman_urdf_path = "/home/kzzwang/Documents/code/DigitTwin/Assets/realman/rmc_aida_l_ros1-develop2/rmc_aida_l_ros1-develop/src/dual_arm_control/dual_arm_description/overseas_75_b_v_description/urdf/overseas_75_b_v_description.urdf"
realman_discription_path = "/home/kzzwang/Documents/code/DigitTwin/Assets/Lula/realman.yaml"

add_reference_to_stage(
    usd_path=realman_usd_path,
    prim_path="/World/Realman"
)
realman = Robot(
    prim_path="/World/Realman",
    name="realman",
    position=(0, 0, 0.7),
    orientation=(1, 0, 0, 0)  # 四元数 (w,x,y,z)
)

realman_articulation = SingleArticulation("/World/Realman")
# 6. 初始化场景
world.reset()
realman.initialize()
realman_articulation.initialize()
# cprint(f"Articulation is initialized: {realman_articulation.is_initialized}", "blue")
cprint(f"realman base position: {realman.get_world_pose()[0]} orientation: {realman.get_world_pose()[1]}", "blue")

realman_kinematics_solver = RealmanKinematicsSolver(
    realman_description_path=realman_discription_path,
    urdf_path=realman_urdf_path,
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
    world.step(render=True)
    if step % 10000 == 0:
        realman_kinematics_solver.update(realman.get_world_pose()[0], realman.get_world_pose()[1])
        

# 7. 关闭应用
simulation_app.close()
