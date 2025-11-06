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
from Utils.Parser import parse_args_record

def main(args):
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

    cprint("Starting recording...", "yellow")
    base_env.start_record(task_name="realman_demo")
    # stage1: hold the box

    if args["env_random_flag"]:
        cprint("Environment is random", "yellow")   
        x_random = np.random.uniform(0.17, 0.35)
        y_random = np.random.uniform(0.24, 0.42)
        base_env.box.set_local_pose(translation=np.array([x_random, y_random, 0.544]))
    
    for i in range(50):
        base_env.step()

    box_position_before_picking = base_env.box.get_world_pose()[0]
    cprint("Box Position Before Picking: {}".format(base_env.box.get_world_pose()[0]), "yellow")

    realman.set_joint_positions(positions=np.array([-4.6530871e+00 ,-3.8103994e-02 ,1.2183394e+00 ,-3.6342379e-01 ,
    -3.8440518e+00 ,-1.3539194e-01  ,5.0000000e-01  ,4.2105007e+00,
    1.7042177e+00 ,-2.9733958e+00 ,-2.6501727e-01 ,-1.0542235e-07,
    -1.0971073e+00 ,1.2717236e+00 ,-4.1895804e-01 ,-7.7700865e-01,
    1.2963485e+00 ,6.1120838e-01 ,9.6495047e-02 ,-1.6289116e+00,
    1.9037319e+00 ,8.5985887e-01 ,-2.7240545e-01 ,1.3391192e+00,
    -1.6014029e+00 ,4.6082916e+00  ,6.8182266e-01]))

    cprint("Get Realman Ready", "green")
    cprint("Realman qposition After Getting Ready: {}".format(realman.get_joint_positions()), "yellow")

    realman._left_target.set_world_pose(position=np.array([box_position_before_picking[0] + 0.4, box_position_before_picking[1] - 0.09, box_position_before_picking[2] + 0.2]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))
    realman._right_target.set_world_pose(position=np.array([box_position_before_picking[0] - 0.4, box_position_before_picking[1] - 0.09, box_position_before_picking[2] + 0.2]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))

    realman.dense_step_action()
    cprint("finish step 1", "green")

    realman._left_target.set_world_pose(position=np.array([box_position_before_picking[0] + 0.4, box_position_before_picking[1] - 0.09, box_position_before_picking[2] + 0.02]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))
    realman._right_target.set_world_pose(position=np.array([box_position_before_picking[0] - 0.4, box_position_before_picking[1] - 0.09, box_position_before_picking[2] + 0.02]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))
    realman.dense_step_action()
    cprint("finish step 2", "green")

    realman._left_target.set_world_pose(position=np.array([box_position_before_picking[0] + 0.225, box_position_before_picking[1] - 0.09, box_position_before_picking[2] + 0.02]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))
    realman._right_target.set_world_pose(position=np.array([box_position_before_picking[0] - 0.225, box_position_before_picking[1] - 0.09, box_position_before_picking[2] + 0.02]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))
    realman.dense_step_action()
    cprint("finish step 3", "green")

    realman._left_target.set_world_pose(position=np.array([box_position_before_picking[0] + 0.225, box_position_before_picking[1] - 0.09, box_position_before_picking[2] + 0.2]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))
    realman._right_target.set_world_pose(position=np.array([box_position_before_picking[0] - 0.225, box_position_before_picking[1] - 0.09, box_position_before_picking[2] + 0.2]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))
    realman.dense_step_action()
    cprint("finish step 4", "green")
    box_position_after_picking = base_env.box.get_world_pose()[0]
    cprint(f"Box Position After Picking: {box_position_after_picking}", "yellow")
    if box_position_after_picking[2] - box_position_before_picking[2] > 0.05:
        cprint("Box is picked successfully", "green")
    else:
        cprint("Box is not picked successfully", "red")

    cprint("Stop recording...", "yellow")
    base_env.stop_record()

    while simulation_app.is_running():
        base_env.step()

if __name__ == '__main__':
    parser = parse_args_record()
    main(vars(parser.parse_args()))
