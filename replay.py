from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api.objects import GroundPlane
from isaacsim.core.api.robots import Robot
import omni.replicator.core as rep
import omni.usd
import numpy as np
import sys
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation, Articulation
from isaacsim.core.api.controllers import ArticulationController
from isaacsim.robot_motion.motion_generation import KinematicsSolver
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from termcolor import cprint
from Kinematics.Kinematics_Solver import RealmanKinematicsSolver
from Environment.Simple_Env import BaseEnv
from Robot.realman_robot import Realman
import matplotlib.pyplot as plt
from Utils.Parser import parse_args_record
from Utils.Object_Tools import set_prim_visible_group
from isaacsim.core.utils.prims import set_prim_visibility

def main(args):
    base_env = BaseEnv()
    realman = Realman(
        prim_path="/World/Realman",
        name="realman",
        world=base_env.world
    )
    realman.apply_action
   

if __name__ == '__main__':
    parser = parse_args_record()
    main(vars(parser.parse_args()))