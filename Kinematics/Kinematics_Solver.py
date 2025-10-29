import numpy as np
import os
import carb
from termcolor import cprint

from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from isaacsim.robot_motion.motion_generation import interface_config_loader
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.api import World

from Utils.Transforms import get_pose_relat, quat_to_rot_matrix, Rotation
from Utils.Code_Tools import dense_trajectory_points_generation
from scipy.spatial.transform import Rotation as ScipyRotation 
class RealmanKinematicsSolver:
    def __init__(self, realman_description_path: str, urdf_path: str, realman_articulation: Articulation = None):
        self._kinematics_solver = LulaKinematicsSolver(
            robot_description_path=realman_description_path,
            urdf_path=urdf_path
        )
        self._left_arm_kinematics_solver = ArticulationKinematicsSolver(
            robot_articulation=realman_articulation,
            kinematics_solver=self._kinematics_solver,
            end_effector_frame_name="l_link7"
        )
        self._right_arm_kinematics_solver = ArticulationKinematicsSolver(
            robot_articulation=realman_articulation,
            kinematics_solver=self._kinematics_solver,
            end_effector_frame_name="r_link7" 
        )
        self._left_action_indices = np.array([12, 15, 17, 19, 21, 23, 25])
        
        self._right_action_indices = np.array([13, 16, 18, 20, 22, 24, 26])
        self._articulation = realman_articulation

    def get_left_arm_kinematics_solver(self) -> ArticulationKinematicsSolver:
        return self._left_arm_kinematics_solver

    def get_right_arm_kinematics_solver(self) -> ArticulationKinematicsSolver:
        return self._right_arm_kinematics_solver
    
    def get_kinematics_solver(self) -> LulaKinematicsSolver:
        return self._kinematics_solver
    