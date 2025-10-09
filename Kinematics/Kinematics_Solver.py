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

from Utils.Transforms import get_pose_relat 
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
        
        self._right_action_masks = np.array([13, 16, 18, 20, 22, 24, 26])
        self._articulation = realman_articulation

        self._left_target = VisualCuboid(
            prim_path="/World/LeftTarget",
            name="LeftTarget",
            scale=np.array([0.05, 0.05, 0.05]),
            translation=np.array([1.6, -1.54, 1.36]),
            color=np.array([1.0, 0.0, 0.0]),
            visible=True
        )

        self._right_target = VisualCuboid(
            prim_path="/World/RightTarget",
            name="RightTarget", 
            scale=np.array([0.05, 0.05, 0.05]),
            translation=np.array([0.7, -1.4, 1.4]),
            color=np.array([0.0, 1.0, 0.0]),
            visible=True
        )

    def get_left_arm_kinematics_solver(self) -> ArticulationKinematicsSolver:
        return self._left_arm_kinematics_solver

    def get_right_arm_kinematics_solver(self) -> ArticulationKinematicsSolver:
        return self._right_arm_kinematics_solver
    
    def get_kinematics_solver(self) -> LulaKinematicsSolver:
        return self._kinematics_solver
    
    def update(self, robot_base_pos, robot_base_ori):
        left_target_pos, left_target_ori = self._left_target.get_world_pose()
        left_target_ori = quat_to_rot_matrix(left_target_ori)

        right_target_pos, right_target_ori = self._right_target.get_world_pose()
        right_target_ori = quat_to_rot_matrix(right_target_ori)

        base_pose, base_ori = self._articulation.get_world_pose()
        base_ori = quat_to_rot_matrix(base_ori)

        rel_left_target_pos, rel_left_target_ori = get_pose_relat(
            trans=left_target_pos,
            rot=left_target_ori,
            robot_pos=base_pose,
            robot_rot=base_ori
        )

        rel_right_target_pos, rel_right_target_ori = get_pose_relat(
            trans=right_target_pos,
            rot=right_target_ori,
            robot_pos=base_pose,
            robot_rot=base_ori
        )

        rel_left_target_ori = euler_angles_to_quats(np.radians([90, 180, 0]))
        rel_right_target_ori = euler_angles_to_quats(np.radians([90, 0, 0]))

        self._kinematics_solver.set_robot_base_pose(robot_position=robot_base_pos, robot_orientation=robot_base_ori)
        left_action, left_success = self._left_arm_kinematics_solver.compute_inverse_kinematics(
            target_position=left_target_pos,
            target_orientation=rel_left_target_ori,
            position_tolerance=0.06,
            orientation_tolerance=0.2
        )

        right_action, right_success = self._right_arm_kinematics_solver.compute_inverse_kinematics(
            target_position=right_target_pos,
            target_orientation=rel_right_target_ori,
            position_tolerance=0.06,
            orientation_tolerance=0.2
        )
        joint_articulation_action = ArticulationAction()
        joint_articulation_action.joint_positions = []
        joint_articulation_action.joint_indices = []
        if left_success:
            cprint("Left IK Success", "green")
            for i, joint_index in enumerate(left_action.joint_indices):
                if joint_index in self._left_action_indices:
                    joint_articulation_action.joint_positions.append(left_action.joint_positions[i])
                    joint_articulation_action.joint_indices.append(joint_index)       
        else:
            cprint("Left IK Failed", "red")

        if right_success:
            cprint("Right IK Success", "green")
            for i, joint_index in enumerate(right_action.joint_indices):
                if joint_index in self._right_action_masks:
                    joint_articulation_action.joint_positions.append(right_action.joint_positions[i])
                    joint_articulation_action.joint_indices.append(joint_index)
        else:
            cprint("Right IK Failed", "red")

        self._articulation.apply_action(joint_articulation_action)

        return