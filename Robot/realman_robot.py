import numpy as np
import os
import carb
from termcolor import cprint

from isaacsim.core.api.robots import Robot
from isaacsim.core.api import World
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.prims import SingleArticulation, Articulation
from Kinematics.Kinematics_Solver import RealmanKinematicsSolver
from isaacsim.core.api.objects import VisualCuboid
from Utils.Transforms import get_pose_relat, quat_to_rot_matrix
from Utils.Code_Tools import dense_trajectory_points_generation
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.sensors.camera import Camera

class Realman(Robot):
    def __init__(self, prim_path: str, name: str, world: World):
        super().__init__(prim_path=prim_path, name=name)
        self.world = world
        self._name = "Realman"
        self._prim_path = prim_path
        self._articulation = SingleArticulation(prim_path=self._prim_path)
        self.realman_urdf_path = "/home/kzzwang/Documents/code/DigitTwin/Assets/realman/rmc_aida_l_ros1-develop2/rmc_aida_l_ros1-develop/src/dual_arm_control/dual_arm_description/overseas_75_b_v_description/urdf/overseas_75_b_v_description.urdf"
        self.realman_discription_path = "/home/kzzwang/Documents/code/DigitTwin/Assets/Lula/realman_platform_half.yaml"
        
        world.scene.add(self)


        self._left_target = VisualCuboid(
            prim_path="/World/LeftTarget",
            name="LeftTarget",
            scale=np.array([0.05, 0.05, 0.05]),
            translation=np.array([0.356, -0.55, 0.71]),
            color=np.array([1.0, 0.0, 0.0]),
            visible=True
        )

        self._right_target = VisualCuboid(
            prim_path="/World/RightTarget",
            name="RightTarget", 
            scale=np.array([0.05, 0.05, 0.05]),
            translation=np.array([-0.181, -0.573, 0.72]),
            color=np.array([0.0, 1.0, 0.0]),
            visible=True
        )

        self.head_camera = Camera(
            prim_path=self._prim_path + "/realman_with_pads_camera/realman_with_pads/overseas_75_b_v_description_lockplatform2/overseas_75_b_v_description/head_link2/camera_link/rsd455/RSD455/Camera_OmniVision_OV9782_Color",
            name="HeadCamera"
        )
        # Add any Realman-specific initialization here

    def initialize(self, physics_sim_view=None):
        super().initialize(physics_sim_view=physics_sim_view)
        self._articulation.initialize()
        
        # 初始化相机
        self.head_camera.initialize()

        self.ki_solver = RealmanKinematicsSolver(
            realman_description_path=self.realman_discription_path,
            urdf_path=self.realman_urdf_path,
            realman_articulation=self._articulation
        )
        # /World/Realman/realman_with_pads_camera/realman_with_pads/overseas_75_b_v_description_lockplatform2/overseas_75_b_v_description/ClampingPad_Right
        # /World/Realman/realman_with_pads_camera/realman_with_pads/overseas_75_b_v_description_lockplatform2/overseas_75_b_v_description/ClampingPad_Left
        # Additional initialization steps for Realman can be added here
        self.left_end_effector = SingleRigidPrim(
            prim_path=self._prim_path + "/realman_with_pads_camera/realman_with_pads/overseas_75_b_v_description_lockplatform2/overseas_75_b_v_description/ClampingPad_Left",
            name="LeftEndEffector"
        )
        self.right_end_effector = SingleRigidPrim(
            prim_path=self._prim_path + "/realman_with_pads_camera/realman_with_pads/overseas_75_b_v_description_lockplatform2/overseas_75_b_v_description/ClampingPad_Right",
            name="RightEndEffector"
        )
        self.left_end_effector.initialize(physics_sim_view=physics_sim_view)
        self.right_end_effector.initialize(physics_sim_view=physics_sim_view)

    def step_action(self, robot_base_pos, robot_base_ori):
        left_target_pos, left_target_ori = self._left_target.get_world_pose()
        left_target_ori = quat_to_rot_matrix(left_target_ori)

        right_target_pos, right_target_ori = self._right_target.get_world_pose()
        right_target_ori = quat_to_rot_matrix(right_target_ori)

        base_pose, base_ori = self._articulation.get_world_pose()
        cprint(f"Robot base position: {base_pose}, orientation: {base_ori}", "blue")
        cprint(f"Robot base pos: {robot_base_pos}, orientation: {robot_base_ori}", "blue")
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

        self.ki_solver._kinematics_solver.set_robot_base_pose(robot_position=robot_base_pos, robot_orientation=robot_base_ori)
        left_action, left_success = self.ki_solver._left_arm_kinematics_solver.compute_inverse_kinematics(
            target_position=left_target_pos,
            target_orientation=rel_left_target_ori,
            position_tolerance=0.03,
            orientation_tolerance=0.1
        )

        right_action, right_success = self.ki_solver._right_arm_kinematics_solver.compute_inverse_kinematics(
            target_position=right_target_pos,
            target_orientation=rel_right_target_ori,
            position_tolerance=0.03,
            orientation_tolerance=0.1
        )
        joint_articulation_action = ArticulationAction()
        joint_articulation_action.joint_positions = []
        joint_articulation_action.joint_indices = []
        if left_success:
            cprint("Left IK Success", "green")
            # print("Left end effector pose: pos {} ori {}".format(left_target_pos, rel_left_target_ori))
            # print("Left target pose: pos {} ori {}".format(left_target_pos, rel_left_target_ori))
            for i, joint_index in enumerate(left_action.joint_indices):
                if joint_index in self.ki_solver._left_action_indices:
                    joint_articulation_action.joint_positions.append(left_action.joint_positions[i])
                    joint_articulation_action.joint_indices.append(joint_index)       
        else:
            cprint("Left IK Failed", "red")

        if right_success:
            cprint("Right IK Success", "green")
            # print("Right end effector pose: pos {} ori {}".format(right_target_pos, rel_right_target_ori))
            # print("Right target pose: pos {} ori {}".format(right_target_pos, rel_right_target_ori))
            for i, joint_index in enumerate(right_action.joint_indices):
                if joint_index in self.ki_solver._right_action_masks:
                    joint_articulation_action.joint_positions.append(right_action.joint_positions[i])
                    joint_articulation_action.joint_indices.append(joint_index)
        else:
            cprint("Right IK Failed", "red")
        if len(joint_articulation_action.joint_indices) > 0:
            self._articulation.apply_action(joint_articulation_action)

        for _ in range(20):
            self.world.step(render=True)
        return


    def dense_step_action(self, robot_base_pos, robot_base_ori, dense_sample_scale:float = 0.01):
        left_target_pos, left_target_ori = self._left_target.get_world_pose()
        left_target_ori = quat_to_rot_matrix(left_target_ori)

        right_target_pos, right_target_ori = self._right_target.get_world_pose()
        right_target_ori = quat_to_rot_matrix(right_target_ori)

        base_pose, base_ori = self._articulation.get_world_pose()
        cprint(f"Robot base position: {base_pose}, orientation: {base_ori}", "blue")
        cprint(f"Robot base pos: {robot_base_pos}, orientation: {robot_base_ori}", "blue")
        base_ori = quat_to_rot_matrix(base_ori)

        rel_left_target_ori = euler_angles_to_quats(np.radians([90, 180, 0]))
        rel_right_target_ori = euler_angles_to_quats(np.radians([90, 0, 0]))

        self.ki_solver._kinematics_solver.set_robot_base_pose(robot_position=robot_base_pos, robot_orientation=robot_base_ori)
        
        current_ee_left_pos, current_ee_left_ori = self.left_end_effector.get_world_pose()
        current_ee_right_pos, current_ee_right_ori = self.right_end_effector.get_world_pose()

        dense_sample_num = int(max(np.linalg.norm(left_target_pos - current_ee_left_pos), np.linalg.norm(right_target_pos - current_ee_right_pos)) // dense_sample_scale)
        cprint(f"dense_sample_num: {dense_sample_num}", "blue")
        left_interp_pos = dense_trajectory_points_generation(
            start_pos=current_ee_left_pos, 
            end_pos=left_target_pos,
            num_points=dense_sample_num,
        )
        print(f"left interp pos: {left_interp_pos}")
        right_interp_pos = dense_trajectory_points_generation(
            start_pos=current_ee_right_pos, 
            end_pos=right_target_pos,
            num_points=dense_sample_num,
        )
        print(f"right interp pos: {right_interp_pos}")
        for i in range(dense_sample_num):
            left_target_pos_i = left_interp_pos[i]
            right_target_pos_i = right_interp_pos[i]
            left_action, left_success = self.ki_solver._left_arm_kinematics_solver.compute_inverse_kinematics(
                target_position=left_target_pos_i,
                target_orientation=rel_left_target_ori,
                position_tolerance=0.03,
                orientation_tolerance=0.1
            )

            right_action, right_success = self.ki_solver._right_arm_kinematics_solver.compute_inverse_kinematics(
                target_position=right_target_pos_i,
                target_orientation=rel_right_target_ori,
                position_tolerance=0.03,
                orientation_tolerance=0.1
            )
            if left_success and right_success:
                joint_articulation_action = ArticulationAction()
                joint_articulation_action.joint_positions = []
                joint_articulation_action.joint_indices = []
        
                for i, joint_index in enumerate(left_action.joint_indices):
                    if joint_index in self.ki_solver._left_action_indices:
                        joint_articulation_action.joint_positions.append(left_action.joint_positions[i])
                        joint_articulation_action.joint_indices.append(joint_index)       

                for i, joint_index in enumerate(right_action.joint_indices):
                    if joint_index in self.ki_solver._right_action_masks:
                        joint_articulation_action.joint_positions.append(right_action.joint_positions[i])
                        joint_articulation_action.joint_indices.append(joint_index)
                
                self._articulation.apply_action(joint_articulation_action)
                self.world.step()
                
        if left_success and right_success:
            print("\033[32mfinish moving!\033[0m")
        else:
            if not left_success and not right_success:
                print("\033[31mboth hand failed to move completely!\033[0m")
            elif not left_success:
                print("\033[31mleft hand failed to move completely!\033[0m")
            elif not right_success:
                print("\033[31mright hand failed to move completely!\033[0m")
        return