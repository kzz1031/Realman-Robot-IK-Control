## **一.项目介绍**

- 项目概述

  开发并集成一个具身双臂机器人的ROS功能包，集成了机械臂、移动底盘、视觉传感器以及头部舵机控制模块。

- 系统组成
  1. **机械臂模块**：搭配两台7/6轴机械臂，具备单臂独立操作和双臂协同操作能力。
  2. **移动底盘模块（`AGV`）**:具备运动、导航、避障能力。
  3. **视觉传感器模块（`D435`）**：`Intel RealSense D435`相机提供了深度图像、`RGB`图像以及指定像素点的深度值，支持机器人进行环境感知、物体识别与定位等功能。
  4. **头部舵机模块**：通过控制头部舵机，机器人能够灵活地调整视角，增强对环境的观测能力。

![dual_lift_robot.png](./pictures/dual_lift_robot.png.png)

​                                                                                                                         图1 具身双臂机器人

- 代码版本

  `V1.1.2`
  
- 硬件环境

  | 部件名称                                                     | 硬件版本信息                                                 | 软件版本信息                                           |
  | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------ |
  | 机械臂                                                       | RM75-B/RM65-B                                                | 控制器V1.4.10及以上，API V4.2.8及以上，ROS功能包V2.0.1 |
  | 相机                                                         | Realsense D435C                                              | realsense-ros-2.3.2                                    |
  | 主控                                                         | jetson xavier NX                                             | ubuntu20.04 、ros-noetic                               |
  | 底盘                                                         | 悟时                                                         |                                                        |
  | 头部舵机                                                     | LX-224HV 1                                                   | 串口通信                                               |
  | 末端工具（可选）                                             | RMG24平行夹爪/EG2-4C2夹爪/灵巧手（右手RM56DFX-2R/左手RM56DFX-2L/傲翼™（OHand™） |                                                        |
  | 更多信息参考：https://develop.realman-robotics.com/robot/versionComparisonTable.html ROS包下载：https://github.com/RealManRobot |                                                              |                                                        |
  
  
  

## 二.环境要求

- 系统：`Ubuntu `20.04

- ROS：`noetic`

  

## **三.代码结构**


```

rmc_aida_l_ros1
├── 具身双臂机器人ROS1话题服务列表.md
├── Embodied dual-arm robot ROS1 topic service list.en.md
├── README_CN.md
├── README.md
└── src
    ├── agv_demo  调用悟时底盘ros接口样例包
    │   ├── 底盘说明文件
    │   │   ├── 底盘链接.txt
    │   │   ├── 移动机器人的ROS接口（对外） - WOOSH Robotics.pdf
    │   │   ├── Woosh Design用户指南.pdf
    │   │   ├── Woosh Design User Guide_ZH.pdf
    │   │   ├── Woosh Mobile交互软件用户指南.pdf
    │   │   ├── Woosh Mobile User Guide_ZH.pdf
    │   │   └── woosh_robot_sdk_interface_v1.1.62.pdf
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── agv_demo
    │   ├── package.xml
    │   ├── README_AGV.md
    │   └── scripts
    │       └── agv_demo.py 样例代码
    ├── camera_demo 相机调用样例代码
    │   ├── d435_demo d435相机调用样例包
    │   │   ├── CMakeLists.txt
    │   │   ├── launch
    │   │   │   ├── d435_pub.launch
    │   │   │   └── d435_sub.launch
    │   │   ├── package.xml
    │   │   └── scripts
    │   │       ├── __init__.py
    │   │       ├── video_stream_pub.py
    │   │       └── video_stream_sub.py
    │   ├── README_D435.md  相机包调用命令文档
    │   └── usb_camera_demo usb相机调用样例包
    │       ├── CMakeLists.txt
    │       ├── config
    │       │   └── camera_params.yaml
    │       ├── include
    │       │   └── usb_camera
    │       ├── launch
    │       │   ├── usb_camera_pub.launch
    │       │   └── usb_camera_sub.launch
    │       ├── package.xml
    │       ├── scripts
    │       │   ├── video_stream_pub.py
    │       │   └── video_stream_sub.py
    │       └── src
    ├── dual_arm_control 双臂包
    │   ├── arm_control
    │   │   ├── CMakeLists.txt
    │   │   ├── package.xml
    │   │   └── src
    │   │       ├── arm_control.cpp
    │   │       └── cubicSpline.h
    │   ├── arm_driver
    │   │   ├── CMakeLists.txt
    │   │   ├── launch
    │   │   │   ├── dual_arm_65_driver.launch
    │   │   │   └── dual_arm_75_driver.launch
    │   │   ├── package.xml
    │   │   └── src
    │   │       ├── arm_driver.cpp
    │   │       ├── cJSON.c
    │   │       ├── cJSON.h
    │   │       └── rm_robot.h
    │   ├── arm_servo
    │   │   ├── CMakeLists.txt
    │   │   └── package.xml
    │   ├── dual_arm_description
    │   │   ├── overseas_65_b_v_description
    │   │   │   ├── CMakeLists.txt
    │   │   │   ├── config
    │   │   │   │   └── joint_names_overseas_65_b_v_description.yaml
    │   │   │   ├── export.log
    │   │   │   ├── launch
    │   │   │   │   ├── display.launch
    │   │   │   │   ├── display_rmg24.launch
    │   │   │   │   └── gazebo.launch
    │   │   │   ├── meshes
    │   │   │   │   ├── base_link2.STL
    │   │   │   │   ├── base_link_underpan.STL
    │   │   │   │   ├── bl_Link.STL
    │   │   │   │   ├── body_base_link.STL
    │   │   │   │   ├── br_Link.STL
    │   │   │   │   ├── camera_link.STL
    │   │   │   │   ├── fl_Link.STL
    │   │   │   │   ├── fr_Link.STL
    │   │   │   │   ├── head_link1.STL
    │   │   │   │   ├── head_link2.STL
    │   │   │   │   ├── l_base_link1.STL
    │   │   │   │   ├── l_base_link.STL
    │   │   │   │   ├── l_hand_base_link.STL
    │   │   │   │   ├── Link_finger1.STL
    │   │   │   │   ├── Link_finger2.STL
    │   │   │   │   ├── link_left_wheel.STL
    │   │   │   │   ├── link_right_wheel.STL
    │   │   │   │   ├── link_swivel_wheel_1_1.STL
    │   │   │   │   ├── link_swivel_wheel_1_2.STL
    │   │   │   │   ├── link_swivel_wheel_2_1.STL
    │   │   │   │   ├── link_swivel_wheel_2_2.STL
    │   │   │   │   ├── link_swivel_wheel_3_1.STL
    │   │   │   │   ├── link_swivel_wheel_3_2.STL
    │   │   │   │   ├── link_swivel_wheel_4_1.STL
    │   │   │   │   ├── link_swivel_wheel_4_2.STL
    │   │   │   │   ├── l_link1.STL
    │   │   │   │   ├── l_link2.STL
    │   │   │   │   ├── l_link3.STL
    │   │   │   │   ├── l_link4.STL
    │   │   │   │   ├── l_link5.STL
    │   │   │   │   ├── l_link6.STL
    │   │   │   │   ├── platform_base_link.STL
    │   │   │   │   ├── r_base_link1.STL
    │   │   │   │   ├── r_base_link.STL
    │   │   │   │   ├── r_hand_base_link.STL
    │   │   │   │   ├── r_link1.STL
    │   │   │   │   ├── r_link2.STL
    │   │   │   │   ├── r_link3.STL
    │   │   │   │   ├── r_link4.STL
    │   │   │   │   ├── r_link5.STL
    │   │   │   │   └── r_link6.STL
    │   │   │   ├── package.xml
    │   │   │   └── urdf
    │   │   │       ├── body_head_platform_transmission.xacro
    │   │   │       ├── body_head_platform.urdf.xacro
    │   │   │       ├── common_gazebo.xacro
    │   │   │       ├── joint_rmg24.urdf.xacro
    │   │   │       ├── joint.urdf.xacro
    │   │   │       ├── left_hand_rmg24_transmission.xacro
    │   │   │       ├── left_hand_rmg24.urdf.xacro
    │   │   │       ├── left_hand.urdf.xacro
    │   │   │       ├── overseas_65_b_v_description.csv
    │   │   │       ├── overseas_65_b_v_description_rmg24.urdf.xacro
    │   │   │       ├── overseas_65_b_v_description.urdf
    │   │   │       ├── overseas_65_b_v_description.urdf.xacro
    │   │   │       ├── right_hand_rmg24_transmission.xacro
    │   │   │       ├── right_hand_rmg24.urdf.xacro
    │   │   │       ├── right_hand.urdf.xacro
    │   │   │       ├── rm65_b_v_left_transmission.xacro
    │   │   │       ├── rm65_b_v_left.urdf.xacro
    │   │   │       ├── rm65_b_v_right_transmission.xacro
    │   │   │       ├── rm65_b_v_right.urdf.xacro
    │   │   │       └── woosh_agv.urdf.xacro
    │   │   └── overseas_75_b_v_description
    │   │       ├── CMakeLists.txt
    │   │       ├── config
    │   │       │   └── joint_names_overseas_75_b_v_description.yaml
    │   │       ├── export.log
    │   │       ├── launch
    │   │       │   ├── display.launch
    │   │       │   ├── display_rmg24.launch
    │   │       │   └── gazebo.launch
    │   │       ├── meshes
    │   │       │   ├── base_link2.STL
    │   │       │   ├── base_link_underpan.STL
    │   │       │   ├── bl_Link.STL
    │   │       │   ├── body_base_link.STL
    │   │       │   ├── br_Link.STL
    │   │       │   ├── camera_link.STL
    │   │       │   ├── fl_Link.STL
    │   │       │   ├── fr_Link.STL
    │   │       │   ├── head_link1.STL
    │   │       │   ├── head_link2.STL
    │   │       │   ├── l_base_link1.STL
    │   │       │   ├── l_base_link.STL
    │   │       │   ├── l_hand_base_link.STL
    │   │       │   ├── l_hand_link.STL
    │   │       │   ├── Link_finger1.STL
    │   │       │   ├── Link_finger2.STL
    │   │       │   ├── link_left_wheel.STL
    │   │       │   ├── link_right_wheel.STL
    │   │       │   ├── link_swivel_wheel_1_1.STL
    │   │       │   ├── link_swivel_wheel_1_2.STL
    │   │       │   ├── link_swivel_wheel_2_1.STL
    │   │       │   ├── link_swivel_wheel_2_2.STL
    │   │       │   ├── link_swivel_wheel_3_1.STL
    │   │       │   ├── link_swivel_wheel_3_2.STL
    │   │       │   ├── link_swivel_wheel_4_1.STL
    │   │       │   ├── link_swivel_wheel_4_2.STL
    │   │       │   ├── l_link1.STL
    │   │       │   ├── l_link2.STL
    │   │       │   ├── l_link3.STL
    │   │       │   ├── l_link4.STL
    │   │       │   ├── l_link5.STL
    │   │       │   ├── l_link6.STL
    │   │       │   ├── l_link7.STL
    │   │       │   ├── platform_base_link.STL
    │   │       │   ├── r_base_link1.STL
    │   │       │   ├── r_base_link.STL
    │   │       │   ├── r_hand_base_link.STL
    │   │       │   ├── r_hand.STL
    │   │       │   ├── r_link1.STL
    │   │       │   ├── r_link2.STL
    │   │       │   ├── r_link3.STL
    │   │       │   ├── r_link4.STL
    │   │       │   ├── r_link5.STL
    │   │       │   ├── r_link6.STL
    │   │       │   └── r_link7.STL
    │   │       ├── no_gravity_world.world
    │   │       ├── package.xml
    │   │       └── urdf
    │   │           ├── body_head_platform_transmission.xacro
    │   │           ├── body_head_platform.urdf.xacro
    │   │           ├── common_gazebo.xacro
    │   │           ├── joint_rmg24.urdf.xacro
    │   │           ├── joint.urdf.xacro
    │   │           ├── left_hand_rmg24_transmission.xacro
    │   │           ├── left_hand_rmg24.urdf.xacro
    │   │           ├── left_hand.urdf.xacro
    │   │           ├── overseas_75_b_v_description.csv
    │   │           ├── overseas_75_b_v_description_rmg24.urdf.xacro
    │   │           ├── overseas_75_b_v_description.urdf
    │   │           ├── overseas_75_b_v_description.urdf.xacro
    │   │           ├── right_hand_rmg24_transmission.xacro
    │   │           ├── right_hand_rmg24.urdf.xacro
    │   │           ├── right_hand.urdf.xacro
    │   │           ├── rm75_b_v_left_transmission.xacro
    │   │           ├── rm75_b_v_left.urdf.xacro
    │   │           ├── rm75_b_v_right_transmission.xacro
    │   │           ├── rm75_b_v_right.urdf.xacro
    │   │           └── woosh_agv.urdf.xacro
    │   ├── dual_arm_gazebo
    │   │   ├── dual_65B_arm_gazebo
    │   │   │   ├── CMakeLists.txt
    │   │   │   ├── config
    │   │   │   │   ├── arm_gazebo_joint_states.yaml
    │   │   │   │   ├── rm_65_trajectory_control_rmg24.yaml
    │   │   │   │   └── rm_65_trajectory_control.yaml
    │   │   │   ├── launch
    │   │   │   │   ├── arm_65_bringup_moveit.launch
    │   │   │   │   ├── arm_65_bringup_moveit_rmg24.launch
    │   │   │   │   ├── arm_65_trajectory_controller.launch
    │   │   │   │   ├── arm_65_trajectory_controller_rmg24.launch
    │   │   │   │   ├── arm_gazebo_states.launch
    │   │   │   │   ├── arm_world.launch
    │   │   │   │   └── arm_world_rmg24.launch
    │   │   │   ├── package.xml
    │   │   │   └── worlds
    │   │   │       └── no_gravity_world.world
    │   │   └── dual_75B_arm_gazebo
    │   │       ├── CMakeLists.txt
    │   │       ├── config
    │   │       │   ├── arm_gazebo_joint_states.yaml
    │   │       │   ├── rm_75_trajectory_control_rmg24.yaml
    │   │       │   └── rm_75_trajectory_control.yaml
    │   │       ├── launch
    │   │       │   ├── arm_75_bringup_moveit.launch
    │   │       │   ├── arm_75_bringup_moveit_rmg24.launch
    │   │       │   ├── arm_75_trajectory_controller.launch
    │   │       │   ├── arm_75_trajectory_controller_rmg24.launch
    │   │       │   ├── arm_gazebo_states.launch
    │   │       │   ├── arm_world.launch
    │   │       │   └── arm_world_rmg24.launch
    │   │       ├── package.xml
    │   │       └── worlds
    │   │           └── no_gravity_world.world
    │   ├── dual_arm_moveit
    │   │   ├── dual_65B_arm_moveit_config
    │   │   │   ├── CMakeLists.txt
    │   │   │   ├── config
    │   │   │   │   ├── cartesian_limits.yaml
    │   │   │   │   ├── chomp_planning.yaml
    │   │   │   │   ├── controllers.yaml
    │   │   │   │   ├── fake_controllers.yaml
    │   │   │   │   ├── gazebo_controllers.yaml
    │   │   │   │   ├── gazebo_overseas_65_b_v_description.urdf
    │   │   │   │   ├── joint_limits.yaml
    │   │   │   │   ├── kinematics.yaml
    │   │   │   │   ├── ompl_planning.yaml
    │   │   │   │   ├── overseas_65_b_v_description.srdf
    │   │   │   │   ├── ros_controllers.yaml
    │   │   │   │   ├── sensors_3d.yaml
    │   │   │   │   ├── simple_moveit_controllers.yaml
    │   │   │   │   └── stomp_planning.yaml
    │   │   │   ├── launch
    │   │   │   │   ├── chomp_planning_pipeline.launch.xml
    │   │   │   │   ├── default_warehouse_db.launch
    │   │   │   │   ├── demo_gazebo.launch
    │   │   │   │   ├── demo.launch
    │   │   │   │   ├── demo_realrobot.launch
    │   │   │   │   ├── dual_65B_arm_robot_moveit_controller_manager.launch.xml
    │   │   │   │   ├── dual_65B_arm_robot_moveit_sensor_manager.launch.xml
    │   │   │   │   ├── fake_moveit_controller_manager.launch.xml
    │   │   │   │   ├── gazebo.launch
    │   │   │   │   ├── joystick_control.launch
    │   │   │   │   ├── move_group.launch
    │   │   │   │   ├── moveit_65B_planning_execution_gazebo.launch
    │   │   │   │   ├── moveit_planning_execution.launch
    │   │   │   │   ├── moveit.rviz
    │   │   │   │   ├── moveit_rviz.launch
    │   │   │   │   ├── ompl-chomp_planning_pipeline.launch.xml
    │   │   │   │   ├── ompl_planning_pipeline.launch.xml
    │   │   │   │   ├── overseas_65_b_v_description_moveit_sensor_manager.launch.xml
    │   │   │   │   ├── pilz_industrial_motion_planner_planning_pipeline.launch.xml
    │   │   │   │   ├── planning_context.launch
    │   │   │   │   ├── planning_pipeline.launch.xml
    │   │   │   │   ├── ros_controllers.launch
    │   │   │   │   ├── ros_control_moveit_controller_manager.launch.xml
    │   │   │   │   ├── run_benchmark_ompl.launch
    │   │   │   │   ├── sensor_manager.launch.xml
    │   │   │   │   ├── setup_assistant.launch
    │   │   │   │   ├── simple_moveit_controller_manager.launch.xml
    │   │   │   │   ├── stomp_planning_pipeline.launch.xml
    │   │   │   │   ├── trajectory_execution.launch.xml
    │   │   │   │   ├── warehouse.launch
    │   │   │   │   └── warehouse_settings.launch.xml
    │   │   │   └── package.xml
    │   │   ├── dual_65B_arm_rmg24_moveit_config
    │   │   │   ├── CMakeLists.txt
    │   │   │   ├── config
    │   │   │   │   ├── cartesian_limits.yaml
    │   │   │   │   ├── chomp_planning.yaml
    │   │   │   │   ├── controllers.yaml
    │   │   │   │   ├── fake_controllers.yaml
    │   │   │   │   ├── gazebo_controllers.yaml
    │   │   │   │   ├── gazebo_overseas_65_b_v_description.urdf
    │   │   │   │   ├── joint_limits.yaml
    │   │   │   │   ├── kinematics.yaml
    │   │   │   │   ├── ompl_planning.yaml
    │   │   │   │   ├── overseas_65_b_v_description.srdf
    │   │   │   │   ├── ros_controllers.yaml
    │   │   │   │   ├── sensors_3d.yaml
    │   │   │   │   ├── simple_moveit_controllers.yaml
    │   │   │   │   └── stomp_planning.yaml
    │   │   │   ├── launch
    │   │   │   │   ├── chomp_planning_pipeline.launch.xml
    │   │   │   │   ├── default_warehouse_db.launch
    │   │   │   │   ├── demo_gazebo.launch
    │   │   │   │   ├── demo.launch
    │   │   │   │   ├── demo_realrobot.launch
    │   │   │   │   ├── dual_65B_arm_robot_moveit_controller_manager.launch.xml
    │   │   │   │   ├── fake_moveit_controller_manager.launch.xml
    │   │   │   │   ├── gazebo.launch
    │   │   │   │   ├── joystick_control.launch
    │   │   │   │   ├── move_group.launch
    │   │   │   │   ├── moveit_65B_planning_execution_gazebo.launch
    │   │   │   │   ├── moveit_planning_execution.launch
    │   │   │   │   ├── moveit.rviz
    │   │   │   │   ├── moveit_rviz.launch
    │   │   │   │   ├── ompl-chomp_planning_pipeline.launch.xml
    │   │   │   │   ├── ompl_planning_pipeline.launch.xml
    │   │   │   │   ├── overseas_65_b_v_description_moveit_sensor_manager.launch.xml
    │   │   │   │   ├── pilz_industrial_motion_planner_planning_pipeline.launch.xml
    │   │   │   │   ├── planning_context.launch
    │   │   │   │   ├── planning_pipeline.launch.xml
    │   │   │   │   ├── ros_controllers.launch
    │   │   │   │   ├── ros_control_moveit_controller_manager.launch.xml
    │   │   │   │   ├── run_benchmark_ompl.launch
    │   │   │   │   ├── sensor_manager.launch.xml
    │   │   │   │   ├── setup_assistant.launch
    │   │   │   │   ├── simple_moveit_controller_manager.launch.xml
    │   │   │   │   ├── stomp_planning_pipeline.launch.xml
    │   │   │   │   ├── trajectory_execution.launch.xml
    │   │   │   │   ├── warehouse.launch
    │   │   │   │   └── warehouse_settings.launch.xml
    │   │   │   └── package.xml
    │   │   ├── dual_75B_arm_moveit_config
    │   │   │   ├── CMakeLists.txt
    │   │   │   ├── config
    │   │   │   │   ├── cartesian_limits.yaml
    │   │   │   │   ├── chomp_planning.yaml
    │   │   │   │   ├── controllers.yaml
    │   │   │   │   ├── fake_controllers.yaml
    │   │   │   │   ├── gazebo_controllers.yaml
    │   │   │   │   ├── gazebo_overseas_75_b_v_description.urdf
    │   │   │   │   ├── joint_limits.yaml
    │   │   │   │   ├── kinematics.yaml
    │   │   │   │   ├── ompl_planning.yaml
    │   │   │   │   ├── overseas_75_b_v_description.srdf
    │   │   │   │   ├── ros_controllers.yaml
    │   │   │   │   ├── sensors_3d.yaml
    │   │   │   │   ├── simple_moveit_controllers.yaml
    │   │   │   │   └── stomp_planning.yaml
    │   │   │   ├── launch
    │   │   │   │   ├── chomp_planning_pipeline.launch.xml
    │   │   │   │   ├── default_warehouse_db.launch
    │   │   │   │   ├── demo_gazebo.launch
    │   │   │   │   ├── demo.launch
    │   │   │   │   ├── demo_realrobot.launch
    │   │   │   │   ├── dual_75B_arm_robot_moveit_controller_manager.launch.xml
    │   │   │   │   ├── dual_75B_arm_robot_moveit_sensor_manager.launch.xml
    │   │   │   │   ├── fake_moveit_controller_manager.launch.xml
    │   │   │   │   ├── gazebo.launch
    │   │   │   │   ├── joystick_control.launch
    │   │   │   │   ├── move_group.launch
    │   │   │   │   ├── moveit_75B_planning_execution_gazebo.launch
    │   │   │   │   ├── moveit_planning_execution.launch
    │   │   │   │   ├── moveit.rviz
    │   │   │   │   ├── moveit_rviz.launch
    │   │   │   │   ├── ompl-chomp_planning_pipeline.launch.xml
    │   │   │   │   ├── ompl_planning_pipeline.launch.xml
    │   │   │   │   ├── overseas_75_b_v_description_moveit_sensor_manager.launch.xml
    │   │   │   │   ├── pilz_industrial_motion_planner_planning_pipeline.launch.xml
    │   │   │   │   ├── planning_context.launch
    │   │   │   │   ├── planning_pipeline.launch.xml
    │   │   │   │   ├── ros_controllers.launch
    │   │   │   │   ├── ros_control_moveit_controller_manager.launch.xml
    │   │   │   │   ├── run_benchmark_ompl.launch
    │   │   │   │   ├── sensor_manager.launch.xml
    │   │   │   │   ├── setup_assistant.launch
    │   │   │   │   ├── simple_moveit_controller_manager.launch.xml
    │   │   │   │   ├── stomp_planning_pipeline.launch.xml
    │   │   │   │   ├── trajectory_execution.launch.xml
    │   │   │   │   ├── warehouse.launch
    │   │   │   │   └── warehouse_settings.launch.xml
    │   │   │   └── package.xml
    │   │   └── dual_75B_arm_rmg24_moveit_config
    │   │       ├── CMakeLists.txt
    │   │       ├── config
    │   │       │   ├── cartesian_limits.yaml
    │   │       │   ├── chomp_planning.yaml
    │   │       │   ├── controllers.yaml
    │   │       │   ├── fake_controllers.yaml
    │   │       │   ├── gazebo_controllers.yaml
    │   │       │   ├── gazebo_overseas_75_b_v_description.urdf
    │   │       │   ├── joint_limits.yaml
    │   │       │   ├── kinematics.yaml
    │   │       │   ├── ompl_planning.yaml
    │   │       │   ├── overseas_75_b_v_description.srdf
    │   │       │   ├── ros_controllers.yaml
    │   │       │   ├── sensors_3d.yaml
    │   │       │   ├── simple_moveit_controllers.yaml
    │   │       │   └── stomp_planning.yaml
    │   │       ├── launch
    │   │       │   ├── chomp_planning_pipeline.launch.xml
    │   │       │   ├── default_warehouse_db.launch
    │   │       │   ├── demo_gazebo.launch
    │   │       │   ├── demo.launch
    │   │       │   ├── demo_realrobot.launch
    │   │       │   ├── dual_75B_arm_robot_moveit_controller_manager.launch.xml
    │   │       │   ├── fake_moveit_controller_manager.launch.xml
    │   │       │   ├── gazebo.launch
    │   │       │   ├── joystick_control.launch
    │   │       │   ├── move_group.launch
    │   │       │   ├── moveit_75B_planning_execution_gazebo.launch
    │   │       │   ├── moveit_planning_execution.launch
    │   │       │   ├── moveit.rviz
    │   │       │   ├── moveit_rviz.launch
    │   │       │   ├── ompl-chomp_planning_pipeline.launch.xml
    │   │       │   ├── ompl_planning_pipeline.launch.xml
    │   │       │   ├── overseas_75_b_v_description_moveit_sensor_manager.launch.xml
    │   │       │   ├── pilz_industrial_motion_planner_planning_pipeline.launch.xml
    │   │       │   ├── planning_context.launch
    │   │       │   ├── planning_pipeline.launch.xml
    │   │       │   ├── ros_controllers.launch
    │   │       │   ├── ros_control_moveit_controller_manager.launch.xml
    │   │       │   ├── run_benchmark_ompl.launch
    │   │       │   ├── sensor_manager.launch.xml
    │   │       │   ├── setup_assistant.launch
    │   │       │   ├── simple_moveit_controller_manager.launch.xml
    │   │       │   ├── stomp_planning_pipeline.launch.xml
    │   │       │   ├── trajectory_execution.launch.xml
    │   │       │   ├── warehouse.launch
    │   │       │   └── warehouse_settings.launch.xml
    │   │       └── package.xml
    │   └── dual_arm_msgs
    │       ├── CMakeLists.txt
    │       ├── msg
    │       │   ├── Arm_Analog_Output.msg
    │       │   ├── Arm_Current_State copy.msg
    │       │   ├── Arm_Current_State.msg
    │       │   ├── Arm_Digital_Output.msg
    │       │   ├── Arm_IO_State.msg
    │       │   ├── Arm_Joint_Speed_Max.msg
    │       │   ├── Arm_Pose_Euler.msg
    │       │   ├── Arm_Software_Version.msg
    │       │   ├── ArmState.msg
    │       │   ├── Cabinet.msg
    │       │   ├── CarteFdPose.msg
    │       │   ├── CartePos.msg
    │       │   ├── ChangeTool_Name.msg
    │       │   ├── ChangeTool_State.msg
    │       │   ├── ChangeWorkFrame_Name.msg
    │       │   ├── ChangeWorkFrame_State.msg
    │       │   ├── Force_Position_Move_Joint.msg
    │       │   ├── Force_Position_Move_Pose.msg
    │       │   ├── Force_Position_State.msg
    │       │   ├── GetArmState_Command copy.msg
    │       │   ├── GetArmState_Command.msg
    │       │   ├── Gripper_Pick.msg
    │       │   ├── Gripper_Set.msg
    │       │   ├── Hand_Angle.msg
    │       │   ├── Hand_Force.msg
    │       │   ├── Hand_Posture.msg
    │       │   ├── Hand_Seq.msg
    │       │   ├── Hand_Speed.msg
    │       │   ├── IO_Update.msg
    │       │   ├── Joint_Current.msg
    │       │   ├── Joint_Enable.msg
    │       │   ├── Joint_Error_Code.msg
    │       │   ├── Joint_Max_Speed.msg
    │       │   ├── JointPos.msg
    │       │   ├── Joint_Step.msg
    │       │   ├── Joint_Teach.msg
    │       │   ├── Lift_Height.msg
    │       │   ├── Lift_Speed.msg
    │       │   ├── LiftState.msg
    │       │   ├── Manual_Set_Force_Pose.msg
    │       │   ├── MoveC.msg
    │       │   ├── MoveJ.msg
    │       │   ├── MoveJ_P.msg
    │       │   ├── MoveJ_PO.msg
    │       │   ├── MoveL.msg
    │       │   ├── Ort_Teach.msg
    │       │   ├── Plan_State.msg
    │       │   ├── Pos_Teach.msg
    │       │   ├── Servo_GetAngle.msg
    │       │   ├── Servo_Move.msg
    │       │   ├── Set_Force_Position.msg
    │       │   ├── Set_Realtime_Push.msg
    │       │   ├── Six_Force.msg
    │       │   ├── Socket_Command.msg
    │       │   ├── Start_Multi_Drag_Teach.msg
    │       │   ├── Stop.msg
    │       │   ├── Stop_Teach.msg
    │       │   ├── Tool_Analog_Output.msg
    │       │   ├── Tool_Digital_Output.msg
    │       │   ├── Tool_IO_State.msg
    │       │   └── Turtle_Driver.msg
    │       └── package.xml
    ├── dual_arm_robot_demo 示例ros包
    │   ├── CMakeLists.txt
    │   ├── launch
    │   │   ├── dual_arm_65_robot_start.launch
    │   │   ├── dual_arm_75_robot_start.launch
    │   │   ├── startlaunch_grippers.launch
    │   │   └── startlaunch_hand.launch
    │   ├── msg
    │   │   └── ObjectPose.msg
    │   ├── package.xml
    │   └── scripts
    │       ├── aoyi_hand.py
    │       ├── camera_pose.py
    │       ├── catch2object_grippers.py
    │       ├── catch2object_hand.py
    │       ├── config.yaml
    │       ├── detect_object.py
    │       ├── __init__.py
    │       ├── lift_control.py
    │       ├── pose_udp.py
    │       ├── use_65_demo_all.py
    │       ├── use_75_demo_all.py
    │       └── weight
    │           └── best.pt
    └── servo_control 舵机ros包
        ├── README_SERVO.en.md
        ├── README_SERVO.md
        ├── servo_demo
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   └── scripts
        │       └── servo_control_demo.py
        └── servo_ros
            ├── CMakeLists.txt
            ├── launch
            │   └── servo_start.launch
            ├── msg
            │   ├── ServoAngle.msg
            │   └── ServoMove.msg
            ├── package.xml
            └── src
                └── servo_controller.cpp

```



## **四.编译方法**

    1.cd rmc_aida_l_ros1
    2.catkin build dual_arm_msgs
    3.catkin build



## 五.运行指令



### 5.1**启动具身双臂机器人机械臂ROS控制节点**：

- `cd rmc_aida_l_ros1`
- `source devel/setup.bash`
- `roslaunch arm_driver dual_arm_<type>_driver.launch`     #  type 为 65 或 75



### 5.2 启动具身双臂机器人整体案例

#### 5.2.1 具身机器人整体部件活动案例（机械臂、舵机、`USB`相机、`D435`相机、底盘、升降）



- `cd rmc_aida_l_ros1`
- `source devel/setup.bash`
- `roslaunch dual_arm_robot_demo dual_arm_<type>_robot_start.launch`     #  type 为 65 或 75

e.g. 如果具身机器人双臂是65臂的话 ，运行指令

```
roslaunch dual_arm_robot_demo dual_arm_65_robot_start.launch
```

dual_arm_65_robot_start.launch内容如下：

```
<launch>

    <!-- 包含 servo_start.launch 启动舵机-->
    <include file="$(find servo_ros)/launch/servo_start.launch"/>



    <!-- 包含 dual_arm_driver.launch 启动机械臂-->
    <include file="$(find arm_driver)/launch/dual_arm_65_driver.launch"/>



    <!-- 包含 d435_pub.launch 启动d435相机 其中一个相机-->
    <include file="$(find d435_demo)/launch/d435_pub.launch">
            <arg name="idx" value="0"/>
    </include>


    <!-- 包含 usb_camera_demo.launch 启动usb相机 -->
    <include file="$(find usb_camera_demo)/launch/usb_camera_pub.launch">
            <arg name="idx" value="0"/>
    </include>


    <!-- 包含 底盘导航-->
    <node pkg="agv_demo" name="agv_demo" type="agv_demo.py" cwd="node" output="screen"/> 


    <!-- 整体部件运动 -->
    <node pkg="dual_arm_robot_demo" name="robot_demo" type="use_65_demo_all.py" cwd="node" output="screen"/> 
    

</launch>


```

注意，运行整体案例前，如果需要在案例中使用底盘，则需要使用底盘建图软件建立地图点位"A"，相关底盘软件和使用教程请参考  [底盘相关资料](.//src/agv_demo/底盘说明文件)

如果不使用底盘，请在上面launch文件中注释这行

>​    <node pkg="agv_demo" name="agv_demo" type="agv_demo.py" cwd="node" output="screen"/> 



#### 5.2.2 具身机器人移动识别抓取案例

当前案例使用的末端工具是傲意`Ohand`灵巧手/`RMG24`夹爪

##### 5.2.2.1 案例运行前准备

1. 设置相机设备ID

   查看是否安装了`realsense-viewer` ，如果安装，跳过下面安装步骤

   - 安装`realsense-viewer` 

     **注册服务器的公钥**

     ```
     sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
     sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
     
     ```

     **将服务器添加到存储库列表中**

     ```
     sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
     
     ```

     **安装SDK2**

     ```
     sudo apt-get install librealsense2-utils
     sudo apt-get install librealsense2-dev 
     ```

     **说明：如果在非Jetson Xavier NX设备上，建议用以下方式安装**

     ```
     sudo apt-get install librealsense2-dkms
     sudo apt-get install librealsense2-utils
     ```

   - 查看机械臂右臂d435相机ID

     打开机器人主机terminal，输入:

     ```4
     realsense-viewer
     ```

     在显示的界面上点击Add Source选择D435相机

     ![image-20241224152610093](./images/image-20241224152610093.png)

     ​													图2 刚打开的`realsense-viewer`

     打开各个D435相机的RGB Camera，根据打开的相机界面，判断哪个是机械臂右臂上的相机

     ![image-20241224154959640](./images/image-20241224154959640.png)

     ​                                                                                                                                   图3 

     选择右臂相机的ID

     ![image-20241224155254773](./images/image-20241224155254773.png)

     ​                                                                                                                        图4
     
     将右臂相机的ID填入到[配置文件](src/dual_arm_robot_demo/scripts/config.yaml)
     
     ![image-20241224155839852](./images/image-20241224155839852.png)
     
     ​                                                                                                                       图5
     
     

2. 将瓶子放置在一个固定平台上（桌子或者其他平台）

​	![IMG_20241224_195748](./images/IMG_20241224_195748.jpg)

​                                                                                                             图6 抓取前具身机器人和瓶子的放置位置



3. 让右臂处于识别位姿

​	将机器人的正面对准瓶子，打开一个终端，执行下面命令启动机械臂driver：

```
source devel/setup.bash
roslaunch arm_driver dual_arm_65_driver.launch
```

​	打开另外一个终端，执行下面命令让机械臂到达相机拍照的姿态：

```
rosrun dual_arm_robot_demo camera_pose.py
```

​	此时机械臂右臂的姿态如图（图片里的末端工具是`RMG24`平行夹爪）：

![f299091b8c60ca1b51d483f395200003](./images/f299091b8c60ca1b51d483f395200003.jpg)

​                                                                                                                图7 具身机器人识别物体时机械臂位姿



4.打开相机识别模型：

打开一个终端，执行下面命令：

```
source devel/setup.bash
rosrun dual_arm_robot_demo detect_object.py
```

​	此时弹出一个视觉检测界面，如下图：

![1735039999963](./images/1735039999963.jpg)

​														 图8  识别界面



​        

5.调整机器人升降高度

如果第四步瓶子已经出现在视觉检测页面里，则不需要操作这一步，否则，执行下面命令来调整升降高度来使瓶子出现在视觉检测页面里：

```
source devel/setup.bash
rosrun dual_arm_robot_demo lift_control.py
```

##### 5.2.2.2 运行案例

**关闭**上面的几个打开的服务

执行下面命令执行整个抓取demo

末端工具为`RMG24`夹爪，执行下面程序：

```
source devel/setup.bash
roslaunch dual_arm_robot_demo startlaunch_grippers.launch
```

末端工具为傲意灵巧手，执行下面程序：

```
source devel/setup.bash
roslaunch dual_arm_robot_demo startlaunch_hand.launch
```



如果需要在案例中使用底盘，则需要使用底盘建图软件建立地图点位"A"（建立底盘点位时A机器人的方向需要使机器人面向瓶子，并且建立`dianA`时后需要机器人需要执行[**5.2.2.1 案例运行前准备**](#5.2.2.1 案例运行前准备)，相关底盘软件和使用教程请参考  [底盘相关资料](.//src/agv_demo/底盘说明文件)

并且**取消注释**`startlaunch.launch`中运行底盘指令的命令：

```
<launch>
    <!-- 包含 底盘导航-->
    <node pkg="agv_demo" name="agv_demo" type="agv_demo.py" cwd="node" output="screen"/> 
    
    <include file="$(find arm_driver)/launch/dual_arm_65_driver.launch"/>
    <env name="PYTHONPATH" value="$(find dual_arm_robot_demo)/scripts:$(env PYTHONPATH)"/>
    <node pkg="tf2_ros" type="static_transform_publisher" name="camera" args="-0.0974 0.041261 -0.005  0.04798 -0.0376 -0.677 0.73  /end /camera" output="screen" />
    <node pkg="dual_arm_robot_demo" name="model_detect" type="detect_object.py" cwd="node" output="screen"/>
    <node pkg="dual_arm_robot_demo" name="pose_pub" type="pose_udp.py" cwd="node" output="screen"/>
    <node pkg="dual_arm_robot_demo" name="catch" type="" cwd="node" output="screen"/>
</launch>

```



### 5.3**在`rviz`中显示机器人模型**



```shell
cd embodied_robot

source devel/setup.bash

roslaunch overseas_{65,75}_b_v_description display[_rmg24].launch  
```



其中<arm_type>可更换为当前的机械臂型号类型 65或者75，如果具身机器人使用的是65臂的话，使用如下指令:

- 末端工具为RMG24夹爪时：

  ```
  roslaunch overseas_65_b_v_description display_rmg24.launch
  ```


- 末端工具为傲翼™灵巧手时：

  ```
  roslaunch overseas_65_b_v_description display.launch
  ```

  

正常情况下如图9所示（末端工具傲翼™灵巧手）：
![image-20241115170248370](./pictures/image-20241115170248370.png)

​                                                                                                   图9 在`rviz`中显示具身双臂机器人

如果 `rviz` 中未显示模型，则手动修改“Fixed Frame”为“body_base_link”，然后点击左侧下方的 Add 按钮在弹出的界面中找到“`RobotModel`”添加即可



### 5.4 **运行 具身双臂机器人的`MoveIt!`演示demo** 



打开终端进入工作空间执行以下命令运行具身机器人的`MoveIt!`演示 demo：

```shell
cd embodied_robot

source devel/setup.bash

roslaunch dual_{65,75}B_arm[_rmg24]_moveit_config demo.launch
```

如在具身双臂机器人使用 `RM65 `机械臂时:

- 末端工具为RMG24夹爪：

  ```
  roslaunch dual_65B_arm_rmg24_moveit_config demo.launch
  ```

- 末端工具为傲翼™灵巧手：

  ```
  roslaunch dual_65B_arm_moveit_config demo.launch
  ```

启动成功后，可以看到如图所示的界面：

![image-20241115183544803](./pictures/image-20241115183544803.png)

​									                                 图10 `MoveIt! `demo的启动界面



**拖动规划**

拖动机械臂的前端，可以改变机械臂的姿态。然后在 Planning 标签页中点击“Plan & Execute”按钮，`MoveIt!`开始规划路径，并且控制机器人向目标位置移

动，从右侧界面可以看到机器人运动的全部过程。



![image-20241115183720521](./pictures/image-20241115183720521.png)

​                                                                                                                图11 拖动规划的运动效果

**选择目标姿态规划**

在 Planning 标签页中点击 Goal State 的下拉列表可以选择机械臂的目标姿态，然后点击“Plan & Execute”按钮，`MoveIt!`开始规划路径，并且控制机器人向

目标位置移动。



### 5.5  **使用** **`MoveIt!`控制Gazebo** **中的机械臂**

执行以下命令运行 `MoveIt!`和 Gazebo:

```shell

cd embodied_robot

source devel/setup.bash

roslaunch dual_{65,75}B_arm_gazebo arm_{65,75}_bringup_moveit[_rmg24].launch 

```

当具身双臂机器人使用的是`RM65` 机械臂时:

- 末端工具为RMG24夹爪：

  ```
  roslaunch dual_65B_arm_gazebo arm_65_bringup_moveit_rmg24.launch 
  ```

- 末端工具为傲翼™灵巧手：

  ```
  roslaunch dual_65B_arm_gazebo arm_65_bringup_moveit.launch 
  ```

  

启动后打开的 Gazebo 如图12：

![image-20241115180100361](./pictures/image-20241115180100361.png)

​                                                                                               图12 具身双臂机器人在Gazebo中显示效果

​                                                              

启动后打开的 `rviz `如图13所示：

![image-20241115180136399](./pictures/image-20241115180136399.png)

​								                                                  图13 `rviz`中显示具身双臂机器人

接下来使用 `MoveIt!`规划运动的几种方式就可以控制 Gazebo 中的机器人了，例如图14拖动机器人机械臂末端到一个位置，然后点击“Plan & Execute”按钮，可以看到`rviz`中机器人开始规划执行并且可以看到Gazebo中的机器人开始运动且与`rviz `中的机器人模型保持一致，如图 15所示。

![image-20241115180609419](./pictures/image-20241115180609419.png)

​                                                                                                          图14 使用`MoveIt!`拖动规划执行

![image-20241115180726522](./pictures/image-20241115180726522.png)

​								                             图15 Gazebo中机器人按`rviz`规划同步执行效果



### 5.6使用 `MoveIt!`控制真实机械臂

执行以下命令运行 `MoveIt!`和机械臂:

```shell
cd rmc_aida_l_ros1

source devel/setup.bash

roslaunch dual_{65,75}B_arm[_rmg24]_moveit_config moveit_planning_execution.launch 

```



当具身机器人是`RM65 `机械臂时，需要运行如下指令。

- 末端工具为RMG24夹爪:

  ```
  roslaunch dual_65B_arm_rmg24_moveit_config moveit_planning_execution.launch 
  ```

- 末端工具为傲翼™灵巧手：

  ```
  roslaunch dual_65B_arm_moveit_config moveit_planning_execution.launch 
  ```

  ​                                                              

启动后打开的 `rviz `如图16所示：

![image-20241115180136399](./pictures/image-20241115180136399.png)

​                                                                                                              图16 `rviz`中显示具身双臂机器人

接下来使用 `MoveIt!`规划运动的几种方式就可以控制 Gazebo 中的机器人了，例如图17拖动机器人末端到一个位置，然后点击“Plan & Execute”按钮，可以看到具身双臂机器人的真实机械臂开始执行，到达和`moveIt!`中机械臂一样的位置。

![	](./pictures/image-20241115180609419.png)

​                                                                                                         图17 使用`MoveIt!`拖动规划执行



## 六.版本更新

| 修订版本 |          内容更新          |  生效日期  |
| :------: | :------------------------: | :--------: |
|  V1.0.0  |        首次提交代码        | 2024-12-07 |
|  V1.1.0  |        增加抓取demo        | 2024-12-26 |
|  V1.1.1  |       增加README.md        | 2024-12-27 |
|  V1.1.2  | d435相机和底盘话题冲突问题 | 2025-01-09 |
|  V1.2.0  |      增加自研夹爪仿真      | 2025-02-09 |

