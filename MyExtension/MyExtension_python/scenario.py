# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid, GroundPlane
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils import distance_metrics
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats, quats_to_rot_matrices
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot_motion.motion_generation import ArticulationMotionPolicy, RmpFlow
from isaacsim.robot_motion.motion_generation.interface_config_loader import load_supported_motion_policy_config
from isaacsim.storage.native import get_assets_root_path


class VehicleControlExampleScript:
    def __init__(self):
        self._articulation = None
        self._ground_plane = None
        self._script_generator = None

    def load_example_assets(self):
        # 添加车辆
        vehicle_prim_path = "/vehicle"
        path_to_vehicle_usd = "/home/ros/isaacsim_assets/Assets/Isaac/4.5/Isaac/Robots/AgilexRobotics/limo/limo.usd"
        add_reference_to_stage(path_to_vehicle_usd, vehicle_prim_path)

        self._articulation = SingleArticulation(vehicle_prim_path)

        # 添加地面
        self._ground_plane = GroundPlane("/World/Ground")

        # 返回添加到舞台的资产，以便它们可以注册到 core.World 中
        return self._articulation, self._ground_plane

    def setup(self):
        # 设置合适的相机视角
        set_camera_view(eye=[1, 0.4, 0.5], target=[0, 0, 0], camera_prim_path="/OmniverseKit_Persp")

        # 创建一个脚本生成器来执行 my_script()
        self._script_generator = self.my_script()

    def reset(self):
        # 通过重新创建生成器来重新开始脚本
        self._script_generator = self.my_script()

    def update(self, step: float):
        if self._script_generator is None:
            self._script_generator = self.my_script()
        try:
            result = next(self._script_generator)
        except StopIteration:
            return True
        return False

    def my_script(self):
        while True:
            # 向前移动2秒
            yield from self.move_vehicle(3, forward=True)

            # 向后移动1秒
            yield from self.move_vehicle(3, forward=False)

    def move_vehicle(self, duration, forward=True):
        # 设置移动的方向和速度
        velocity = 10 if forward else -10

        velocities = {
            'front_left_wheel': velocity,
            'front_right_wheel': velocity,
            'rear_left_wheel': velocity,
            'rear_right_wheel': velocity
        }

        # 获取关节状态
        joints_state = self._articulation.get_joints_state()
        
        # 打印关节状态以进行调试
        print("Joint positions:", joints_state.positions)
        print("Joint velocities:", joints_state.velocities)

        # 获取所有关节的名称和索引
        joint_names = self._articulation.dof_names
        print("Joint names and indices:", list(enumerate(joint_names)))

        # 设置每个轮子关节的速度
        joint_velocities = np.zeros(len(joint_names))
        for joint_name, velocity in velocities.items():
            if joint_name in joint_names:
                joint_index = joint_names.index(joint_name)
                joint_velocities[joint_index] = velocity
                print(f"Setting velocity: {velocity} for '{joint_name}' at index {joint_index}")
            else:
                print(f"Joint '{joint_name}' not found in articulation.")

        self._articulation.set_joint_velocities(joint_velocities)
        
        for _ in range(int(duration * 60)):  # 持续移动指定时间
            yield
        
        # 时间到了，将所有轮子速度设置为零
        self._articulation.set_joint_velocities(np.zeros(len(joint_names)))
        print("All wheel velocities set to zero.")










