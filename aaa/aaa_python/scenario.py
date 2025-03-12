# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
from isaacsim.core.api.objects import GroundPlane, FixedCuboid
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.types import ArticulationActions
from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy

class AuboControlExampleScript:
    def __init__(self):
        self._articulation = None
        self._ground_plane = None
        self._script_generator = None

    def load_example_assets(self):
        # 添加机器人
        robot_prim_path = "/World/robot"
        path_to_robot_usd = "/home/ros/Downloads/Usd/copy.usd"
        add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        self._articulation = Articulation(robot_prim_path)

        # 添加地面
        self._ground_plane = GroundPlane("/World/Ground")

        # 添加障碍物
        self._obstacle = FixedCuboid(prim_path="/World/obstacle", position=[-0.5, 0.0, 0.25], scale=[0.1, 1, 0.5], size=1)

        # 返回添加到舞台的资产，以便它们可以注册到 core.World 中
        return self._articulation, self._ground_plane, self._obstacle

    def setup(self):
        # 设置合适的相机视角
        set_camera_view(eye=[0.1/1.5, 5/1.5, 1/1.5], target=[-1, -1, 0], camera_prim_path="/OmniverseKit_Persp") 

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
        # 机器人目标位置 [0.0, -1.23, 1.75, 0.66, 0.08, -0.14]
        action = ArticulationActions(joint_positions=np.array([-1.57, -0.57, 1.05, 0.66, 0.08, -0.14]))    
        self._articulation.apply_action(action)

        yield
        