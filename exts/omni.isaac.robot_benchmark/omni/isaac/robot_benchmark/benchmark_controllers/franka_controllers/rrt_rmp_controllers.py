# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.motion_generation.lula import RRT, RmpFlow
from omni.isaac.motion_generation import interface_config_loader

from omni.isaac.robot_benchmark.benchmark_controllers import RrtRmpCarrotController


class FrankaRrtRmpCarrotController(RrtRmpCarrotController):
    def load_rrt(self):
        rrt_config = interface_config_loader.load_supported_path_planner_config("Franka", "RRT")
        rrt = RRT(**rrt_config)

        return rrt

    def load_rmp(self):
        rmp_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
        rmp_flow = RmpFlow(**rmp_config)
        return rmp_flow
