# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.motion_generation.lula import RRT
from omni.isaac.motion_generation import interface_config_loader
from omni.isaac.franka import KinematicsSolver

from omni.isaac.robot_benchmark.benchmark_controllers import RrtLinearInterpolationController


class FrankaRrtLinearInterpolationController(RrtLinearInterpolationController):
    def load_rrt(self):
        rrt_config = interface_config_loader.load_supported_path_planner_config("Franka", "RRT")
        rrt = RRT(**rrt_config)
        return rrt

    def load_articulation_kinematics_solver(self, robot_articulation):
        rrt_config = interface_config_loader.load_supported_path_planner_config("Franka", "RRT")
        solver = KinematicsSolver(robot_articulation, rrt_config["end_effector_frame_name"])

        return solver
