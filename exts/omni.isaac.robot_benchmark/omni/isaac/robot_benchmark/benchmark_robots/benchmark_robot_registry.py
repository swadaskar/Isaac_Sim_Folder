# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from . import BenchmarkFrankaLoader, BenchmarkUR10Loader, BaseBenchmarkRobotLoader
import carb


class BenchmarkRobotRegistry:
    _instance = None
    robot_loaders = None

    def __new__(self):
        """
        Make BenchmarkRobotRegistry a singleton class so that the user may instantiate and modify it in the /user folder
        """
        if self._instance is None:
            self._instance = object.__new__(self)
        return self._instance

    def __init__(self):
        # Names of key must match arg to robot_loader
        # The name given to a RobotLoader here will appear on the drop-down menu in RobotBenchmark
        if self.robot_loaders is None:
            self.robot_loaders = dict()

            self.register_robot("Franka", BenchmarkFrankaLoader("Franka"))
            self.register_robot("UR10", BenchmarkUR10Loader("UR10"))

    def get_robot_options(self):
        return self.robot_loaders

    def get_robot_loader(self, robot_name):
        return self.robot_loaders[robot_name]

    def register_robot(self, robot_name: str, robot_loader: BaseBenchmarkRobotLoader):
        """This function may be called from a script in the robot_benchmark/user folder
        It allows the override of an existing robot loader by name, so be careful

        Args:
            robot_name (str): _description_
            robot_loader (BaseBenchmarkRobotLoader): _description_
        """
        if robot_name in self.robot_loaders:
            carb.log_warn("Overriding the robot loader with key {} in BenchmarkRobotRegistry".format(robot_name))

        self.robot_loaders[robot_name] = robot_loader
