# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.robot_benchmark.benchmark_controllers import BaseBenchmarkController

from typing import Sequence
from collections.abc import Callable

import carb


class BaseBenchmarkRobotLoader:
    """
    The BaseBenchmarkRobotLoader class handles a set of functions that omni.isaac.robot_benchmark extension uses to populate the
    drop-down menu in the RobotBenchmark extension UI, and to load the robot and controllers at the appropriate times in the 
    robot_benchmark extension.  
    """

    def __init__(self, name, **robot_kwargs):
        self._name = name

        self._robot_kwargs = robot_kwargs

        self.benchmark_controllers = dict()
        self.benchmark_controller_kwargs = dict()

    def load_robot(self, prim_path: str):
        """Load a robot and place is on the USD stage at the specified prim_path
        The self._robot_kwargs keyword arguments should be used appropriately in the implemented load function.
            i.e. When a RobotLoader is initialized, keyword arguments may be passed to the __init__ function;
                these keyword arguments are stored, and should be used here in the user-defined load function

        Args:
            prim_path (str): Path at which the USD robot will be placed on the stage.  RobotBenchmark will always pass
                the path "/Robot"
        """
        return

    def get_name(self) -> str:
        """Return the name of this RobotLoader

        Returns:
            str: Name of this RobotLoader
        """
        return self._name

    def load_controller(self, controller_name: str) -> BaseBenchmarkController:
        """Load a registered controller by the name that was passed to the register_controller function

        Args:
            controller_name (str): name under which the controller was registered
        """
        if controller_name in self.benchmark_controllers:
            return self.benchmark_controllers[controller_name](**self.benchmark_controller_kwargs[controller_name])
        else:
            carb.log_error("Name " + controller_name + " not found in " + str(list(self.benchmark_controllers.keys())))
            return None

    def get_benchmark_controller_names(self) -> Sequence[str]:
        """Get a list of all the controllers that have been registered under this robot

        Returns:
            Sequence[str]: List of controller names
        """
        return list(self.benchmark_controllers.keys())

    def register_controller(self, name: str, load_function: Callable, **controller_kwargs) -> None:
        """Register a controller that is available for this specific robot

        Args:
            name (str): Name of the controller
            load_function (Callable): A function that returns a BaseBenchmarkController object when called with
                load_function(**controller_kwargs)
        """
        if name in self.benchmark_controllers:
            carb.log_error(
                "Attempted to register controller {} for robot {}.  But that name is already taken".format(
                    name, self._name
                )
            )
            return

        self.benchmark_controllers[name] = load_function
        self.benchmark_controller_kwargs[name] = controller_kwargs
