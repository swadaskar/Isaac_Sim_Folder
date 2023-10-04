# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.robot_benchmark.benchmark_controllers import franka_controllers
from omni.isaac.franka import Franka
from omni.isaac.robot_benchmark.benchmark_robots.base_benchmark_robot_loader import BaseBenchmarkRobotLoader


class BenchmarkFrankaLoader(BaseBenchmarkRobotLoader):
    def __init__(self, name, **robot_kwargs):
        BaseBenchmarkRobotLoader.__init__(self, name, **robot_kwargs)

        self._articulation = None
        self._controller = None

        self.register_controller("RMPflow", self.load_rmp_flow_benchmark_controller)
        self.register_controller(
            "RRT Linear Interpolation",
            self.load_rrt_linear_interpolator,
            controller_name="RRT Linear Interpolation",
            num_seeds=1,
            iterations_per_seed=8000,
        )
        self.register_controller("RRT+RMP Taskspace Carrot", self.load_rrt_rmp_taskspace_carrot)

    def load_rrt_rmp_taskspace_carrot(self):
        self._controller = franka_controllers.FrankaRrtRmpCarrotController(
            "RRT RMP Taskspace Carrot", self._articulation, carrot_dist=0.2, cspace_carrot=False
        )
        return self._controller

    def load_rrt_linear_interpolator(self, **kwargs):
        self._controller = franka_controllers.FrankaRrtLinearInterpolationController(
            kwargs["controller_name"],
            self._articulation,
            num_seeds=kwargs["num_seeds"],
            iterations_per_seed=kwargs["iterations_per_seed"],
        )
        return self._controller

    def load_rmp_flow_benchmark_controller(self):
        self._controller = franka_controllers.RMPFlowBenchmarkController("RmpFlow Controller", self._articulation)
        return self._controller

    def load_robot(self, prim_path: str, name: str = "Franka"):
        self._articulation = Franka(prim_path, name, **self._robot_kwargs)

        return self._articulation

    def get_robot_articulation(self):
        return self._articulation
