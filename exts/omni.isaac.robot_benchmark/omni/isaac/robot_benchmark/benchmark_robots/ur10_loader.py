# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.robot_benchmark.benchmark_controllers import ur10_controllers

from typing import Optional, Sequence
from collections.abc import Callable
from omni.isaac.universal_robots import UR10
import numpy as np
import carb
from omni.isaac.robot_benchmark.benchmark_robots.base_benchmark_robot_loader import BaseBenchmarkRobotLoader


class BenchmarkUR10Loader(BaseBenchmarkRobotLoader):
    def __init__(self, name, **robot_kwargs):
        BaseBenchmarkRobotLoader.__init__(self, name, **robot_kwargs)

        self._articulation = None
        self._controller = None

        self.register_controller("RMPflow", self.load_rmp_flow_benchmark_controller)

    def load_rmp_flow_benchmark_controller(self):
        self._controller = ur10_controllers.RMPFlowBenchmarkController("RmpFlow Controller", self._articulation)
        return self._controller

    def load_robot(
        self,
        prim_path: str,
        name: str = "UR10",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ):

        self._articulation = UR10(prim_path, name, usd_path=usd_path, position=position, orientation=orientation)

        return self._articulation

    def get_robot_articulation(self):
        return self._articulation
