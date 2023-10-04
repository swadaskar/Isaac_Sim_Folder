# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test

import carb
import numpy as np
import omni.graph.core as og

from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.dynamic_control import utils as dc_utils
from omni.isaac.core.utils.rotations import quat_to_euler_angles
from omni.isaac.core.utils.extensions import get_extension_path_from_name

from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core_nodes.scripts.utils import set_target_prims


async def init_robot_sim(dc, art_path, graph_path="/ActionGraph"):

    art = dc.get_articulation(art_path)
    chassis = dc.get_articulation_root_body(art)
    starting_pos = dc.get_rigid_body_pose(chassis)
    starting_pos.p = [0, 0, 0.1]
    starting_pos.r = [0, 0, 0, 1]
    # reset pose
    dc.set_rigid_body_pose(chassis, starting_pos)
    # reset velocity
    dc.set_rigid_body_linear_velocity(chassis, (0, 0, 0))
    dc.set_rigid_body_angular_velocity(chassis, (0, 0, 0))
    # reset controller
    og.Controller.attribute(graph_path + "/DifferentialController.inputs:linearVelocity").set(0)
    og.Controller.attribute(graph_path + "/DifferentialController.inputs:angularVelocity").set(0)
    # wait for robot to drop
    for i in range(10):
        await omni.kit.app.get_app().next_update_async()

    return


def setup_robot_og(graph_path, lwheel_name, rwheel_name, robot_path, wheel_rad, wheel_dist):
    keys = og.Controller.Keys
    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("DifferentialController", "omni.isaac.wheeled_robots.DifferentialController"),
                ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                ("computeOdom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "DifferentialController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "computeOdom.inputs:execIn"),
                ("DifferentialController.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                ("DifferentialController.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("DifferentialController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ],
            keys.SET_VALUES: [
                ("ArticulationController.inputs:jointNames", [lwheel_name, rwheel_name]),
                ("ArticulationController.inputs:robotPath", robot_path),
                ("DifferentialController.inputs:wheelRadius", wheel_rad),
                ("DifferentialController.inputs:wheelDistance", wheel_dist),
            ],
        },
    )

    set_target_prims(primPath="/ActionGraph/computeOdom", inputName="inputs:chassisPrim", targetPrimPaths=[robot_path])

    return graph, nodes[3]


def set_physics_frequency(frequency=60):
    import carb

    carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
    carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(frequency))
    carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(frequency))
