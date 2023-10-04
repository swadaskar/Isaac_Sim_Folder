# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.dofbot.tasks import FollowTarget
from omni.isaac.dofbot.controllers import RMPFlowController
from omni.isaac.core import World
from omni.isaac.dofbot import KinematicsSolver
import carb
import numpy as np

my_world = World(stage_units_in_meters=1.0)
my_task = FollowTarget(name="follow_target_task")
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("follow_target_task").get_params()
dofbot_name = task_params["robot_name"]["value"]
target_name = task_params["target_name"]["value"]
my_dofbot = my_world.scene.get_object(dofbot_name)
my_controller = KinematicsSolver(my_dofbot)
articulation_controller = my_dofbot.get_articulation_controller()
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        observations = my_world.get_observations()

        # IK does not work well on dofbot with orientation targets
        actions, succ = my_controller.compute_inverse_kinematics(target_position=observations[target_name]["position"])
        # actions,succ = my_controller.compute_inverse_kinematics(target_position=observations[target_name]["position"],
        #     target_orientation=observations[target_name]["orientation"], orientation_tolerance = np.pi/2)
        if succ:
            articulation_controller.apply_action(actions)
        else:
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")

simulation_app.close()
