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

from omni.isaac.universal_robots.tasks import BinFilling
from omni.isaac.universal_robots.controllers import PickPlaceController
from omni.isaac.core import World
import numpy as np
from omni.isaac.core.utils.rotations import euler_angles_to_quat

my_world = World(stage_units_in_meters=1.0)
my_task = BinFilling()
my_world.add_task(my_task)
my_world.reset()
task_params = my_task.get_params()
my_ur10 = my_world.scene.get_object(task_params["robot_name"]["value"])
my_controller = PickPlaceController(name="pick_place_controller", gripper=my_ur10.gripper, robot_articulation=my_ur10)
articulation_controller = my_ur10.get_articulation_controller()

i = 0
added_screws = False
while simulation_app.is_running():
    if my_world.is_playing():
        my_world.step(render=True)
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
            added_screws = False
        observations = my_world.get_observations()
        actions = my_controller.forward(
            picking_position=observations[task_params["bin_name"]["value"]]["position"],
            placing_position=observations[task_params["bin_name"]["value"]]["target_position"],
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            end_effector_offset=np.array([0, -0.098, 0.03]),
            end_effector_orientation=euler_angles_to_quat(np.array([np.pi, 0, np.pi / 2.0])),
        )
        if not added_screws and my_controller.get_current_event() == 6 and not my_controller.is_paused():
            my_controller.pause()
            my_task.add_screws(screws_number=20)
            added_screws = True
        if my_controller.is_done():
            print("done picking and placing")
        articulation_controller.apply_action(actions)
    else:
        my_world.render()
simulation_app.close()
