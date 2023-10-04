# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import SimulationContext

simulation_context = SimulationContext()

from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.stage import add_reference_to_stage

assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"

simulation_context = SimulationContext(stage_units_in_meters=1.0)
add_reference_to_stage(asset_path, "/Franka")
# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()
dc = _dynamic_control.acquire_dynamic_control_interface()
art = dc.get_articulation("/Franka")
dof_ptr = dc.find_articulation_dof(art, "panda_joint2")


def step_callback_1(step_size):
    dc.wake_up_articulation(art)
    dc.set_dof_position_target(dof_ptr, -1.5)
    return


def step_callback_2(step_size):
    dof_state = dc.get_dof_state(dof_ptr, _dynamic_control.STATE_POS)
    print(
        "Current joint 2 position @ step "
        + str(simulation_context.current_time_step_index)
        + " : "
        + str(dof_state.pos)
    )
    print("TIME: ", simulation_context.current_time)
    return


def render_callback(event):
    print("Render Frame")


simulation_context.add_physics_callback("physics_callback_1", step_callback_1)
simulation_context.add_physics_callback("physics_callback_2", step_callback_2)
simulation_context.add_render_callback("render_callback", render_callback)
simulation_context.stop()
simulation_context.play()
# Simulate 60 timesteps
for i in range(60):
    simulation_context.step(render=False)
# Render one frame
simulation_context.render()

simulation_context.stop()
simulation_app.close()
