from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from kuka_follow_target import FollowTarget
import numpy as np

my_world = World(stage_units_in_meters=1.0)
#Initialize the Follow Target task with a target location for the cube to be followed by the end effector
my_task = FollowTarget(name="denso_follow_target", target_position=np.array([0.5, 0, 0.5]))
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("denso_follow_target").get_params()
target_name = task_params["target_name"]["value"]
denso_name = task_params["robot_name"]["value"]
my_denso = my_world.scene.get_object(denso_name)
articulation_controller = my_denso.get_articulation_controller()
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        observations = my_world.get_observations()
        print(observations)
simulation_app.close()