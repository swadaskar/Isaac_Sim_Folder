from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import SurfaceGripper
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.prims import GeometryPrim, XFormPrim
import numpy as np

assets_root_path = get_assets_root_path()

my_world = World(stage_units_in_meters=1.0)
#TODO: change this to your own path
asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/robot_arm_nj.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/kuka")
#define the gripper
gripper_usd = assets_root_path + "/Isaac/Robots/UR10/Props/short_gripper.usd"
add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/kuka/tool0")
# gripper = my_world.scene.add(XFormPrim(prim_path=f'/World/kuka/tool0', name="gripper")) # declares in the world

# ## add part
# gripper.set_world_pose(position=np.array([2.715, 0, 0.63385]), orientation=np.array([0.70711, 0, 0, 0]))

gripper = SurfaceGripper(end_effector_prim_path="/World/kuka/tool0", translate=0, direction="x")

kuka = my_world.scene.add(SingleManipulator(prim_path="/World/kuka", name="kuka", end_effector_prim_name="tool0", gripper=gripper))
# #define the manipulator
# my_denso = my_world.scene.add(SingleManipulator(prim_path="/World/kuka", name="kuka_robot",
#                                                 end_effector_prim_name="onrobot_rg6_base_link", gripper=gripper))
# #set the default positions of the other gripper joints to be opened so
# #that its out of the way of the joints we want to control when gripping an object for instance.
# joints_default_positions = np.zeros(12)
# joints_default_positions[7] = 0.628
# joints_default_positions[8] = 0.628
# my_denso.set_joints_default_state(positions=joints_default_positions)
my_world.scene.add_default_ground_plane()
my_world.reset()

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
#     if my_world.is_playing():
#         if my_world.current_time_step_index == 0:
#             my_world.reset()
#         # i += 1
#         # gripper_positions = my_denso.gripper.get_joint_positions()
#         # if i < 500:
#         #     #close the gripper slowly
#         #     my_denso.gripper.apply_action(
#         #         ArticulationAction(joint_positions=[gripper_positions[0] + 0.1, gripper_positions[1] - 0.1]))
#         # if i > 500:
#         #     #open the gripper slowly
#         #     my_denso.gripper.apply_action(
#         #         ArticulationAction(joint_positions=[gripper_positions[0] - 0.1, gripper_positions[1] + 0.1]))
#         # if i == 1000:
#         #     i = 0

simulation_app.close()