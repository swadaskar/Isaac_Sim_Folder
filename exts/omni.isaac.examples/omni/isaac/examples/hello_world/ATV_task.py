from omni.isaac.core.prims import GeometryPrim, XFormPrim
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.universal_robots.controllers.pick_place_controller import PickPlaceController
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.types import ArticulationAction
# This extension includes several generic controllers that could be used with multiple robots
from omni.isaac.motion_generation import WheelBasePoseController
# Robot specific controller
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.controllers import BaseController
from omni.isaac.core.tasks import BaseTask
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import SurfaceGripper
import numpy as np
from omni.isaac.core.objects import VisualCuboid, DynamicCuboid
from omni.isaac.core.utils import prims
from pxr import UsdLux, Sdf, UsdGeom
import omni.usd
from omni.isaac.dynamic_control import _dynamic_control

from omni.isaac.universal_robots import KinematicsSolver
import carb
from collections import deque, defaultdict
import time

class ATVTask(BaseTask):
    def __init__(self, name, offset=None):
        super().__init__(name=name, offset=offset)
        self._task_event = 0
        self.task_done = [False]*1000

        self._bool_event = 0
        self.delay=0
        return

    def set_up_scene(self, scene):
        super().set_up_scene(scene)


        # large_robot_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/Collected_full_warehouse_microfactory/Collected_mobile_platform_improved/Collected_mobile_platform_unfinished/mobile_platform_flattened.usd"

        # large_robot_asset_path = "/home/lm-2023/Isaac_Sim/navigation/Collected_real_microfactory_show/Collected_full_warehouse_microfactory/Collected_mobile_platform_improved/Collected_mobile_platform/mobile_platform_ag.usd"

        large_robot_asset_path = "/home/lm-2023/Isaac_Sim/navigation/Collected_real_microfactory_multiple_mp/Collected_real_microfactory_show/Collected_full_warehouse_microfactory/Collected_mobile_platform_improved/Collected_mobile_platform/mobile_platform_ag.usd"
        
        # add floor
        _,num = self.name.split("_")
        # add moving platform
        self.moving_platform = scene.add(
            WheeledRobot(
                prim_path=f"/mock_robot_{num}",
                name=f"moving_platform_{num}",
                wheel_dof_names=["wheel_tl_joint", "wheel_tr_joint", "wheel_bl_joint", "wheel_br_joint"],
                create_robot=True,
                usd_path=large_robot_asset_path,
                # position=np.array([2.5, 5.65, 0.03551]),  orientation=np.array([0,0,0,1]), # start position
                position=np.array([-0.378+int(num)*2, 5.65, 0.03551]),  orientation=np.array([0,0,0,1]), # start position
                # position=np.array([-4.78521, -10.1757,0.03551]), orientation=np.array([0.70711, 0, 0, -0.70711]),# initial before fuel cell
                # position=np.array([-9.60803, -17.35671, 0.03551]), orientation=np.array([0, 0, 0, 1]),# initial before battery cell
                # position=np.array([-32.5-int(num)*2, 3.516, 0.03551]), orientation=np.array([0.70711, 0, 0, 0.70711]),# initial before trunk cell
                # position=np.array([-19.86208-int(num)*2, 9.65617, 0.03551]), orientation=np.array([1, 0, 0, 0]),# initial before wheel cell
                # position=np.array([-20.84299, 6.46358, 0.03551]), orientation=np.array([-0.70711, 0, 0, -0.70711]),# initial before wheel cell
                # position=np.array([-21.13755, -15.54504, 0.03551]), orientation=np.array([0.70711, 0, 0, -0.70711]),# initial before cover cell
                # position=np.array([-27.52625, -7.11835, 0.03551]), orientation=np.array([0.70711, 0, 0, -0.70711]),# initial before light cell
            )
        )

        return

    def get_observations(self):
        current_mp_position, current_mp_orientation = self.moving_platform.get_world_pose()

        observations= {
            self.name+"_event": self._task_event,
            self.moving_platform.name: {
                "position": current_mp_position,
                "orientation": current_mp_orientation
            }
        }
        return observations

    def get_params(self):
        params_representation = {}
        params_representation["mp_name"] = {"value": self.moving_platform.name, "modifiable": False}
        return params_representation
    
    def check_prim_exists(self, prim):
        if prim:
            return True
        return False
    
    def give_location(self, prim_path):
        dc=_dynamic_control.acquire_dynamic_control_interface()
        object=dc.get_rigid_body(prim_path)
        object_pose=dc.get_rigid_body_pose(object)
        return object_pose # position: object_pose.p, rotation: object_pose.r
    

    def pre_step(self, control_index, simulation_time):
        return

    def post_reset(self):
        self._task_event = 0
        return