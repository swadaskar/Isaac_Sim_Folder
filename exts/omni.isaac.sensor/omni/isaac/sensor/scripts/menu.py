# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
from omni.isaac.core.utils.prims import set_prim_visibility
import omni.kit.commands
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
from pxr import Sdf, UsdGeom, Gf
import weakref
import sys


class IsaacSensorMenu:
    def __init__(self, ext_id: str):
        menu_items = [
            make_menu_item_description(ext_id, "Contact Sensor", lambda a=weakref.proxy(self): a._add_contact_sensor()),
            make_menu_item_description(ext_id, "Imu Sensor", lambda a=weakref.proxy(self): a._add_imu_sensor()),
        ]
        if sys.platform != "win32":
            menu_items.append(
                MenuItemDescription(
                    name="RTX Lidar",
                    sub_menu=[
                        make_menu_item_description(
                            ext_id, "Rotating", lambda a=weakref.proxy(self): a._add_rtx_rotating_lidar()
                        ),
                        make_menu_item_description(
                            ext_id, "Solid State", lambda a=weakref.proxy(self): a._add_rtx_solid_lidar()
                        ),
                    ],
                )
            )
        self._menu_items = [
            MenuItemDescription(
                name="Isaac", glyph="plug.svg", sub_menu=[MenuItemDescription(name="Sensors", sub_menu=menu_items)]
            )
        ]
        add_menu_items(self._menu_items, "Create")

    def _get_stage_and_path(self):
        self._stage = omni.usd.get_context().get_stage()
        selectedPrims = omni.usd.get_context().get_selection().get_selected_prim_paths()

        if len(selectedPrims) > 0:
            curr_prim = selectedPrims[-1]
        else:
            curr_prim = None
        return curr_prim

    def _add_contact_sensor(self, *args, **kargs):
        result, prim = omni.kit.commands.execute(
            "IsaacSensorCreateContactSensor",
            path="/Contact_Sensor",
            parent=self._get_stage_and_path(),
            min_threshold=0.0,
            max_threshold=100000.0,
            color=Gf.Vec4f(1, 0, 0, 1),
            radius=-1,
            sensor_period=-1,
            translation=Gf.Vec3d(0, 0, 0),
        )

    def _add_imu_sensor(self, *args, **kargs):
        result, prim = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/Imu_Sensor",
            parent=self._get_stage_and_path(),
            sensor_period=-1,
            translation=Gf.Vec3d(0, 0, 0),
        )
        # Make lidar invisible on stage as camera
        set_prim_visibility(prim=prim, visible=False)

    def _add_rtx_rotating_lidar(self, *args, **kwargs):
        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/rtx_lidar",
            parent=self._get_stage_and_path(),
            config="Example_Rotary",
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(0.5, 0.5, -0.5, -0.5),
        )
        # Make lidar invisible on stage as camera
        set_prim_visibility(prim=prim, visible=False)

    def _add_rtx_solid_lidar(self, *args, **kwargs):
        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/rtx_lidar",
            parent=self._get_stage_and_path(),
            config="Example_Solid_State",
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(0.5, 0.5, -0.5, -0.5),
        )
        # Make lidar invisible on stage as camera
        set_prim_visibility(prim=prim, visible=False)

    def shutdown(self):
        remove_menu_items(self._menu_items, "Create")
        self.menus = None
