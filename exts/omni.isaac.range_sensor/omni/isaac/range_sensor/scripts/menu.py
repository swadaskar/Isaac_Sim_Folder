# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
import omni.kit.commands
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
from pxr import Sdf, UsdGeom, Gf
import weakref


class RangeSensorMenu:
    def __init__(self, ext_id: str):
        menu_items = [
            MenuItemDescription(
                name="Lidar",
                sub_menu=[
                    make_menu_item_description(ext_id, "Rotating", lambda a=weakref.proxy(self): a._add_lidar()),
                    make_menu_item_description(ext_id, "Generic", lambda a=weakref.proxy(self): a._add_generic()),
                ],
            ),
            MenuItemDescription(
                name="Ultrasonic",
                sub_menu=[
                    make_menu_item_description(
                        ext_id, "Array", lambda a=weakref.proxy(self): a._add_ultrasonic_array()
                    ),
                    make_menu_item_description(
                        ext_id, "Emitter", lambda a=weakref.proxy(self): a._add_ultrasonic_emitter()
                    ),
                    make_menu_item_description(
                        ext_id, "FiringGroup", lambda a=weakref.proxy(self): a._add_ultrasonic_firing_group()
                    ),
                ],
            ),
        ]

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

    def _add_lidar(self, *args, **kwargs):
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/Lidar",
            parent=self._get_stage_and_path(),
            min_range=0.4,
            max_range=100.0,
            draw_points=False,
            draw_lines=False,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=20.0,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=False,
        )

    def _add_ultrasonic_array(self, *args, **kwargs):
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicArray",
            path="/UltrasonicArray",
            parent=self._get_stage_and_path(),
            min_range=0.4,
            max_range=3.0,
            draw_points=False,
            draw_lines=False,
            horizontal_fov=15.0,
            vertical_fov=10.0,
            horizontal_resolution=0.5,
            vertical_resolution=0.5,
            num_bins=224,
            emitter_prims=[],
            firing_group_prims=[],
        )

    def _add_ultrasonic_emitter(self, *args, **kwargs):
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicEmitter",
            path="/UltrasonicEmitter",
            parent=self._get_stage_and_path(),
            per_ray_intensity=1.0,
            yaw_offset=0.0,
            adjacency_list=[],
        )

    def _add_ultrasonic_firing_group(self, *args, **kwargs):
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicFiringGroup",
            path="/UltrasonicFiringGroup",
            parent=self._get_stage_and_path(),
            emitter_modes=[],
            receiver_modes=[],
        )

    def _add_generic(self, *args, **kwargs):
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateGeneric",
            path="/GenericSensor",
            parent=self._get_stage_and_path(),
            min_range=0.4,
            max_range=100.0,
            draw_points=False,
            draw_lines=False,
            sampling_rate=60,
        )

    def shutdown(self):
        remove_menu_items(self._menu_items, "Create")
        self.menus = None
