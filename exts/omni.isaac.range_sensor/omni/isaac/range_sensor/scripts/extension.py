# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
import omni.ext
from functools import partial

from omni.isaac import RangeSensorSchema
from .. import _range_sensor
from .menu import RangeSensorMenu
from pxr import UsdShade, Sdf


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._lidar = _range_sensor.acquire_lidar_sensor_interface()
        self._ultrasonic = _range_sensor.acquire_ultrasonic_sensor_interface()
        self._generic = _range_sensor.acquire_generic_sensor_interface()

        self._menu = RangeSensorMenu(ext_id)
        self._registered = False
        manager = omni.kit.app.get_app().get_extension_manager()
        self._hook = manager.subscribe_to_extension_enable(
            on_enable_fn=lambda _: self._register_property_menu(),
            on_disable_fn=lambda _: self._unregister_property_menu(),
            ext_name="omni.kit.property.usd",
            hook_name="omni.isaac.range_sensor omni.kit.property.usd listener",
        )

    def on_shutdown(self):
        self._hook = None
        if self._registered:
            self._unregister_property_menu()
        self._menu.shutdown()
        self._menu = None

        _range_sensor.release_lidar_sensor_interface(self._lidar)
        _range_sensor.release_ultrasonic_sensor_interface(self._ultrasonic)
        _range_sensor.release_generic_sensor_interface(self._generic)

    def _register_property_menu(self):
        self._registered = True
        # +add menu item(s)
        from omni.kit.property.usd import PrimPathWidget

        context_menu = omni.kit.context_menu.get_instance()
        if context_menu is None:
            self._menu_button1 = None
            carb.log_error("context_menu is disabled!")
            return None

        self._menu_button1 = PrimPathWidget.add_button_menu_entry(
            "USS Material", show_fn=partial(self._is_material), onclick_fn=partial(self._apply_uss_material)
        )

    def _unregister_property_menu(self):
        # prevent unregistering multiple times
        if self._registered is False:
            return
        from omni.kit.property.usd import PrimPathWidget

        PrimPathWidget.remove_button_menu_entry(self._menu_button1)
        self._registered = False

    def _is_material(self, objects):
        if not "prim_list" in objects:
            return False
        prim_list = objects["prim_list"]
        stage = objects["stage"]
        if prim_list and stage is not None:
            for prim_path in prim_list:
                if isinstance(prim_path, Sdf.Path):
                    prim = stage.GetPrimAtPath(prim_path)
                    if prim.IsA(UsdShade.Material):
                        return True

        return False

    def _apply_uss_material(self, payload):
        stage = payload.get_stage()
        if stage:
            selected_prims = omni.usd.get_context().get_selection().get_selected_prim_paths()
            for prim_path in selected_prims:
                prim = stage.GetPrimAtPath(prim_path)
                RangeSensorSchema.UltrasonicMaterialAPI.Apply(prim)
        else:
            carb.log_error("_apply_uss_material stage not found")
        return
