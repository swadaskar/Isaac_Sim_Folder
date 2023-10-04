# Copyright (c) 2022-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import omni.ext
import omni.ui as ui
import gc
import carb
import omni.kit.commands
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
from pxr import Sdf, UsdGeom, Gf
import weakref
from ..bindings._omni_isaac_conveyor import acquire_interface as _acquire
from ..bindings._omni_isaac_conveyor import release_interface as _release

from .style import UI_STYLES

from .ui.conveyor_builder import ConveyorBuilderWidget

EXTENSION_NAME = "Conveyor Utility"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.widget = None
        menu_items = [
            MenuItemDescription(
                name="Warehouse Items",
                sub_menu=[
                    make_menu_item_description(ext_id, "Conveyor", lambda a=weakref.proxy(self): a._add_conveyor())
                ],
            )
        ]
        self._menu_items_2 = [
            make_menu_item_description(ext_id, "Conveyor Track Builder", lambda a=weakref.proxy(self): a.create_ui())
        ]

        self._menu_items = [MenuItemDescription(name="Isaac", glyph="plug.svg", sub_menu=menu_items)]

        add_menu_items(self._menu_items, "Create")
        add_menu_items(self._menu_items_2, "Tools")

        self.__interface = _acquire()
        theme = "NvidiaDark"
        self.ext_id = ext_id
        ext_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)
        self.ext_path = ext_path + "/omni/isaac/conveyor"
        self.config_file = ext_path + "/data/track_types.json"
        self.mdl_file = ext_path + "/data/GhostVolumetric.mdl"
        self._style = UI_STYLES[theme]
        for i in [a for a in self._style.keys() if "{}" in self._style[a].get("image_url", "")]:
            self._style[i]["image_url"] = self._style[i]["image_url"].format(self.ext_path)

    def create_ui(self):
        self._window = ui.Window("Conveyor Builder", open=True, dock=ui.DockPreference.LEFT_BOTTOM)
        self._window.set_visibility_changed_fn(self.on_visibility_changed)
        with self._window.frame:
            self.widget = ConveyorBuilderWidget(self.config_file, mdl_file=self.mdl_file, style=self._style)

    def on_visibility_changed(self, value):
        if not value:
            self.widget.shutdown()

    def on_shutdown(self):
        remove_menu_items(self._menu_items, "Create")
        remove_menu_items(self._menu_items_2, "Tools")
        if self.widget:
            self.widget.shutdown()
        _release(self.__interface)
        self.__interface = None
        gc.collect()

    def _add_conveyor(self, *args, **kwargs):
        _, prim = omni.kit.commands.execute("CreateConveyorBelt")
