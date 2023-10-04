# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
from click.termui import style
import omni
import omni.ext
import omni.timeline
import omni.kit.commands
import omni.ui as ui
import weakref
from .style import UI_STYLES
from ..widgets.documents_widget import *
from ..widgets.content_widget import *
from ..widgets.assembly_widget import *
from .usd_generator import *
import threading
import asyncio
from functools import partial
import weakref
from omni.kit.window.filepicker import FilePickerDialog
import os
import shutil
import time
from omni.client._omniclient import Result
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.kit.window.preferences import register_page, unregister_page

from .preferences import OnshapeImporterPreferences

# from ..widgets.properties import OnshapePropertiesWidget

from omni.isaac.onshape import SETTINGS_PATH

from pxr import UsdGeom

EXTENSION_NAME = "Onshape Importer"


def on_filter_folder(item) -> bool:
    if item and item.is_folder:
        return True
    else:
        return False


class OnshapeImporter(omni.ext.IExt):
    def __init__(self):
        self._window = None

    def _register_preferences(self):
        self._resource_monitor_preferences = register_page(OnshapeImporterPreferences())

    def _unregister_preferences(self):
        if self._resource_monitor_preferences:
            unregister_page(self._resource_monitor_preferences)
            self._resource_monitor_preferences = None

    # def _register_widget(self):
    #     import omni.kit.window.property as p

    #     w = p.get_window()
    #     w.register_widget(
    #         "prim", "onshape_connect", OnshapePropertiesWidget(title="Onshape Data", collapsed=True), False
    #     )
    #     self._registered = True

    # def _unregister_widget(self):
    #     import omni.kit.window.property as p

    #     w = p.get_window()
    #     if w:
    #         w.unregister_widget("prim", "onshape_connect")
    #         self._registered = False

    def on_startup(self, ext_id: str):
        self._window = None
        theme = "NvidiaDark"
        self.ext_id = ext_id
        self.ext_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)
        self._window = None
        self._style = UI_STYLES[theme]
        for i in [
            "Button.Image::filter",
            "Button.Image::accept",
            "Button.Image::cancel",
            "Button.Image::options",
            "Image::processing",
            "Image::error",
            "Image::warning",
            "Image::changed",
            "Image::assembly",
            "Image::part_studio",
            "Image::blob",
            "Image::bom",
            "Image::part",
            "Image::arrow_up",
            "Image::arrow_right",
            "Image::arrow_down",
            "Button.Image::arrow_up",
            "Button.Image::arrow_right",
            "Button.Image::arrow_down",
        ]:
            self._style[i]["image_url"] = self._style[i]["image_url"].format(self.ext_path)
        self._filter_option = -1
        self.orders = ["desc", "asc"]
        self.order_icons = ["arrow_up", "arrow_down"]
        self.order = 1
        self.refresh = False
        self.element_details = None
        self._element_details = None
        self._timeline = omni.timeline.get_timeline_interface()

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            self.ext_id,
            "import_from_onshape",
            partial(self.menu_click, None, True),
            display_name="Import from Onshape",
            description="Import documents from Onshape",
            tag="Import from Onshape",
        )
        self._menu = [
            MenuItemDescription(
                name="Import from Onshape",
                glyph="none.svg",
                appear_after="Import",
                onclick_action=(self.ext_id, "import_from_onshape"),
            )
        ]
        add_menu_items(self._menu, "File", -1000000000000000, rebuild_menus=True)
        self.usd_gen = None

        self._folder_picker = FilePickerDialog(
            "Select output Folder",
            allow_multi_selection=False,
            apply_button_label="Select folder",
            click_apply_handler=lambda a, b, c=weakref.proxy(self): c._on_open_folder_selected(a, b),
            click_cancel_handler=lambda a, b, c=weakref.proxy(self): c._on_picker_cancel(a, b),
            item_filter_fn=on_filter_folder,
        )
        self._folder_picker.hide()
        self.asset_importer = None
        self._preferences = OnshapeImporterPreferences()

        self._hooks = []

        manager = omni.kit.app.get_app().get_extension_manager()

        self._hooks.append(
            manager.subscribe_to_extension_enable(
                on_enable_fn=lambda _: self._register_preferences(),
                on_disable_fn=lambda _: self._unregister_preferences(),
                ext_name="omni.isaac.onshape",
                hook_name="omni.isaac.onshape omni.kit.window.preferences listener",
            )
        )
        # self._register_widget()

    def menu_click(self, menu, value):
        self.show_window(menu, value)

    def on_shutdown(self):
        remove_menu_items(self._menu, "File")
        # self._unregister_widget()
        self.deregister_actions()
        self._menu = None
        if self._folder_picker:
            self._folder_picker.destroy()
            self._folder_picker = None
        if self.usd_gen:
            self.usd_gen.on_shutdown()
            del self.usd_gen
            self.usd_gen = None
        self.element_details = None
        self._window = None

    def _on_picker_cancel(self, a, b):
        if self._folder_picker:
            self._folder_picker.hide()

    def _on_picker_cancel(self, a, b):
        if self._folder_picker:
            self._folder_picker.hide()

    def show_window(self, menu, value):
        if value:
            self.build_ui()

    def on_rig_physics_changed(self, value):
        pass

    def on_filter_unsupported(self, value):
        carb.settings.get_settings().set("{}/filter_unsupported".format(SETTINGS_PATH), value)
        self.content_browser._docs_model.on_update_filter_unsupported(value)

    def on_element_selected(self, item):
        if self._element_details:
            self._element_details = None
            if self.usd_gen:
                self.usd_gen.on_shutdown()
                del self.usd_gen
                self.usd_gen = None

        if item.get_selected_element():
            element = item.get_selected_element()[0]
            if element["type"] == "Assembly":
                self.element_details = ui.Window(
                    EXTENSION_NAME + " - Assembly Viewer", width=900, height=400, open=True
                )
                self.element_details.dock_in(self._window, ui.DockPosition.SAME)
                with self.element_details.frame:
                    # print(element)
                    with ui.VStack(height=ui.Fraction(1), style=self._style):
                        with ui.ScrollingFrame(height=ui.Fraction(1)):
                            if element["type"] == "Assembly":
                                model = OnshapeAssemblyModel(
                                    item,
                                    element,
                                    assembly_loaded_fn=lambda a=weakref.proxy(self): a.assembly_reloaded(),
                                    rig_physics=self._preferences.rig_physics,
                                )
                                self.usd_gen = UsdGenerator(
                                    item,
                                    model,
                                    UsdGeom.GetStageMetersPerUnit(omni.usd.get_context().get_stage()),
                                    assembly_done_fn=partial(OnshapeImporter.on_stage_imported, self),
                                )
                                self.usd_gen.rig_physics = self._preferences.rig_physics
                                self._element_details = AssemblyDetailsWidget(
                                    model,
                                    self.usd_gen,
                                    style=self._style,
                                    mesh_imported_fn=lambda a, b, c=weakref.proxy(self): c.on_mesh_imported(a, b),
                                )
                                model._get_assembly_definition()
                self._window.visible = False

                self.element_details.focus()

    def _select_folder(self, btn_widget):
        self._folder_picker.show()

    def _on_open_folder_selected(self, menu, path):
        self._folder_picker.hide()
        # if self._flatten_cb.model.get_value_as_bool():
        #     self.usd_gen.save_flattened(path, self.close_window)
        #     if self._element_details:
        #         self._element_details = None
        #         if self.usd_gen:
        #             self.usd_gen.on_shutdown()
        #             del self.usd_gen
        #             self.usd_gen = None
        # else:
        self._finish_import(path)

    def on_stage_imported(self, stage_path):
        if not self.prim:
            stage = omni.usd.get_context().get_stage()
            _selection = omni.usd.get_context().get_selection()
            selected = _selection.get_selected_prim_paths()
            if selected:
                selected = stage.GetPrimAtPath(selected[0]).GetPath()
            else:
                selected = stage.GetDefaultPrim().GetPath()
            base_name = self.usd_gen.assembly.get_name()
            prim_path = omni.usd.get_stage_next_free_path(
                stage, selected.AppendChild(pxr.Tf.MakeValidIdentifier(base_name)), False
            )
            self.prim = UsdGeom.Xform.Define(stage, prim_path).GetPrim()
            self.prim.GetPayloads().AddPayload(stage_path)
        self._element_details.loading_image.visible = False
        # else:
        #     self.prim.Load()

    def close_window(self, a=None, b=None):
        self.usd_gen = None
        self._window.visible = False
        self.element_details.visible = False

    def _finish_import(self, output_dir):
        # setting asset importer parameters to upload
        # if self.asset_importer is None:
        #     from omni.kit.tool.asset_importer import importer

        #     self.asset_importer = importer.Importer()
        #     self.asset_importer.on_startup()

        # upload_absolute_paths, upload_relative_paths = self.usd_gen.get_abs_and_rel_paths()

        # async def upload():
        #     await self.asset_importer.create_import_task(
        #         False, upload_absolute_paths, upload_relative_paths, output_dir, None
        #     )
        #     omni.usd.get_context().open_stage(
        #         output_dir
        #         + "/"
        #         + self.usd_gen.document.get_name()
        #         + "/{}.usd".format(self.usd_gen.document.get_name()),
        #         weakref.proxy(self).close_window,
        #     )
        # print(self.usd_gen.tempdir, output_dir)
        dst = os.path.join(output_dir, os.path.basename(self.usd_gen.tempdir))
        if os.path.exists(dst):
            shutil.rmtree(dst)
        omni.client.copy(self.usd_gen.tempdir, dst)
        r = omni.client.list(dst)

        # asyncio.ensure_future(omni.client.copy_async(self.usd_gen.tempdir, output_dir))
        if self._element_details:
            self._element_details = None
            if self.usd_gen:
                self.usd_gen.on_shutdown()
                del self.usd_gen
                self.usd_gen = None
        if r[0] == Result.OK:
            root_file = os.path.join(dst, [a for a in r[1] if a.flags == 3][0].relative_path)

            async def open_stage():
                while not omni.usd.get_context().can_open_stage():
                    await omni.kit.app.get_app().next_update_async()
                omni.usd.get_context().open_stage(root_file)

            asyncio.ensure_future(open_stage())
        self.close_window()

    def assembly_reloaded(self):
        # if self.prim:
        #     self.prim.Unload()
        if self._element_details.model.config_changed:
            # for part in self._element_details._parts_widget.model._children:
            #         for c in ["suppressed", "configuration", "fullConfiguration"]:
            #             print(part.get_item(c), self._element_details.model._parts_flat[part.get_key()].get_item(c))
            #             part.set_item(c, self._element_details.model._parts_flat[part.get_key()].get_item(c))

            self._element_details.model.config_changed = False
            self.usd_gen.reset_assembly()
            self.refresh = True
        if self.refresh:
            self.usd_gen.assembly = self._element_details.model
            self.usd_gen.build_assemblies()
            self.refresh = False

    def reload_assembly(self):
        if self._timeline.is_playing():
            self._timeline.stop()
        self._element_details.loading_image.visible = True
        self.usd_gen.reset_assembly()

        self._element_details.model._get_assembly_definition()

    def on_mesh_imported(self, item, done_importing=True):
        if item is not None:
            self.usd_gen.create_part_stage(item)

    def build_ui(self):
        if OnshapeClient.authenticate(self.build_ui):
            self.prim = None
            if self._window is None:
                # Do a first call on Onshape Client to prime authentication
                self._window = ui.Window(
                    EXTENSION_NAME,
                    width=800,
                    height=400,
                    menu_path="Isaac/" + EXTENSION_NAME,
                    open=True,
                    dock=ui.DockPreference.LEFT_BOTTOM,
                )
                self._window.set_visibility_changed_fn(self.on_visibility_change)

                self._docs_delegate = DocumentListDelegate(self._style)
                self._filters = []
                with self._window.frame:
                    with ui.ScrollingFrame(
                        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED, width_min=455
                    ):
                        with ui.HStack(style=self._style):
                            self.content_browser = OnshapeContentWidget(
                                filter_unsupported=self._preferences.filter_unsupported
                            )

                            def selected(model, item):
                                model.on_element_selected(item)

                            self.content_browser.set_on_mouse_double_clicked(partial(selected, weakref.proxy(self)))
                self.element_details = ui.Window(
                    EXTENSION_NAME + " - Assembly Viewer", width=900, height=400, open=True
                )

                main_dockspace = ui.Workspace.get_window(EXTENSION_NAME)
                main_dockspace.deferred_dock_in("Overview", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
                main_dockspace.dock_order = 5
                ext2 = ui.Workspace.get_window(EXTENSION_NAME + " - Assembly Viewer")
                ext2.deferred_dock_in(EXTENSION_NAME, ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
                main_dockspace.dock_order = 6
                self.element_details.visible = False
                # self.element_details.dock_in(self._window, ui.DockPosition.SAME, 1.0)
            self._window.visible = True

    def on_visibility_change(self, a):
        self.show_window(self._menu, a)

    def deregister_actions(self,):
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_all_actions_for_extension(self.ext_id)
