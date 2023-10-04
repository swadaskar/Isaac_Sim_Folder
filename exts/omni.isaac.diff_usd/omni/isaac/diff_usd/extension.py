# Copyright (c) 2022-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ext
import omni.kit.commands
import omni.ui
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description

import carb

from pxr import Sdf, Usd

import difflib as dl
import weakref

from typing import List
from typing import Optional


class DiffUSD(omni.kit.commands.Command):
    """
    The DiffUSD command compares two selected prims using difflib's unified diff format.
    """

    def __init__(self, text_diff=False, **kwargs):
        """
        Constructor.

        Args:
            text_diff: Diff USD Text if true.  Otherwise, diff a list of prims.
            kwargs["src_diff_paths"]: contains the two paths to compare.
        """
        self._src_diff_paths = None
        self._diff_prim_text = ""
        self._text_diff = text_diff

        usd_context = omni.usd.get_context()

        if "src_diff_paths" in kwargs:
            self._src_diff_paths = kwargs["src_diff_paths"]
        else:
            self._src_diff_paths = usd_context.get_selection().get_selected_prim_paths()

    def do(self):
        usd_context = omni.usd.get_context()
        stage = usd_context.get_stage()
        self._diff_prim_text = ""

        if len(self._src_diff_paths) < 2:
            return self._diff_prim_text

        if self._text_diff:
            prim_left = stage.GetPrimAtPath(self._src_diff_paths[0])
            prim_right = stage.GetPrimAtPath(self._src_diff_paths[-1])

            if prim_left and prim_right:
                properties_left = self._get_prim_as_properties(prim_left)
                properties_right = self._get_prim_as_properties(prim_right)

                for line in dl.unified_diff(properties_left, properties_right):
                    self._diff_prim_text += str(line) + "\n"
        else:
            prim_left = Sdf.Path(self._src_diff_paths[0])
            prim_right = Sdf.Path(self._src_diff_paths[-1])

            if prim_left and prim_right:
                usd_left = self._get_prim_as_usda(stage, [prim_left])
                usd_right = self._get_prim_as_usda(stage, [prim_right])

                for line in dl.unified_diff(usd_left, usd_right):
                    self._diff_prim_text += str(line) + "\n"

        carb.log_info(self._diff_prim_text)

        return self._diff_prim_text

    def undo(self):
        return

    def _update_property_paths(self, prim_spec, old_path, new_path):
        if not prim_spec:
            return

        for rel in prim_spec.relationships:
            rel.targetPathList.explicitItems = [
                path.ReplacePrefix(old_path, new_path) for path in rel.targetPathList.explicitItems
            ]

        for attr in prim_spec.attributes:
            attr.connectionPathList.explicitItems = [
                path.ReplacePrefix(old_path, new_path) for path in attr.connectionPathList.explicitItems
            ]

        for child in prim_spec.nameChildren:
            self._update_property_paths(child, old_path, new_path)

    def _get_prim_as_properties(self, prim) -> Optional[str]:
        prop_list = [str(prim.GetName())] + [
            str(prop.GetName()) + str(type(prop)) + str(prop.GetAllMetadata())
            for prop in prim.GetProperties()
            if not prop.IsHidden()
        ]
        return prop_list

    def _get_prim_as_usda(self, stage: Usd.Stage, prim_paths: List[Sdf.Path]) -> Optional[str]:
        if not prim_paths:
            return

        flatten_layer = stage.Flatten()
        anonymous_layer = Sdf.Layer.CreateAnonymous(prim_paths[0].name + ".usda")
        paths_map = {}

        for i in range(0, len(prim_paths)):
            item_name = str.format("Item_{:02d}", i)
            Sdf.PrimSpec(anonymous_layer, item_name, Sdf.SpecifierDef)
            prim_path = prim_paths[i]
            anonymous_path = Sdf.Path.absoluteRootPath.AppendChild(item_name).AppendChild(prim_path.name)

            # Copy
            Sdf.CopySpec(flatten_layer, prim_path, anonymous_layer, anonymous_path)

            paths_map[prim_path] = anonymous_path

        for prim in anonymous_layer.rootPrims:
            for source_path, target_path in paths_map.items():
                self._update_property_paths(prim, source_path, target_path)

        return anonymous_layer.ExportToString().split("\n")


class DiffUSDExtension(omni.ext.IExt):
    """
    The DiffUSDExtension wraps the DiffUSD command.
    """

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def on_startup(self, ext_id: str):
        self._window = omni.ui.Window("Diff USD", visiable=False)

        with self._window.frame:
            with omni.ui.VStack(height=33, spacing=9):
                with omni.ui.HStack():
                    omni.ui.Label("Diff Type")
                    self._diff_type_combo = omni.ui.ComboBox(0, "USDA Text Diff", "Property List Diff")
                omni.ui.Button("Diff Selected", clicked_fn=lambda b=None: self.on_click(b))

        self._menu_items = [
            make_menu_item_description(ext_id, "Diff USD", lambda a=weakref.proxy(self): a._menu_callback())
        ]
        add_menu_items(self._menu_items, "Isaac Utils")
        self._window.visible = False

    def on_click(self, button):
        prim_text = self._diff_type_combo.model.get_item_value_model().get_value_as_int() > 0
        omni.kit.commands.execute("DiffUSD", text_diff=prim_text)

    def on_shutdown(self):
        remove_menu_items(self._menu_items, "Isaac Utils")
        self._window = None
        return


def get_extension():
    return DiffUSDExtension()
