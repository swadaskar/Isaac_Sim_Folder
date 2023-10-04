# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import gc
import omni.ext
import omni.usd
import omni.ui as ui
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
import omni.kit.utils
import omni.kit.commands
from pxr import Usd, UsdGeom, UsdPhysics
import weakref
from omni.physx.scripts import utils
from omni.isaac.ui.ui_utils import cb_builder, btn_builder, multi_btn_builder, progress_bar_builder, dropdown_builder
import asyncio

EXTENSION_NAME = "Physics Utilities"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._usd_context = omni.usd.get_context()
        self._selected_prim = None
        self._selection = self._usd_context.get_selection()

        self._window = omni.ui.Window(
            EXTENSION_NAME, width=600, height=400, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.deferred_dock_in("Console", omni.ui.DockPolicy.DO_NOTHING)
        self._menu_items = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        add_menu_items(self._menu_items, "Isaac Utils")

        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                ui.Label("Collision APIs:")
                self._children_checkbox = cb_builder(
                    label="Apply To Children",
                    default_val=True,
                    tooltip="Apply collision API to child prims if possible",
                )
                self._visible_checkbox = cb_builder(
                    label="Visible Only", default_val=True, tooltip="Only apply collision API to prims that are visible"
                )
                self._collision_type = dropdown_builder(
                    label="Collision Type", items=["Triangle Mesh", "Convex Hull", "Convex Decomposition"]
                )
                btn_builder(
                    label="For Current Selection",
                    text="Apply Static",
                    tooltip="The following buttons work on the currently selected prims",
                    on_clicked_fn=self.apply_collision_on_selected,
                )
                ui.Line()
                ui.Label("Remove APIs on Current Selection:")
                multi_btn_builder(
                    label="For Current Selection",
                    text=["Remove Collision API", "Remove All Physics APIs"],
                    tooltip=[
                        "The following buttons work on the currently selected prims",
                        "Remove Collision API on selected",
                        "NOTE: This cannot delete usd prims on referenced assets, but will still unapply USD APIs",
                    ],
                    on_clicked_fn=[self.clear_collision_on_selected, self.remove_physics_apis_on_selected],
                )
                ui.Line()
                self._progress_bar = progress_bar_builder("Current Progress")
                self._progress_bar.set_value(0)

        pass

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def apply_collision_on_selected(self):
        async def _task():
            self._stage = self._usd_context.get_stage()
            selection = self._selection.get_selected_prim_paths()
            if (len(selection)) == 0:
                return
            all_prims = self.traverse_prims(
                selection, include_xform=False, visible_only=self._visible_checkbox.get_value_as_bool()
            )
            count = len(all_prims)
            index = 0

            approximation_type = ["none", "convexHull", "convexDecomposition"]

            for prim in all_prims:
                selection = self._collision_type.get_item_value_model().as_int
                self.apply_collision_to_prim(prim, approximation_type[selection])
                index = index + 1
                self._progress_bar.set_value(index / count)
                await omni.kit.app.get_app().next_update_async()

        asyncio.ensure_future(_task())

    def clear_collision_on_selected(self):
        async def _task():
            self._stage = self._usd_context.get_stage()
            selection = self._selection.get_selected_prim_paths()
            if (len(selection)) == 0:
                return
            all_prims = self.traverse_prims(
                selection, include_xform=True, visible_only=self._visible_checkbox.get_value_as_bool()
            )
            count = len(all_prims)
            index = 0
            for prim in all_prims:
                self.unapply_collision_on_prim(prim)
                index = index + 1
                self._progress_bar.set_value(index / count)
                await omni.kit.app.get_app().next_update_async()

        asyncio.ensure_future(_task())

        pass

    def remove_physics_apis_on_selected(self):
        async def _task():
            self._stage = self._usd_context.get_stage()
            selection = self._selection.get_selected_prim_paths()
            if (len(selection)) == 0:
                return
            all_prims = self.traverse_prims(selection, include_xform=True, ignore_rigid=False, visible_only=False)
            count = len(all_prims)
            index = 0
            for prim in all_prims:
                self.remove_physics_apis_on_prim(prim)
                index = index + 1
                self._progress_bar.set_value(index / count)
                await omni.kit.app.get_app().next_update_async()

        asyncio.ensure_future(_task())

        pass

    def traverse_prims(self, selection, include_xform=False, ignore_rigid=True, visible_only=True):
        prims = []
        for s in selection:
            curr_prim = self._stage.GetPrimAtPath(s)
            if self._children_checkbox.get_value_as_bool():
                prim_range_iter = iter(Usd.PrimRange(curr_prim))
                for prim in prim_range_iter:
                    # process the current prim if its an instance, but prune children as we cannot process them anyways
                    if prim.IsInstanceable():
                        prim_range_iter.PruneChildren()
                    imageable = UsdGeom.Imageable(prim)
                    # ignore non stage prims
                    if prim.GetMetadata("hide_in_stage_window"):
                        prim_range_iter.PruneChildren()
                        continue
                    # If a prim is hidden and visible only is checked, skip all children of that prim
                    if visible_only:
                        if imageable:
                            visibility = imageable.ComputeVisibility(Usd.TimeCode.Default())
                            if visibility == UsdGeom.Tokens.invisible:
                                prim_range_iter.PruneChildren()
                                continue
                    # Ignore rigid bodies and its children
                    if ignore_rigid and utils.hasSchema(prim, "PhysicsRigidBodyAPI"):
                        prim_range_iter.PruneChildren()
                        continue
                    if self.prim_is_valid(
                        prim, include_xform or prim.IsInstanceable(), self._visible_checkbox.get_value_as_bool()
                    ):
                        prims.append(prim)
            else:
                if self.prim_is_valid(
                    curr_prim, include_xform or curr_prim.IsInstanceable(), self._visible_checkbox.get_value_as_bool()
                ):
                    prims.append(curr_prim)
        return prims

    def prim_is_valid(self, prim, include_xform=False, visible_only=True):
        if (
            prim.IsA(UsdGeom.Cylinder)
            or prim.IsA(UsdGeom.Capsule)
            or prim.IsA(UsdGeom.Cone)
            or prim.IsA(UsdGeom.Sphere)
            or prim.IsA(UsdGeom.Cube)
        ):
            return True
        elif prim.IsA(UsdGeom.Mesh):
            return True
        elif include_xform and prim.IsA(UsdGeom.Xformable):
            return True
        return False
        pass

    def apply_collision_to_prim(self, prim, approximationShape="none"):
        # TODO: add checks for rigid body parent type, we cannot use regular collision mesh in that case
        if prim.IsInstanceable():
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim)
        else:
            utils.setCollider(prim, approximationShape)

    def unapply_collision_on_prim(self, prim):
        utils.removeCollider(prim)

    def remove_physics_apis_on_prim(self, prim):
        apis = [
            "PhysicsRigidBodyAPI",
            "PhysicsCollisionAPI",
            "PhysicsArticulationRootAPI",
            "CharacterControllerAPI",
            "ContactReportAPI",
            "PhysicsFilteredPairsAPI",
            "TriggerAPI",
            "PhysicsMaterialAPI",
            "PhysicsMassAPI",
            "PhysicsRevoluteJoint",
            "PhysicsPrismaticJoint",
            "PhysicsSphericalJoint",
            "PhysicsDistanceJoint",
            "PhysicsFixedJoint",
            "PhysicsLimit",
            "PhysicsDrive",
        ]
        for component in apis:
            omni.kit.commands.execute("RemovePhysicsComponentCommand", usd_prim=prim, component=component)

    def on_shutdown(self):
        remove_menu_items(self._menu_items, "Isaac Utils")
        gc.collect()
        pass
