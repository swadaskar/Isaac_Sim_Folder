# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from typing import OrderedDict
import weakref
import asyncio
import gc
import carb
import omni
from pxr import Usd, UsdGeom
from omni.kit.window.property.templates import LABEL_WIDTH
import omni.ui as ui
import omni.usd
import omni.timeline
import omni.kit.commands
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.core.utils.prims import get_prim_object_type, get_prim_at_path
from omni.isaac.core.utils.numpy.rotations import quats_to_rot_matrices
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.articulations import Articulation
from .collision_sphere_editor import CollisionSphereEditor

from omni.isaac.ui.widgets import DynamicComboBoxModel

from omni.isaac.ui.ui_utils import (
    add_line_rect_flourish,
    btn_builder,
    state_btn_builder,
    float_builder,
    int_builder,
    xyz_builder,
    color_picker_builder,
    setup_ui_headers,
    get_style,
    str_builder,
)
import omni.physx as _physx
import numpy as np
import os
import yaml

EXTENSION_NAME = "Lula Robot Description Editor"

MAX_DOF_NUM = 100


def is_yaml_file(path: str):
    _, ext = os.path.splitext(path.lower())
    return ext in [".yaml", ".YAML"]


def on_filter_item(item) -> bool:
    if not item or item.is_folder:
        return not (item.name == "Omniverse" or item.path.startswith("omniverse:"))
    return is_yaml_file(item.path)


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""

        # Events
        self._usd_context = omni.usd.get_context()
        self._physxIFace = _physx.acquire_physx_interface()
        self._physx_subscription = None
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

        # Build Window
        self._window = ui.Window(
            title=EXTENSION_NAME, width=600, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        # UI
        self._models = {}
        self._ext_id = ext_id
        self._menu_items = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        # self._menu_items = [MenuItemDescription(name="Workflows", sub_menu=menu_items)]
        add_menu_items(self._menu_items, "Isaac Utils")

        # Selection
        self._new_window = True
        self.new_selection = True
        self._selected_index = None
        self._selected_prim_path = None
        self._prev_art_prim_path = None

        # Articulation
        self.articulation = None
        self.num_dof = 0
        self.dof_names = []
        self.upper_joint_limits = np.zeros(MAX_DOF_NUM)
        self.lower_joint_limits = np.zeros(MAX_DOF_NUM)

        # Animation
        self._set_joint_positions_on_step = False

        # Sphere generation
        self._selected_link = None
        self._sphere_gen_link_2_mesh = OrderedDict()
        self._preview_spheres = True

        # Connect Spheres
        self._connect_sphere_0_options = []
        self._connect_sphere_1_options = []

        # Link Visibility
        self._hiding_link = False
        self._hiding_robot = False
        self._prev_link = None

        # Active Joints
        self._joint_positions = np.zeros(MAX_DOF_NUM)
        self._active_joints = np.zeros(MAX_DOF_NUM, dtype=bool)

        self._collision_sphere_editor = CollisionSphereEditor()

    def on_shutdown(self):
        self._show_robot_if_hidden()
        self._collision_sphere_editor.clear_spheres(store_op=False)
        self._collision_sphere_editor.clear_preview()
        self._usd_context = None
        self._stage_event_sub = None
        self._timeline_event_sub = None
        self._physx_subscription = None
        self._models = {}
        remove_menu_items(self._menu_items, "Isaac Utils")
        if self._window:
            self._window = None
        gc.collect()

    def _on_window(self, visible):
        if self._window.visible:
            # Subscribe to Stage and Timeline Events
            self._usd_context = omni.usd.get_context()
            events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
            stream = self._timeline.get_timeline_event_stream()
            self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)

            self._build_ui()
            if not self._new_window and self.articulation:
                self._refresh_ui(self.articulation)
            self._new_window = False
        else:
            self._usd_context = None
            self._stage_event_sub = None
            self._timeline_event_sub = None

    def _menu_callback(self):
        self._window.visible = not self._window.visible
        # Update the Selection Box if the Timeline is already playing
        if self._timeline.is_playing():
            self._refresh_selection_combobox()

    def _build_ui(self):
        # if not self._window:
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):

                self._build_info_ui()

                self._build_selection_ui()

                self._build_command_ui()

                self._build_editor_ui()

                self._build_tools_ui()

        async def dock_window():
            await omni.kit.app.get_app().next_update_async()

            def dock(space, name, location, pos=0.5):
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_NAME, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    def _on_selection(self, prim_path):
        """Creates an Articulation Object from the selected articulation prim path.
           Updates the UI with the Selected articulation.

        Args:
            prim_path (string): path to selected articulation
        """
        if prim_path == self._prev_art_prim_path:
            return
        else:
            self._prev_art_prim_path = prim_path

        self.new_selection = True
        self._prev_link = None

        if self.articulation_list and prim_path != "None":

            # Create and Initialize the Articulation
            self.articulation = Articulation(prim_path)
            if not self.articulation.handles_initialized:
                self.articulation.initialize()

            # Get list of all links and populate link selection combobox
            self.get_all_sphere_gen_meshes()
            self._refresh_sphere_gen_link_combobox()

            # Update the entire UI with the selected articulaiton
            self._refresh_ui(self.articulation)

            # start event subscriptions
            if not self._physx_subscription:
                self._physx_subscription = self._physxIFace.subscribe_physics_step_events(self._on_physics_step)

        # Deselect and Reset
        else:
            if self.articulation is not None:
                self._show_robot_if_hidden()
                self._reset_ui()
                self._refresh_selection_combobox()
                self._refresh_sphere_gen_link_combobox()
            self.articulation = None
            # carb.log_warn("Resetting Articulation Inspector")

    def _on_combobox_selection(self, model=None, val=None):
        # index = model.get_item_value_model().as_int
        index = self._models["ar_selection_model"].get_item_value_model().as_int
        if index >= 0 and index < len(self.articulation_list):
            self._selected_index = index
            item = self.articulation_list[index]
            self._selected_prim_path = item
            self._on_selection(item)

    def _refresh_selection_combobox(self):
        self.articulation_list = self.get_all_articulations()
        if self._prev_art_prim_path is not None and self._prev_art_prim_path not in self.articulation_list:
            self._reset_ui()
        self._models["ar_selection_model"] = DynamicComboBoxModel(self.articulation_list)
        self._models["ar_selection_combobox"].model = self._models["ar_selection_model"]
        self._models["ar_selection_combobox"].model.add_item_changed_fn(self._on_combobox_selection)
        # If something was already selected, reselect after refresh
        if self._selected_index is not None and self._selected_prim_path is not None:
            # If the item is still in the articulation list
            if self._selected_prim_path in self.articulation_list:
                self._models["ar_selection_combobox"].model.set_item_value_model(
                    ui.SimpleIntModel(self._selected_index)
                )

    def _clear_selection_combobox(self):
        self._selected_index = None
        self._selected_prim_path = None
        self.articulation_list = []
        self._models["ar_selection_model"] = DynamicComboBoxModel(self.articulation_list)
        self._models["ar_selection_combobox"].model = self._models["ar_selection_model"]
        self._models["ar_selection_combobox"].model.add_item_changed_fn(self._on_combobox_selection)

    def _on_select_sphere_gen_link(self, model, val):
        index = model.get_item_value_model().as_int
        item = list(self._sphere_gen_link_2_mesh.keys())[index]
        self._selected_sphere_gen_link = item
        self._models["sphere_gen_mesh_selection_model"] = DynamicComboBoxModel(self._sphere_gen_link_2_mesh[item])
        self._models["sphere_gen_mesh_selection_model_combobox"].model = self._models["sphere_gen_mesh_selection_model"]
        self._models["sphere_gen_mesh_selection_model"].add_item_changed_fn(
            self._trigger_preview_generate_spheres_for_link
        )
        self._generate_spheres_for_link()

        self._refresh_collision_sphere_comboboxes()

        self._collision_sphere_editor.set_sphere_colors(self._get_selected_link_path())

        if self._hiding_link != self._hiding_robot:
            self._hide_link(self._get_selected_link())
            if self._prev_link is not None:
                self._hide_link(self._prev_link)

        self._prev_link = self._get_selected_link()

    def get_all_articulations(self):
        """Get all the articulation objects from the Stage.

        Returns:
            list(str): list of prim_paths as strings
        """
        articulations = ["None"]
        stage = self._usd_context.get_stage()
        if stage:
            for prim in Usd.PrimRange(stage.GetPrimAtPath("/")):
                path = str(prim.GetPath())
                # Get prim type get_prim_object_type
                type = get_prim_object_type(path)
                if type == "articulation":
                    articulations.append(path)

        return articulations

    def _refresh_sphere_gen_link_combobox(self):
        self._models["sphere_gen_link_selection_model"] = DynamicComboBoxModel(
            list(self._sphere_gen_link_2_mesh.keys())
        )
        self._models["sphere_gen_link_selection_model_combobox"].model = self._models["sphere_gen_link_selection_model"]
        self._models["sphere_gen_link_selection_model_combobox"].model.add_item_changed_fn(
            self._on_select_sphere_gen_link
        )
        self._on_select_sphere_gen_link(self._models["sphere_gen_link_selection_model"], None)

        self._refresh_collision_sphere_comboboxes()

    def _refresh_collision_sphere_comboboxes(self, keep_sphere_selection=False):
        sphere_0_name, _ = self._get_selected_collision_spheres()

        sphere_names = self._collision_sphere_editor.get_sphere_names_by_link(self._get_selected_link_path())
        self._connect_sphere_0_options = sphere_names
        self._models["connect_sphere_selection_0"] = DynamicComboBoxModel(sphere_names)
        self._models["connect_sphere_selection_0_combobox"].model = self._models["connect_sphere_selection_0"]
        self._models["connect_sphere_selection_0"].add_item_changed_fn(self._on_collision_sphere_select_0)

        # Keep currently selected collision sphere when reloading if it still exists
        if keep_sphere_selection and sphere_0_name in sphere_names:
            self._models["connect_sphere_selection_0"].get_item_value_model().set_value(
                int(sphere_names.index(sphere_0_name))
            )

        self._on_collision_sphere_select_0(None, None)

    def _on_collision_sphere_select_0(self, model, val):
        sphere_0_name, sphere_1_name = self._get_selected_collision_spheres()
        if sphere_0_name is not None:
            sphere_names = self._connect_sphere_0_options
            pruned_names = sphere_names[:]  # shallow copy
            pruned_names.pop(sphere_names.index(sphere_0_name))
        else:
            pruned_names = []

        self._connect_sphere_1_options = pruned_names

        name = "connect_sphere_selection_1"
        self._models[name] = DynamicComboBoxModel(pruned_names)
        self._models[name + "_combobox"].model = self._models[name]

        if sphere_1_name in pruned_names:
            self._models[name].get_item_value_model().set_value(int(pruned_names.index(sphere_1_name)))

    def get_all_sphere_gen_meshes(self):
        stage = self._usd_context.get_stage()
        self._sphere_gen_link_2_mesh = OrderedDict()
        if stage and self.articulation is not None:
            for prim in Usd.PrimRange(stage.GetPrimAtPath(self.articulation.prim_path)):
                path = str(prim.GetPath())
                # Get prim type get_prim_object_type
                type = get_prim_object_type(path)

                if type == "xform":
                    geom_mesh = UsdGeom.Mesh(prim)
                    if geom_mesh.GetPointsAttr().HasValue():
                        rel_path = path[len(self.articulation.prim_path) :]
                        div_index = rel_path[1:].find("/") + 1
                        key = rel_path[:div_index]
                        l = self._sphere_gen_link_2_mesh.get(key, [])
                        l.append(rel_path[div_index:])
                        self._sphere_gen_link_2_mesh[key] = l

    def get_articulation_values(self, articulation):
        """Get and store the latest dof_properties from the articulation.
           Update the Properties UI.

        Args:
            articulation (Articulation): Selected Articulation
        """
        # Update static dof properties on new selection
        if self.new_selection:
            self.num_dof = articulation.num_dof
            self.dof_names = articulation.dof_names
            self.new_selection = False

            self._joint_positions = articulation.get_joint_positions()
            self._active_joints = np.zeros(MAX_DOF_NUM, dtype=bool)

            self.lower_joint_limits = articulation.dof_properties["lower"]
            self.upper_joint_limits = articulation.dof_properties["upper"]

    def _refresh_ui(self, articulation):
        """Updates the GUI with a new Articulation's properties.

        Args:
            articulation (Articulation): [description]
        """
        # Get the latest articulation values and update the Properties UI
        self.get_articulation_values(articulation)

        self._update_editor_ui()

        self._models["frame_command_ui"].collapsed = False
        self._models["frame_command_ui"].enabled = True

        self._models["sphere_editor_ui"].collapsed = False
        self._models["sphere_editor_ui"].enabled = True

        self._models["editor_tools_ui"].collapsed = False
        self._models["editor_tools_ui"].enabled = True

        self._models["save_spheres_ui"].enabled = True

        self._models["load_spheres_ui"].enabled = True

        self._update_command_ui()

    def _reset_ui(self):
        """Reset / Hide UI Elements.
        """
        self._clear_selection_combobox()
        self._prev_art_prim_path = None

        self._show_robot_if_hidden()

        # Reset & Disable Button
        self._models["frame_command_ui"].collapsed = True
        self._models["frame_command_ui"].enabled = False

        for i in range(MAX_DOF_NUM):
            self._models[f"joint_{i}_frame"].visible = False

        self._models["sphere_editor_ui"].collapsed = True
        self._models["sphere_editor_ui"].enabled = False

        self._models["editor_tools_ui"].collapsed = True
        self._models["editor_tools_ui"].enabled = False

        self._models["save_spheres_ui"].collapsed = True
        self._models["save_spheres_ui"].enabled = False

        self._models["load_spheres_ui"].collapsed = True
        self._models["load_spheres_ui"].enabled = False

    ##################################
    # Callbacks
    ##################################

    def _on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """

        # On every stage event check if any articulations have been added/removed from the Stage
        self._refresh_selection_combobox()

        if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
            # self._on_selection_changed()
            self._collision_sphere_editor.copy_all_sphere_data()
            self._refresh_collision_sphere_comboboxes(keep_sphere_selection=True)
            pass

        elif event.type == int(omni.usd.StageEventType.OPENED) or event.type == int(omni.usd.StageEventType.CLOSED):
            # stage was opened or closed, cleanup
            self._physx_subscription = None

    def _on_physics_step(self, step):
        """Callback for Physics Step.
           
        Args:
            step ([type]): [description]
        """
        if self.articulation is not None:
            if not self.articulation.handles_initialized:
                self.articulation.initialize()
            # Get the latest values from the articulation
            self.get_articulation_values(self.articulation)

            # Handle animation
            if self._set_joint_positions_on_step:
                self._set_joint_positions(step)
        return

    def _on_timeline_event(self, e):
        """Callback for Timeline Events

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """

        if e.type == int(omni.timeline.TimelineEventType.PLAY):
            # BUG: get_all_articulations returns ['None'] after STOP/PLAY <-- articulations show up as xforms
            self._refresh_selection_combobox()
        elif e.type == int(omni.timeline.TimelineEventType.STOP):
            self._reset_ui()

    def _set_joint_positions(self, step):
        if self.articulation is not None:
            joint_velocities = np.zeros_like(self._joint_positions)
            self.articulation.set_joint_positions(self._joint_positions)
            self.articulation.set_joint_velocities(joint_velocities)
        self._set_joint_positions_on_step = False
        return

    ##################################
    # UI Builders
    ##################################

    def _build_info_ui(self):
        title = EXTENSION_NAME
        doc_link = "https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html"

        overview = "This utility is used to help generate a Lula robot_description.yaml file required to use Lula-based algorithms like RmpFlow, RRT, and Lula Kinematics.  "
        overview += "A Lula robot_description file contains a collision sphere representation of the robot that is used for collision avoidance, and information that is required to interpret the robot URDF.\n\n"

        overview += (
            "To begin using this editor, load a robot USD file onto the stage and press the 'Play' button.  In the 'Selection Panel', select your robot from the 'Select Articulation' drop-down menu.  "
            + "The 'Select Link' drop-down menu will populate once an Articulation has been selected.  The user may create collision spheres for the robot one link at a time. \n\n"
        )

        overview += (
            "Command Panel:\nIn the Command Panel, the user may select the default positions of robot joints and choose a subset of joints that are considered 'Active Joints'. "
            + "'Active Joints' are considered by Lula to be directly controllable, while 'Fixed Joints' are assumed to never move."
            + "The default positions that 'Active Joints' are set to are used by Lula algorithms to resolve null-space behavior.  For example, RmpFlow is typically configured to control"
            + "only the joints in a robot arm, and assume the gripper to be in a fixed position.  While moving the gripper to a target, it will choose a path that moves the robot close to"
            + " the default 'Active Joints' configuration.  By default, all joints are marked as 'Fixed Joints', which will cause Lula not to control the robot at all.  The user must determine"
            + "a set of joints that should be considered 'Active'.\n\n"
        )

        overview += (
            "Adding Collision Spheres:\nIn the 'Link Sphere Editor' panel paired with 'Editor Tools', the user may add collision spheres on a per-link basis.  Spheres are added with positions specified relative to the base of the "
            + "selected link, with their position relative to the link being fixed.  Once a sphere has been created, the user may move it around, resize or delete it on the USD stage until it looks right.  "
            + "Additionally, the user may generate spheres for a link automatically or select any two spheres under a link and linearly interpolate to create more spheres connecting them.  In general,"
            + " the user will want to fully cover the robot in spheres, using around 40-60 spheres total.  It is easiest to create such a set of spheres when individual spheres are allowed to slightly exceed"
            + " the volume of the robot. \n\n"
        )

        overview += "Importing and Exporting:\nThe user may import a pre-existing robot_description YAML file in the 'Import Robot Description File' panel.  And the user may export their work to a yaml file using the `Export Robot Description File` panel."

        setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)

    def _build_selection_ui(self):
        frame = ui.CollapsableFrame(
            title="Selection Panel",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                # Create a dynamic ComboBox for Articulation Selection
                self.articulation_list = []
                self._models["ar_selection_model"] = DynamicComboBoxModel(self.articulation_list)
                with ui.HStack():
                    ui.Label(
                        "Select Articulation",
                        width=LABEL_WIDTH,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="Select Articulation",
                    )
                    self._models["ar_selection_combobox"] = ui.ComboBox(self._models["ar_selection_model"])
                    add_line_rect_flourish(False)
                self._models["ar_selection_combobox"].model.add_item_changed_fn(self._on_combobox_selection)

                name = "sphere_gen_link_selection_model"
                self._models[name] = DynamicComboBoxModel(list(self._sphere_gen_link_2_mesh.keys()))

                with ui.HStack():
                    ui.Label(
                        "Select Link",
                        width=LABEL_WIDTH,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="Select under which to generate spheres.  Only links with nested meshes can be chosen",
                    )
                    self._models[name + "_combobox"] = ui.ComboBox(self._models[name])
                    add_line_rect_flourish(False)

                self._models[name + "_combobox"].model.add_item_changed_fn(self._on_select_sphere_gen_link)

    def _build_command_ui(self):
        self._models["frame_command_ui"] = ui.CollapsableFrame(
            title="Command Panel",
            name="Command Panel",
            height=0,
            collapsed=True,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        self._models["frame_command_ui"].enabled = False

        with self._models["frame_command_ui"]:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                frame = ui.CollapsableFrame(
                    title="Set Joint Positions",
                    name="subFrame",
                    height=0,
                    collapsed=False,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        for i in range(MAX_DOF_NUM):
                            name = f"joint_{i}_frame"
                            frame = ui.CollapsableFrame(
                                title=name,
                                name="subFrame",
                                height=0,
                                collapsed=False,
                                style=get_style(),
                                style_type_name_override="CollapsableFrame",
                                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                            )
                            if self.articulation is None:
                                frame.visible = False
                            self._models[f"joint_{i}_frame"] = frame

                            kwargs = {
                                "label": "Joint Position",
                                "default_val": 0,
                                "tooltip": f"Desired Position for Robot Joint: {i}",
                            }

                            def on_set_joint_position(i, model):
                                model.set_max(self.upper_joint_limits[i])
                                model.set_min(self.lower_joint_limits[i])
                                joint_val = model.get_value_as_float()
                                if self.upper_joint_limits[i] < joint_val:
                                    joint_val = self.upper_joint_limits[i]
                                    model.set_value(float(joint_val + 1))
                                    return
                                elif self.lower_joint_limits[i] > joint_val:
                                    joint_val = self.lower_joint_limits[i]
                                    model.set_value(float(joint_val - 1))
                                    return

                                self._joint_positions[i] = joint_val
                                self._set_joint_positions_on_step = True

                            def update_active_joints(i, model):
                                self._active_joints[i] = model.get_item_value_model().as_int

                            with frame:
                                with ui.VStack(style=get_style(), spacing=5, height=0):
                                    self._models["joint_{}_position".format(i)] = float_builder(**kwargs)
                                    self._models["joint_{}_position".format(i)].add_value_changed_fn(
                                        lambda model, index=i: on_set_joint_position(index, model)
                                    )

                                    with ui.HStack():
                                        name = f"joint_{i}_status"
                                        self._models[name] = DynamicComboBoxModel(["Fixed Joint", "Active Joint"])

                                        ui.Label(
                                            "Joint Status",
                                            width=LABEL_WIDTH,
                                            alignment=ui.Alignment.LEFT_CENTER,
                                            tooltip="Active Joint: Lula will directly control this joint, using a default position equal to the value set above.\n"
                                            + "Fixed Joint: Lula will assume a fixed position of the joint equal to the value set above.",
                                        )
                                        self._models[name + "_combobox"] = ui.ComboBox(self._models[name])
                                        add_line_rect_flourish(False)

                                        self._models[name + "_combobox"].model.add_item_changed_fn(
                                            lambda model, value, index=i: update_active_joints(index, model)
                                        )

    def _build_editor_ui(self):
        self._models["sphere_editor_ui"] = ui.CollapsableFrame(
            title="Link Sphere Editor",
            height=0,
            collapsed=True,
            style=get_style(),
            name="editorFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        self._models["sphere_editor_ui"].enabled = False

        with self._models["sphere_editor_ui"]:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                ###################################################################
                #                          Generate Spheres
                ###################################################################

                frame = ui.CollapsableFrame(
                    title="Generate Spheres",
                    name="subFrame",
                    height=0,
                    collapsed=True,
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                self._models["sphere_generator_ui"] = frame
                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        num_sphere_kwargs = {
                            "label": "Number of Spheres",
                            "default_val": 0,
                            "tooltip": "Number of Spheres to Generate for Link",
                        }

                        rad_offset_kwargs = {
                            "label": "Radius Offset",
                            "default_val": 0.01,
                            "tooltip": "Extent to which spheres may extend beyond the mesh.  A positive value means that spheres may exceed the mesh by up to the given value.\n A negative value specifies that all spheres are at least radius_offset from the mesh surface.",
                        }

                        with frame:
                            with ui.VStack(style=get_style(), spacing=5, height=0):
                                name = "sphere_gen_mesh_selection_model"
                                self._models[name] = DynamicComboBoxModel([])

                                with ui.HStack():
                                    ui.Label(
                                        "Select Mesh",
                                        width=LABEL_WIDTH,
                                        alignment=ui.Alignment.LEFT_CENTER,
                                        tooltip="Select Mesh to be Used for Sphere Generation",
                                    )
                                    self._models[name + "_combobox"] = ui.ComboBox(self._models[name])
                                    add_line_rect_flourish(False)

                                self._models["sphere_gen_num_spheres"] = int_builder(**num_sphere_kwargs)
                                self._models["sphere_gen_num_spheres"].add_value_changed_fn(
                                    self._trigger_preview_generate_spheres_for_link
                                )
                                self._models["sphere_gen_radius_offset"] = float_builder(**rad_offset_kwargs)
                                self._models["sphere_gen_radius_offset"].add_value_changed_fn(
                                    self._trigger_preview_generate_spheres_for_link
                                )

                                self._models["sphere_gen_preview"] = state_btn_builder(
                                    label="Preview Spheres",
                                    b_text="Show Preview",
                                    a_text="Hide Preview",
                                    tooltip="Show a preview of the spheres that will be generated.",
                                    on_clicked_fn=self._preview_collision_spheres,
                                )

                                def generate_spheres():
                                    self._generate_spheres_for_link(preview=False)
                                    self._refresh_collision_sphere_comboboxes(keep_sphere_selection=True)
                                    self._models["sphere_gen_num_spheres"].set_value(int(0))

                                self._models["sphere_gen_add"] = btn_builder(
                                    label="Generate Spheres",
                                    text="Generate Spheres",
                                    tooltip="Generate Spheres for Robot Link",
                                    on_clicked_fn=generate_spheres,
                                )

                ###################################################################
                #                            Add Sphere
                ###################################################################

                frame = ui.CollapsableFrame(
                    title="Add Sphere",
                    name="subFrame",
                    height=0,
                    collapsed=True,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )

                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        kwargs = {"label": "Radius", "default_val": 0.1, "min": 0.001, "tooltip": "Desired Radius"}
                        self._models["add_sphere_radius"] = float_builder(**kwargs)

                        kwargs = {
                            "label": "Relative Translation",
                            "tooltip": "Relative translation of sphere in the local frame of the selected Prim path.",
                            "axis_count": 3,
                            "default_val": [0.0, 0.0, 0.0],
                        }

                        val_models = xyz_builder(**kwargs)
                        self._models["add_sphere_translation_x"] = val_models[0]
                        self._models["add_sphere_translation_y"] = val_models[1]
                        self._models["add_sphere_translation_z"] = val_models[2]

                        def on_add_sphere():
                            radius = self._models["add_sphere_radius"].get_value_as_float()
                            translation = np.zeros(3)
                            translation[0] = self._models["add_sphere_translation_x"].get_value_as_float()
                            translation[1] = self._models["add_sphere_translation_y"].get_value_as_float()
                            translation[2] = self._models["add_sphere_translation_z"].get_value_as_float()
                            link_path = self._get_selected_link_path()

                            self._collision_sphere_editor.add_sphere(link_path, translation, radius)
                            self._refresh_collision_sphere_comboboxes(keep_sphere_selection=True)

                        self._models["add_sphere_btn"] = btn_builder(
                            "Add Sphere", text="Add Sphere", on_clicked_fn=on_add_sphere
                        )

                ###################################################################
                #                           Connect Spheres
                ###################################################################
                frame = ui.CollapsableFrame(
                    title="Connect Spheres",
                    name="subFrame",
                    height=0,
                    collapsed=True,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )

                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):

                        name = "connect_sphere_selection_0"
                        self._models[name] = DynamicComboBoxModel([])

                        with ui.HStack():
                            ui.Label(
                                "Select Collision Sphere",
                                width=LABEL_WIDTH,
                                alignment=ui.Alignment.LEFT_CENTER,
                                tooltip="Select First Collision Sphere to Connect",
                            )
                            self._models[name + "_combobox"] = ui.ComboBox(self._models[name])
                            add_line_rect_flourish(False)
                        self._models[name + "_combobox"].model.add_item_changed_fn(self._on_collision_sphere_select_0)

                        name = "connect_sphere_selection_1"
                        self._models[name] = DynamicComboBoxModel([])

                        with ui.HStack():
                            ui.Label(
                                "Select Collision Sphere",
                                width=LABEL_WIDTH,
                                alignment=ui.Alignment.LEFT_CENTER,
                                tooltip="Select First Collision Sphere to Connect",
                            )
                            self._models[name + "_combobox"] = ui.ComboBox(self._models[name])
                            add_line_rect_flourish(False)

                        kwargs = {
                            "label": "Number of Spheres",
                            "default_val": 0,
                            "tooltip": "Create the specified number of spheres interpolated between the selected spheres",
                        }
                        self._models["connect_sphere_num"] = int_builder(**kwargs)

                        def on_connect_spheres():
                            c0, c1 = self._get_selected_collision_spheres()
                            link_path = self._get_selected_link_path()
                            if c1 is None:
                                carb.log_warn("Please select two distinct collision spheres to Connect Spheres")

                            num = self._models["connect_sphere_num"].get_value_as_int()

                            self._collision_sphere_editor.interpolate_spheres(link_path + c0, link_path + c1, num)
                            self._refresh_collision_sphere_comboboxes(keep_sphere_selection=True)

                        self._models["connect_sphere_btn"] = btn_builder(
                            "Connect Spheres", text="Connect Spheres", on_clicked_fn=on_connect_spheres
                        )
                        self._models["connect_sphere_btn"].enabled = True

                ###################################################################
                #                           Scale Spheres
                ###################################################################
                frame = ui.CollapsableFrame(
                    title="Scale Spheres in Link",
                    name="subFrame",
                    height=0,
                    collapsed=True,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )

                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        kwargs = {
                            "label": "Scaling Factor",
                            "default_val": 1.0,
                            "min": 0.001,
                            "tooltip": "Scaling factor for the radii of the specified spheres",
                        }
                        self._models["scale_spheres_factor"] = float_builder(**kwargs)

                        def on_scale_spheres():
                            path = self._get_selected_link_path()
                            factor = self._models["scale_spheres_factor"].get_value_as_float()

                            self._collision_sphere_editor.scale_spheres(path, factor)

                        self._models["scale_sphere_btn"] = btn_builder(
                            "Scale Spheres", text="Scale Spheres", on_clicked_fn=on_scale_spheres
                        )
                        self._models["scale_sphere_btn"].enabled = True

                ###################################################################
                #                           Clear Spheres
                ###################################################################
                frame = ui.CollapsableFrame(
                    title="Clear Spheres in Link",
                    name="subFrame",
                    height=0,
                    collapsed=True,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):

                        def clear_link_spheres_fn():
                            self._collision_sphere_editor.clear_link_spheres(self._get_selected_link_path())
                            self._refresh_collision_sphere_comboboxes()

                        self._models["link_clear_btn"] = btn_builder(
                            "Clear Link Spheres", text="Clear", on_clicked_fn=clear_link_spheres_fn
                        )
                        self._models["link_clear_btn"].enabled = True

    def _build_tools_ui(self):
        self._models["editor_tools_ui"] = ui.CollapsableFrame(
            title="Editor Tools",
            height=0,
            collapsed=True,
            style=get_style(),
            name="editorFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        self._models["editor_tools_ui"].enabled = False

        with self._models["editor_tools_ui"]:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                self._models["undo_btn"] = btn_builder(
                    "Undo", text="Undo", on_clicked_fn=self._collision_sphere_editor.undo
                )
                self._models["undo_btn"].enabled = True

                self._models["redo_btn"] = btn_builder(
                    "Redo", text="Redo", on_clicked_fn=self._collision_sphere_editor.redo
                )
                self._models["redo_btn"].enabled = True

                kwargs = {
                    "label": "Toggle Link Visibility",
                    "a_text": " Hide",
                    "b_text": "Show",
                    "tooltip": "Hide the Selected Link",
                    "on_clicked_fn": self._on_toggle_link_visible,
                }
                self._models["hide_link_btn"] = state_btn_builder(**kwargs)

                kwargs = {
                    "label": "Toggle Robot Visibility",
                    "a_text": "Hide",
                    "b_text": "Show",
                    "tooltip": "Hide the Robot",
                    "on_clicked_fn": self._on_toggle_robot_visible,
                }
                self._models["hide_robot_btn"] = state_btn_builder(**kwargs)

                def on_link_color_change(a1, a2):
                    sphere_color = []
                    for item in self._models["link_color_picker"].get_item_children():
                        val = self._models["link_color_picker"].get_item_value_model(item).get_value_as_float()
                        sphere_color.append(val)
                    sphere_color = np.array(sphere_color[:3])
                    self._collision_sphere_editor.set_sphere_colors(
                        self._get_selected_link_path(), color_in=sphere_color
                    )

                kwargs = {
                    "label": "Link Sphere Color",
                    "default_val": self._collision_sphere_editor.filter_in_sphere_color,
                    "tooltip": "Set the color of all collision spheres in the selected link",
                }
                self._models["link_color_picker"] = color_picker_builder(**kwargs)
                self._models["link_color_picker"].add_end_edit_fn(on_link_color_change)

                def on_color_change(a1, a2):
                    sphere_color = []
                    for item in self._models["color_picker"].get_item_children():
                        val = self._models["color_picker"].get_item_value_model(item).get_value_as_float()
                        sphere_color.append(val)
                    sphere_color = np.array(sphere_color[:3])
                    self._collision_sphere_editor.set_sphere_colors(
                        self._get_selected_link_path(), color_out=sphere_color
                    )

                kwargs = {
                    "label": "Base Sphere Color",
                    "default_val": self._collision_sphere_editor.filter_out_sphere_color,
                    "tooltip": "Set the color of all collision spheres outside the selected link",
                }
                self._models["color_picker"] = color_picker_builder(**kwargs)
                self._models["color_picker"].add_end_edit_fn(on_color_change)

                def clear_spheres_fn():
                    self._collision_sphere_editor.clear_spheres()
                    self._refresh_collision_sphere_comboboxes()

                self._models["clear_btn"] = btn_builder(
                    "Clear All Spheres", text="Clear", on_clicked_fn=clear_spheres_fn
                )
                self._models["clear_btn"].enabled = True

                frame = ui.CollapsableFrame(
                    title="Scale All Spheres",
                    name="subFrame",
                    height=0,
                    collapsed=True,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        kwargs = {
                            "label": "Scaling Factor",
                            "default_val": 1.0,
                            "min": 0.001,
                            "tooltip": "Scaling factor for the radii of the specified spheres",
                        }
                        self._models["scale_all_spheres_factor"] = float_builder(**kwargs)

                        def on_scale_all_spheres():
                            path = self.articulation.prim_path
                            factor = self._models["scale_all_spheres_factor"].get_value_as_float()

                            self._collision_sphere_editor.scale_spheres(path, factor)

                        self._models["scale_all_sphere_btn"] = btn_builder(
                            "Scale All Spheres", text="Scale All Spheres", on_clicked_fn=on_scale_all_spheres
                        )
                        self._models["scale_all_sphere_btn"].enabled = True

        ###################################################################
        #                            Save to File
        ###################################################################
        frame = ui.CollapsableFrame(
            title="Export Robot Description File",
            name="subFrame",
            height=0,
            collapsed=True,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        self._models["save_spheres_ui"] = frame
        frame.enabled = False

        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def check_file_type(model=None):
                    path = model.get_value_as_string()
                    if is_yaml_file(path) and "omniverse:" not in path.lower():
                        self._models["export_btn"].enabled = True
                    else:
                        self._models["export_btn"].enabled = False
                        carb.log_warn(f"Invalid path to Robot Desctiption YAML: {path}")

                kwargs = {
                    "label": "Output File",
                    "default_val": "",
                    "tooltip": "Click the Folder Icon to Set Filepath",
                    "use_folder_picker": True,
                    "item_filter_fn": on_filter_item,
                    "folder_dialog_title": "Write all sphere to a YAML file",
                    "folder_button_title": "Select YAML",
                }
                self._models["output_file"] = str_builder(**kwargs)
                self._models["output_file"].add_value_changed_fn(check_file_type)

                self._models["export_btn"] = btn_builder(
                    "Save", text="Save", on_clicked_fn=self._save_robot_description_file
                )
                self._models["export_btn"].enabled = False

        ###################################################################
        #                   Import Robot Description File
        ###################################################################

        frame = ui.CollapsableFrame(
            title="Import Robot Description File",
            name="subFrame",
            height=0,
            collapsed=True,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        self._models["load_spheres_ui"] = frame
        frame.enabled = False

        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def check_file_type(model=None):
                    path = model.get_value_as_string()
                    if is_yaml_file(path) and "omniverse:" not in path.lower() and self.articulation is not None:
                        self._models["import_btn"].enabled = True
                    elif self.articulation is None:
                        self._models["import_btn"].enabled = False
                        carb.log_warn(
                            "Robot Articulation must be selected in the Selection Panel in order to import spheres for a robot"
                        )
                    else:
                        self._models["import_btn"].enabled = False
                        carb.log_warn(f"Invalid path to Robot Desctiption YAML: {path}")

                kwargs = {
                    "label": "Input File",
                    "default_val": "",
                    "tooltip": "Click the Folder Icon to Set Filepath",
                    "use_folder_picker": True,
                    "item_filter_fn": on_filter_item,
                    "folder_dialog_title": "Select Robot Description YAML file, clearing all spheres",
                    "folder_button_title": "Select YAML",
                }
                self._models["input_file"] = str_builder(**kwargs)
                self._models["input_file"].add_value_changed_fn(check_file_type)

                self._models["import_btn"] = btn_builder(
                    "Import", text="Import", on_clicked_fn=self._load_robot_description_file
                )
                self._models["import_btn"].enabled = False

    def _update_editor_ui(self):
        self._models["sphere_editor_ui"].collapsed = False
        self._models["sphere_editor_ui"].visible = True

        if is_yaml_file(self._models["input_file"].get_value_as_string()):
            self._models["import_btn"].enabled = True

        self._refresh_collision_sphere_comboboxes()

    def _update_command_ui(self):
        if self.articulation is None:
            return

        self._models["frame_command_ui"].enabled = True

        for i, joint_name in enumerate(self.dof_names):
            self._models[f"joint_{i}_frame"].title = joint_name
            self._models[f"joint_{i}_frame"].visible = True

            self._models[f"joint_{i}_position"].set_value(float(self._joint_positions[i]))

            self._models[f"joint_{i}_status"].get_item_value_model().set_value(int(self._active_joints[i]))

        for i in range(self.num_dof, MAX_DOF_NUM):
            self._models[f"joint_{i}_frame"].visible = False

    def _trigger_preview_generate_spheres_for_link(self, model=None, val=None):
        self._generate_spheres_for_link()

    def _generate_spheres_for_link(self, preview=True):
        if preview and not self._preview_spheres:
            return

        link = self._get_selected_link()
        mesh_index = self._models["sphere_gen_mesh_selection_model"].get_item_value_model().as_int
        mesh = self._sphere_gen_link_2_mesh[link][mesh_index]

        num_spheres = self._models["sphere_gen_num_spheres"].get_value_as_int()
        if num_spheres <= 0:
            self._collision_sphere_editor.clear_preview()
            return

        radius_offset = self._models["sphere_gen_radius_offset"].get_value_as_float()

        link_path = self.articulation.prim_path + link
        mesh_path = link_path + mesh
        geom_mesh = UsdGeom.Mesh(get_prim_at_path(mesh_path))
        points = np.array(geom_mesh.GetPointsAttr().Get())
        face_inds = np.array(geom_mesh.GetFaceVertexIndicesAttr().Get())
        vert_cts = np.array(geom_mesh.GetFaceVertexCountsAttr().Get())

        # Transform coordinates of points into Link frame
        mesh_xform = XFormPrim(mesh_path)
        link_xform = XFormPrim(link_path)

        mesh_trans, mesh_rot = mesh_xform.get_world_pose()
        link_trans, link_rot = link_xform.get_world_pose()
        link_rot, mesh_rot = quats_to_rot_matrices(np.array([link_rot, mesh_rot]))

        inv_rot = link_rot.T @ mesh_rot
        inv_trans = (link_rot.T @ (mesh_trans - link_trans)).reshape((3, 1))

        link_frame_points = (inv_rot @ points.T + inv_trans).T

        self._collision_sphere_editor.generate_spheres(
            link_path, link_frame_points, face_inds, vert_cts, num_spheres, radius_offset, preview
        )

    def _get_selected_collision_spheres(self):
        if not self._connect_sphere_0_options:
            return None, None

        name = "connect_sphere_selection_0"
        c0 = self._connect_sphere_0_options[self._models[name].get_item_value_model().as_int]

        if not self._connect_sphere_1_options:
            return c0, None

        name = "connect_sphere_selection_1"
        c1 = self._connect_sphere_1_options[self._models[name].get_item_value_model().as_int]
        return c0, c1

    def _get_selected_link_path(self):
        link = self._get_selected_link()
        if link is None:
            return None
        return self.articulation.prim_path + link

    def _get_selected_link(self):
        if self.articulation is None:
            return None

        link_index = self._models["sphere_gen_link_selection_model"].get_item_value_model().as_int
        link = list(self._sphere_gen_link_2_mesh.keys())[link_index]

        return link

    def _preview_collision_spheres(self, model=None):
        if self._preview_spheres:
            self._preview_spheres = False
            self._collision_sphere_editor.clear_preview()
        else:
            self._preview_spheres = True
            self._generate_spheres_for_link()

    def _hide_link(self, link_name):
        meshes = self._sphere_gen_link_2_mesh[link_name]
        link_path = self.articulation.prim_path + link_name
        mesh_paths = []
        for mesh in meshes:
            mesh_path = link_path + mesh
            mesh_paths.append(mesh_path)
        omni.kit.commands.execute("ToggleVisibilitySelectedPrims", selected_paths=mesh_paths)

    def _on_toggle_link_visible(self, model=None):
        self._hide_link(self._get_selected_link())
        self._hiding_link = not self._hiding_link

    def _on_toggle_robot_visible(self, model=None):
        selected_link = self._get_selected_link()
        links = list(self._sphere_gen_link_2_mesh.keys())
        for link in links:
            if selected_link != link:
                self._hide_link(link)

        self._hiding_robot = not self._hiding_robot

    def _show_robot_if_hidden(self):
        if self._hiding_robot:
            self._models["hide_robot_btn"].call_clicked_fn()
        if self._hiding_link:
            self._models["hide_link_btn"].call_clicked_fn()

    def _load_robot_description_file(self, model=None):
        if self.articulation is None:
            return
        path = self._models["input_file"].get_value_as_string()

        with open(path, "r") as stream:
            try:
                parsed_file = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                carb.log_error("Attempted to load invalid yaml file " + str(exc))

        self._active_joints = np.zeros(MAX_DOF_NUM, dtype=bool)
        dof_names = np.array(self.dof_names)

        cspace = parsed_file["cspace"]
        default_q = parsed_file["default_q"]

        in_mask = np.in1d(cspace, dof_names)
        if not np.all(in_mask):
            carb.log_warn(
                "Some joints listed in the cspace of the provided robot_description YAML file are not present in the robot Articulation:"
                + f" {cspace[~in_mask]}"
            )
            cspace = cspace[in_mask]

        for i, joint in enumerate(cspace):
            ind = self.dof_names.index(joint)
            self._active_joints[ind] = True
            self._joint_positions[ind] = default_q[i]

        fixed_joints = parsed_file["cspace_to_urdf_rules"]
        if fixed_joints is not None:
            for item in fixed_joints:
                if item["rule"] != "fixed":
                    continue
                joint_name = item["name"]
                if joint_name not in self.dof_names:
                    carb.log_warn(
                        f"Fixed joint specified for a joint that is not present in the robot Articulation: {joint_name}"
                    )
                    return
                ind = self.dof_names.index(item["name"])
                self._active_joints[ind] = False
                self._joint_positions[ind] = item["value"]

        self._collision_sphere_editor.load_spheres(self.articulation, path)

        self._update_command_ui()

    def _save_robot_description_file(self, model=None):
        if self.articulation is None:
            return

        active_joints_mask = self._active_joints[: self.num_dof]
        if np.sum(active_joints_mask) == 0:
            carb.log_error(
                "There are no Active Joints in this robot description (Reference the Information Panel subsection: Command Panel).  This means that Lula will not control the robot at all.  Aborting Save Operation."
            )
            return

        fixed_joints_mask = ~active_joints_mask

        dof_names = np.array(self.dof_names)

        path = self._models["output_file"].get_value_as_string()
        if not path:
            carb.log_error(f"Cannot Save to Invalid Path {path}")
            return

        with open(path, "w") as f:
            f.write(
                "# The robot description defines the generalized coordinates and how to map those\n"
                + "# to the underlying URDF dofs.\n\n"
                + "api_version: 1.0\n\n"
                + "# Defines the generalized coordinates. Each generalized coordinate is assumed\n"
                + "# to have an entry in the URDF.\n"
                + "# Lula will only use these joints to control the robot position.\n"
                + "cspace:\n"
            )
            for name in dof_names[active_joints_mask]:
                f.write(f"    - {name}\n")

            f.write("default_q: [\n")
            f.write("    ")
            for joint_pos in self._joint_positions[active_joints_mask][:-1]:
                pos = np.around(joint_pos, 4)
                f.write(f"{str(pos)},")
            f.write(f"{str(np.around(self._joint_positions[active_joints_mask][-1],4))}\n")
            f.write("]\n\n")

            f.write("# Most dimensions of the cspace have a direct corresponding element\n")
            f.write("# in the URDF. This list of rules defines how unspecified coordinates\n")
            f.write("# should be extracted or how values in the URDF should be overwritten.\n\n")

            f.write("cspace_to_urdf_rules:\n")
            for name, position in zip(dof_names[fixed_joints_mask], self._joint_positions[fixed_joints_mask]):
                pos = np.around(position, 4)
                f.write(f"    - {{name: {name}, rule: fixed, value: {str(pos)}}}\n")
            f.write("\n")

            f.write("# Lula uses collision spheres to define the robot geometry in order to avoid\n")
            f.write("# collisions with external obstacles.  If no spheres are specified, Lula will\n")
            f.write("# not be able to avoid obstacles.\n\n")

            self._collision_sphere_editor.save_spheres(self.articulation, f)

        def _get_urdf_root_link(self):
            from omni.isaac.urdf import _urdf

            urdf_path = self._models["urdf_file"].get_value_as_string()

            urdf_interface = _urdf.acquire_urdf_interface()

            # setup config params
            import_config = _urdf.ImportConfig()
            import_config.set_merge_fixed_joints(False)
            import_config.set_fix_base(True)

            # parse and import file
            # imported_robot = urdf_interface.parse_urdf(urdf_path, "robot_urdf", import_config)

            imported_robot = omni.kit.commands.execute(
                "URDFParseFile", urdf_path=urdf_path, import_config=import_config
            )

            # TODO: Decide if this function needs to get completed and used or deleted
