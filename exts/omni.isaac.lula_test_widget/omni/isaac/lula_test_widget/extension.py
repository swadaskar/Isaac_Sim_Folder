# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import weakref
import asyncio
import gc
import carb
import omni
from pxr import Usd
from omni.kit.window.property.templates import LABEL_WIDTH
import omni.ui as ui
import omni.usd
import omni.timeline
import omni.kit.commands
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.core.utils.prims import get_prim_object_type
from omni.isaac.core.articulations import Articulation

from omni.isaac.ui.widgets import DynamicComboBoxModel

from .test_scenarios import LulaTestScenarios

from omni.isaac.ui.ui_utils import (
    add_line_rect_flourish,
    btn_builder,
    state_btn_builder,
    float_builder,
    setup_ui_headers,
    get_style,
    str_builder,
)
from omni.kit.window.extensions import SimpleCheckBox
import omni.physx as _physx
import numpy as np
import os

EXTENSION_NAME = "Lula Test Widget"

MAX_DOF_NUM = 100


def is_yaml_file(path: str):
    _, ext = os.path.splitext(path.lower())
    return ext in [".yaml", ".YAML"]


def is_urdf_file(path: str):
    _, ext = os.path.splitext(path.lower())
    return ext in [".urdf", ".URDF"]


def on_filter_yaml_item(item) -> bool:
    if not item or item.is_folder:
        return not (item.name == "Omniverse" or item.path.startswith("omniverse:"))
    return is_yaml_file(item.path)


def on_filter_urdf_item(item) -> bool:
    if not item or item.is_folder:
        return not (item.name == "Omniverse" or item.path.startswith("omniverse:"))
    return is_urdf_file(item.path)


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
        self.link_names = []

        # Lula Config Files
        self._selected_robot_description_file = None
        self._selected_robot_urdf_file = None
        self._robot_description_file = None
        self._robot_urdf_file = None
        self._ee_frame_options = []

        self._rmpflow_config_yaml = None

        # Lula Test Scenarios
        self._test_scenarios = LulaTestScenarios()

        # Visualize End Effector
        self._visualize_end_effector = True

    def on_shutdown(self):
        self._test_scenarios.full_reset()
        self.articulation = None
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

                self._build_kinematics_ui()

                self._build_trajectory_generation_ui()

                self._build_rmpflow_ui()

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

            # Update the entire UI with the selected articulaiton
            self._refresh_ui(self.articulation)

            # start event subscriptions
            if not self._physx_subscription:
                self._physx_subscription = self._physxIFace.subscribe_physics_step_events(self._on_physics_step)

        # Deselect and Reset
        else:
            if self.articulation is not None:
                self._reset_ui()
                self._refresh_selection_combobox()
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

    def _refresh_ee_frame_combobox(self):
        if self._robot_description_file is not None and self._robot_urdf_file is not None:
            self._test_scenarios.initialize_ik_solver(self._robot_description_file, self._robot_urdf_file)
            ee_frames = self._test_scenarios.get_ik_frames()

        else:
            ee_frames = []

        name = "ee_frame"
        self._models[name] = DynamicComboBoxModel(ee_frames)
        self._models[name + "_combobox"].model = self._models[name]

        if len(ee_frames) > 0:
            self._models[name].get_item_value_model().set_value(len(ee_frames) - 1)

        self._models[name].add_item_changed_fn(self._reset_scenario)

        self._ee_frame_options = ee_frames

    def _reset_scenario(self, model=None, value=None):
        self._enable_lula_dropdowns()
        self._set_enable_trajectory_panel(False)

        if self.articulation is not None:
            self.articulation.post_reset()

    def _refresh_ui(self, articulation):
        """Updates the GUI with a new Articulation's properties.

        Args:
            articulation (Articulation): [description]
        """
        # Get the latest articulation values and update the Properties UI
        self.get_articulation_values(articulation)

        if is_yaml_file(self._models["input_robot_description_file"].get_value_as_string()):
            self._enable_load_button

    def _reset_ui(self):
        """Reset / Hide UI Elements.
        """
        self._clear_selection_combobox()
        self._test_scenarios.full_reset()
        self._prev_art_prim_path = None
        self._visualize_end_effector = True

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

            action = self._get_next_action()
            self.articulation.get_articulation_controller().apply_action(action)

        return

    def _get_next_action(self):
        if self._test_scenarios.scenario_name == "Sinusoidal Target":
            w_xy = self._models["rmpflow_follow_sinusoid_w_xy"].get_value_as_float()
            w_z = self._models["rmpflow_follow_sinusoid_w_z"].get_value_as_float()
            rad_z = self._models["rmpflow_follow_sinusoid_rad_z"].get_value_as_float()
            rad_xy = self._models["rmpflow_follow_sinusoid_rad_xy"].get_value_as_float()
            height = self._models["rmpflow_follow_sinusoid_height"].get_value_as_float()

            return self._test_scenarios.get_next_action(w_xy=w_xy, w_z=w_z, rad_z=rad_z, rad_xy=rad_xy, height=height)
        else:
            return self._test_scenarios.get_next_action()

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

    ##################################
    # UI Builders
    ##################################

    def _build_info_ui(self):
        title = EXTENSION_NAME
        doc_link = "https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html"

        overview = "This utility is used to help generate and refine the collision sphere representation of a robot.  "
        overview += "Select the Articulation for which you would like to edit spheres from the dropdown menu.  Then select a link from the robot Articulation to begin using the Sphere Editor."
        overview += "\n\nPress the 'Open in IDE' button to view the source code."

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

                # Select Robot Description YAML file

                def check_file_type(model=None):
                    path = model.get_value_as_string()
                    if is_yaml_file(path):
                        self._selected_robot_description_file = model.get_value_as_string()
                        self._enable_load_button()
                    else:
                        self._selected_robot_description_file = None
                        carb.log_warn(f"Invalid path to Robot Desctiption YAML: {path}")

                kwargs = {
                    "label": "Robot Description YAML",
                    "default_val": "",
                    "tooltip": "Click the Folder Icon to Set Filepath",
                    "use_folder_picker": True,
                    "item_filter_fn": on_filter_yaml_item,
                    "folder_dialog_title": "Select Robot Description YAML file",
                    "folder_button_title": "Select YAML",
                }
                self._models["input_robot_description_file"] = str_builder(**kwargs)
                self._models["input_robot_description_file"].add_value_changed_fn(check_file_type)

                # Select Robot URDF file

                def check_urdf_file_type(model=None):
                    path = model.get_value_as_string()
                    if is_urdf_file(path):
                        self._selected_robot_urdf_file = model.get_value_as_string()
                        self._enable_load_button()
                    else:
                        self._selected_robot_urdf_file = None
                        carb.log_warn(f"Invalid path to Robot URDF: {path}")

                kwargs = {
                    "label": "Robot URDF",
                    "default_val": "",
                    "tooltip": "Click the Folder Icon to Set Filepath",
                    "use_folder_picker": True,
                    "item_filter_fn": on_filter_urdf_item,
                    "folder_dialog_title": "Select Robot URDF file",
                    "folder_button_title": "Select URDF",
                }
                self._models["input_robot_urdf_file"] = str_builder(**kwargs)
                self._models["input_robot_urdf_file"].add_value_changed_fn(check_urdf_file_type)

                # Load the currently selected config files
                def on_load_config(model=None, val=None):
                    self._robot_description_file = self._selected_robot_description_file
                    self._robot_urdf_file = self._selected_robot_urdf_file
                    self._refresh_ee_frame_combobox()
                    self._enable_lula_dropdowns()
                    self._set_enable_trajectory_panel(False)

                self._models["load_config_btn"] = btn_builder(
                    label="Load Selected Config",
                    text="Load",
                    tooltip="Load the selected Lula config files",
                    on_clicked_fn=on_load_config,
                )

                # Select End Effector Frame Name
                name = "ee_frame"
                self._models[name] = DynamicComboBoxModel([])

                with ui.HStack():
                    ui.Label(
                        "Select End Effector Frame",
                        width=LABEL_WIDTH,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="End Effector Frame to Use when following a target",
                    )
                    self._models[name + "_combobox"] = ui.ComboBox(self._models[name])
                    add_line_rect_flourish(False)

                self._models[name].add_item_changed_fn(self._reset_scenario)

                # Button for ignoring IK targets
                def on_clicked_fn(use_orientation):
                    self._test_scenarios.set_use_orientation(use_orientation)

                with ui.HStack(width=0):
                    label = "Use Orientation Targets"
                    ui.Label(label, width=LABEL_WIDTH - 12, alignment=ui.Alignment.LEFT_TOP)
                    cb = ui.SimpleBoolModel(default_value=1)
                    SimpleCheckBox(1, on_clicked_fn, model=cb)

                # Button for visualizing end effector
                def on_vis_ee_clicked_fn(visualize_ee):
                    self._visalize_end_effector = visualize_ee
                    if visualize_ee:
                        self._test_scenarios.visualize_ee_frame(self.articulation, self._get_selected_ee_frame())
                    else:
                        self._test_scenarios.stop_visualize_ee_frame()

                with ui.HStack(width=0):
                    label = "Visualize End Effector Pose"
                    ui.Label(label, width=LABEL_WIDTH - 12, alignment=ui.Alignment.LEFT_TOP)
                    cb = ui.SimpleBoolModel(default_value=1)
                    SimpleCheckBox(1, on_vis_ee_clicked_fn, model=cb)

    def _build_kinematics_ui(self):
        frame = ui.CollapsableFrame(
            title="Lula Kinematics Solver",
            height=0,
            collapsed=True,
            enabled=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        self._models["kinematics_frame"] = frame

        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def ik_follow_target(model=None):
                    ee_frame = self._get_selected_ee_frame()
                    self.articulation.post_reset()
                    self._test_scenarios.on_ik_follow_target(self.articulation, ee_frame)

                self._models["kinematics_follow_target_btn"] = btn_builder(
                    label="Follow Target",
                    text="Follow Target",
                    tooltip="Use IK to follow a target",
                    on_clicked_fn=ik_follow_target,
                )

    def _build_trajectory_generation_ui(self):
        frame = ui.CollapsableFrame(
            title="Lula Trajectory Generator",
            height=0,
            collapsed=True,
            enabled=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        self._models["trajectory_frame"] = frame

        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def on_custom_trajectory(model=None, val=None):
                    self.articulation.post_reset()
                    self._test_scenarios.on_custom_trajectory(self._robot_description_file, self._robot_urdf_file)
                    self._set_enable_trajectory_panel(True)

                self._models["custom_trajectory_btn"] = btn_builder(
                    label="Custom Trajectory",
                    text="Custom Trajectory",
                    tooltip="Create a basic customizable trajectory and unlock the Custom Trajectory Panel",
                    on_clicked_fn=on_custom_trajectory,
                )

                frame = ui.CollapsableFrame(
                    title="Custom Trajectory Panel",
                    height=0,
                    collapsed=True,
                    enabled=False,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )

                self._models["trajectory_panel"] = frame

                def follow_trajectory(model=None, val=None):
                    self._test_scenarios.create_trajectory_controller(self.articulation, self._get_selected_ee_frame())

                def on_add_waypoint(model=None, val=None):
                    self._test_scenarios.add_waypoint()

                def on_delete_waypoint(model=None, val=None):
                    self._test_scenarios.delete_waypoint()

                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        self._models["follow_trajectory_btn"] = btn_builder(
                            label="Follow Trajectory",
                            text="Follow Trajectory",
                            tooltip="Follow the trajectory shown in front of the robot",
                            on_clicked_fn=follow_trajectory,
                        )

                        self._models["add_trajectory_waypoint_btn"] = btn_builder(
                            label="Add Waypoint",
                            text="Add Waypoint",
                            tooltip="Add waypoint to trajectory",
                            on_clicked_fn=on_add_waypoint,
                        )

                        self._models["remove_trajectory_waypoint_btn"] = btn_builder(
                            label="Remove Waypoint",
                            text="Remove Waypoint",
                            tooltip="Remove waypoint from trajectory",
                            on_clicked_fn=on_delete_waypoint,
                        )

    def _build_rmpflow_ui(self):
        frame = ui.CollapsableFrame(
            title="RmpFlow",
            height=0,
            collapsed=True,
            enabled=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        self._models["rmpflow_frame"] = frame

        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def check_file_type(model=None):
                    path = model.get_value_as_string()
                    if is_yaml_file(path):
                        self._rmpflow_config_yaml = model.get_value_as_string()
                        self._set_enable_rmpflow_buttons(True)
                    else:
                        self._rmpflow_config_yaml = None
                        self._set_enable_rmpflow_buttons(False)
                        carb.log_warn(f"Invalid path to RmpFlow config YAML: {path}")

                kwargs = {
                    "label": "RmpFlow Config YAML",
                    "default_val": "",
                    "tooltip": "Click the Folder Icon to Set Filepath",
                    "use_folder_picker": True,
                    "item_filter_fn": on_filter_yaml_item,
                    "folder_dialog_title": "Select RmpFlow config YAML file",
                    "folder_button_title": "Select YAML",
                }
                self._models["input_rmp_config_file"] = str_builder(**kwargs)
                self._models["input_rmp_config_file"].add_value_changed_fn(check_file_type)
                # TODO: remove hard coded line below
                # self._rmpflow_config_yaml = (
                #     "/home/arudich/Desktop/Denso/Cobotta_Pro_900_Assets/cobotta_rmpflow_config_final.yaml"
                # )

                def toggle_rmpflow_debug_mode(model=None):
                    self._test_scenarios.toggle_rmpflow_debug_mode()

                self._models["rmpflow_debug_mode"] = state_btn_builder(
                    label="Debugger",
                    a_text="Debugging Mode",
                    b_text="Normal Mode",
                    tooltip="Toggle Debugging Mode",
                    on_clicked_fn=toggle_rmpflow_debug_mode,
                )

                ######################################################
                #                    Follow Target
                ######################################################

                def rmpflow_follow_target(model=None):
                    ee_frame = self._get_selected_ee_frame()
                    rmpflow_config_dict = {
                        "end_effector_frame_name": ee_frame,
                        "maximum_substep_size": 0.0034,
                        "ignore_robot_state_updates": False,
                        "robot_description_path": self._robot_description_file,
                        "urdf_path": self._robot_urdf_file,
                        "rmpflow_config_path": self._rmpflow_config_yaml,
                    }
                    self.articulation.post_reset()
                    self._test_scenarios.on_rmpflow_follow_target_obstacles(self.articulation, **rmpflow_config_dict)

                self._models["rmpflow_follow_target_btn"] = btn_builder(
                    label="Follow Target",
                    text="Follow Target",
                    tooltip="Use RmpFlow to follow a target",
                    on_clicked_fn=rmpflow_follow_target,
                )
                self._models["rmpflow_follow_target_btn"].enabled = False

                #######################################################
                #                Sinusoidal Target
                #######################################################

                def rmpflow_follow_sinusoidal_target(model=None):
                    ee_frame = self._get_selected_ee_frame()
                    rmpflow_config_dict = {
                        "end_effector_frame_name": ee_frame,
                        "maximum_substep_size": 0.0034,
                        "ignore_robot_state_updates": False,
                        "robot_description_path": self._robot_description_file,
                        "urdf_path": self._robot_urdf_file,
                        "rmpflow_config_path": self._rmpflow_config_yaml,
                    }
                    self.articulation.post_reset()
                    self._test_scenarios.on_rmpflow_follow_sinusoidal_target(self.articulation, **rmpflow_config_dict)

                self._models["rmpflow_follow_sinusoid_btn"] = btn_builder(
                    label="Follow Sinusoid",
                    text="Follow Sinusoid",
                    tooltip="Use RmpFlow to follow a rotating sinusoidal target",
                    on_clicked_fn=rmpflow_follow_sinusoidal_target,
                )
                self._models["rmpflow_follow_sinusoid_btn"].enabled = False

                frame = ui.CollapsableFrame(
                    title="Sinusoid Parameters",
                    height=0,
                    collapsed=True,
                    enabled=False,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )

                self._models["rmpflow_sinusoidal_target_frame"] = frame

                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):

                        self._models["rmpflow_follow_sinusoid_w_z"] = float_builder(
                            label="Vertical Wave Frequency",
                            default_val=0.05,
                            tooltip="Speed [rad/sec] at which the target makes vertical oscilations",
                        )
                        self._models["rmpflow_follow_sinusoid_rad_z"] = float_builder(
                            label="Vertical Wave Radius", default_val=0.2, tooltip="Height [m] of vertical oscilations"
                        )

                        self._models["rmpflow_follow_sinusoid_w_xy"] = float_builder(
                            label="Z Axis Rotation Frequency",
                            default_val=0.05,
                            tooltip="Speed [rad/sec] at which the target makes a full circle about the z axis",
                        )
                        self._models["rmpflow_follow_sinusoid_rad_xy"] = float_builder(
                            label="Distance From Origin",
                            default_val=0.5,
                            tooltip="Distance on the XY plane from the origin [m] of the target",
                        )

                        self._models["rmpflow_follow_sinusoid_height"] = float_builder(
                            label="Sinusoid Height", default_val=0.5, tooltip="Average height of target [m]"
                        )

    def _disable_lula_dropdowns(self):
        frame_names = ["kinematics_frame", "trajectory_frame", "rmpflow_frame", "trajectory_panel"]
        for n in frame_names:
            frame = self._models[n]
            frame.enabled = False
            frame.collapsed = True

    def _enable_load_button(self):
        self._models["load_config_btn"].enabled = True

    def _enable_lula_dropdowns(self):
        if self.articulation is None or self._robot_description_file is None or self._robot_urdf_file is None:
            return

        frame_names = ["kinematics_frame", "trajectory_frame", "rmpflow_frame"]
        for n in frame_names:
            frame = self._models[n]
            frame.enabled = True

        self._test_scenarios.scenario_reset()
        self._test_scenarios.initialize_ik_solver(self._robot_description_file, self._robot_urdf_file)

        if self._visualize_end_effector:
            self._test_scenarios.visualize_ee_frame(self.articulation, self._get_selected_ee_frame())

    def _set_enable_trajectory_panel(self, enable):
        frame = self._models["trajectory_panel"]
        frame.enabled = enable
        frame.collapsed = not enable

    def _set_enable_rmpflow_buttons(self, enable):
        self._models["rmpflow_follow_target_btn"].enabled = enable
        self._models["rmpflow_follow_sinusoid_btn"].enabled = enable

    def _get_selected_ee_frame(self):
        name = "ee_frame"
        return self._ee_frame_options[self._models[name].get_item_value_model().as_int]
