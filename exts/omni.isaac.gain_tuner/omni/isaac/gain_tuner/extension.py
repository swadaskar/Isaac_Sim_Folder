# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
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
import random
import omni
from omni.isaac.core.utils.types import ArticulationAction
from pxr import Usd
from omni.kit.window.property.templates import LABEL_WIDTH
import omni.ui as ui
import omni.usd
import omni.timeline
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.prims import get_prim_object_type
from omni.isaac.core.articulations import Articulation

from omni.isaac.ui.widgets import DynamicComboBoxModel

from omni.isaac.ui.ui_utils import (
    add_line_rect_flourish,
    btn_builder,
    float_builder,
    setup_ui_headers,
    get_style,
    state_btn_builder,
)
import omni.physx as _physx
import numpy as np


EXTENSION_NAME = "Gain Tuner"

MAX_DOF_NUM = 100


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
            title=EXTENSION_NAME, width=500, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        # UI
        self._models = {}
        self._ext_id = ext_id
        menu_items = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        self._menu_items = [MenuItemDescription(name="Workflows", sub_menu=menu_items)]
        add_menu_items(self._menu_items, "Isaac Utils")

        # Selection
        self.new_selection = True
        self._selected_index = None
        self._selected_prim_path = None
        self._force_gains_update = False

        # Articulation
        self.articulation = None
        self.num_dof = None
        self.dof_names = None

        # Animation
        self._send_joint_target_pos = False
        self._random_joint_positions = None
        self._send_joint_target_vels = False
        self._random_joint_velocities = None

    def on_shutdown(self):
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

                self._build_gains_ui()

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
        self.new_selection = True

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

            # Enable Buttons / Layouts in GUI
            self._models["gains_zero_all_btn"].enabled = True
            self._models["disable_gravity_btn"].enabled = True
            self._models["gains_scalar_kp_btn"].enabled = True
            self._models["gains_scalar_kd_btn"].enabled = True
            self._models["randomize_joint_target_pos_btn"].enabled = True
            self._models["send_joint_target_pos_btn"].enabled = True
            self._models["randomize_joint_target_vels_btn"].enabled = True
            self._models["send_joint_target_vels_btn"].enabled = True

        # Deselect and Reset
        else:
            if self.articulation is not None:
                self._reset_ui()
                self._refresh_selection_combobox()
            self.articulation = None
            if self.num_dof is not None:
                self._toggle_gains_callbacks(False)
            # carb.log_warn("Resetting Articulation Inspector")

    def _on_combobox_selection(self, model, val):
        index = model.get_item_value_model().as_int
        if index >= 0 and index < len(self.articulation_list):
            self._selected_index = index
            item = self.articulation_list[index]
            self._selected_prim_path = item
            self._on_selection(item)

    def _refresh_selection_combobox(self):
        self.articulation_list = self.get_all_articulations()
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
                # carb.log_warn(f"{path}:\t{type}")
                if type == "articulation":
                    articulations.append(path)
        # carb.log_warn(f"ALL ARTICULATIONS:\t{articulations}")
        return articulations

    def get_articulation_values(self, articulation):
        """Get and store the latest dof_properties from the articulation.
           Update the Properties UI.

        Args:
            articulation (Articulation): Selected Articulation
        """
        # Update static dof properties on new selection
        if self.new_selection:
            self.stiffness = articulation.dof_properties["stiffness"]
            self.damping = articulation.dof_properties["damping"]
            self.num_dof = articulation.num_dof
            self.dof_names = articulation.dof_names
            self.types = articulation.dof_properties["type"]
            self.lower_limits = articulation.dof_properties["lower"]
            self.upper_limits = articulation.dof_properties["upper"]
            self.max_efforts = articulation.dof_properties["maxEffort"]
            self.new_selection = False

            self._random_joint_positions = articulation.get_joint_positions()
            self._random_joint_velocities = articulation.get_joint_velocities()

        else:
            # Use gains from UI after initial update
            if self._force_gains_update:
                self.articulation.get_articulation_controller().set_gains(self.stiffness, self.damping, True)
                self._force_gains_update = False
            # else:
            #     # Check if the gains have been updated externally from a different extension
            #     if not (self.stiffness == stiffness).all():
            #         for i in range(self.num_dof):
            #             if self.stiffness[i] != stiffness[i]:
            #                 self._models[f"gains_{i}_kp_field"].set_value(stiffness[i])
            #         self.stiffness = stiffness

            #     if not (self.damping == damping).all():
            #         for i in range(self.num_dof):
            #             if self.damping[i] != damping[i]:
            #                 self._models[f"gains_{i}_kd_field"].set_value(damping[i])
            #         self.damping = damping

            self.articulation.get_articulation_controller().set_gains(self.stiffness, self.damping)

    def _refresh_ui(self, articulation):
        """Updates the GUI with a new Articulation's properties.

        Args:
            articulation (Articulation): [description]
        """
        # Get the latest articulation values and update the Properties UI
        self.get_articulation_values(articulation)

        # Hide the dof and gains frames in case the selected articulation
        # has a different number of joints
        for i in range(MAX_DOF_NUM):
            # self.dof_frames[i].visible = False
            self.gains_frames[i].visible = False

        # self._update_controllers_ui()
        self._models["frame_command_ui"].collapsed = False
        self._models["frame_gains_ui"].collapsed = False
        self._update_gains_ui()

        # Turn controller ui callbacks back on
        self._toggle_gains_callbacks(True)

    def _reset_ui(self):
        """Reset / Hide UI Elements.
        """
        self._clear_selection_combobox()

        self._random_joint_positions = None
        self._random_joint_velocities = None

        # Reset & Disable Button
        self._models["gains_zero_all_btn"].enabled = False
        self._models["disable_gravity_btn"].text = "DISABLE"
        self._models["disable_gravity_btn"].enabled = False
        self._models["gains_scalar_kp_btn"].enabled = False
        self._models["gains_scalar_kd_btn"].enabled = False
        self._models["randomize_joint_target_pos_btn"].enabled = False
        self._send_joint_target_pos = False
        self._models["send_joint_target_pos_btn"].text = "START"
        self._models["send_joint_target_pos_btn"].enabled = False
        self._models["randomize_joint_target_vels_btn"].enabled = False
        self._send_joint_target_vels = False
        self._models["send_joint_target_vels_btn"].text = "START"
        self._models["send_joint_target_vels_btn"].enabled = False

        for i in range(MAX_DOF_NUM):
            # self.dof_frames[i].visible = False
            self.gains_frames[i].visible = False

        self._models["frame_command_ui"].collapsed = True
        self._models["frame_gains_ui"].collapsed = True

    ##################################
    # Callbacks
    ##################################

    def _on_gains_value_changed(self, name, model, id):
        """Callback for when Gains Slider is modified.

        Args:
            name (string): ui.Label.text
            model (ui.AbstractValueModel): ui.FloatField model
            id (int): Index of DOF
        """
        if self.num_dof is not None:
            if id > -1 and id < self.num_dof:
                val = model.get_value_as_float()
                if "kp" in name.lower():
                    name = "kp"
                    self.stiffness[id] = val
                elif "kd" in name.lower():
                    name = "kd"
                    self.damping[id] = val
                else:
                    carb.log_warn(f"VALUE FROM UNKNOWN INPUT: {name}, {id}, {val}")

                # Update the Articulation
                if self.articulation is not None:
                    self.articulation.get_articulation_controller().set_gains(self.stiffness, self.damping, True)
                    # self.update_properties_ui_dynamic()
                else:
                    carb.log_warn("Invalid Articulation.")

    def _toggle_gains_callbacks(self, val):
        """Add / Remove callbacks from DOF and Gains UI

        Args:
            val (bool): Toggle flag
        """
        for i in range(self.num_dof):
            # Add / Remove callbacks from Gains UI
            for name in self._gains_keys:
                key = f"gains_{i}_" + name
                if val:
                    self._models[key + "_fn"] = self._models[key + "_field"].add_value_changed_fn(
                        lambda m, n=name, id=i: self._on_gains_value_changed(n, m, id)
                    )
                else:
                    self._models[key + "_field"].remove_value_changed_fn(self._models[key + "_fn"])

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

            # Handle animation
            if self._send_joint_target_pos or self._send_joint_target_vels:
                self._send_joint_targets(step)
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

    def _send_joint_targets(self, step):
        if self.articulation is not None:
            joint_positions = None
            joint_velocities = None
            if self._send_joint_target_pos:
                joint_positions = self._random_joint_positions
            if self._send_joint_target_vels:
                joint_velocities = self._random_joint_velocities
            self.articulation.get_articulation_controller().apply_action(
                ArticulationAction(joint_positions=joint_positions, joint_velocities=joint_velocities)
            )
        return

    ##################################
    # UI Builders
    ##################################

    def _build_info_ui(self):
        title = EXTENSION_NAME
        doc_link = "https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html"

        overview = "This utility is used to help tune the gains of an articulation.  "
        overview += "Select the Articulation you would like to tune from the dropdown menu."
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
                        tooltip="Select Articulation to Inspect",
                    )
                    self._models["ar_selection_combobox"] = ui.ComboBox(self._models["ar_selection_model"])
                    add_line_rect_flourish(False)
                self._models["ar_selection_combobox"].model.add_item_changed_fn(self._on_combobox_selection)

    def _build_command_ui(self):
        self._models["frame_command_ui"] = ui.CollapsableFrame(
            title="Command Panel",
            name="groupFrame",
            height=0,
            collapsed=True,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with self._models["frame_command_ui"]:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                frame = ui.CollapsableFrame(
                    title="Test Joint Positions",
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

                        def on_randomize_joint_target_pos():
                            if self.articulation is not None:
                                joint_targets = []
                                for i in range(self.num_dof):
                                    joint_targets.append(random.uniform(self.upper_limits[i], self.lower_limits[i]))
                                self._random_joint_positions = joint_targets
                            else:
                                carb.log_warn("Invalid Articulation.")
                            return

                        kwargs = {
                            "label": "Randomize Position Targets",
                            "text": "RANDOMIZE",
                            "tooltip": "Randomize Joint Position Targets",
                            "on_clicked_fn": on_randomize_joint_target_pos,
                        }
                        self._models["randomize_joint_target_pos_btn"] = btn_builder(**kwargs)
                        self._models["randomize_joint_target_pos_btn"].enabled = False

                        def on_send_joint_target_pos(val):
                            self._send_joint_target_pos = val

                        kwargs = {
                            "label": "Send Position Targets",
                            "a_text": "START",
                            "b_text": "STOP",
                            "tooltip": "Send Random Position Targets to the Robot's Controller",
                            "on_clicked_fn": on_send_joint_target_pos,
                        }
                        self._models["send_joint_target_pos_btn"] = state_btn_builder(**kwargs)
                        self._models["send_joint_target_pos_btn"].enabled = False

                frame = ui.CollapsableFrame(
                    title="Test Joint Velocities",
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

                        def on_randomize_joint_target_vels():
                            if self.articulation is not None:
                                joint_velocities = []
                                for i in range(self.num_dof):
                                    joint_velocities.append(random.uniform(-10, 10))
                                self._random_joint_velocities = joint_velocities
                            else:
                                carb.log_warn("Invalid Articulation.")
                            return

                        kwargs = {
                            "label": "Randomize Velocity Targets",
                            "text": "RANDOMIZE",
                            "tooltip": "Randomize Joint Velocity Targets",
                            "on_clicked_fn": on_randomize_joint_target_vels,
                        }
                        self._models["randomize_joint_target_vels_btn"] = btn_builder(**kwargs)
                        self._models["randomize_joint_target_vels_btn"].enabled = False

                        def on_send_joint_target_vels(val):
                            self._send_joint_target_vels = val

                        kwargs = {
                            "label": "Send Velocity Targets",
                            "a_text": "START",
                            "b_text": "STOP",
                            "tooltip": "Send Random Velocity Targets to the Robot's Controller",
                            "on_clicked_fn": on_send_joint_target_vels,
                        }
                        self._models["send_joint_target_vels_btn"] = state_btn_builder(**kwargs)
                        self._models["send_joint_target_vels_btn"].enabled = False

    def _build_gains_ui(self):
        self._models["frame_gains_ui"] = ui.CollapsableFrame(
            title="Gains Panel",
            height=0,
            collapsed=True,
            style=get_style(),
            name="groupFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with self._models["frame_gains_ui"]:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                frame = ui.CollapsableFrame(
                    title="Reset Gains",
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

                        def on_disable_gravity(val):
                            if self.articulation is not None:
                                if val:
                                    self.articulation.disable_gravity()
                                else:
                                    self.articulation.enable_gravity()

                        kwargs = {
                            "label": "Disable Gravity",
                            "a_text": "Disable",
                            "b_text": "Enable",
                            "tooltip": "Disable Gravity for the articulation while tuning",
                            "on_clicked_fn": on_disable_gravity,
                        }
                        self._models["disable_gravity_btn"] = state_btn_builder(**kwargs)
                        self._models["disable_gravity_btn"].enabled = False

                        def on_zero_all_gains():
                            for i in range(self.num_dof):
                                self._models[f"gains_{i}_kp_field"].set_value(0.001)
                                self._models[f"gains_{i}_kd_field"].set_value(0.0001)

                        kwargs = {
                            "label": "Zero All Gains",
                            "text": "Zero",
                            "tooltip": "Zero all joint gains for tuning",
                            "on_clicked_fn": on_zero_all_gains,
                        }
                        self._models["gains_zero_all_btn"] = btn_builder(**kwargs)
                        self._models["gains_zero_all_btn"].enabled = False

                frame = ui.CollapsableFrame(
                    title="Scale Gains",
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
                            "label": "Stiffness Multiplier",
                            "default_val": 10,
                            "tooltip": "Multiplier for Scaling All Stiffness Gains",
                        }
                        self._models["gains_kp_scalar"] = float_builder(**kwargs)

                        def on_scale_gains_kp():
                            scalar = self._models["gains_kp_scalar"].get_value_as_float()
                            for i in range(self.num_dof):
                                val = self._models[f"gains_{i}_kp_field"].get_value_as_float()
                                self._models[f"gains_{i}_kp_field"].set_value(val * scalar)
                            return

                        kwargs = {
                            "label": "Scale Stiffness Gains",
                            "text": "Scale",
                            "tooltip": "Scale All Stiffness (kp) Gains by the Multiplier",
                            "on_clicked_fn": on_scale_gains_kp,
                        }
                        self._models["gains_scalar_kp_btn"] = btn_builder(**kwargs)
                        self._models["gains_scalar_kp_btn"].enabled = False

                        kwargs = {
                            "label": "Damping Multiplier",
                            "default_val": 10,
                            "tooltip": "Multiplier for Scaling All Damping Gains",
                        }
                        self._models["gains_kd_scalar"] = float_builder(**kwargs)

                        def on_scale_gains_kd():
                            scalar = self._models["gains_kd_scalar"].get_value_as_float()
                            for i in range(self.num_dof):
                                # val = self._models[f"gains_{i}_kp_field"].get_value_as_float()
                                # self._models[f"gains_{i}_kp_field"].set_value(val * scalar)
                                val = self._models[f"gains_{i}_kd_field"].get_value_as_float()
                                self._models[f"gains_{i}_kd_field"].set_value(val * scalar)
                            return

                        kwargs = {
                            "label": "Scale Damping Gains",
                            "text": "Scale",
                            "tooltip": "Scale All Damping (kd) Gains by the Multiplier",
                            "on_clicked_fn": on_scale_gains_kd,
                        }
                        self._models["gains_scalar_kd_btn"] = btn_builder(**kwargs)
                        self._models["gains_scalar_kd_btn"].enabled = False

                self.gains_frames = []
                # Add the Gain Pairs per joint
                for i in range(MAX_DOF_NUM):
                    name = f"DOF {i}"
                    frame = ui.CollapsableFrame(
                        title=name,
                        height=0,
                        collapsed=False,
                        style=get_style(),
                        name="subFrame",
                        horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    )
                    frame.visible = False
                    self.gains_frames.append(frame)

                    with frame:
                        with ui.VStack(style=get_style(), spacing=5, height=0):

                            self._gains_keys = ["kp", "kd"]
                            gains_labels = ["Stiffness (kp)", "Damping (kd)"]

                            for j in range(len(self._gains_keys)):
                                name = self._gains_keys[j]
                                label = gains_labels[j]
                                kwargs = {"label": label, "tooltip": "DOF " + label, "step": 0.0001, "format": "%.4f"}
                                self._models[f"gains_{i}_" + name + "_field"] = float_builder(**kwargs)

    def _update_gains_ui(self):
        """Updates the DOF and Gains UI with updated values.
        """
        for i in range(self.num_dof):
            self.gains_frames[i].visible = True
            self.gains_frames[i].title = f"DOF {i}: {self.dof_names[i]}"
            for name in self._gains_keys:
                key = f"gains_{i}_" + name
                if name == "kp":
                    self._models[key + "_field"].set_value(float(self.stiffness[i]))
                elif name == "kd":
                    self._models[key + "_field"].set_value(float(self.damping[i]))
