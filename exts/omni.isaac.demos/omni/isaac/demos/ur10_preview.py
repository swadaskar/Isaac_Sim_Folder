# Copyright (c) 2020-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb.input
import omni.kit.commands
import omni.ext
import omni.ui as ui

import weakref

from omni.isaac.motion_planning import _motion_planning
from omni.isaac.dynamic_control import _dynamic_control
import omni.physx as _physx
from omni.physx.bindings._physx import SimulationEvent
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description

from omni.isaac.demos.ur10_scenarios.scenario import Scenario
from omni.isaac.demos.ur10_scenarios import bin_stack
from omni.isaac.demos.ur10_scenarios.fill_bin import FillBin
from omni.isaac.core.utils.viewports import set_camera_view

import asyncio

EXTENSION_NAME = "UR10 Preview"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._timeline = omni.timeline.get_timeline_interface()
        self._usd_context = omni.usd.get_context()
        self._stage = self._usd_context.get_stage()

        self._first_step = True
        self._is_playing = False

        self._mp = _motion_planning.acquire_motion_planning_interface()
        self._dc = _dynamic_control.acquire_dynamic_control_interface()

        self._physxIFace = _physx.acquire_physx_interface()

        self._settings = carb.settings.get_settings()

        self._appwindow = omni.appwindow.get_default_app_window()
        self._sub_stage_event = self._usd_context.get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event
        )
        self._physx_subs = _physx.get_physx_interface().subscribe_physics_step_events(self._on_simulation_step)
        self._scenario = Scenario(self._dc, self._mp)

        self._window = None
        self._selected_scenario = None
        self._create_UR10_btn = None
        self._perform_task_btn = None
        self._stop_task_btn = None
        self._pause_task_btn = None
        self._open_gripper_btn = None
        self._add_new_bins_btn = None

        self._menu_items = [
            MenuItemDescription(
                name="Demos",
                sub_menu=[
                    make_menu_item_description(
                        ext_id, "UR10 Palletizing", lambda a=weakref.proxy(self): a._menu_callback()
                    )
                ],
            )
        ]
        add_menu_items(self._menu_items, "Isaac Examples")

    def _menu_callback(self):
        self._build_ui()

    def _build_ui(self):
        if not self._window:
            self._window = ui.Window(
                title=EXTENSION_NAME, width=300, height=300, dockPreference=ui.DockPreference.LEFT_BOTTOM
            )
            self._app_update_sub = (
                omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update_ui)
            )
            with self._window.frame:
                with ui.VStack():
                    with ui.HStack(height=0):
                        ui.Label("Selected Scenario", width=0)
                        ui.Spacer(width=5)
                        self._selected_scenario = ui.ComboBox(0, "Stack Bins", "Fill Bin")

                    self._create_UR10_btn = ui.Button("Create Scenario", clicked_fn=self._on_environment_setup)

                    self._perform_task_btn = ui.Button("Perform Task", clicked_fn=self._on_perform_task)
                    self._perform_task_btn.enabled = False

                    self._stop_task_btn = ui.Button("Reset Task", clicked_fn=self._on_stop_tasks)
                    self._stop_task_btn.enabled = False

                    self._pause_task_btn = ui.Button("Pause Task", clicked_fn=self._on_pause_tasks)
                    self._pause_task_btn.enabled = False

                    self._open_gripper_btn = ui.Button(
                        "Attach/Detach Suction Gripper", clicked_fn=self._on_open_gripper
                    )
                    self._open_gripper_btn.enabled = False

                    self._add_new_bins_btn = ui.Button("Add Bin", clicked_fn=self._on_add_bin)
                    self._add_new_bins_btn.enabled = False
        self._window.visible = True

    def _on_clear_scenario(self):
        # wait for new stage before creating franka
        asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        self._create_UR10_btn.text = "Create Scenario"
        self._create_UR10_btn.set_clicked_fn(self._on_environment_setup)
        self._selected_scenario.enabled = True

    def _on_environment_setup(self):
        # wait for new stage before creating franka
        task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._on_create_UR10(task))
        self._create_UR10_btn.text = "Clear Scenario"
        self._create_UR10_btn.set_clicked_fn(self._on_clear_scenario)

    async def _on_create_UR10(self, task):

        done, pending = await asyncio.wait({task})
        if task not in done:
            await omni.kit.app.get_app().next_update_async()
            return
        selected_scenario = self._selected_scenario.model.get_item_value_model().as_int

        self._stage = self._usd_context.get_stage()
        if selected_scenario == 0:
            self._scenario = bin_stack.BinStack(self._dc, self._mp)
            set_camera_view(
                eye=[3.70, 1.35, 0.60], target=[-0.8341, -1.2678, -0.8028], camera_prim_path="/OmniverseKit_Persp"
            )
        if selected_scenario == 1:
            self._scenario = FillBin(self._dc, self._mp)
            self._add_new_bins_btn.text = "Drop Parts"
            set_camera_view(
                eye=[-1.4207, 2.8472, 1.1153], target=[-1.406, 2.827, 1.106], camera_prim_path="/OmniverseKit_Persp"
            )

        self._first_step = True
        self._selected_scenario.enabled = False

        self._timeline.stop()
        self._physxIFace.release_physics_objects()

        self._settings.set("/rtx/reflections/halfRes", True)
        self._settings.set("/rtx/shadows/denoiser/quarterRes", True)
        self._settings.set("/rtx/translucency/reflectionCutoff", 0.1)

        self._scenario.create_UR10()

        self._physxIFace.release_physics_objects()
        self._physxIFace.force_load_physics_from_usd()

        self._physxIFace.release_physics_objects()
        self._physxIFace.force_load_physics_from_usd()
        self._stop_task_btn.enabled = True
        self._pause_task_btn.enabled = True
        self._add_new_bins_btn.enabled = True

    def _on_stop_tasks(self, *args):
        if self._scenario:
            self._scenario.stop_tasks()

    def _on_pause_tasks(self, *args):
        self._open_gripper_btn.enabled = self._scenario.pause_tasks()
        self._perform_task_btn.enabled = not self._open_gripper_btn.enabled
        self._stop_task_btn.enabled = True

    def _on_open_gripper(self, *args):
        self._scenario.open_gripper()

    def _on_add_bin(self, *args):
        self._scenario.add_bin()

    def _on_simulation_step(self, step):
        if self._first_step:
            self._scenario.register_assets()
            self._first_step = False
        self._scenario.step(step)

    def _on_stage_event(self, event):
        if self._window:
            self.stage = self._usd_context.get_stage()
            if event.type == int(omni.usd.StageEventType.OPENED):
                self._create_UR10_btn.enabled = True
                self._selected_scenario.enabled = True
                self._perform_task_btn.enabled = False
                self._stop_task_btn.enabled = False
                self._pause_task_btn.enabled = False
                self._open_gripper_btn.enabled = False
                self._add_new_bins_btn.enabled = False
                self._timeline.stop()
                self._on_stop_tasks()
                self._scenario = Scenario(self._dc, self._mp)

    def _on_perform_task(self, *args):
        self._perform_task_btn.enabled = False
        self._pause_task_btn.enabled = True
        self._stop_task_btn.enabled = True
        self._open_gripper_btn.enabled = False
        self._scenario.perform_tasks()

    def _on_update_ui(self, step):
        is_stopped = self._timeline.is_stopped()
        if is_stopped and self._is_playing:
            self._on_stop_tasks()
        self._is_playing = not is_stopped

        if self._timeline.is_playing() or self._scenario.is_created():
            self._perform_task_btn.enabled = self._scenario._paused
            self._add_new_bins_btn.enabled = self._scenario._add_bin_enabled
            self._perform_task_btn.text = "Perform Task"
            if not self._scenario.is_created():
                self._perform_task_btn.enabled = False
                self._perform_task_btn.text = "Press Create To Enable"
        if not self._timeline.is_playing():
            self._perform_task_btn.enabled = False
            self._open_gripper_btn.enabled = False
            self._add_new_bins_btn.enabled = False
            self._perform_task_btn.text = "Press Play To Enable"
            if not self._scenario.is_created():
                self._create_UR10_btn.enabled = True
                self._perform_task_btn.text = "Press Create To Enable"

    def on_shutdown(self):
        self._timeline.stop()
        self._on_stop_tasks()
        self._scenario = None
        self._app_update_sub = None
        self._physx_subs = None
        remove_menu_items(self._menu_items, "Isaac Examples")
        self._window = None
