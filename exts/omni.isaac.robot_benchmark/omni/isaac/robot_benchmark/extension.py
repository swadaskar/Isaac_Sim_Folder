# Copyright (c) 2021-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ext
import omni.ui as ui
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription

import gc
import asyncio
import weakref
import omni.physx as _physx
from .robot_benchmarking import RobotBenchmark
from .benchmark_robots import BenchmarkRobotRegistry
from omni.isaac.benchmark_environments.environments import EnvironmentCreator
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.core.utils.viewports import set_camera_view
import carb
import os
import json

EXTENSION_NAME = "Robot Benchmark"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._window = ui.Window(EXTENSION_NAME, width=800, height=400, visible=False)
        self._window.set_visibility_changed_fn(self._on_window)
        self._menu_items = [
            make_menu_item_description(ext_id, "Default Benchmarks", lambda a=weakref.proxy(self): a._menu_callback())
        ]

        add_menu_items(self._menu_items, "Robot Benchmark")
        self._timeline = omni.timeline.get_timeline_interface()

        self._benchmarking = RobotBenchmark()
        # Simple button style that grays out the button if disabled
        self._button_style = {":disabled": {"color": 0xFF000000}}

        self._selected_environment = None

        self.ext_id = ext_id
        self.benchmark_robot_registry = BenchmarkRobotRegistry()

        self.env_creator = EnvironmentCreator()
        self.get_robot_options()

        with self._window.frame:
            with omni.ui.VStack(style=self._button_style):
                with ui.HStack(height=30):
                    ui.Label("Selected Environment", width=0)
                    ui.Spacer(width=5)
                    self._selected_environment = ui.ComboBox(0, *self.env_creator.get_environment_names())
                    s = self._selected_environment.model.get_item_value_model().subscribe_value_changed_fn(
                        self.on_env_selection
                    )
                    self.env_selection_subscription = s

                with ui.HStack(height=30):
                    ui.Label("Selected Robot", width=0)
                    ui.Spacer(width=5)
                    self._robot_frame = ui.Frame()
                    with self._robot_frame:
                        self._selected_robot = ui.ComboBox(0, *self.robot_options)
                        s = self._selected_robot.model.get_item_value_model().subscribe_value_changed_fn(
                            self.on_robot_selection
                        )
                        self.robot_selection_subscription = s

                with ui.HStack(height=30):
                    ui.Label("Selected Motion Controller", width=0)
                    ui.Spacer(width=5)
                    self._policy_frame = ui.Frame()
                    self.get_motion_controller_options()
                    with self._policy_frame:
                        self._selected_policy = ui.ComboBox(0, *self.controller_options)

                self._create_robot_btn = ui.Button("Load Robot", enabled=True)
                self._create_robot_btn.set_clicked_fn(self._on_setup_environment)
                self._create_robot_btn.set_tooltip("Load robot and environment")

                self._test_btn = ui.Button("Start Test", enabled=False)
                self._test_btn.set_clicked_fn(self._benchmarking.toggle_testing)
                self._test_btn.set_tooltip("Begin Test")

                self._reset_btn = ui.Button("Reset", enabled=False)
                self._reset_btn.set_clicked_fn(self._benchmarking.reset)
                self._reset_btn.set_tooltip("Reset Robot to default position")

    def on_env_selection(self, option):
        """
        callback any time a new environment is selected from the drop-down menu:
            Reload the list of possible robots and motion policies in the selected environment.
        """
        with self._robot_frame:
            self.get_robot_options()
            self._selected_robot = ui.ComboBox(0, *self.robot_options)
            s = self._selected_robot.model.get_item_value_model().subscribe_value_changed_fn(self.on_robot_selection)
            self.robot_selection_subscription = s

        with self._policy_frame:
            self.get_motion_controller_options()
            self._selected_policy = ui.ComboBox(0, *self.controller_options)

    def on_robot_selection(self, option):
        """
        callback any time a new robot is selected in the drop-down menu:
            Reload the list of possible motion policies for the selected robot.
        """
        with self._policy_frame:
            self.get_motion_controller_options()
            self._selected_policy = ui.ComboBox(0, *self.controller_options)

    def get_robot_options(self):
        """
        Given the environment selected in the drop-down menu, return a list of the robots that have at least one
        motion policy configured in the motion_generation extension, and are not explicitly excluded for this
        environment
        """

        if self._selected_environment is None:
            env_name = self.env_creator.get_environment_names()[0]
        else:
            selected_environment = self._selected_environment.model.get_item_value_model().as_int
            env_name = self.env_creator.get_environment_names()[selected_environment]

        robot_exclusion_list = self.env_creator.get_robot_exclusion_list(env_name)

        all_robot_options = self.benchmark_robot_registry.get_robot_options()

        robot_options = []
        for op in all_robot_options:
            if op not in robot_exclusion_list:
                robot_options.append(op)
        self.robot_options = robot_options

    def get_motion_controller_options(self):
        """
        Given the robot selected in the drop down menu, return the motion policies that have default configs
        for the robot in the motion_generation extension
        """
        selected_robot = self.robot_options[self._selected_robot.model.get_item_value_model().as_int]

        if self._selected_environment is None:
            env_name = self.env_creator.get_environment_names()[0]
        else:
            selected_environment_idx = self._selected_environment.model.get_item_value_model().as_int
            env_name = self.env_creator.get_environment_names()[selected_environment_idx]

        controller_exclusion_list = self.env_creator.get_motion_policy_exclusion_list(env_name)

        all_controller_options = self.benchmark_robot_registry.get_robot_loader(
            selected_robot
        ).get_benchmark_controller_names()

        controller_options = []
        for op in all_controller_options:
            if op not in controller_exclusion_list:
                controller_options.append(op)
        self.controller_options = controller_options

    def get_environment_params(self, env_name, robot_name):
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        benchmark_extension_path = ext_manager.get_extension_path(self.ext_id)

        self._benchmark_config_dir = os.path.join(benchmark_extension_path, "benchmark_config")
        with open(os.path.join(self._benchmark_config_dir, "benchmark_config_map.json")) as benchmark_config_map:
            self._benchmark_config_map = json.load(benchmark_config_map)

        local_env_config_path = (
            self._benchmark_config_map.get(robot_name, {}).get("environment_config_paths", {}).get(env_name, None)
        )
        if local_env_config_path is None:
            return {}

        env_config_path = os.path.join(self._benchmark_config_dir, local_env_config_path)

        if os.path.exists(env_config_path):
            with open(env_config_path) as env_file:
                config = json.load(env_file)
        else:
            carb.log_error(
                "Invalid path to config file specified in benchmark_config_map.json for the "
                + robot_name
                + " in the "
                + env_name
                + "environment"
            )
            config = {}

        return config

    def _on_window(self, status):
        if status:
            self._sub_stage_event = (
                omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(self._on_stage_event)
            )
            self._physx_subs = _physx.get_physx_interface().subscribe_physics_step_events(self._on_simulation_step)
            self._timeline_sub = self._timeline.get_timeline_event_stream().create_subscription_to_pop(
                self._on_timeline_event
            )
        else:
            self._sub_stage_event = None
            self._physx_subs = None
            self._timeline_sub = None

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def _on_stage_event(self, event):
        """This function is called when stage events occur.
        Enables UI elements when stage is opened.
        Prevents tasks from being started until all assets are loaded

        Arguments:
            event (int): event type
        """
        if event.type == int(omni.usd.StageEventType.OPENED):
            self._create_robot_btn.enabled = True
            self._test_btn.enabled = False

            self._reset_btn.enabled = False

            self._timeline.stop()
            self._benchmarking.stop_tasks()

    def _on_simulation_step(self, step):
        if self._benchmarking.created:
            self._create_robot_btn.text = "Reload Robot"
            if self._timeline.is_playing():
                self._benchmarking.step(step)

            else:
                self._test_btn.text = "Press Play To Enable"

        else:
            self._create_robot_btn.text = "Load Robot"
            self._test_btn.text = "Press Load Robot To Enable"

    def _on_timeline_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.PLAY):
            self._test_btn.enabled = True
            self._reset_btn.enabled = True

        if e.type == int(omni.timeline.TimelineEventType.STOP) or e.type == int(omni.timeline.TimelineEventType.PAUSE):
            self._test_btn.enabled = False

    def _on_setup_environment(self):
        self._timeline.stop()
        task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._on_create_robot(task))

    async def _on_create_robot(self, task):
        done, pending = await asyncio.wait({task})
        if task not in done:
            return

        selected_environment = self._selected_environment.model.get_item_value_model().as_int
        env_name = self.env_creator.get_environment_names()[selected_environment]

        robot_name = self.robot_options[self._selected_robot.model.get_item_value_model().as_int]
        controller_name = self.controller_options[self._selected_policy.model.get_item_value_model().as_int]

        env_kwargs = self.get_environment_params(env_name, robot_name)
        env = self.env_creator.create_environment(env_name, **env_kwargs)

        robot_loader = self.benchmark_robot_registry.get_robot_loader(robot_name)

        self._benchmarking.initialize_test(env, robot_loader, controller_name, benchmark_logger=None)

        set_camera_view(eye=env.camera_position, target=env.camera_target, camera_prim_path="/OmniverseKit_Persp")

        self._reset_btn.enabled = True

    def on_shutdown(self):
        self._physx_subs = None
        self._sub_stage_event = None
        self._timeline_sub = None

        self._timeline.stop()
        self._benchmarking.stop_tasks()
        self._benchmarking = None
        remove_menu_items(self._menu_items, "Robot Benchmark")
        gc.collect()
        pass
