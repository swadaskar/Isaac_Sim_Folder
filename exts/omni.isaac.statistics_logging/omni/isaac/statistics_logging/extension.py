# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import datetime
import os
import time
import yaml

import carb.settings
import carb.tokens
import omni
from .statistics import get_memory_stats, plot_statistics_log


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):

        # Get settings
        settings = carb.settings.get_settings()
        self.reset_log_on_start = settings.get("/exts/omni.isaac.statistics_logging/resetLogOnStart")
        self.log_file_path = settings.get("/exts/omni.isaac.statistics_logging/logFilePath")
        self.log_mode = settings.get("/exts/omni.isaac.statistics_logging/logMode")
        self.log_verbose = settings.get("/exts/omni.isaac.statistics_logging/verbose")
        self.log_every_n_seconds = settings.get("/exts/omni.isaac.statistics_logging/logEveryNSeconds")
        self.log_every_n_frames = settings.get("/exts/omni.isaac.statistics_logging/logEveryNFrames")
        self.output_plot = settings.get("/exts/omni.isaac.statistics_logging/outputPlot")

        if self.log_mode not in ["seconds", "frames"]:
            raise ValueError(
                "Isaac Statistics log mode setting: '{}' must be either 'seconds' or 'frames'.".format(self.log_mode)
            )

        # Default log file path, if needed
        if self.log_file_path == "":
            self.log_file_path = carb.tokens.get_tokens_interface().resolve("${logs}") + "/isaac_statistics/log.yaml"

        omni.kit.app.get_app().print_and_log(f"[omni.isaac.statistics_logging] Logging to file: {self.log_file_path}")

        # Create log path, if needed
        log_path = os.path.dirname(self.log_file_path)
        if not os.path.exists(log_path):
            os.makedirs(log_path, exist_ok=True)

        # Create log file, if needed
        create_new_file = self.reset_log_on_start or not os.path.exists(self.log_file_path)
        if create_new_file:
            open(self.log_file_path, "w").close()

        # Initialize vars
        self._frame_counter = 0
        self._timer = 0
        self._last_update_time = time.time()

        self._update_event_subscription = (
            omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update_event)
        )

    def on_shutdown(self):
        if self.output_plot:
            plot_statistics_log(self.log_file_path)

        self._update_event_subscription = None

    def _on_update_event(self, delta):
        if self.log_mode == "seconds":
            current_time = time.time()
            time_delta = current_time - self._last_update_time
            self._last_update_time = current_time

            self._timer += time_delta
            while self._timer > self.log_every_n_seconds:
                self._log_stamp()
                self._timer -= self.log_every_n_seconds
        else:
            self._frame_counter += 1
            if self._frame_counter >= self.log_every_n_frames:
                self._log_stamp()
                self._frame_counter = 0

    def _log_stamp(self):
        memory_stats = get_memory_stats()

        if not self.log_verbose:
            # Filter-out statistics
            memory_stats = {"System Memory": memory_stats["System Memory"]}

        memory_stats["mode"] = self.log_mode
        if self.log_mode == "seconds":
            memory_stats["step"] = self.log_every_n_seconds
        else:
            memory_stats["step"] = self.log_every_n_frames

        # Key log entry with timestamp
        timestamp = str(datetime.datetime.now())
        memory_stats = {timestamp: memory_stats}

        # Append to the end of log
        with open(self.log_file_path, "a") as f:
            yaml.safe_dump(memory_stats, f)
