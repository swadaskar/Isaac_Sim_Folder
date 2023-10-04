# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import numpy as np


class VibraFSM:
    _amplitudes = {
        "stop": np.array((0.0, 0.0, 0.0), dtype=np.float32),  # [m]
        "run_feed": np.array((0.000, 0.03, 0.02), dtype=np.float32),  # [m]
        "backward": np.array((0.000, -0.03, 0.02), dtype=np.float32),  # [m]
        "realign": np.array((-0.03, 0.0, 0.02), dtype=np.float32),  # [m]
    }
    _motion_frequency = 60.0  # [Hz]

    # configure unblock-cycle:
    _feed_time = 5.0
    _stop_time = 5.0
    _backward_time = 1.5
    _realign_time = 1.5

    def __init__(self, dt=None):
        self.reset()
        self._i = 0
        if dt is not None:
            self._dt = dt

    def reset(self):
        self._dt = 1.0 / 240.0
        self._time = 0.0
        self.state = "stop"
        self._after_delay_state = None

    def start_feed(self):
        self.state = "run_feed"
        # kick off unblock cycle
        self._set_delayed_state_change(delay_sec=self._feed_time, nextState="backward")

    def stop_feed_after_delay(self, delay_sec: float):
        self.state = "run_feed"
        self._set_delayed_state_change(delay_sec=delay_sec, nextState="stop")

    def _set_delayed_state_change(self, delay_sec: float, nextState: str):
        self._after_delay_state = nextState
        self._wait_end_time = self._time + delay_sec

    def update(self):
        self._time += self._dt
        # process wait if necessary
        if self._after_delay_state is not None and self._time > self._wait_end_time:
            self.state = self._after_delay_state
            # auto-unblock cycle
            if self._state == "run_feed":
                self.stop_feed_after_delay(self._stop_time)
            elif self._state == "backward":
                self._set_delayed_state_change(delay_sec=self._backward_time, nextState="realign")
            elif self._state == "realign":
                self._set_delayed_state_change(delay_sec=self._realign_time, nextState="run_feed")
            else:
                self._after_delay_state = None
        return self._motion_amplitude

    def is_stopped(self):
        return self._state == "stop"

    def is_stopping(self):
        return self.is_stopped() or self._after_delay_state == "stop"

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, newState):
        self._state = newState
        if self._state in self._amplitudes:
            self._motion_amplitude = self._amplitudes[self._state]
        else:
            self._motion_amplitude = self._amplitudes["stop"]
