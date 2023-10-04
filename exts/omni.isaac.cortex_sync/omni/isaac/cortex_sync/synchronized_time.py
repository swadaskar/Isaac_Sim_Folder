# Copyright (c) 2022-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

""" Helpers for representing the synchronized time information collected through the command-ack
messaging protocol when communicating to off-board real-time controllers.
"""

import math
from typing import Optional

import rospy

from cortex_control.msg import JointPosVelAccCommand, CortexCommandAck


class CycleTime:
    """ Collects information about both the corrected current time and the measured period between
    the last cycle and this one.

    Args:
        time: Corrected wall-clock time based on difference between cortex and control clocks.
        period: Time between last cycle and this cycle. Note the period may be None if this is the
            first cycle.
    """

    def __init__(self, time: rospy.Time, period: Optional[rospy.Duration] = None):
        self.time = time
        self.period = period
        if period is None:
            self.is_period_available = False
        else:
            self.is_period_available = True


class SynchronizedTime:
    """ Represents the online updated synchronized time information resulting from the command-ack
    protocol when communicating with off-board controllers (see cortex_control for details).

    The main issue is two fold:
    1. The cortex system may be running at slower than real-time (< 60hz) if processing gets bogged
       down. In that case, we still want to maintain consistent effective motion policy integration
       that matches the real-world wall-clock time. Reactivity in those cases might slow, but the
       behavior overall will remain the same speed. (To the cortex system it'll seem like the world
       is moving fast, and it'll adjust the way it integrates accordingly.)
    2. Even when maintaining real-time integration on the cortex machine, the cortex machine's clock
       might progress at a slightly different rate overall than a robot's internal microcontroller's
       clock. This was observed when controlling the Franka early on -- the real-time clock would
       run slightly fast so that over a number of hours the evaluation point of the incrementally
       generated splines would eventually overrun the spline interpolation buffer. The command-ack
       synchronization protocol automatically estimates that clock offset over time to eliminate
       this drift.

    The synchronized time is returned as a CycleTime object. Issue 1 is addressed by the CycleTime's
    period field (representing an exponentially weighted average of the measured cycle time), and
    Issue 2 is addressed by the CycleTime's time field (giving the corrected wall-clock time using
    the estimated corrections from the command-ack protocol).

    Args:
        skip_cycles: The number of cycles to skip before processing the messages. Early messages
            timings can be off if the loop runner isn't running at rate up front.
    """

    def __init__(self, skip_cycles: Optional[int] = 0):
        """ Initialize this synchronized time.

        skip_cycles defines the number of cycles to skip before starting to measure cycle time and
        offsets to compensate for differences in clock rate between the cortex machine and real-time
        control machine.
        """
        self.skip_cycles = skip_cycles
        self.sub = rospy.Subscriber("/cortex/arm/command/ack", CortexCommandAck, self.callback)
        self.reset()

    def __del__(self):
        """ Simply unregisters the subscriber.
        """
        self.sub.unregister()

    def reset(self) -> None:
        """ Reset the statistics. It will start estimating the cycle time and period fresh from here.
        """
        self.cycle_count = 0
        self.latest_message = CortexCommandAck()
        self.cycle_start_time = rospy.Time.now()
        self.current_offset = rospy.Duration(0)

    def callback(self, data: CortexCommandAck) -> None:
        """ Basic ack callback, stores the information as the latest message.

        Args:
            data: The incomine message.
        """
        self.latest_message = data

    def next_adaptive_cycle_time(self) -> CycleTime:
        """ Ticks the time synchronization algorithm.

        This method is called once per cortex cycle to both estimate the cortex cycle and use the
        ack information to estimate the offset between between clocks.

        The offset between clocks is used in the call to self.now_nonblocking() and returned as the
        corrected time in the returned CycleTime object. The period is estimated as an exponentially
        weighted average based on measured information from the acks and cycle time. The exponential
        weights scaled to work well with a 60hz cycle.

        Returns: A CycleTime object representing the latest adaptive cycle time.
        """
        self.cycle_count += 1
        now = self.now_nonblocking()

        if self.cycle_count <= max(1, self.skip_cycles):
            ret = CycleTime(now)
        else:
            command_period = now - self.cycle_start_time
            ret = CycleTime(now, command_period)

            new_offset_measurement = self.latest_message.time_offset.to_sec()
            nominal_eps = math.pow(0.9999, 250.0)
            reg_decay = math.pow(nominal_eps, command_period.to_sec())
            ss = command_period.to_sec()
            self.current_offset = rospy.Duration(reg_decay * self.current_offset.to_sec() + ss * new_offset_measurement)

        self.cycle_start_time = now
        return ret

    def now_nonblocking(self) -> rospy.Time:
        """ Accessor for the corrected time based on the current measured time offset.

        Returns: The corrected time.
        """
        return rospy.Time.now() + self.current_offset
