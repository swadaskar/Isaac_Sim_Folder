# Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.usd
import carb.events


class BaseResetNode:
    """
        Base class for nodes that automatically reset when stop is pressed.
    """

    def __init__(self, initialize=False):
        self.initialized = initialize

        timeline = omni.timeline.get_timeline_interface()

        self.timeline_event_sub = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_stop_play, name="IsaacSimOGNCoreNodesStageEventHandler"
        )

    def on_stop_play(self, event: carb.events.IEvent):
        self.custom_reset()
        self.initialized = False

    # Defined by subclass
    def custom_reset(self):
        pass

    def reset(self):
        self.timeline_event_sub = None
        self.initialized = None
