# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ext
from .. import _motion_planning


class Extension(omni.ext.IExt):
    def on_startup(self):
        self._mp = _motion_planning.acquire_motion_planning_interface()

    def on_shutdown(self):
        _motion_planning.release_motion_planning_interface(self._mp)
