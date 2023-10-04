# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ext
import omni.kit.commands
import gc
from .. import _dynamic_control

EXTENSION_NAME = "Dynamic Control"


class Extension(omni.ext.IExt):
    def on_startup(self):
        self._dc = _dynamic_control.acquire_dynamic_control_interface()

    def on_shutdown(self):
        _dynamic_control.release_dynamic_control_interface(self._dc)
        gc.collect()
