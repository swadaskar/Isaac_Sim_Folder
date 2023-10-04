# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ext

from .. import _debug_draw


class Extension(omni.ext.IExt):
    def on_startup(self):
        self._draw = _debug_draw.acquire_debug_draw_interface()

    def on_shutdown(self):
        _debug_draw.release_debug_draw_interface(self._draw)
