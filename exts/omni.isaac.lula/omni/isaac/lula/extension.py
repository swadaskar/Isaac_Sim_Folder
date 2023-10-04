# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ext
import lula


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        lula.set_log_level(lula.LogLevel.WARNING)

    def on_shutdown(self):
        pass
