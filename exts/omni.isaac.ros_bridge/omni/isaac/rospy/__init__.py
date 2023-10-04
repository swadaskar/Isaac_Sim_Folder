# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os, sys
import omni.ext


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._ros_extension_path = ext_manager.get_extension_path(ext_id)
        sys.path.append(
            os.path.join(os.path.dirname(self._ros_extension_path + "/noetic/lib/python3.7/site-packages/"))
        )

    def on_shutdown(self):
        sys.path.remove(
            os.path.join(os.path.dirname(self._ros_extension_path + "/noetic/lib/python3.7/site-packages/"))
        )
