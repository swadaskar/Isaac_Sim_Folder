# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.kit import SimulationApp

# The most basic usage for creating a simulation app
kit = SimulationApp()

import carb
server_check = carb.settings.get_settings().get_as_string("/persistent/isaac/asset_root/default")
print(server_check)

for i in range(100):
    kit.update()

kit.close()  # Cleanup application
