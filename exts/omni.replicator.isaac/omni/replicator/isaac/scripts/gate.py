# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.replicator.core.scripts.utils import ReplicatorItem, ReplicatorWrapper, create_node


@ReplicatorWrapper
def on_interval(interval):
    """ 
        Args:
            interval (int): The frequency interval for randomization. The interval is incremented
                            by omni.replicator.isaac.physics_view.step_randomization() call. 
    """
    node = create_node("omni.replicator.isaac.OgnIntervalFiltering")
    trigger_node = ReplicatorItem.get_context()

    node.get_attribute("inputs:interval").set(interval)
    trigger_node.get_attribute("outputs:execOut").connect(node.get_attribute("inputs:execIn"), True)
    trigger_node.get_attribute("outputs:frameNum").connect(node.get_attribute("inputs:frameCounts"), True)

    return node


@ReplicatorWrapper
def on_env_reset():
    node = create_node("omni.replicator.isaac.OgnIntervalFiltering")
    trigger_node = ReplicatorItem.get_context()

    node.get_attribute("inputs:ignoreInterval").set(True)
    trigger_node.get_attribute("outputs:execOut").connect(node.get_attribute("inputs:execIn"), True)
    trigger_node.get_attribute("outputs:resetInds").connect(node.get_attribute("inputs:indices"), True)

    return node
