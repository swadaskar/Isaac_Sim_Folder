# Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni
from pxr import Usd
from omni.isaac.core_nodes.impl.extension import cache_node_template_activation, cache_writer_attach
import copy
import omni.replicator.core as rep
from omni.isaac.core.utils.prims import set_targets


def set_target_prims(primPath: str, targetPrimPaths: list, inputName: str = "inputs:targetPrim"):
    stage = omni.usd.get_context().get_stage()
    try:
        set_targets(stage.GetPrimAtPath(primPath), inputName, targetPrimPaths)
    except Exception as e:
        print(e, primPath)


def submit_node_template_activation(
    template_name: str,
    render_product_path_index: int = -1,
    render_product_paths: list = None,
    attributes: dict = None,
    stage: Usd.Stage = None,
) -> None:
    """Submit a request to activate a node template for the next update.

            Args:
                template_name : name of the activated node
                attribute_names : list of node attribute names to retrieve the value
                render_product_path_index : if the node template is associated to a render product, index of the associated render product in the render product path list
                render_product_paths : render product path list to be used for specifying the render product of the node template and its dependencies to activate

            """

    cache_node_template_activation(
        template_name, render_product_path_index, copy.deepcopy(render_product_paths), copy.deepcopy(attributes), stage
    )


def submit_writer_attach(writer: rep.Writer, render_product_path: str) -> None:
    """Submit a request to attach a writer for the next update.

            Args:
                writer : writer object we want to attach
                render_product_path to attach to writer

            """

    cache_writer_attach(writer, render_product_path)
