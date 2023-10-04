# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.replicator.core.scripts.utils import ReplicatorItem
from typing import Dict, List

import numpy as np
import json


def set_distribution_params(distribution: ReplicatorItem, parameters: Dict) -> None:
    """ 
        Args:
            distribution (ReplicatorItem): The replicator distribution object to be modified.
            parameters (Dict): A dictionary where the keys are the names of the replicator 
                               distribution parameters and the values are the parameter values 
                               to be set.
    """
    node = distribution.node

    for parameter, value in parameters.items():
        attribute_name = "inputs:" + parameter
        if not node.get_attribute_exists(attribute_name):
            raise ValueError(f"This distribution does not have a parameter: `{parameter}`")
        node.get_attribute(attribute_name).set(value)


def get_distribution_params(distribution: ReplicatorItem, parameters: List[str]) -> List:
    """ 
        Args:
            distribution (ReplicatorItem): A replicator distribution object.
            parameters (List[str]): A list of the names of the replicator distribution parameters.
        Returns:
            List[str]: A list of the distribution parameters of the given replicator distribution object.

    """
    node = distribution.node
    params = list()

    for parameter in parameters:
        attribute_name = "inputs:" + parameter
        if not node.get_attribute_exists(attribute_name):
            raise ValueError(f"This distribution does not have a parameter: `{parameter}`")
        value = node.get_attribute(attribute_name).get_array(
            on_gpu=False, get_for_write=False, reserved_element_count=0
        )
        params.append(value)
    return params


def get_semantics(
    num_semantics,
    num_semantic_tokens,
    instance_semantic_map,
    min_semantic_idx,
    max_semantic_hierarchy_depth,
    semantic_token_map,
    required_semantic_types,
):

    instance_to_semantic = instance_semantic_map - min_semantic_idx

    id_to_parents = {}
    # Mapping from a semantic itself to its parents.
    for i in range(0, len(instance_to_semantic), max_semantic_hierarchy_depth):
        curr_semantic_id = instance_to_semantic[i]
        id_to_parents[curr_semantic_id] = []
        for j in range(1, max_semantic_hierarchy_depth):
            parent_semantic_id = instance_to_semantic[i + j]
            if parent_semantic_id != 65535:  # Avoid invalid data
                id_to_parents[curr_semantic_id].append(parent_semantic_id)

    # Mapping from index to semantic labels of each prim
    index_to_labels = {}

    # Iterate through all semantic tokens, and choose those who required by the semantic types.
    valid_semantic_entity_count = 0
    prim_paths = []

    for i in range(num_semantics):
        # TODO Is there a validity check that needs to be performed here?
        is_valid = True
        if is_valid:
            index_to_labels[valid_semantic_entity_count] = {}

            # Find labels of itself and parent labels
            self_labels = semantic_token_map[i * num_semantic_tokens : (i + 1) * num_semantic_tokens]
            parent_labels = []

            if i in id_to_parents.keys():
                for parent_semantic_id in id_to_parents[i]:
                    parent_labels.extend(
                        semantic_token_map[
                            parent_semantic_id * num_semantic_tokens : (parent_semantic_id + 1) * num_semantic_tokens
                        ]
                    )

            all_labels = self_labels + parent_labels

            prim_paths.append(all_labels[0])
            for label_string in all_labels:
                for label in label_string.split(" "):
                    if ":" not in label:
                        continue
                    semantic_type, semantic_data = label.split(":")
                    if semantic_type in required_semantic_types:
                        index_to_labels[valid_semantic_entity_count].setdefault(semantic_type, set()).add(semantic_data)
            # TODO I don't remember why I put this todo here :(   Maybe the bouding box node this code is taken from would provide a clue? Maybe it has to do with semantic filtering?
            valid_semantic_entity_count += 1

    semantic_ids = []
    labels_to_id = {}
    id_to_labels = {}
    id_count = 0

    for index, labels in index_to_labels.items():
        labels_str = str(labels)
        if labels_str not in labels_to_id:
            labels_to_id[labels_str] = id_count
            id_to_labels[id_count] = {}

            for label in labels:
                id_to_labels[id_count] = {k: ",".join(sorted(v)) for k, v in labels.items()}

            semantic_ids.append(id_count)
            id_count += 1
        else:
            semantic_ids.append(labels_to_id[labels_str])

    serialized_index_to_labels = json.dumps(id_to_labels)

    return serialized_index_to_labels, semantic_ids, valid_semantic_entity_count, prim_paths


def get_image_space_points(points, view_proj_matrix):
    """
        Args: 
            points: numpy array of N points (N, 3) in the world space. Points will be projected into the image space. 
            view_proj_matrix: Desired view projection matrix, transforming points from world frame to image space of desired camera
        Returns: 
            numpy array of shape (N, 3) of points projected into the image space. 
    """

    homo = np.pad(points, ((0, 0), (0, 1)), constant_values=1.0)
    tf_points = np.dot(homo, view_proj_matrix)
    tf_points = tf_points / (tf_points[..., -1:])
    tf_points[..., :2] = 0.5 * (tf_points[..., :2] + 1)
    image_space_points = tf_points[..., :3]

    return image_space_points


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)
