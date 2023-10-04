"""
This is the implementation of the OGN node defined in OgnDope.ogn
"""

import numpy as np
import omni.graph.core as og
from omni.isaac.core.utils.transformations import tf_matrix_from_pose, pose_from_tf_matrix
from omni.isaac.core.utils.rotations import euler_angles_to_quat

from omni.syntheticdata.scripts.helpers import get_bbox_3d_corners

from omni.replicator.isaac.scripts.utils import get_image_space_points, get_semantics


class OgnDope:
    """
        Gets pose information of assets with semantic labels. Information is used to train a DOPE model. 
    """

    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""

        return_data_dtype = np.dtype(
            [
                ("semanticId", "<u4"),
                ("visibility", "<f4"),
                ("location", "<f4", (3,)),
                ("rotation", "<f4", (4,)),  # Quaternion
                ("projected_cuboid", "<f4", (9, 2)),  # Includes Center
            ]
        )

        bbox_3d_dtype = np.dtype(
            [
                ("semanticId", "<u4"),
                ("x_min", "<f4"),
                ("y_min", "<f4"),
                ("z_min", "<f4"),
                ("x_max", "<f4"),
                ("y_max", "<f4"),
                ("z_max", "<f4"),
                ("transform", "<f4", (4, 4)),  # Local to World Transform Matrix
                ("occlusionRatio", "<f4"),
            ]
        )

        bboxes_3d = np.frombuffer(db.inputs.boundingBox3d.tobytes(), dtype=bbox_3d_dtype)

        # Semantics
        num_semantics = db.inputs.sdIMNumSemantics
        num_semantic_tokens = db.inputs.sdIMNumSemanticTokens

        if num_semantics == 0:
            return True

        instance_semantic_map = db.inputs.sdIMInstanceSemanticMap.view(np.uint16)
        min_semantic_idx = db.inputs.sdIMMinSemanticIndex
        max_semantic_hierarchy_depth = db.inputs.sdIMMaxSemanticHierarchyDepth
        semantic_token_map = db.inputs.sdIMSemanticTokenMap

        required_semantic_types = db.inputs.semanticTypes

        serialized_index_to_labels, _, _, _ = get_semantics(
            num_semantics,
            num_semantic_tokens,
            instance_semantic_map,
            min_semantic_idx,
            max_semantic_hierarchy_depth,
            semantic_token_map,
            required_semantic_types,
        )

        # Get Camera Parameters
        cameraViewTransform = db.inputs.cameraViewTransform.reshape((4, 4))
        cameraProjection = db.inputs.cameraProjection

        # Desired view projection matrix, transforming points from world frame to image space of desired camera
        default_camera_to_desired_camera = tf_matrix_from_pose(
            translation=(0.0, 0.0, 0.0),
            orientation=euler_angles_to_quat(np.array(db.inputs.cameraRotation), degrees=True),
        )

        world_to_default_camera_row_major = np.asarray(cameraViewTransform).reshape((4, 4))
        default_camera_to_image_row_major = np.asarray(cameraProjection).reshape((4, 4))

        world_to_default_camera_row_major = np.asarray(cameraViewTransform).reshape((4, 4))
        world_to_default_camera = np.transpose(world_to_default_camera_row_major)

        all_cuboid_points = get_bbox_3d_corners(bboxes_3d)

        semantic_ids = []
        visibilities = []
        locations = []
        rotations = []
        projected_cuboids = []

        for idx, bbox in enumerate(bboxes_3d):
            semantic_ids.append(bbox["semanticId"])
            prim_to_world = np.copy(bbox["transform"])  # Row Major

            center = prim_to_world[-1][:3]

            # Get Center and Rotation of object in camera frame
            prim_to_world[:-1, :-1] = prim_to_world[:-1, :-1] * 100.0
            prim_to_world = np.transpose(prim_to_world)  # Convert to Column Major

            prim_to_default_camera = world_to_default_camera @ prim_to_world

            prim_to_desired_camera = default_camera_to_desired_camera @ prim_to_default_camera

            # location, rotation in 3D camera space
            location, rotation = pose_from_tf_matrix(prim_to_desired_camera)

            locations.append(location * 100)  # Convert to cm
            rotations.append(rotation)

            # Get Cuboid Points of object in image frame
            cuboid_points = np.concatenate((all_cuboid_points[idx], center.reshape(1, 3)))

            # Default view projection matrix, transforming points from world frame to image space of default camera
            world_to_default_image = world_to_default_camera_row_major @ default_camera_to_image_row_major

            default_camera_to_desired_camera_row_major = np.transpose(default_camera_to_desired_camera)

            view_proj_matrix = world_to_default_image @ default_camera_to_desired_camera_row_major

            # Convert Points from World Frame (3D) -> Image Frame (2D)
            image_space_points = get_image_space_points(cuboid_points, view_proj_matrix)

            resolution = np.array([[db.inputs.width, db.inputs.height, 1.0]])
            image_space_points *= resolution

            projected_cuboid_points = [
                [pixel_coordinate[0], pixel_coordinate[1]] for pixel_coordinate in image_space_points
            ]

            # points returned as: [RUB, LUB, RDB, LDB, RUF, LUF, RDF, LDF]
            # but DOPE expects  : [LUF, RUF, RDF, LDF, LUB, RUB, RDB, LDB]
            projected_cuboid = [
                projected_cuboid_points[5],
                projected_cuboid_points[4],
                projected_cuboid_points[6],
                projected_cuboid_points[7],
                projected_cuboid_points[1],
                projected_cuboid_points[0],
                projected_cuboid_points[2],
                projected_cuboid_points[3],
                projected_cuboid_points[8],  # center
            ]

            projected_cuboids.append(projected_cuboid)

            visibilities.append(1.0 - bbox["occlusionRatio"])

        data = np.zeros(len(bboxes_3d), dtype=return_data_dtype)

        data["semanticId"] = np.array(semantic_ids, dtype=np.dtype([("semanticId", "<u4")]))
        data["visibility"] = np.array(visibilities, dtype=np.dtype([("visibility", "<f4")]))

        if len(projected_cuboids) > 0:
            data["projected_cuboid"] = np.array(
                projected_cuboids, dtype=np.dtype([("projected_cuboid", "<f4", (9, 2))])
            )
        if len(locations) > 0:
            data["location"] = np.array(locations, dtype=np.dtype([("location", "<f4", (3,))]))
        if len(rotations) > 0:
            data["rotation"] = np.array(rotations, dtype=np.dtype([("rotation", "<f4", (4,))]))

        # TO-DO:
        # Pass camera location and camera rotation (in world frame) to writer

        db.outputs.data = np.frombuffer(data.tobytes(), dtype=np.uint8)
        db.outputs.idToLabels = serialized_index_to_labels

        db.outputs.exec = og.ExecutionAttributeState.ENABLED
        db.outputs.swhFrameNumber = db.inputs.swhFrameNumber
        db.outputs.bufferSize = 0
        db.outputs.height = db.inputs.height
        db.outputs.width = db.inputs.width

        return True
