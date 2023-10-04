__copyright__ = "Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved."
__license__ = """
NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

import io
import os
from pxr import Usd, UsdGeom
from typing import Dict, List
import numpy as np
from scipy.io import savemat
from PIL import Image
from omni.replicator.core import AnnotatorRegistry, BackendDispatch, Writer, WriterRegistry
from omni.syntheticdata import SyntheticData
from omni.isaac.core.utils.mesh import get_mesh_vertices_relative_to

NodeTemplate, NodeConnectionTemplate = SyntheticData.NodeTemplate, SyntheticData.NodeConnectionTemplate

__version__ = "0.0.1"


class YCBVideoWriter(Writer):
    """Writer capable of writing annotator groundtruth in the YCB Video Dataset format.

    Attributes:
        output_dir:
            Output directory string that indicates the directory to save the results.
        num_frames:
            Total number of frames to be generated.
        semantic_types:
            List of semantic types to consider when filtering annotator data. Default: ["class"]
        rgb:
            Boolean value that indicates whether the rgb annotator will be activated
            and the data will be written or not. Default: False.
        bounding_box_2d_tight:
            Boolean value that indicates whether the bounding_box_2d_tight annotator will be activated
            and the data will be written or not. Default: False.
        semantic_segmentation:
            Boolean value that indicates whether the semantic_segmentation annotator will be activated
            and the data will be written or not. Default: False.
        distance_to_image_plane:
            Boolean value that indicates whether the distance_to_image_plane annotator will be activated
            and the data will be written or not. Default: False.
        image_output_format:
            String that indicates the format of saved RGB images. Default: "png"
        pose:
            Boolean value that indicates whether the pose annotator will be activated
            and the data will be written or not. Default: False.
        class_name_to_index_map:
            Mapping between semantic label and index used in the YCB Video Dataset. This indices are used in the 
            'cls_indexes' field of the generated meta.mat file, in addition to being used to color the semantic 
            segmentation (where pixels are colored according to the grayscale class index).
        factor_depth:
            Depth scaling factor used in the YCB Video Dataset. Default: 10000.
        intrinsic_matrix:
            Camera intrinsic matrix. shape is (3, 3).
    """

    def __init__(
        self,
        output_dir: str,
        num_frames: int,
        semantic_types: List[str] = None,
        rgb: bool = False,
        bounding_box_2d_tight: bool = False,
        semantic_segmentation: bool = False,
        distance_to_image_plane: bool = False,
        image_output_format: str = "png",
        pose: bool = False,
        class_name_to_index_map: Dict = None,
        factor_depth: int = 10000,
        intrinsic_matrix: np.ndarray = None,
    ):
        self.backend = BackendDispatch({"paths": {"out_dir": output_dir}})
        self._backend = self.backend  # Kept for backwards compatibility
        self._output_dir = self.backend.output_dir
        self.num_frames = num_frames
        self._frame_id = 0
        self._image_output_format = image_output_format
        self._output_data_format = {}
        self.annotators = []
        self.version = __version__
        self.class_to_index = class_name_to_index_map
        self.factor_depth = factor_depth
        self.intrinsic_matrix = intrinsic_matrix

        # Specify the semantic types that will be included in output
        if semantic_types is None:
            semantic_types = ["class"]

        # RGB
        if rgb:
            self.annotators.append(AnnotatorRegistry.get_annotator("rgb"))

        # Bounding Box 2D
        if bounding_box_2d_tight:
            self.annotators.append(
                AnnotatorRegistry.get_annotator("bounding_box_2d_tight", init_params={"semanticTypes": semantic_types})
            )

        # Semantic Segmentation
        if semantic_segmentation:
            self.annotators.append(
                AnnotatorRegistry.get_annotator(
                    "semantic_segmentation", init_params={"semanticTypes": semantic_types, "colorize": False}
                )
            )

        # Depth
        if distance_to_image_plane:
            self.annotators.append(AnnotatorRegistry.get_annotator("distance_to_image_plane"))

        # Pose Data
        if pose:
            self.annotators.append(
                AnnotatorRegistry.get_annotator("pose", init_params={"semanticTypes": semantic_types})
            )

        self._create_output_folders()
        self._create_train_text_file()

    def write(self, data: dict):
        """Write function called from the OgnWriter node on every frame to process annotator output.

        Args:
            data: A dictionary containing the annotator data for the current frame.
        """

        for annotator in data.keys():
            annotator_split = annotator.split("-")
            render_product_path = ""
            multi_render_prod = 0
            # multiple render_products
            if len(annotator_split) > 1:
                multi_render_prod = 1
                render_product_name = annotator_split[-1]
                render_product_path = f"{render_product_name}/"

            if annotator.startswith("rgb"):
                if multi_render_prod:
                    render_product_path += "rgb/"
                self._write_rgb(data, render_product_path, annotator)

            if annotator.startswith("distance_to_image_plane"):
                if multi_render_prod:
                    render_product_path += "distance_to_image_plane/"
                self._write_distance_to_image_plane(data, render_product_path, annotator)

            if annotator.startswith("semantic_segmentation"):
                if multi_render_prod:
                    render_product_path += "semantic_segmentation/"
                self._write_semantic_segmentation(data, render_product_path, annotator)

            if annotator.startswith("bounding_box_2d_tight"):
                if multi_render_prod:
                    render_product_path += "bounding_box_2d_tight/"
                self._write_bounding_box_data(data, render_product_path, annotator)

            if annotator.startswith("pose"):
                self._write_pose(data, render_product_path, annotator)

        self._frame_id += 1

    def save_mesh_vertices(mesh_prim: UsdGeom.Mesh, coord_prim: Usd.Prim, model_name: str, output_folder: str):
        """Create points.xyz file representing vertices of the mesh_prim, defined in the frame of the coord_prim. The
        points.xyz file will be saved in the output_folder/data/models/model_name/ directory.

        Args:
            mesh_prim (UsdGeom.Mesh): mesh prim to get the vertice points.
            coord_prim (Usd.Prim): prim's coordinate used to define the vertices with respect to.
            model_name (str): name of the part to get the vertices of. Note: This corresponds to the name used for
                              the part in the YCB Video Dataset, and is unrelated to the name of the part in the scene.
            output_folder (str): path of the base output directory.
        """

        file_path = os.path.join(output_folder, "data", "models", model_name, "points.xyz")
        dirname = os.path.dirname(file_path)
        os.makedirs(dirname, exist_ok=True)

        points = get_mesh_vertices_relative_to(mesh_prim, coord_prim)
        np.savetxt(file_path, points, fmt="%.6f", delimiter=" ", newline="\n")

    def _write_rgb(self, data: dict, render_product_path: str, annotator: str):
        """Saves a RGB image for the YCB Video Dataset.

        Args:
            data (dict): A dictionary containing the annotator data for the current frame.
            render_product_path (str): Directory name to save data to, corresponding to a specific render product.
            annotator (str): Annotator name used as a key in the data dictionary, which can also be used to retrieve the
                             annotator from the annotator registry.
        """

        image_id = "{:06d}".format(self._frame_id)

        file_path = f"{self.vid_dir}/{render_product_path}{image_id}-color.{self._image_output_format}"

        self._backend.write_image(file_path, data[annotator])

    def _write_distance_to_image_plane(self, data: dict, render_product_path: str, annotator: str):
        """Saves a depth image for the YCB Video Dataset. Note: Depth images are only for visualization and testing, and
           would need to be adapted to conform to the exact format used in the YCB Video Dataset.

        Args:
            data (dict): A dictionary containing the annotator data for the current frame.
            render_product_path (str): Directory name to save data to, corresponding to a specific render product.
            annotator (str): Annotator name used as a key in the data dictionary, which can also be used to retrieve the
                             annotator from the annotator registry.
        """

        dis_to_img_plane_data = data[annotator]
        dis_to_img_plane_data = dis_to_img_plane_data.squeeze()

        # Convert linear depth to inverse depth for better visualization
        dis_to_img_plane_data = dis_to_img_plane_data * 100
        if np.max(dis_to_img_plane_data) > 0:
            dis_to_img_plane_data = np.reciprocal(dis_to_img_plane_data)

        # Save ground truth data as png
        dis_to_img_plane_data[dis_to_img_plane_data == 0.0] = 1e-5
        dis_to_img_plane_data = np.clip(dis_to_img_plane_data, 0, 255)
        dis_to_img_plane_data -= np.min(dis_to_img_plane_data)

        if np.max(dis_to_img_plane_data) > 0:
            dis_to_img_plane_data /= np.max(dis_to_img_plane_data)

        depth_img = Image.fromarray((dis_to_img_plane_data * 255.0).astype(np.uint8))

        image_id = "{:06d}".format(self._frame_id)
        file_path = f"{self.vid_dir}/{render_product_path}{image_id}-depth.{self._image_output_format}"

        self._backend.write_image(file_path, depth_img)

    def _write_semantic_segmentation(self, data: dict, render_product_path: str, annotator: str):
        """Saves a segmentation label image file for the YCB Video Dataset. Segmentation label is saved as a grayscale
           image.

        Args:
            data (dict): A dictionary containing the annotator data for the current frame.
            render_product_path (str): Directory name to save data to, corresponding to a specific render product.
            annotator (str): Annotator name used as a key in the data dictionary, which can also be used to retrieve the
                             annotator from the annotator registry.
        """

        semantic_seg_data = data[annotator]["data"]

        id_to_labels = data[annotator]["info"]["idToLabels"]

        max_semantic_id = 0
        for semantic_id_str in id_to_labels.keys():
            semantic_id = int(semantic_id_str)
            if semantic_id > max_semantic_id:
                max_semantic_id = semantic_id

        # Array indexed by semantic id, so add 1 to max semantic id.
        semantic_id_to_class_index_map = np.zeros(max_semantic_id + 1, dtype=np.uint8)

        for semantic_id_str, label_dict in id_to_labels.items():

            semantic_id = int(semantic_id_str)

            if "class" in label_dict and label_dict["class"] in self.class_to_index:
                semantic_id_to_class_index_map[semantic_id] = self.class_to_index[label_dict["class"]]
            else:
                semantic_id_to_class_index_map[semantic_id] = 0

        segmentation_data_remapped = np.take(semantic_id_to_class_index_map, semantic_seg_data)

        # Save ground truth data as png
        img = Image.fromarray(np.uint8(segmentation_data_remapped), "L")

        image_id = "{:06d}".format(self._frame_id)
        file_path = f"{self.vid_dir}/{render_product_path}{image_id}-label.{self._image_output_format}"

        self._backend.write_image(file_path, img)

    def _write_bounding_box_data(self, data: dict, render_product_path: str, annotator: str):
        """Saves a text file describing bounding boxes of semantically-labeled objects in view for the YCB Video
           Dataset. Note: Lines of the bounding box text file consist of a class name and the position of the bounding
           box. The positions of the bounding boxes are represented by the upper-left coordinate, followed by the
           bottom-right coordinate. Coordinates are expressed in pixels, where the origin of the image is the top-left
           corner, with +x to the right and +y down.

        Args:
            data (dict): A dictionary containing the annotator data for the current frame.
            render_product_path (str): Directory name to save data to, corresponding to a specific render product.
            annotator (str): Annotator name used as a key in the data dictionary, which can also be used to retrieve the
                             annotator from the annotator registry.
        """

        bbox_data = data[annotator]["data"]
        id_to_labels = data[annotator]["info"]["idToLabels"]

        buf = io.BytesIO()

        for bbox in bbox_data:

            bbox_id = bbox[0]
            semantic_labels = id_to_labels[str(bbox_id)]["class"]
            semantic_label = semantic_labels.split(",")[0]

            bbox_str = f"{semantic_label} {bbox[1]} {bbox[2]} {bbox[3]} {bbox[4]}\n"  # "class_name x1 y1 x2 y2"

            buf.write(bbox_str.encode())

        image_id = "{:06d}".format(self._frame_id)
        file_path = f"{self.vid_dir}/{render_product_path}{image_id}-box.txt"

        self._backend.write_blob(file_path, buf.getvalue())

    def _write_pose(self, data: dict, render_product_path: str, annotator: str):
        """Saves a metadata ".mat" file for the YCB Video Dataset, containing:
           - Class indexes (from a pre-defined mapping) corresponding to each semantically-labeled object in view.
           - A depth image scaling factor.
           - The intrinsic matrix of the camera.
           - Poses from the frame of each semantically-labeled object in view to the world frame, represented as a
             rotation matrix and a translation.
           - The center (in pixel coordinates) of each semantically-labeled object in view. Pixel coordinates are
             expressed relative to the top-left corner of the image, with +x to the right and +y down.

        Args:
            data (dict): A dictionary containing the annotator data for the current frame.
            render_product_path (str): Directory name to save data to, corresponding to a specific render product.
            annotator (str): Annotator name used as a key in the data dictionary, which can also be used to retrieve the
                             annotator from the annotator registry.
        """

        pose_data = data[annotator]["data"]

        n = len(pose_data)

        if n > 0:
            transform_matrices = np.zeros((n, 4, 4))
        else:
            transform_matrices = np.array([[[]]])

        id_to_labels = data[annotator]["info"]["idToLabels"]

        cls_indexes = []
        centers = []

        for i, (semantic_id, pose, center) in enumerate(pose_data):

            # Class indexes
            semantic_labels = id_to_labels[str(semantic_id)]["class"]
            semantic_label = semantic_labels.split(",")[0]
            semantic_index = self.class_to_index[semantic_label]
            cls_indexes.append(semantic_index)

            # Poses
            transform_matrices[i, ...] = pose

            # Centers
            centers.append([center[0], center[1]])

        if n > 0:
            # Make poses have a shape of (3, 4, n)
            poses = np.moveaxis(transform_matrices[:, :-1, :], 0, -1)
        else:
            # Make empty poses have a shape of (1, 1, 0)
            poses = transform_matrices

        meta_dict = {
            "cls_indexes": np.asarray(cls_indexes, dtype=np.uint8),
            "factor_depth": np.array([self.factor_depth], dtype=np.uint16),
            "intrinsic_matrix": self.intrinsic_matrix,
            "poses": poses,
            "center": np.array(centers, dtype=np.float64),
        }

        buf = io.BytesIO()
        savemat(buf, meta_dict)

        image_id = "{:06d}".format(self._frame_id)
        file_path = f"{self.vid_dir}/{render_product_path}{image_id}-meta.mat"

        self._backend.write_blob(file_path, buf.getvalue())

    def _create_output_folders(self):
        """Creates an output directory structure (if necessary), similar to that used in the YCB Video Dataset. Note: A
           single video directory is used to hold all the generated synthetic data, rather than several directories
           (each representing a separate video file, as in the YCB Video Dataset).
        """

        if not os.path.exists(self._output_dir):
            os.mkdir(self._output_dir)

        data_dir = os.path.join(self._output_dir, "data")
        if not os.path.exists(data_dir):
            os.mkdir(data_dir)

        self.ycb_video_dir = os.path.join(data_dir, "YCB_Video")
        if not os.path.exists(self.ycb_video_dir):
            os.mkdir(self.ycb_video_dir)

        ycb_video_data_dir = os.path.join(self.ycb_video_dir, "data")
        if not os.path.exists(ycb_video_data_dir):
            os.mkdir(ycb_video_data_dir)

        self.vid_dir = os.path.join(ycb_video_data_dir, "0000")
        if not os.path.exists(self.vid_dir):
            os.mkdir(self.vid_dir)

    def _create_train_text_file(self):
        """Creates a text file to specify the set of YCB Video Dataset samples to be used during training of a model.
           Lines include the video basename corresponding to the video that the sample is from, and the image ID of the
           sample. Training samples are written as if a single video is being used (see the note in
           create_output_folders()). Additionally, it is assumed data is generated only for model training (rather than
           for testing or validation).
        """

        train_filename = os.path.join(self.ycb_video_dir, "train.txt")
        with open(train_filename, "w") as f:
            vid_dir_basename = os.path.basename(os.path.normpath(self.vid_dir))
            for i in range(self.num_frames):
                train_file_str = f"{vid_dir_basename}/{i:06d}\n"
                f.write(train_file_str)


WriterRegistry.register(YCBVideoWriter)
