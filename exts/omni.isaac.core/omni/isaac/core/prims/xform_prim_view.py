# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional, Tuple, Union, List
from pxr import Gf, Usd, UsdGeom, UsdShade
import omni.kit.app
from omni.isaac.core.utils.types import XFormPrimViewState
from omni.isaac.core.materials import PreviewSurface, OmniGlass, OmniPBR, VisualMaterial
from omni.isaac.core.simulation_context.simulation_context import SimulationContext
from omni.isaac.core.utils.prims import (
    get_prim_at_path,
    is_prim_non_root_articulation_link,
    is_prim_path_valid,
    find_matching_prim_paths,
    get_prim_parent,
)
import numpy as np
import carb
from omni.isaac.core.utils.stage import get_current_stage
import torch


class XFormPrimView(object):
    """Provides high level functions to deal with an Xform prim view (1 or more XForm prims and its descendants) 
        as well as its attributes/ properties.
        This object wraps all matching XForms found at the regex provided at the prim_paths_expr.

        Note: each prim will have "xformOp:orient", "xformOp:translate" and "xformOp:scale" only post init,
                unless it is a non-root articulation link.

        Args:
            prim_paths_expr (str): prim paths regex to encapsulate all prims that match it.
                                    example: "/World/Env[1-5]/Franka" will match /World/Env1/Franka, 
                                    /World/Env2/Franka..etc.
                                    (a non regex prim path can also be used to encapsulate one XForm).
            name (str, optional): shortname to be used as a key by Scene class. 
                                    Note: needs to be unique if the object is added to the Scene. 
                                    Defaults to "xform_prim_view".
            positions (Optional[Union[np.ndarray, torch.Tensor]], optional): 
                                                            default positions in the world frame of the prim. 
                                                            shape is (N, 3).
                                                            Defaults to None, which means left unchanged.
            translations (Optional[Union[np.ndarray, torch.Tensor]], optional): 
                                                            default translations in the local frame of the prims
                                                            (with respect to its parent prims). shape is (N, 3).
                                                            Defaults to None, which means left unchanged.
            orientations (Optional[Union[np.ndarray, torch.Tensor]], optional): 
                                                            default quaternion orientations in the world/ local frame of the prim
                                                            (depends if translation or position is specified).
                                                            quaternion is scalar-first (w, x, y, z). shape is (N, 4).
                                                            Defaults to None, which means left unchanged.
            scales (Optional[Union[np.ndarray, torch.Tensor]], optional): local scales to be applied to 
                                                            the prim's dimensions. shape is (N, 3).
                                                            Defaults to None, which means left unchanged.
            visibilities (Optional[Union[np.ndarray, torch.Tensor]], optional): set to false for an invisible prim in 
                                                                                the stage while rendering. shape is (N,). 
                                                                                Defaults to None.
            reset_xform_properties (bool, optional): True if the prims don't have the right set of xform properties 
                                                    (i.e: translate, orient and scale) ONLY and in that order.
                                                    Set this parameter to False if the object were cloned using using 
                                                    the cloner api in omni.isaac.cloner. Defaults to True.

        Raises:
            Exception: if translations and positions defined at the same time.
            Exception: No prim was matched using the prim_paths_expr provided.
    """

    def __init__(
        self,
        prim_paths_expr: str,
        name: str = "xform_prim_view",
        positions: Optional[Union[np.ndarray, torch.Tensor]] = None,
        translations: Optional[Union[np.ndarray, torch.Tensor]] = None,
        orientations: Optional[Union[np.ndarray, torch.Tensor]] = None,
        scales: Optional[Union[np.ndarray, torch.Tensor]] = None,
        visibilities: Optional[Union[np.ndarray, torch.Tensor]] = None,
        reset_xform_properties: bool = True,
    ) -> None:
        self._non_root_link = False
        self._prim_paths = find_matching_prim_paths(prim_paths_expr)
        if len(self._prim_paths) == 0:
            raise Exception(
                "Prim path expression {} is invalid, a prim matching the expression needs to created before wrapping it as view".format(
                    prim_paths_expr
                )
            )
        self._name = name
        self._count = len(self._prim_paths)
        self._prims = []
        self._regex_prim_paths = prim_paths_expr
        for prim_path in self._prim_paths:
            self._prims.append(get_prim_at_path(prim_path))

        if SimulationContext.instance() is not None:
            self._backend = SimulationContext.instance().backend
            self._device = SimulationContext.instance().device
            self._backend_utils = SimulationContext.instance().backend_utils
        else:
            import omni.isaac.core.utils.numpy as np_utils

            self._backend = "numpy"
            self._device = None
            self._backend_utils = np_utils

        self._default_state = None
        self._applied_visual_materials = [None] * self._count
        self._binding_apis = [None] * self._count
        self._non_root_link = is_prim_non_root_articulation_link(prim_path=self._prim_paths[0])
        if not self._non_root_link and reset_xform_properties:
            self._set_xform_properties()
        if not self._non_root_link:
            if translations is not None and positions is not None:
                raise Exception("You can not define translation and position at the same time")
            if positions is not None or orientations is not None or translations is not None:
                if translations is not None:
                    self.set_local_poses(translations, orientations)
                else:
                    self.set_world_poses(positions, orientations)
            if scales is not None:
                XFormPrimView.set_local_scales(self, scales)
        if visibilities is not None:
            XFormPrimView.set_visibilities(self, visibilities=visibilities)
        if not self._non_root_link:
            default_positions, default_orientations = self.get_world_poses()
            self._default_state = XFormPrimViewState(positions=default_positions, orientations=default_orientations)
        return

    @property
    def prim_paths(self) -> List[str]:
        """
        Returns:
            List[str]: list of prim paths in the stage encapsulated in this view.
        """
        return self._prim_paths

    @property
    def name(self) -> str:
        """
        Returns:
            str: name given to the prims view when instantiating it.
        """
        return self._name

    @property
    def count(self) -> int:
        """
        Returns:
            int: Number of prims encapsulated in this view.
        """
        return self._count

    @property
    def prims(self) -> List[Usd.Prim]:
        """
        
        Returns:
            List[Usd.Prim]: List of USD Prim objects encapsulated in this view.
        """
        return self._prims

    @property
    def is_non_root_articulation_link(self) -> bool:
        """ 
        Returns:
            bool: True if the prim corresponds to a non root link in an articulation. Otherwise False.
        """
        return self._non_root_link

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        return

    def _set_xform_properties(self) -> None:
        current_positions, current_orientations = self.get_world_poses()
        properties_to_remove = [
            "xformOp:rotateX",
            "xformOp:rotateXZY",
            "xformOp:rotateY",
            "xformOp:rotateYXZ",
            "xformOp:rotateYZX",
            "xformOp:rotateZ",
            "xformOp:rotateZYX",
            "xformOp:rotateZXY",
            "xformOp:rotateXYZ",
            "xformOp:transform",
        ]
        for i in range(self._count):
            prop_names = self._prims[i].GetPropertyNames()
            xformable = UsdGeom.Xformable(self._prims[i])
            xformable.ClearXformOpOrder()
            for prop_name in prop_names:
                if prop_name in properties_to_remove:
                    self._prims[i].RemoveProperty(prop_name)
            if "xformOp:scale" not in prop_names:
                xform_op_scale = xformable.AddXformOp(UsdGeom.XformOp.TypeScale, UsdGeom.XformOp.PrecisionDouble, "")
                xform_op_scale.Set(Gf.Vec3d([1.0, 1.0, 1.0]))
            else:
                xform_op_scale = UsdGeom.XformOp(self._prims[i].GetAttribute("xformOp:scale"))

            if "xformOp:translate" not in prop_names:
                xform_op_tranlsate = xformable.AddXformOp(
                    UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, ""
                )
            else:
                xform_op_tranlsate = UsdGeom.XformOp(self._prims[i].GetAttribute("xformOp:translate"))

            if "xformOp:orient" not in prop_names:
                xform_op_rot = xformable.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
            else:
                xform_op_rot = UsdGeom.XformOp(self._prims[i].GetAttribute("xformOp:orient"))
            xformable.SetXformOpOrder([xform_op_tranlsate, xform_op_rot, xform_op_scale])
        self.set_world_poses(positions=current_positions, orientations=current_orientations)
        return

    def set_visibilities(
        self,
        visibilities: Union[np.ndarray, torch.Tensor],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the visibilities of the prims in stage.

        Args:
            visibilities (Union[np.ndarray, torch.Tensor]): flag to set the visibilities of the usd prims in stage. 
                                                            Shape (M,). Where M <= size of the encapsulated prims in the view.
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                 to manipulate. Shape (M,).
                                                                                 Defaults to None (i.e: all prims in the view).
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        read_idx = 0
        for i in indices:
            imageable = UsdGeom.Imageable(self._prims[i.tolist()])
            if visibilities[read_idx]:
                imageable.MakeVisible()
            else:
                imageable.MakeInvisible()
            read_idx += 1
        return

    def get_visibilities(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> Union[np.ndarray, torch.Tensor]:
        """Returns the current visibilities of the prims in stage.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            Union[np.ndarray, torch.Tensor]: Shape (M,) with type bool, where each item holds True 
                                             if the prim is visible in stage. False otherwise.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        visibilities = self._backend_utils.create_zeros_tensor(
            shape=[indices.shape[0]], dtype="bool", device=self._device
        )
        write_idx = 0
        for i in indices:
            visibilities[write_idx] = (
                UsdGeom.Imageable(self._prims[i.tolist()]).ComputeVisibility(Usd.TimeCode.Default())
                != UsdGeom.Tokens.invisible
            )
            write_idx += 1
        return visibilities

    def post_reset(self) -> None:
        """Resets the prims to its default state (positions and orientations).
        """
        if not self._non_root_link:
            self.set_world_poses(self._default_state.positions, self._default_state.orientations)
        return

    def get_default_state(self) -> XFormPrimViewState:
        """
        Returns:
            XFormPrimViewState: returns the default state of the prims (positions and orientations) that is used after each reset.
        """
        if self._non_root_link:
            carb.log_warn("This view corresponds to non root links that are included in an articulation")
        return self._default_state

    def set_default_state(
        self,
        positions: Optional[np.ndarray] = None,
        orientations: Optional[np.ndarray] = None,
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the default state of the prims (positions and orientations), that will be used after each reset.

        Args:
            positions (Optional[np.ndarray], optional):  positions in the world frame of the prim. shape is (M, 3).
                                                       Defaults to None, which means left unchanged.
            orientations (Optional[np.ndarray], optional): quaternion orientations in the world frame of the prim. 
                                                          quaternion is scalar-first (w, x, y, z). shape is (M, 4).
                                                          Defaults to None, which means left unchanged.
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        if self._non_root_link:
            carb.log_warn("This view corresponds to non root links that are included in an articulation")
            return
        if positions is not None:
            if indices is None:
                self._default_state.positions = positions
            else:
                self._default_state.positions[indices] = positions
        if orientations is not None:
            if indices is None:
                self._default_state.orientations = orientations
            else:
                self._default_state.orientations[indices] = orientations
        return

    def apply_visual_materials(
        self,
        visual_materials: Union[VisualMaterial, List[VisualMaterial]],
        weaker_than_descendants: Optional[Union[bool, List[bool]]] = None,
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Used to apply visual material to the prims and optionally its prim descendants.

        Args:
            visual_materials (Union[VisualMaterial, List[VisualMaterial]]): visual materials to be applied to the prims. Currently supports
                                                                            PreviewSurface, OmniPBR and OmniGlass. If a list is provided then
                                                                            its size has to be equal the view's size or indices size. 
                                                                            If one material is provided it will be applied to all prims in the view.
            weaker_than_descendants (Optional[Union[bool, List[bool]]], optional):  True if the material shouldn't override the descendants  
                                                                                    materials, otherwise False. Defaults to False. 
                                                                                    If a list of visual materials is provided then a list
                                                                                    has to be provided with the same size for this arg as well.
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Raises:
            Exception: length of visual materials != length of prims indexed
            Exception: length of visual materials != length of weaker descendants bools arg
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)

        if isinstance(visual_materials, list):
            if indices.shape[0] != len(visual_materials):
                raise Exception("length of visual materials != length of prims indexed")
            if weaker_than_descendants is None:
                weaker_than_descendants = [False] * len(visual_materials)
            if len(visual_materials) != len(weaker_than_descendants):
                raise Exception("length of visual materials != length of weaker descendants bools arg")
        if isinstance(visual_materials, list):
            read_idx = 0
            for i in indices:
                if self._binding_apis[i.tolist()] is None:
                    if self._prims[i.tolist()].HasAPI(UsdShade.MaterialBindingAPI):
                        self._binding_apis[i.tolist()] = UsdShade.MaterialBindingAPI(self._prims[i.tolist()])
                    else:
                        self._binding_apis[i.tolist()] = UsdShade.MaterialBindingAPI.Apply(self._prims[i.tolist()])
                if weaker_than_descendants[read_idx]:
                    self._binding_apis[i.tolist()].Bind(
                        visual_materials[read_idx].material, bindingStrength=UsdShade.Tokens.weakerThanDescendants
                    )
                else:
                    self._binding_apis[i.tolist()].Bind(
                        visual_materials[read_idx].material, bindingStrength=UsdShade.Tokens.strongerThanDescendants
                    )
                self._applied_visual_materials[i.tolist()] = visual_materials[read_idx]
                read_idx += 1
            return
        else:
            if weaker_than_descendants is None:
                weaker_than_descendants = False
            for i in indices:
                if self._binding_apis[i.tolist()] is None:
                    if self._prims[i.tolist()].HasAPI(UsdShade.MaterialBindingAPI):
                        self._binding_apis[i.tolist()] = UsdShade.MaterialBindingAPI(self._prims[i.tolist()])
                    else:
                        self._binding_apis[i.tolist()] = UsdShade.MaterialBindingAPI.Apply(self._prims[i.tolist()])
                if weaker_than_descendants:
                    self._binding_apis[i.tolist()].Bind(
                        visual_materials.material, bindingStrength=UsdShade.Tokens.weakerThanDescendants
                    )
                else:
                    self._binding_apis[i.tolist()].Bind(
                        visual_materials.material, bindingStrength=UsdShade.Tokens.strongerThanDescendants
                    )
                self._applied_visual_materials[i.tolist()] = visual_materials
        return

    def get_applied_visual_materials(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> List[VisualMaterial]:
        """

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            List[VisualMaterial]: a list of the current applied visual materials to the prims if its type is currently supported.
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = [None] * indices.shape[0]
        write_idx = 0
        for i in indices:
            if self._binding_apis[i.tolist()] is None:
                if self._prims[i.tolist()].HasAPI(UsdShade.MaterialBindingAPI):
                    self._binding_apis[i.tolist()] = UsdShade.MaterialBindingAPI(self._prims[i.tolist()])
                else:
                    self._binding_apis[i.tolist()] = UsdShade.MaterialBindingAPI.Apply(self._prims[i.tolist()])
            if self._applied_visual_materials[i.tolist()] is not None:
                result[write_idx] = self._applied_visual_materials[i.tolist()]
                write_idx += 1
            else:
                visual_binding = self._binding_apis[i.tolist()].GetDirectBinding()
                material_path = str(visual_binding.GetMaterialPath())
                if material_path == "":
                    result[write_idx] = None
                    write_idx += 1
                else:
                    stage = get_current_stage()
                    material = UsdShade.Material(stage.GetPrimAtPath(material_path))
                    # getting the shader
                    shader_info = material.ComputeSurfaceSource()
                    if shader_info[0].GetPath() != "":
                        shader = shader_info[0]
                    elif is_prim_path_valid(material_path + "/shader"):
                        shader_path = material_path + "/shader"
                        shader = UsdShade.Shader(get_prim_at_path(shader_path))
                    elif is_prim_path_valid(material_path + "/Shader"):
                        shader_path = material_path + "/Shader"
                        shader = UsdShade.Shader(get_prim_at_path(shader_path))
                    else:
                        carb.log_warn(
                            "the shader on xform prim {} is not supported".format(self._prim_paths[i.tolist()])
                        )
                        result[write_idx] = None
                        write_idx += 1
                        continue
                    implementation_source = shader.GetImplementationSource()
                    asset_sub_identifier = shader.GetPrim().GetAttribute("info:mdl:sourceAsset:subIdentifier").Get()
                    shader_id = shader.GetShaderId()
                    if implementation_source == "id" and shader_id == "UsdPreviewSurface":
                        self._applied_visual_materials[i] = PreviewSurface(prim_path=material_path, shader=shader)
                        result[write_idx] = self._applied_visual_materials[i.tolist()]
                        write_idx += 1
                    elif asset_sub_identifier == "OmniGlass":
                        self._applied_visual_materials[i] = OmniGlass(prim_path=material_path, shader=shader)
                        result[write_idx] = self._applied_visual_materials[i.tolist()]
                        write_idx += 1
                    elif asset_sub_identifier == "OmniPBR":
                        self._applied_visual_materials[i.tolist()] = OmniPBR(prim_path=material_path, shader=shader)
                        result[write_idx] = self._applied_visual_materials[i.tolist()]
                        write_idx += 1
                    else:
                        carb.log_warn(
                            "the shader on xform prim {} is not supported".format(self._prim_paths[i.tolist()])
                        )
                        result[write_idx] = None
                        write_idx += 1
        return result

    def is_visual_material_applied(self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None) -> List[bool]:
        """
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            List[bool]: True if there is a visual material applied is applied to the corresponding prim in the view. False otherwise.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = [None] * indices.shape[0]
        write_idx = 0
        for i in indices:
            if self._binding_apis[i.tolist()] is None:
                if self._prims[i.tolist()].HasAPI(UsdShade.MaterialBindingAPI):
                    self._binding_apis[i.tolist()] = UsdShade.MaterialBindingAPI(self._prims[i.tolist()])
                else:
                    self._binding_apis[i.tolist()] = UsdShade.MaterialBindingAPI.Apply(self._prims[i.tolist()])
            visual_binding = self._binding_apis[i.tolist()].GetDirectBinding()
            material_path = str(visual_binding.GetMaterialPath())
            if material_path == "":
                result[write_idx] = False
                write_idx += 1
            else:
                result[write_idx] = True
                write_idx += 1
        return result

    def get_world_poses(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]:
        """ Returns the poses (positions and orientations) of the prims in the view with respect to the world frame.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: first index is positions in the world frame of the prims. shape is (M, 3). 
                                                                                     second index is quaternion orientations in the world frame of the prims.
                                                                                     quaternion is scalar-first (w, x, y, z). shape is (M, 4).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        positions = self._backend_utils.create_zeros_tensor([indices.shape[0], 3], dtype="float32", device=self._device)
        orientations = self._backend_utils.create_zeros_tensor(
            [indices.shape[0], 4], dtype="float32", device=self._device
        )
        write_idx = 0
        for i in indices:
            prim_tf = UsdGeom.Xformable(self._prims[i.tolist()]).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            transform = Gf.Transform()
            transform.SetMatrix(prim_tf)
            positions[write_idx] = self._backend_utils.create_tensor_from_list(
                transform.GetTranslation(), dtype="float32", device=self._device
            )
            orientations[write_idx] = self._backend_utils.gf_quat_to_tensor(
                transform.GetRotation().GetQuat(), device=self._device
            )
            write_idx += 1
        return positions, orientations

    def set_world_poses(
        self,
        positions: Optional[Union[np.ndarray, torch.Tensor]] = None,
        orientations: Optional[Union[np.ndarray, torch.Tensor]] = None,
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets prim poses in the view with respect to the world's frame.

        Args:
            positions (Optional[Union[np.ndarray, torch.Tensor]], optional): positions in the world frame of the prims. shape is (M, 3).
                                                                             Defaults to None, which means left unchanged.
            orientations (Optional[Union[np.ndarray, torch.Tensor]], optional): quaternion orientations in the world frame of the prims. 
                                                                                quaternion is scalar-first (w, x, y, z). shape is (M, 4).
                                                                                Defaults to None, which means left unchanged.
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        if positions is None or orientations is None:
            current_positions, current_orientations = self.get_world_poses(indices=indices)
            if positions is None:
                positions = current_positions
            if orientations is None:
                orientations = current_orientations
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        parent_transforms = self._backend_utils.create_zeros_tensor(
            shape=[indices.shape[0], 4, 4], dtype="float32", device=self._device
        )
        write_idx = 0
        for i in indices:
            parent_transforms[write_idx] = self._backend_utils.create_tensor_from_list(
                UsdGeom.Xformable(get_prim_parent(self._prims[i.tolist()])).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()
                ),
                dtype="float32",
                device=self._device,
            )
            write_idx += 1
        calculated_translations, calculated_orientations = self._backend_utils.get_local_from_world(
            parent_transforms, positions, orientations, self._device
        )
        XFormPrimView.set_local_poses(
            self, translations=calculated_translations, orientations=calculated_orientations, indices=indices
        )
        return

    def get_local_poses(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]:
        """Gets prim poses in the view with respect to the local's frame (the prim's parent frame).

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: 
                                          first index is translations in the local frame of the prims. shape is (M, 3). 
                                            second index is quaternion orientations in the local frame of the prims.
                                            quaternion is scalar-first (w, x, y, z). shape is (M, 4).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        translations = self._backend_utils.create_zeros_tensor(
            shape=[indices.shape[0], 3], dtype="float32", device=self._device
        )
        orientations = self._backend_utils.create_zeros_tensor(
            shape=[indices.shape[0], 4], dtype="float32", device=self._device
        )
        write_idx = 0
        for i in indices:
            translations[write_idx] = self._backend_utils.create_tensor_from_list(
                self._prims[i.tolist()].GetAttribute("xformOp:translate").Get(), dtype="float32", device=self._device
            )
            orientations[write_idx] = self._backend_utils.gf_quat_to_tensor(
                self._prims[i.tolist()].GetAttribute("xformOp:orient").Get(), device=self._device
            )
            write_idx += 1
        return translations, orientations

    def set_local_poses(
        self,
        translations: Optional[Union[np.ndarray, torch.Tensor]] = None,
        orientations: Optional[Union[np.ndarray, torch.Tensor]] = None,
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets prim poses in the view with respect to the local frame (the prim's parent frame).

        Args:
            translations (Optional[Union[np.ndarray, torch.Tensor]], optional): 
                                                          translations in the local frame of the prims
                                                          (with respect to its parent prim). shape is (M, 3).
                                                          Defaults to None, which means left unchanged.
            orientations (Optional[Union[np.ndarray, torch.Tensor]], optional): 
                                                          quaternion orientations in the local frame of the prims. 
                                                          quaternion is scalar-first (w, x, y, z). shape is (M, 4).
                                                          Defaults to None, which means left unchanged.
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if translations is not None:
            write_idx = 0
            for i in indices:
                properties = self._prims[i.tolist()].GetPropertyNames()
                translation = Gf.Vec3d(*translations[write_idx].tolist())
                if "xformOp:translate" not in properties:
                    carb.log_error(
                        "Translate property needs to be set for {} before setting its position".format(self.name)
                    )
                xform_op = self._prims[i.tolist()].GetAttribute("xformOp:translate")
                xform_op.Set(translation)
                write_idx += 1
        if orientations is not None:
            write_idx = 0
            for i in indices:
                properties = self._prims[i.tolist()].GetPropertyNames()
                if "xformOp:orient" not in properties:
                    carb.log_error(
                        "Orient property needs to be set for {} before setting its orientation".format(self.name)
                    )
                xform_op = self._prims[i.tolist()].GetAttribute("xformOp:orient")
                if xform_op.GetTypeName() == "quatf":
                    rotq = Gf.Quatf(*orientations[write_idx].tolist())
                else:
                    rotq = Gf.Quatd(*orientations[write_idx].tolist())
                xform_op.Set(rotq)
                write_idx += 1
        return

    def get_world_scales(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets prim scales in the view with respect to the world's frame.


        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            Union[np.ndarray, torch.Tensor]: scales applied to the prim's dimensions in the world frame. shape is (M, 3).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        scales = self._backend_utils.create_zeros_tensor(
            shape=[indices.shape[0], 3], dtype="float32", device=self._device
        )
        write_idx = 0
        for i in indices:
            prim_tf = UsdGeom.Xformable(self._prims[i.tolist()]).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            transform = Gf.Transform()
            transform.SetMatrix(prim_tf)
            scales[write_idx] = self._backend_utils.create_tensor_from_list(
                transform.GetScale(), dtype="float32", device=self._device
            )
            write_idx += 1
        return scales

    def set_local_scales(
        self,
        scales: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets prim scales in the view with respect to the local frame (the prim's parent frame).

        Args:
            scales (Optional[Union[np.ndarray, torch.Tensor]]): scales to be applied to the prim's dimensions in the view. 
                                                                shape is (M, 3).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        read_idx = 0
        for i in indices:
            scale = Gf.Vec3d(*scales[read_idx].tolist())
            properties = self._prims[i.tolist()].GetPropertyNames()
            if "xformOp:scale" not in properties:
                carb.log_error("Scale property needs to be set for {} before setting its scale".format(self.name))
            xform_op = self._prims[i.tolist()].GetAttribute("xformOp:scale")
            xform_op.Set(scale)
            read_idx += 1
        return

    def get_local_scales(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets prim scales in the view with respect to the local frame (the parent's frame).

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            Union[np.ndarray, torch.Tensor]: scales applied to the prim's dimensions in the local frame. shape is (M, 3).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        scales = self._backend_utils.create_zeros_tensor(
            shape=[indices.shape[0], 3], dtype="float32", device=self._device
        )
        write_idx = 0
        for i in indices:
            scales[write_idx] = self._backend_utils.create_tensor_from_list(
                self._prims[i.tolist()].GetAttribute("xformOp:scale").Get(), dtype="float32", device=self._device
            )
            write_idx += 1
        return scales

    def is_valid(self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None) -> bool:
        """
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indicies to specify which prims 
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            bool: True if all prim paths specified in the view correspond to a valid prim in stage. False otherwise.
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = True
        for index in indices:
            result = result and is_prim_path_valid(self._prim_paths[index.tolist()])
        return result
