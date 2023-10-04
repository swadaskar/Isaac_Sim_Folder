# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import gc
import omni.ext
import omni.usd
import omni.ui as ui
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
import omni.kit.utils
import omni.kit.commands
from pxr import Usd, UsdGeom, Sdf, UsdShade, Gf
import weakref
import carb

from omni.isaac.ui.style import get_style
from omni.isaac.ui.ui_utils import combo_cb_str_builder, btn_builder, cb_builder, str_builder

EXTENSION_NAME = "Mesh Merge Tool"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Called to load the extension"""

        self._stage = omni.usd.get_context().get_stage()
        self._window = omni.ui.Window(
            EXTENSION_NAME, width=600, height=400, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.deferred_dock_in("Console", omni.ui.DockPolicy.DO_NOTHING)
        self._window.set_visibility_changed_fn(self._on_window)
        self._menu_items = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        add_menu_items(self._menu_items, "Isaac Utils")
        self.models = {}
        self.parent_xform = None

    def build_ui(self):
        with self._window.frame:
            with ui.HStack(spacing=5):
                with ui.VStack(height=0, spacing=5):
                    input_frame = ui.CollapsableFrame(
                        title="Input",
                        style=get_style(),
                        style_type_name_override="CollapsableFrame",
                        horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    )

                    def input_changed(model):
                        if input_frame.collapsed:
                            input_frame.title = "Input ({})".format(model.get_value_as_string())

                    def collapsed_changed_fn(frame, base_txt, model, collapsed):
                        if collapsed:
                            frame.title = base_txt + " ({})".format(model.get_value_as_string())
                        else:
                            frame.title = base_txt

                    with input_frame:
                        with ui.VStack(spacing=2, height=0):
                            self.models["input_mesh"] = str_builder("Source Prim", read_only=True)
                            self.models["input_mesh"].set_value("No Mesh Selected")
                            self.models["input_mesh"].add_value_changed_fn(lambda a: input_changed(a))
                            self.models["submesh"] = str_builder("Submeshes", read_only=True)
                            self.models["subset"] = str_builder("Geometry Subsets", read_only=True)
                            self.models["materials"] = str_builder("Materials", read_only=True)

                    input_frame.set_collapsed_changed_fn(
                        lambda c, f=input_frame, m=self.models["input_mesh"]: collapsed_changed_fn(f, "Input", m, c)
                    )
                    output_frame = ui.CollapsableFrame(
                        title="Output",
                        style=get_style(),
                        style_type_name_override="CollapsableFrame",
                        horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    )
                    with output_frame:
                        with ui.VStack(spacing=2, height=0):
                            self.models["output_mesh"] = str_builder("Destination Prim", read_only=True)
                            self.models["output_mesh"].set_value("No Mesh Selected")

                            self.models["output_subset"] = str_builder("Geometry Subsets", read_only=True)
                    output_frame.set_collapsed_changed_fn(
                        lambda c, f=output_frame, m=self.models["output_mesh"]: collapsed_changed_fn(f, "Output", m, c)
                    )
                with ui.VStack(spacing=3, height=0):
                    self.parent_xform = cb_builder(
                        label="Clear Parent Transform",
                        tooltip="If selected, Creates merged mesh with origin at global orign, otherwise keeps origin at parent's origin",
                    )
                    self.override_looks_directory = combo_cb_str_builder(
                        "Overrride Looks Directory",
                        tooltip="If selected, replaces the path to all selected materials with the prim path provided",
                        on_clicked_fn=lambda a: self._on_stage_event(),
                        default_val=[False, ""],
                    )
                    btn_builder(label="Merge Selected Prim", text="Merge", on_clicked_fn=self._merge_mesh)

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def _on_window(self, visible):
        if self._window.visible:
            self._usd_context = omni.usd.get_context()
            if self._usd_context is not None:
                self._selection = self._usd_context.get_selection()
                self._events = self._usd_context.get_stage_event_stream()
                self._stage_event_sub = self._events.create_subscription_to_pop(
                    self._on_stage_event, name="Mesh merge tool stage event"
                )
            self.build_ui()
            self._on_stage_event()
        else:
            self._stage_event_sub = None

    def _on_stage_event(self, event=None):  # Empty event is a forced update from UI
        if self._window.visible:
            if not event or event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
                selection = self._selection.get_selected_prim_paths()
                stage = self._usd_context.get_stage()
                if len(selection) == 0:
                    curr_prim = None
                    self.models["input_mesh"].set_value("No Mesh Selected")
                    pass
                else:
                    curr_prim = stage.GetPrimAtPath(selection[0])
                    self.models["input_mesh"].set_value(selection[0])
                    total_meshes = 0
                    total_subsets = 0
                    materials = {}
                    for child_prim in Usd.PrimRange(curr_prim, Usd.TraverseInstanceProxies()):
                        imageable = UsdGeom.Imageable(child_prim)
                        visible = imageable.ComputeVisibility(Usd.TimeCode.Default())
                        if child_prim.IsA(UsdGeom.Mesh) and visible != UsdGeom.Tokens.invisible:
                            usdMesh = UsdGeom.Mesh(child_prim)
                            mat, rel = UsdShade.MaterialBindingAPI(usdMesh).ComputeBoundMaterial()
                            mat_path = str(mat.GetPath())
                            if self.override_looks_directory[0].get_value_as_bool():
                                if stage.GetPrimAtPath(self.override_looks_directory[1].get_value_as_string()):
                                    mat_path = "{}/{}".format(
                                        self.override_looks_directory[1].get_value_as_string(),
                                        mat_path.rsplit("/", 1)[-1],
                                    )
                                else:
                                    carb.log_error("override materials path is invalid.")
                            if not rel:
                                mat_path = "/None"
                            if rel:
                                materials[mat_path] = 1
                            subsets = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(child_prim))
                            if len(subsets):
                                total_subsets = total_subsets + len(subsets)
                                for s in subsets:
                                    mat, rel = UsdShade.MaterialBindingAPI(s).ComputeBoundMaterial()
                                    mat_path = str(mat.GetPath())
                                    if self.override_looks_directory[0].get_value_as_bool():
                                        if stage.GetPrimAtPath(self.override_looks_directory[1].get_value_as_string()):
                                            mat_path = "{}/{}".format(
                                                self.override_looks_directory[1].get_value_as_string(),
                                                mat_path.rsplit("/", 1)[-1],
                                            )
                                        else:
                                            carb.log_error("override materials path is invalid.")
                                    if not rel:
                                        mat_path = "/None"
                                    materials[mat_path] = 1
                            else:
                                total_meshes = total_meshes + 1

                    # print(*materials, sep = "\n")

                    self.models["submesh"].set_value(total_meshes)
                    self.models["subset"].set_value(total_subsets)
                    self.models["materials"].set_value(len(materials))
                    merged_path = "/Merged/" + str(curr_prim.GetName())
                    merged_path = omni.usd.get_stage_next_free_path(stage, merged_path, False)
                    self.models["output_mesh"].set_value(merged_path)
                    self.models["output_subset"].set_value(len(materials))

    def _merge_mesh(self):
        stage = omni.usd.get_context().get_stage()
        selectedPrims = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if len(selectedPrims) > 0:
            curr_prim_path = selectedPrims[-1]
        else:
            curr_prim_path = None
            carb.log_warn("Cannot merge, no mesh selected")
            return
        curr_prim = stage.GetPrimAtPath(curr_prim_path)
        prim_transform = omni.usd.utils.get_world_transform_matrix(curr_prim, Usd.TimeCode.Default())
        count = 0
        meshes = []
        for child_prim in Usd.PrimRange(curr_prim, Usd.TraverseInstanceProxies()):
            imageable = UsdGeom.Imageable(child_prim)
            visible = imageable.ComputeVisibility(Usd.TimeCode.Default())
            if (
                child_prim.IsA(UsdGeom.Mesh)
                and visible != UsdGeom.Tokens.invisible
                and imageable.GetPurposeAttr().Get() in ["default", "render"]
            ):
                carb.log_warn(child_prim.GetName())
                usdMesh = UsdGeom.Mesh(child_prim)
                mesh = {}
                mesh["points"] = usdMesh.GetPointsAttr().Get()
                world_mtx = omni.usd.utils.get_world_transform_matrix(child_prim, Usd.TimeCode.Default())
                if self.parent_xform.get_value_as_bool():
                    world_mtx = prim_transform * world_mtx * prim_transform.GetInverse()
                else:
                    world_mtx = world_mtx * prim_transform.GetInverse()
                world_rot = world_mtx.ExtractRotation()
                # print(world_mtx)
                mesh["points"][:] = [world_mtx.TransformAffine(x) for x in mesh["points"]]
                mesh["normals"] = usdMesh.GetNormalsAttr().Get()
                mesh["attr_normals"] = usdMesh.GetPrim().GetAttribute("primvars:normals").Get()
                mesh["attr_normals_indices"] = usdMesh.GetPrim().GetAttribute("primvars:normals:indices").Get()
                if not mesh["attr_normals"]:
                    mesh["attr_normals"] = []
                if not mesh["attr_normals_indices"]:
                    mesh["attr_normals_indices"] = []
                if mesh["normals"]:
                    mesh["normals"][:] = [world_rot.TransformDir(x).GetNormalized() for x in mesh["normals"]]
                else:
                    mesh["normals"] = []
                    carb.log_warn(f"mesh doesn't contain normals: ({child_prim.GetName()})")
                if mesh["attr_normals"]:
                    mesh["attr_normals"][:] = [world_rot.TransformDir(x) for x in mesh["attr_normals"]]
                mesh["vertex_counts"] = usdMesh.GetFaceVertexCountsAttr().Get()
                mesh["vertex_indices"] = usdMesh.GetFaceVertexIndicesAttr().Get()
                # mesh["st"] = usdMesh.GetPrimvar("st").Get()
                mesh["name"] = child_prim.GetName()
                mat, rel = UsdShade.MaterialBindingAPI(usdMesh).ComputeBoundMaterial()
                mat_path = str(mat.GetPath())
                if self.override_looks_directory[0].get_value_as_bool():
                    _mat_path = "{}/{}".format(
                        self.override_looks_directory[1].get_value_as_string(), mat_path.rsplit("/", 1)[-1]
                    )
                    if stage.GetPrimAtPath(_mat_path):
                        mat_path = _mat_path
                    else:
                        carb.log_error(f"Overriden material not found ({mat_path})")
                if not rel:
                    mat_path = "/None"
                # if rel:
                #     mesh["mat"] = str(mat.GetPath())
                # else:
                mesh["mat"] = mat_path
                subsets = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(child_prim))
                mesh["subset"] = []
                for s in subsets:
                    mat, rel = UsdShade.MaterialBindingAPI(s).ComputeBoundMaterial()
                    mat_path = str(mat.GetPath())
                    if self.override_looks_directory[0].get_value_as_bool():
                        _mat_path = "{}/{}".format(
                            self.override_looks_directory[1].get_value_as_string(), mat_path.rsplit("/", 1)[-1]
                        )
                        if stage.GetPrimAtPath(_mat_path):
                            mat_path = _mat_path
                    if not rel:
                        mat_path = "/None"
                    mesh["subset"].append((mat_path, s.GetIndicesAttr().Get()))
                # print(mat.GetPath(), rel)
                # print("INDICES", mesh["normals"])
                meshes.append(mesh)
                # print(count)
                # print(len(mesh["points"]), len(mesh["normals"]), len(mesh["vertex_counts"]), len(mesh["vertex_indices"]))
                count = count + 1
        carb.log_info(f"Merging: {count} meshes")
        all_points = []
        all_normals = []
        all_normals_attr = []
        all_normals_indices = []
        all_vertex_counts = []
        all_vertex_indices = []
        all_mats = {}
        index_offset = 0
        normals_offset = 0
        index = 0
        range_offset = 0
        for mesh in meshes:
            all_points.extend(mesh["points"])
            all_normals.extend(mesh["normals"])
            all_normals_attr.extend(mesh["attr_normals"])
            mesh["attr_normals_indices"][:] = [x + normals_offset for x in mesh["attr_normals_indices"]]
            all_normals_indices.extend(mesh["attr_normals_indices"])
            if mesh["normals"]:
                mesh["normals"][:] = [world_rot.TransformDir(x).GetNormalized() for x in mesh["normals"]]
            all_vertex_counts.extend(mesh["vertex_counts"])
            mesh["vertex_indices"][:] = [x + index_offset for x in mesh["vertex_indices"]]
            all_vertex_indices.extend(mesh["vertex_indices"])
            # all_st.extend(mesh["st"])
            index_offset = index_offset + len(meshes[index]["points"])
            normals_offset = normals_offset + len(mesh["attr_normals_indices"])
            # print("Offset", index_offset)
            index = index + 1
            # create the material entry
            if len(mesh["subset"]) == 0:
                if mesh["mat"] not in all_mats:
                    all_mats[mesh["mat"]] = []
                all_mats[mesh["mat"]].extend([*range(range_offset, range_offset + len(mesh["vertex_counts"]), 1)])
            else:
                for subset in mesh["subset"]:
                    if subset[0] not in all_mats:
                        all_mats[subset[0]] = []
                    all_mats[subset[0]].extend([*(x + range_offset for x in subset[1])])
            range_offset = range_offset + len(mesh["vertex_counts"])
        merged_path = "/Merged/" + str(curr_prim.GetName())
        merged_path = omni.usd.get_stage_next_free_path(stage, merged_path, False)
        carb.log_info(f"Merging to path: {merged_path}")
        merged_mesh = UsdGeom.Mesh.Define(stage, merged_path)
        xform = UsdGeom.Xformable(merged_mesh)
        xform_op_t = xform.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, "")
        xform_op_r = xform.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
        xform_op_t.Set(prim_transform.ExtractTranslation())
        q = prim_transform.ExtractRotation().GetQuaternion()
        xform_op_r.Set(Gf.Quatd(q.GetReal(), q.GetImaginary()))
        # xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
        # if not self.parent_xform.get_value_as_bool():
        # xform_op.Set(prim_transform)
        # merged_mesh.CreateSubdivisionSchemeAttr("none")
        # merged_mesh.CreateTriangleSubdivisionRuleAttr("smooth")
        merged_mesh.CreatePointsAttr(all_points)
        if all_normals:
            merged_mesh.CreateNormalsAttr(all_normals)
            merged_mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        merged_mesh.CreateSubdivisionSchemeAttr("none")
        merged_mesh.CreateFaceVertexCountsAttr(all_vertex_counts)
        merged_mesh.CreateFaceVertexIndicesAttr(all_vertex_indices)
        if all_normals_attr:
            normals_attr = merged_mesh.GetPrim().CreateAttribute(
                "primvars:normals", Sdf.ValueTypeNames.Float3Array, False
            )
            normals_attr.Set(all_normals_attr)
            normals_attr.SetMetadata("interpolation", "vertex")
            merged_mesh.GetPrim().CreateAttribute("primvars:normals:indices", Sdf.ValueTypeNames.IntArray, False).Set(
                all_normals_indices
            )
        extent = merged_mesh.ComputeExtent(all_points)
        merged_mesh.CreateExtentAttr().Set(extent)
        # texCoord = merged_mesh.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying)
        # texCoord.Set(all_st)
        # print(all_mats)
        for name, counts in sorted(all_mats.items(), key=lambda a: a[0].rsplit("/", 1)[-1]):
            subset_name = merged_path + "/{}".format(name.rsplit("/", 1)[-1])
            geomSubset = UsdGeom.Subset.Define(stage, omni.usd.get_stage_next_free_path(stage, subset_name, False))
            geomSubset.CreateElementTypeAttr("face")
            geomSubset.CreateFamilyNameAttr("materialBind")
            # print(mesh["vertex_indices"])
            geomSubset.CreateIndicesAttr(counts)
            if name != "/None":
                material = UsdShade.Material.Get(stage, name)
                binding_api = UsdShade.MaterialBindingAPI(geomSubset)
                binding_api.Bind(material)

        # extent = usdMesh.ComputeExtent(Vertex)
        # usdMesh.GetExtentAttr().Set(extent)

    # def _merge_selected(self):
    #     stage = omni.usd.get_context().get_stage()
    #     selectedPrims = omni.usd.get_context().get_selection().get_selected_prim_paths()
    #     if len(selectedPrims) > 0:
    #         curr_prim_path = selectedPrims[-1]
    #     else:
    #         curr_prim_path = None
    #     curr_prim = stage.GetPrimAtPath(curr_prim_path)
    #     meshes_to_process = None
    #     while True:
    #         for child_prim in Usd.PrimRange(curr_prim):
    #             if (
    #                 child_prim.IsA(UsdGeom.Mesh)
    #                 and child_prim.GetParent() != curr_prim
    #                 and child_prim.GetParent().IsA(UsdGeom.Xformable)
    #             ):
    #                 meshes_to_process = child_prim
    #                 break
    #         if meshes_to_process is None:
    #             break
    #         print("Process", meshes_to_process)
    #         omni.kit.commands.execute(
    #             "MovePrimCommand",
    #             path_from=meshes_to_process.GetPrimPath(),
    #             path_to=curr_prim_path + "/" + meshes_to_process.GetName(),
    #             time_code=Usd.TimeCode.Default(),
    #             keep_world_transform=True,
    #             force_fallback=False,
    #         )
    #         meshes_to_process = None
    def on_shutdown(self):
        """Called when the extesion us unloaded"""
        remove_menu_items(self._menu_items, "Isaac Utils")
        self._window = None
        gc.collect()
