from typing import List
import omni.usd
from pxr import UsdGeom, Gf, Vt, Sdf, PhysxSchema, Usd, UsdUtils
from omni.physx import get_physx_replicator_interface, get_physx_simulation_interface

import numpy as np
import carb


class Cloner:

    """ This class provides a set of simple APIs to make duplication of objects simple. 
        Objects can be cloned using this class to create copies of the same object,
        placed at user-specified locations in the scene.

        Note that the cloning process is performed in a for-loop, so performance should
        be expected to follow linear scaling with an increase of clones.
    """

    def __init__(self):
        self._base_env_path = None
        self._root_path = None

    def define_base_env(self, base_env_path: str):
        """ Creates a USD Scope at base_env_path. This is designed to be the parent that holds all clones.

        Args:
            base_env_path (str): Path to create the USD Scope at.
        """

        UsdGeom.Scope.Define(omni.usd.get_context().get_stage(), base_env_path)
        self._base_env_path = base_env_path

    def generate_paths(self, root_path: str, num_paths: int):

        """ Generates a list of paths under the root path specified. 

        Args:
            root_path (str): Base path where new paths will be created under.
            num_paths (int): Number of paths to generate.

        Returns:
            paths (List[str]): A list of paths
        """

        self._root_path = root_path + "_"
        return [f"{root_path}_{i}" for i in range(num_paths)]

    def _replicate_physics(self, source_prim_path: str, prim_paths: list, base_env_path: str, root_path: str):
        """ Replicates physics properties directly in omni.physics to avoid performance bottlenecks when parsing physics. 

        Args:
            source_prim_path (str): Path of source object.
            prim_paths (List[str]): List of destination paths.
            base_env_path (str): Path to namespace for all environments.
            root_path (str): Prefix path for each environment.
        Raises:
            Exception: Raises exception if base_env_path is None or root_path is None.

        """
        assert base_env_path is not None or self._base_env_path is not None, "base_env_path needs to be specified!"
        assert root_path is not None or self._root_path is not None, "root_path needs to be specified!"

        clone_base_path = self._root_path if root_path is None else root_path
        clone_root = self._base_env_path if base_env_path is None else base_env_path
        num_replications = len(prim_paths) if self._replicate_first else len(prim_paths) - 1

        def replicationAttachFn(stageId):
            exclude_paths = [clone_root]
            return exclude_paths

        def replicationAttachEndFn(stageId):
            get_physx_replicator_interface().replicate(stageId, source_prim_path, num_replications)

        def hierarchyRenameFn(replicatePath, index):
            if self._replicate_first:
                stringPath = clone_base_path + str(index)
            else:
                stringPath = clone_base_path + str(index + 1)
            return stringPath

        stage = omni.usd.get_context().get_stage()
        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        get_physx_replicator_interface().register_replicator(
            stageId, replicationAttachFn, replicationAttachEndFn, hierarchyRenameFn
        )

    def clone(
        self,
        source_prim_path: str,
        prim_paths: List[str],
        positions: np.ndarray = None,
        orientations: np.ndarray = None,
        replicate_physics: bool = False,
        base_env_path: str = None,
        root_path: str = None,
    ):

        """ Clones a source prim at user-specified destination paths. 
            Clones will be placed at user-specified positions and orientations. 

        Args:
            source_prim_path (str): Path of source object.
            prim_paths (List[str]): List of destination paths.
            positions (np.ndarray): Numpy array containing target positions of clones. Dimension must equal length of prim_paths.
                                    Defaults to None. Clones will be placed at (0, 0, 0) if not specified.
            orientations (np.ndarray): Numpy array containing target orientations of clones. Dimension must equal length of prim_paths.
                                    Defaults to None. Clones will have identity orientation (1, 0, 0, 0) if not specified.
            replicate_physics (bool): Uses omni.physics replication. This will replicate physics properties directly for paths beginning with root_path and skip physics parsing for anything under the base_env_path. 
            base_env_path (str): Path to namespace for all environments. Required if replicate_physics=True and define_base_env() not called.
            root_path (str): Prefix path for each environment. Required if replicate_physics=True and generate_paths() not called.
        Raises:
            Exception: Raises exception if source prim path is not valid.

        """

        stage = omni.usd.get_context().get_stage()

        if positions is not None:
            assert len(positions) == len(prim_paths), "dimension mismatch between positions and prim_paths!"
        if orientations is not None:
            assert len(orientations) == len(prim_paths), "dimension mismatch between orientations and prim_paths!"

        # make sure source prim has valid xform properties
        source_prim = stage.GetPrimAtPath(source_prim_path)
        if not source_prim:
            raise Exception("Source prim does not exist")
        properties = source_prim.GetPropertyNames()
        xformable = UsdGeom.Xformable(source_prim)
        # get current position and orientation
        T_p_w = xformable.ComputeParentToWorldTransform(Usd.TimeCode.Default())
        T_l_w = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        T_l_p = Gf.Transform()
        T_l_p.SetMatrix(Gf.Matrix4d(np.matmul(T_l_w, np.linalg.inv(T_p_w)).tolist()))
        current_translation = T_l_p.GetTranslation()
        current_orientation = T_l_p.GetRotation().GetQuat()

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
        xformable.ClearXformOpOrder()
        for prop_name in properties:
            if prop_name in properties_to_remove:
                source_prim.RemoveProperty(prop_name)
        if "xformOp:scale" not in properties:
            xform_op_scale = xformable.AddXformOp(UsdGeom.XformOp.TypeScale, UsdGeom.XformOp.PrecisionDouble, "")
            xform_op_scale.Set(Gf.Vec3d([1.0, 1.0, 1.0]))
        else:
            xform_op_scale = UsdGeom.XformOp(source_prim.GetAttribute("xformOp:scale"))

        if "xformOp:translate" not in properties:
            xform_op_tranlsate = xformable.AddXformOp(
                UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, ""
            )
        else:
            xform_op_tranlsate = UsdGeom.XformOp(source_prim.GetAttribute("xformOp:translate"))
        xform_op_tranlsate.Set(current_translation)

        if "xformOp:orient" not in properties:
            xform_op_rot = xformable.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
        else:
            xform_op_rot = UsdGeom.XformOp(source_prim.GetAttribute("xformOp:orient"))
        xform_op_rot.Set(current_orientation)

        xformable.SetXformOpOrder([xform_op_tranlsate, xform_op_rot, xform_op_scale])
        current_scale = Gf.Vec3d(source_prim.GetAttribute("xformOp:scale").Get())

        # set source actor transform
        self._replicate_first = True  # if source path is not in clone paths
        if source_prim_path in prim_paths:
            self._replicate_first = False
            idx = prim_paths.index(source_prim_path)
            prim = UsdGeom.Xform(stage.GetPrimAtPath(source_prim_path))

            if positions is not None:
                translation = Gf.Vec3d(positions[idx][0], positions[idx][1], positions[idx][2])
            else:
                translation = Gf.Vec3d(0, 0, 0)

            if orientations is not None:
                orientation = Gf.Quatd(
                    orientations[idx][0], Gf.Vec3d(orientations[idx][1], orientations[idx][2], orientations[idx][3])
                )
            else:
                orientation = Gf.Quatd.GetIdentity()

            # overwrite translation and orientation to values specified
            prim.GetPrim().GetAttribute("xformOp:translate").Set(translation)
            prim.GetPrim().GetAttribute("xformOp:orient").Set(orientation)

        has_clones = False
        with Sdf.ChangeBlock():
            for i, prim_path in enumerate(prim_paths):
                if prim_path != source_prim_path:
                    has_clones = True
                    env_spec = Sdf.CreatePrimInLayer(stage.GetRootLayer(), prim_path)
                    env_spec.inheritPathList.Prepend(source_prim_path)

                    if positions is not None:
                        translation = Gf.Vec3d(positions[i][0], positions[i][1], positions[i][2])
                    else:
                        translation = Gf.Vec3d(0, 0, 0)

                    if orientations is not None:
                        orientation = Gf.Quatd(
                            orientations[i][0], Gf.Vec3d(orientations[i][1], orientations[i][2], orientations[i][3])
                        )
                    else:
                        orientation = Gf.Quatd.GetIdentity()

                    op_order_spec = Sdf.AttributeSpec(
                        env_spec, UsdGeom.Tokens.xformOpOrder, Sdf.ValueTypeNames.TokenArray
                    )
                    op_order_spec.default = Vt.TokenArray({"xformOp:translate", "xformOp:orient", "xformOp:scale"})

                    translate_spec = Sdf.AttributeSpec(env_spec, "xformOp:translate", Sdf.ValueTypeNames.Double3)
                    translate_spec.default = translation

                    orient_spec = Sdf.AttributeSpec(env_spec, "xformOp:orient", Sdf.ValueTypeNames.Quatd)
                    orient_spec.default = orientation

                    scale_spec = Sdf.AttributeSpec(env_spec, "xformOp:scale", Sdf.ValueTypeNames.Double3)
                    scale_spec.default = current_scale

        if replicate_physics and has_clones:
            self._replicate_physics(source_prim_path, prim_paths, base_env_path, root_path)

    def filter_collisions(
        self, physicsscene_path: str, collision_root_path: str, prim_paths: List[str], global_paths: List[str] = []
    ):
        """ Filters collisions between clones. Clones will not collide with each other, but can collide with objects specified in global_paths.
        
        Args:
            physicsscene_path (str): Path to PhysicsScene object in stage.
            collision_root_path (str): Path to place collision groups under.
            prim_paths (List[str]): Paths of objects to filter out collision.
            global_paths (List[str]): Paths of objects to generate collision (e.g. ground plane).

        """

        stage = omni.usd.get_context().get_stage()
        physx_scene = PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath(physicsscene_path))

        # We invert the collision group filters for more efficient collision filtering across environments
        physx_scene.CreateInvertCollisionGroupFilterAttr().Set(True)

        collision_scope = UsdGeom.Scope.Define(stage, collision_root_path)

        with Sdf.ChangeBlock():
            if len(global_paths) > 0:
                global_collision_group_path = collision_root_path + "/global_group"
                # add collision group prim
                global_collision_group = Sdf.PrimSpec(
                    stage.GetRootLayer().GetPrimAtPath(collision_root_path),
                    "global_group",
                    Sdf.SpecifierDef,
                    "PhysicsCollisionGroup",
                )
                # prepend collision API schema
                global_collision_group.SetInfo(
                    Usd.Tokens.apiSchemas, Sdf.TokenListOp.Create({"CollectionAPI:colliders"})
                )

                # expansion rule
                expansion_rule = Sdf.AttributeSpec(
                    global_collision_group,
                    "collection:colliders:expansionRule",
                    Sdf.ValueTypeNames.Token,
                    Sdf.VariabilityUniform,
                )
                expansion_rule.default = "expandPrims"

                # includes rel
                global_includes_rel = Sdf.RelationshipSpec(
                    global_collision_group, "collection:colliders:includes", False
                )
                for global_path in global_paths:
                    global_includes_rel.targetPathList.Append(global_path)

                # filteredGroups rel
                global_filtered_groups = Sdf.RelationshipSpec(global_collision_group, "physics:filteredGroups", False)
                # We are using inverted collision group filtering, which means objects by default don't collide across
                # groups. We need to add this group as a filtered group, so that objects within this group collide with
                # each other.
                global_filtered_groups.targetPathList.Append(global_collision_group_path)

            # set collision groups and filters
            for i, prim_path in enumerate(prim_paths):
                collision_group_path = collision_root_path + f"/group{i}"
                # add collision group prim
                collision_group = Sdf.PrimSpec(
                    stage.GetRootLayer().GetPrimAtPath(collision_root_path),
                    f"group{i}",
                    Sdf.SpecifierDef,
                    "PhysicsCollisionGroup",
                )
                # prepend collision API schema
                collision_group.SetInfo(Usd.Tokens.apiSchemas, Sdf.TokenListOp.Create({"CollectionAPI:colliders"}))

                # expansion rule
                expansion_rule = Sdf.AttributeSpec(
                    collision_group,
                    "collection:colliders:expansionRule",
                    Sdf.ValueTypeNames.Token,
                    Sdf.VariabilityUniform,
                )
                expansion_rule.default = "expandPrims"

                # includes rel
                includes_rel = Sdf.RelationshipSpec(collision_group, "collection:colliders:includes", False)
                includes_rel.targetPathList.Append(prim_path)

                # filteredGroups rel
                filtered_groups = Sdf.RelationshipSpec(collision_group, "physics:filteredGroups", False)
                # We are using inverted collision group filtering, which means objects by default don't collide across
                # groups. We need to add this group as a filtered group, so that objects within this group collide with
                # each other.
                filtered_groups.targetPathList.Append(collision_group_path)
                if len(global_paths) > 0:
                    filtered_groups.targetPathList.Append(global_collision_group_path)
                    global_filtered_groups.targetPathList.Append(collision_group_path)
