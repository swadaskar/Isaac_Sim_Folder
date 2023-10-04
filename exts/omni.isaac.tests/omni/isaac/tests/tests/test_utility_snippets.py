# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test

################################################################################
### !!!IMPORTANT!!!
### All of the tests below are for utility snippets from the isaac sim docs.
### If you fix an issue here make sure to update the code in the docs as well
### The idea is that we can catch any api changes and update the docs appropriately
################################################################################


class TestUtilitySnippets(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        pass

    # simple fastcache smoke test
    async def test_physics_scene(self):
        ### Code Start

        import omni
        from pxr import Gf, Sdf, UsdPhysics

        stage = omni.usd.get_context().get_stage()
        # Add a physics scene prim to stage
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
        # Set gravity vector
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

        ### Code End

        ### Code Start

        from pxr import PhysxSchema

        PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/physicsScene"))
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/World/physicsScene")
        physxSceneAPI.CreateEnableCCDAttr(True)
        physxSceneAPI.CreateEnableStabilizationAttr(True)
        physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
        physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
        physxSceneAPI.CreateSolverTypeAttr("TGS")

        ### Code End

        ### Code Start

        import omni
        from pxr import PhysicsSchemaTools

        stage = omni.usd.get_context().get_stage()
        PhysicsSchemaTools.addGroundPlane(stage, "/World/groundPlane", "Z", 100, Gf.Vec3f(0, 0, -100), Gf.Vec3f(1.0))

        ### Code End
        pass

    async def test_enable_physics_collision_convex(self):

        ###
        import omni
        from omni.physx.scripts import utils

        # Create a cube mesh in the stage
        stage = omni.usd.get_context().get_stage()
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Get the prim
        cube_prim = stage.GetPrimAtPath(path)
        # Enable physics on prim
        # If a tighter collision approximation is desired use convexDecomposition instead of convexHull
        utils.setRigidBody(cube_prim, "convexHull", False)

        ###

    async def test_enable_physics_collision_decomp(self):
        ###
        import omni
        from omni.physx.scripts import utils

        # Create a cube mesh in the stage
        stage = omni.usd.get_context().get_stage()
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Get the prim
        cube_prim = stage.GetPrimAtPath(path)
        # Enable physics on prim
        # If a tighter collision approximation is desired use convexDecomposition instead of convexHull
        utils.setRigidBody(cube_prim, "convexDecomposition", False)
        ###
        pass

    async def test_mass_properties(self):
        ###
        import omni
        from pxr import UsdPhysics
        from omni.physx.scripts import utils

        stage = omni.usd.get_context().get_stage()
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Get the prim
        cube_prim = stage.GetPrimAtPath(path)
        # Make it a rigid body
        utils.setRigidBody(cube_prim, "convexHull", False)

        mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
        mass_api.CreateMassAttr(10)
        ### Alternatively set the density
        mass_api.CreateDensityAttr(1000)
        pass

    async def test_traverse_assign_collision(self):
        import omni
        from pxr import Usd, UsdGeom, Gf
        from omni.physx.scripts import utils

        stage = omni.usd.get_context().get_stage()

        def add_cube(stage, path, size: float = 10, offset: Gf.Vec3d = Gf.Vec3d(0, 0, 0)):
            cubeGeom = UsdGeom.Cube.Define(stage, path)
            cubeGeom.CreateSizeAttr(size)
            cubeGeom.AddTranslateOp().Set(offset)

        ### The following prims are added for illustrative purposes
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Torus")
        # all prims under AddCollision will get collisons assigned
        add_cube(stage, "/World/Cube_0", offset=Gf.Vec3d(100, 100, 0))
        # create a prim nested under without a parent
        add_cube(stage, "/World/Nested/Cube", offset=Gf.Vec3d(100, 0, 100))
        ###

        # Traverse all prims in the stage starting at this path
        curr_prim = stage.GetPrimAtPath("/")

        for prim in Usd.PrimRange(curr_prim):
            # only process shapes and meshes
            if (
                prim.IsA(UsdGeom.Cylinder)
                or prim.IsA(UsdGeom.Capsule)
                or prim.IsA(UsdGeom.Cone)
                or prim.IsA(UsdGeom.Sphere)
                or prim.IsA(UsdGeom.Cube)
            ):
                # use a ConvexHull for regular prims
                utils.setCollider(prim, approximationShape="convexHull")
            elif prim.IsA(UsdGeom.Mesh):
                # "None" will use the base triangle mesh if available
                # Can also use "convexDecomposition", "convexHull", "boundingSphere", "boundingCube"
                utils.setCollider(prim, approximationShape="none")
            pass
        pass

    async def test_material(self):
        ###
        import omni
        from pxr import UsdShade, Sdf, Gf

        mtl_created_list = []
        # Create a new material using OmniGlass.mdl
        omni.kit.commands.execute(
            "CreateAndBindMdlMaterialFromLibrary",
            mdl_name="OmniGlass.mdl",
            mtl_name="OmniGlass",
            mtl_created_list=mtl_created_list,
        )
        # Get reference to created material
        stage = omni.usd.get_context().get_stage()
        mtl_prim = stage.GetPrimAtPath(mtl_created_list[0])
        # Set material inputs, these can be determined by looking at the .mdl file
        # or by selecting the Shader attached to the Material in the stage window and looking at the details panel
        omni.usd.create_material_input(mtl_prim, "glass_color", Gf.Vec3f(0, 1, 0), Sdf.ValueTypeNames.Color3f)
        omni.usd.create_material_input(mtl_prim, "glass_ior", 1.0, Sdf.ValueTypeNames.Float)
        # Create a prim to apply the material to
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Get the path to the prim
        cube_prim = stage.GetPrimAtPath(path)
        # Bind the material to the prim
        cube_mat_shade = UsdShade.Material(mtl_prim)
        UsdShade.MaterialBindingAPI(cube_prim).Bind(cube_mat_shade, UsdShade.Tokens.strongerThanDescendants)
        ###
        pass

    async def test_material_texture(self):
        ###
        import omni
        import carb
        from pxr import UsdShade, Sdf
        from omni.isaac.core.utils.nucleus import get_assets_root_path

        assets_root_path = get_assets_root_path()
        mtl_created_list = []
        # Create a new material using OmniPBR.mdl
        omni.kit.commands.execute(
            "CreateAndBindMdlMaterialFromLibrary",
            mdl_name="OmniPBR.mdl",
            mtl_name="OmniPBR",
            mtl_created_list=mtl_created_list,
        )
        stage = omni.usd.get_context().get_stage()
        mtl_prim = stage.GetPrimAtPath(mtl_created_list[0])
        # Set material inputs, these can be determined by looking at the .mdl file
        # or by selecting the Shader attached to the Material in the stage window and looking at the details panel
        omni.usd.create_material_input(
            mtl_prim,
            "diffuse_texture",
            assets_root_path + "/Isaac/Samples/DR/Materials/Textures/marble_tile.png",
            Sdf.ValueTypeNames.Asset,
        )
        # Create a prim to apply the material to
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Get the path to the prim
        cube_prim = stage.GetPrimAtPath(path)
        # Bind the material to the prim
        cube_mat_shade = UsdShade.Material(mtl_prim)
        UsdShade.MaterialBindingAPI(cube_prim).Bind(cube_mat_shade, UsdShade.Tokens.strongerThanDescendants)
        ###
        pass

    async def test_add_transform(self):
        ###
        import omni
        from pxr import Gf, UsdGeom

        # Create a cube mesh in the stage
        stage = omni.usd.get_context().get_stage()
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Get the prim and set its transform matrix
        cube_prim = stage.GetPrimAtPath(path)
        xform = UsdGeom.Xformable(cube_prim)
        transform = xform.AddTransformOp()
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(10.0, 1.0, 1.0))
        mat.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 1, 0), 290))
        transform.Set(mat)
        ###
        pass

    async def test_align_prims(self):
        ###
        import omni
        from pxr import UsdGeom, Gf

        stage = omni.usd.get_context().get_stage()
        # Create a cube
        result, path_a = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        prim_a = stage.GetPrimAtPath(path_a)
        # change the cube pose
        xform = UsdGeom.Xformable(prim_a)
        transform = xform.AddTransformOp()
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(10.0, 1.0, 1.0))
        mat.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 1, 0), 290))
        transform.Set(mat)
        # Create a second cube
        result, path_b = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        prim_b = stage.GetPrimAtPath(path_b)
        # Get the transform of the first cube
        pose = omni.usd.utils.get_world_transform_matrix(prim_a)
        # Clear the transform on the second cube
        xform = UsdGeom.Xformable(prim_b)
        xform.ClearXformOpOrder()
        # Set the pose of prim_b to that of prim_b
        xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
        xform_op.Set(pose)
        ###

    async def test_get_world_transform(self):
        ###
        import omni
        from pxr import UsdGeom, Gf

        usd_context = omni.usd.get_context()
        stage = usd_context.get_stage()

        #### For testing purposes we create and select a prim
        #### This section can be removed if you already have a prim selected
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        cube_prim = stage.GetPrimAtPath(path)
        # change the cube pose
        xform = UsdGeom.Xformable(cube_prim)
        transform = xform.AddTransformOp()
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(10.0, 1.0, 1.0))
        mat.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 1, 0), 290))
        transform.Set(mat)
        omni.usd.get_context().get_selection().set_prim_path_selected(path, True, True, True, False)
        ####

        # Get list of selected primitives
        selected_prims = usd_context.get_selection().get_selected_prim_paths()
        # Get the current timecode
        timeline = omni.timeline.get_timeline_interface()
        timecode = timeline.get_current_time() * timeline.get_time_codes_per_seconds()
        # Loop through all prims and print their transforms
        for s in selected_prims:
            curr_prim = stage.GetPrimAtPath(s)
            print("Selected", s)
            pose = omni.usd.utils.get_world_transform_matrix(curr_prim, timecode)
            print("Matrix Form:", pose)
            print("Translation: ", pose.ExtractTranslation())
            q = pose.ExtractRotation().GetQuaternion()
            print(
                "Rotation: ", q.GetReal(), ",", q.GetImaginary()[0], ",", q.GetImaginary()[1], ",", q.GetImaginary()[2]
            )
        ###
        pass

    async def test_save_to_file(self):
        import omni
        import carb
        from omni.isaac.core.utils.nucleus import get_assets_root_path

        assets_root = get_assets_root_path()
        # Create a prim
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Change the path as needed
        omni.usd.get_context().save_as_stage(assets_root + "/Users/test/saved.usd", None)

    async def test_async_task(self):
        ###
        import asyncio
        import omni

        # Async task that pauses simulation once the incoming task is complete
        async def pause_sim(task):
            done, pending = await asyncio.wait({task})
            if task in done:
                print("Waited until next frame, pausing")
                omni.timeline.get_timeline_interface().pause()

        # Start simulation, then wait a frame and run the pause_sim task
        omni.timeline.get_timeline_interface().play()
        task = asyncio.ensure_future(omni.kit.app.get_app().next_update_async())
        asyncio.ensure_future(pause_sim(task))
        ###

    async def test_camera_intrinsics(self):
        import omni
        from omni.syntheticdata import helpers
        import math

        stage = omni.usd.get_context().get_stage()
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        # Set viewport resolution, changes will occur on next frame
        viewport_api.set_texture_resolution((512, 512))
        # get resolution
        (width, height) = viewport_api.get_texture_resolution()
        aspect_ratio = width / height
        # get camera prim attached to viewport
        camera = stage.GetPrimAtPath(viewport_api.get_active_camera())
        focal_length = camera.GetAttribute("focalLength").Get()
        horiz_aperture = camera.GetAttribute("horizontalAperture").Get()
        vert_aperture = camera.GetAttribute("verticalAperture").Get()
        # Pixels are square so we can also do:
        # vert_aperture = height / width * horiz_aperture
        near, far = camera.GetAttribute("clippingRange").Get()
        fov = 2 * math.atan(horiz_aperture / (2 * focal_length))

        # helper to compute projection matrix
        proj_mat = helpers.get_projection_matrix(fov, aspect_ratio, near, far)

        # compute focal point and center
        focal_x = height * focal_length / vert_aperture
        focal_y = width * focal_length / horiz_aperture
        center_x = height * 0.5
        center_y = width * 0.5

    async def test_get_mesh_size(self):
        import omni
        from pxr import Usd, UsdGeom, Gf

        stage = omni.usd.get_context().get_stage()
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cone")
        # Get the prim
        prim = stage.GetPrimAtPath(path)
        # Get the size
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
        bbox_cache.Clear()
        prim_bbox = bbox_cache.ComputeWorldBound(prim)
        prim_range = prim_bbox.ComputeAlignedRange()
        prim_size = prim_range.GetSize()
        pass
