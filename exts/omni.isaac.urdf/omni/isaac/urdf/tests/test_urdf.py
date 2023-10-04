# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.commands
import os
from pxr import Sdf, Gf, UsdShade, PhysicsSchemaTools, UsdGeom, UsdPhysics
import pxr
import asyncio
import numpy as np


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestUrdf(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.urdf")
        self._extension_path = ext_manager.get_extension_path(ext_id)
        self.dest_path = os.path.abspath(self._extension_path + "/tests_out")
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self):
        # _urdf.release_urdf_interface(self._urdf_interface)
        await omni.kit.app.get_app().next_update_async()
        pass

    # Tests to make sure visual mesh names are incremented
    async def test_urdf_mesh_naming(self):
        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_names.urdf")
        stage = omni.usd.get_context().get_stage()

        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config.merge_fixed_joints = True
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)
        prim = stage.GetPrimAtPath("/test_names/cube/visuals")
        prim_range = prim.GetChildren()
        # There should be a total of 6 visual meshes after import
        self.assertEqual(len(prim_range), 6)

    # basic urdf test: joints and links are imported correctly
    async def test_urdf_basic(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_basic.urdf")
        stage = omni.usd.get_context().get_stage()
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")

        import_config.import_inertia_tensor = True
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)
        await omni.kit.app.get_app().next_update_async()

        prim = stage.GetPrimAtPath("/test_basic")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # make sure the joints exist
        root_joint = stage.GetPrimAtPath("/test_basic/root_joint")
        self.assertNotEqual(root_joint.GetPath(), Sdf.Path.emptyPath)

        wristJoint = stage.GetPrimAtPath("/test_basic/link_2/wrist_joint")
        self.assertNotEqual(wristJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(wristJoint.GetTypeName(), "PhysicsRevoluteJoint")

        fingerJoint = stage.GetPrimAtPath("/test_basic/palm_link/finger_1_joint")
        self.assertNotEqual(fingerJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(fingerJoint.GetTypeName(), "PhysicsPrismaticJoint")
        self.assertAlmostEqual(fingerJoint.GetAttribute("physics:upperLimit").Get(), 0.08)

        fingerLink = stage.GetPrimAtPath("/test_basic/finger_link_2")
        self.assertAlmostEqual(fingerLink.GetAttribute("physics:diagonalInertia").Get()[0], 2.0)
        self.assertAlmostEqual(fingerLink.GetAttribute("physics:mass").Get(), 3)

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()

        self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.0)
        pass

    async def test_urdf_save_to_file(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_basic.urdf")
        dest_path = os.path.abspath(self.dest_path + "/test_basic.usd")
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")

        import_config.import_inertia_tensor = True
        omni.kit.commands.execute(
            "URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config, dest_path=dest_path
        )
        await omni.kit.app.get_app().next_update_async()
        stage = pxr.Usd.Stage.Open(dest_path)
        prim = stage.GetPrimAtPath("/test_basic")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # make sure the joints exist
        root_joint = stage.GetPrimAtPath("/test_basic/root_joint")
        self.assertNotEqual(root_joint.GetPath(), Sdf.Path.emptyPath)

        wristJoint = stage.GetPrimAtPath("/test_basic/link_2/wrist_joint")
        self.assertNotEqual(wristJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(wristJoint.GetTypeName(), "PhysicsRevoluteJoint")

        fingerJoint = stage.GetPrimAtPath("/test_basic/palm_link/finger_1_joint")
        self.assertNotEqual(fingerJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(fingerJoint.GetTypeName(), "PhysicsPrismaticJoint")
        self.assertAlmostEqual(fingerJoint.GetAttribute("physics:upperLimit").Get(), 0.08)

        fingerLink = stage.GetPrimAtPath("/test_basic/finger_link_2")
        self.assertAlmostEqual(fingerLink.GetAttribute("physics:diagonalInertia").Get()[0], 2.0)
        self.assertAlmostEqual(fingerLink.GetAttribute("physics:mass").Get(), 3)

        self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.0)
        stage = None
        pass

    async def test_urdf_textured_obj(self):

        base_path = self._extension_path + "/data/urdf/tests/test_textures_urdf"
        basename = "cube_obj"
        dest_path = "{}/{}/{}.usd".format(self.dest_path, basename, basename)
        mats_path = "{}/{}/materials".format(self.dest_path, basename)
        omni.client.create_folder("{}/{}".format(self.dest_path, basename))
        omni.client.create_folder(mats_path)

        urdf_path = "{}/{}.urdf".format(base_path, basename)
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")

        omni.kit.commands.execute(
            "URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config, dest_path=dest_path
        )
        await omni.kit.app.get_app().next_update_async()
        result = omni.client.list(mats_path)
        self.assertEqual(result[0], omni.client._omniclient.Result.OK)
        self.assertEqual(len(result[1]), 4)  # Metallic texture is unsuported by assimp on OBJ
        pass

    async def test_urdf_textured_in_memory(self):

        base_path = self._extension_path + "/data/urdf/tests/test_textures_urdf"
        basename = "cube_obj"

        urdf_path = "{}/{}.urdf".format(base_path, basename)
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")

        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_urdf_textured_dae(self):

        base_path = self._extension_path + "/data/urdf/tests/test_textures_urdf"
        basename = "cube_dae"
        dest_path = "{}/{}/{}.usd".format(self.dest_path, basename, basename)
        mats_path = "{}/{}/materials".format(self.dest_path, basename)
        omni.client.create_folder("{}/{}".format(self.dest_path, basename))
        omni.client.create_folder(mats_path)

        urdf_path = "{}/{}.urdf".format(base_path, basename)
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")

        omni.kit.commands.execute(
            "URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config, dest_path=dest_path
        )
        await omni.kit.app.get_app().next_update_async()
        result = omni.client.list(mats_path)
        self.assertEqual(result[0], omni.client._omniclient.Result.OK)
        self.assertEqual(len(result[1]), 1)  # only albedo is supported for Collada
        pass

    async def test_urdf_overwrite_file(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_basic.urdf")
        dest_path = os.path.abspath(self._extension_path + "/data/urdf/tests/tests_out/test_basic.usd")
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")

        import_config.import_inertia_tensor = True
        omni.kit.commands.execute(
            "URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config, dest_path=dest_path
        )
        await omni.kit.app.get_app().next_update_async()
        omni.kit.commands.execute(
            "URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config, dest_path=dest_path
        )
        await omni.kit.app.get_app().next_update_async()

        stage = pxr.Usd.Stage.Open(dest_path)
        prim = stage.GetPrimAtPath("/test_basic")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # make sure the joints exist
        root_joint = stage.GetPrimAtPath("/test_basic/root_joint")
        self.assertNotEqual(root_joint.GetPath(), Sdf.Path.emptyPath)

        wristJoint = stage.GetPrimAtPath("/test_basic/link_2/wrist_joint")
        self.assertNotEqual(wristJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(wristJoint.GetTypeName(), "PhysicsRevoluteJoint")

        fingerJoint = stage.GetPrimAtPath("/test_basic/palm_link/finger_1_joint")
        self.assertNotEqual(fingerJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(fingerJoint.GetTypeName(), "PhysicsPrismaticJoint")
        self.assertAlmostEqual(fingerJoint.GetAttribute("physics:upperLimit").Get(), 0.08)

        fingerLink = stage.GetPrimAtPath("/test_basic/finger_link_2")
        self.assertAlmostEqual(fingerLink.GetAttribute("physics:diagonalInertia").Get()[0], 2.0)
        self.assertAlmostEqual(fingerLink.GetAttribute("physics:mass").Get(), 3)

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()
        self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.0)
        stage = None
        pass

    # advanced urdf test: test for all the categories of inputs that an urdf can hold
    async def test_urdf_advanced(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_advanced.urdf")
        stage = omni.usd.get_context().get_stage()

        # enable merging fixed joints
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config.merge_fixed_joints = True
        import_config.default_position_drive_damping = -1  # ignore this setting by making it -1
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)
        await omni.kit.app.get_app().next_update_async()

        # check if object is there
        prim = stage.GetPrimAtPath("/test_advanced")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # check color are imported
        mesh = stage.GetPrimAtPath("/test_advanced/link_1/visuals")
        self.assertNotEqual(mesh.GetPath(), Sdf.Path.emptyPath)
        mat, rel = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()
        shader = UsdShade.Shader(stage.GetPrimAtPath(mat.GetPath().pathString + "/Shader"))
        self.assertTrue(Gf.IsClose(shader.GetInput("diffuse_color_constant").Get(), Gf.Vec3f(0, 0.8, 0), 1e-5))

        # check joint properties
        elbowPrim = stage.GetPrimAtPath("/test_advanced/link_1/elbow_joint")
        self.assertNotEqual(elbowPrim.GetPath(), Sdf.Path.emptyPath)
        self.assertAlmostEqual(elbowPrim.GetAttribute("physxJoint:jointFriction").Get(), 0.1)
        self.assertAlmostEqual(elbowPrim.GetAttribute("drive:angular:physics:damping").Get(), 0.1)

        # check position of a link
        joint_pos = elbowPrim.GetAttribute("physics:localPos0").Get()
        self.assertTrue(Gf.IsClose(joint_pos, Gf.Vec3f(0, 0, 0.40), 1e-5))

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()
        pass

    # test for importing urdf where fixed joints are merged
    async def test_urdf_merge_joints(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_merge_joints.urdf")

        stage = omni.usd.get_context().get_stage()

        # enable merging fixed joints
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config.merge_fixed_joints = True
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)

        # the merged link shouldn't be there
        prim = stage.GetPrimAtPath("/test_merge_joints/link_2")
        self.assertEqual(prim.GetPath(), Sdf.Path.emptyPath)

        pass

    async def test_urdf_mtl(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_mtl.urdf")

        stage = omni.usd.get_context().get_stage()

        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)

        mesh = stage.GetPrimAtPath("/test_mtl/cube/visuals")
        self.assertTrue(UsdShade.MaterialBindingAPI(mesh) is not None)
        mat, rel = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()
        shader = UsdShade.Shader(stage.GetPrimAtPath(mat.GetPath().pathString + "/Shader"))
        print(shader)
        self.assertTrue(Gf.IsClose(shader.GetInput("diffuse_color_constant").Get(), Gf.Vec3f(0.8, 0.0, 0), 1e-5))

    async def test_urdf_mtl_stl(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_mtl_stl.urdf")

        stage = omni.usd.get_context().get_stage()

        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)

        mesh = stage.GetPrimAtPath("/test_mtl_stl/cube/visuals")
        self.assertTrue(UsdShade.MaterialBindingAPI(mesh) is not None)
        mat, rel = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()
        shader = UsdShade.Shader(stage.GetPrimAtPath(mat.GetPath().pathString + "/Shader"))
        print(shader)
        self.assertTrue(Gf.IsClose(shader.GetInput("diffuse_color_constant").Get(), Gf.Vec3f(0.8, 0.0, 0), 1e-5))

    async def test_urdf_carter(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/robots/carter/urdf/carter.urdf")
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config.merge_fixed_joints = False
        status, path = omni.kit.commands.execute(
            "URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config
        )
        self.assertTrue(path, "/carter")
        # TODO add checks here

    async def test_urdf_franka(self):

        urdf_path = os.path.abspath(
            self._extension_path + "/data/urdf/robots/franka_description/robots/panda_arm_hand.urdf"
        )
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)
        # TODO add checks here'

    async def test_urdf_ur10(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/robots/ur10/urdf/ur10.urdf")
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)
        # TODO add checks here'

    async def test_urdf_kaya(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/robots/kaya/urdf/kaya.urdf")
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config.merge_fixed_joints = False
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)
        # TODO add checks here

    async def test_missing(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_missing.urdf")

        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)

    # This sample corresponds to the example in the docs, keep this and the version in the docs in sync
    async def test_doc_sample(self):
        import omni.kit.commands
        from pxr import UsdLux, Sdf, Gf, UsdPhysics

        # setting up import configuration:
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = False

        # Get path to extension data:
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.urdf")
        extension_path = ext_manager.get_extension_path(ext_id)
        # import URDF
        omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=extension_path + "/data/urdf/robots/carter/urdf/carter.urdf",
            import_config=import_config,
        )
        # get stage handle
        stage = omni.usd.get_context().get_stage()

        # enable physics
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
        # set gravity
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

        # add ground plane
        PhysicsSchemaTools.addGroundPlane(stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, -50), Gf.Vec3f(0.5))

        # add lighting
        distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(500)
        ####
        #### Next Docs section
        ####

        # get handle to the Drive API for both wheels
        left_wheel_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/carter/chassis_link/left_wheel"), "angular")
        right_wheel_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/carter/chassis_link/right_wheel"), "angular")

        # Set the velocity drive target in degrees/second
        left_wheel_drive.GetTargetVelocityAttr().Set(150)
        right_wheel_drive.GetTargetVelocityAttr().Set(150)

        # Set the drive damping, which controls the strength of the velocity drive
        left_wheel_drive.GetDampingAttr().Set(15000)
        right_wheel_drive.GetDampingAttr().Set(15000)

        # Set the drive stiffness, which controls the strength of the position drive
        # In this case because we want to do velocity control this should be set to zero
        left_wheel_drive.GetStiffnessAttr().Set(0)
        right_wheel_drive.GetStiffnessAttr().Set(0)

    # Make sure that a urdf with more than 63 links imports
    async def test_64(self):
        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_large.urdf")
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath("/test_large")
        self.assertTrue(prim)

    # basic urdf test: joints and links are imported correctly
    async def test_urdf_floating(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_floating.urdf")
        stage = omni.usd.get_context().get_stage()
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")

        import_config.import_inertia_tensor = True
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)
        await omni.kit.app.get_app().next_update_async()

        prim = stage.GetPrimAtPath("/test_floating")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # make sure the joints exist
        root_joint = stage.GetPrimAtPath("/test_floating/root_joint")
        self.assertNotEqual(root_joint.GetPath(), Sdf.Path.emptyPath)

        link_1 = stage.GetPrimAtPath("/test_floating/link_1")
        self.assertNotEqual(link_1.GetPath(), Sdf.Path.emptyPath)
        link_1_trans = np.array(omni.usd.utils.get_world_transform_matrix(link_1).ExtractTranslation())

        self.assertAlmostEqual(np.linalg.norm(link_1_trans - np.array([0, 0, 0.45])), 0, delta=0.03)
        floating_link = stage.GetPrimAtPath("/test_floating/floating_link")
        self.assertNotEqual(floating_link.GetPath(), Sdf.Path.emptyPath)
        floating_link_trans = np.array(omni.usd.utils.get_world_transform_matrix(floating_link).ExtractTranslation())

        self.assertAlmostEqual(np.linalg.norm(floating_link_trans - np.array([0, 0, 1.450])), 0, delta=0.03)
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()
        pass

    async def test_urdf_scale(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_basic.urdf")
        stage = omni.usd.get_context().get_stage()
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")

        import_config.distance_scale = 1.0
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)
        await omni.kit.app.get_app().next_update_async()

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()

        self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.0)
        pass

    async def test_urdf_drive_none(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_basic.urdf")
        stage = omni.usd.get_context().get_stage()
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        from omni.isaac.urdf._urdf import UrdfJointTargetType

        import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_NONE
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)
        await omni.kit.app.get_app().next_update_async()

        self.assertFalse(stage.GetPrimAtPath("/test_basic/root_joint").HasAPI(UsdPhysics.DriveAPI))
        self.assertTrue(stage.GetPrimAtPath("/test_basic/link_1/elbow_joint").HasAPI(UsdPhysics.DriveAPI))

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()

        pass

    async def test_urdf_usd(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_usd.urdf")
        stage = omni.usd.get_context().get_stage()
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        from omni.isaac.urdf._urdf import UrdfJointTargetType

        import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_NONE
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)
        await omni.kit.app.get_app().next_update_async()

        self.assertNotEqual(stage.GetPrimAtPath("/test_usd/cube/visuals/mesh_0/Cylinder"), Sdf.Path.emptyPath)
        self.assertNotEqual(stage.GetPrimAtPath("/test_usd/cube/visuals/mesh_1/Torus"), Sdf.Path.emptyPath)
        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()

        pass

    # test negative joint limits
    async def test_urdf_limits(self):

        urdf_path = os.path.abspath(self._extension_path + "/data/urdf/tests/test_limits.urdf")
        stage = omni.usd.get_context().get_stage()
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")

        import_config.import_inertia_tensor = True
        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)
        await omni.kit.app.get_app().next_update_async()

        # ensure the import completed.
        prim = stage.GetPrimAtPath("/test_limits")
        self.assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath)

        # ensure the joint limits are set on the elbow
        elbowJoint = stage.GetPrimAtPath("/test_limits/link_1/elbow_joint")
        self.assertNotEqual(elbowJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(elbowJoint.GetTypeName(), "PhysicsRevoluteJoint")
        self.assertTrue(elbowJoint.HasAPI(UsdPhysics.DriveAPI))

        # ensure the joint limits are set on the wrist
        wristJoint = stage.GetPrimAtPath("/test_limits/link_2/wrist_joint")
        self.assertNotEqual(wristJoint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(wristJoint.GetTypeName(), "PhysicsRevoluteJoint")
        self.assertTrue(wristJoint.HasAPI(UsdPhysics.DriveAPI))

        # ensure the joint limits are set on the finger1
        finger1Joint = stage.GetPrimAtPath("/test_limits/palm_link/finger_1_joint")
        self.assertNotEqual(finger1Joint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(finger1Joint.GetTypeName(), "PhysicsPrismaticJoint")
        self.assertTrue(finger1Joint.HasAPI(UsdPhysics.DriveAPI))

        # ensure the joint limits are set on the finger2
        finger2Joint = stage.GetPrimAtPath("/test_limits/palm_link/finger_2_joint")
        self.assertNotEqual(finger2Joint.GetPath(), Sdf.Path.emptyPath)
        self.assertEqual(finger2Joint.GetTypeName(), "PhysicsPrismaticJoint")
        self.assertTrue(finger2Joint.HasAPI(UsdPhysics.DriveAPI))

        # Start Simulation and wait
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(1.0)
        # nothing crashes
        self._timeline.stop()

        pass
