# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni.kit.test
import numpy as np
import carb

from omni.isaac.core.utils.stage import update_stage_async, open_stage_async
import asyncio
from pxr import UsdGeom, UsdPhysics, Gf, PhysxSchema


class TestJointEffort(omni.kit.test.AsyncTestCase):

    # Before running each test
    async def setUp(self):
        self._timeline = omni.timeline.get_timeline_interface()

        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(60))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(60))
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        pass

    async def tearDown(self):
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        pass

    async def test_max_joint_effort(self):
        stage = omni.usd.get_context().get_stage()

        # no gravity (no external force except for joint drives)
        scene = UsdPhysics.Scene.Define(stage, "/physics")
        scene.CreateGravityMagnitudeAttr().Set(0)

        # add two rigid body cube with phyiscs, mass, but no collision
        cube1Geom = UsdGeom.Cube.Define(stage, "/World/Cube1")
        cube1Prim = stage.GetPrimAtPath("/World/Cube1")
        cube1Geom.CreateSizeAttr(1.0)
        rigid_api_1 = UsdPhysics.RigidBodyAPI.Apply(cube1Prim)
        rigid_api_1.CreateRigidBodyEnabledAttr(True)
        mass_api_1 = UsdPhysics.MassAPI.Apply(cube1Prim)
        mass_api_1.CreateMassAttr(1.0)
        await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors

        cube2Geom = UsdGeom.Cube.Define(stage, "/World/Cube2")
        cube2Prim = stage.GetPrimAtPath("/World/Cube2")
        cube2Geom.CreateSizeAttr(1.0)
        cube2Geom.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 5.00))  # move it up 1 meter
        rigid_api_2 = UsdPhysics.RigidBodyAPI.Apply(cube2Prim)
        rigid_api_2.CreateRigidBodyEnabledAttr(True)
        mass_api_2 = UsdPhysics.MassAPI.Apply(cube2Prim)
        mass_api_2.CreateMassAttr(1.0)
        await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors

        # fix joint on Cube 1
        joint_0 = UsdPhysics.FixedJoint.Define(stage, "/World/Cube1/joint_0")
        joint_0.GetBody0Rel().SetTargets(["/World"])
        joint_0.GetBody1Rel().SetTargets(["/World/Cube1"])

        # prismatic joint between two cubes
        joint = UsdPhysics.PrismaticJoint.Define(stage, "/World/Cube2/joint_1")
        joint.CreateAxisAttr("Z")
        joint.GetBody0Rel().SetTargets(["/World/Cube1"])
        joint.GetBody1Rel().SetTargets(["/World/Cube2"])
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 5.0))
        PhysxSchema.JointStateAPI.Apply(joint.GetPrim(), "linear")

        # add drive and set stiffness, damping
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.GetStiffnessAttr().Set(1e8)
        drive.GetDampingAttr().Set(1)

        ## test 1:
        # mode=force; max_force = 1N (small); expected behavior: cube2 drops with terminal acceleration/force = 1N
        drive.GetTypeAttr().Set("force")
        drive.GetMaxForceAttr().Set(1.0)
        drive.GetTargetPositionAttr().Set(-10)
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        # run a fixed number of steps, check acceleration
        for frame in range(30):
            await omni.kit.app.get_app().next_update_async()
        position_1 = np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2]
        for frame in range(30):
            await omni.kit.app.get_app().next_update_async()
        position_2 = np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2]
        joint_force_1 = np.abs(position_2 - position_1)
        self.assertAlmostEqual(joint_force_1, 1, delta=0.1)
        self._timeline.stop()
        await update_stage_async()  # reset

        ## test 2:
        # mode=force; max_force = 3N; expected behavior: cube2 drops 3x faster
        drive.GetMaxForceAttr().Set(3.0)
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        # run a fixed number of steps, check acceleration
        for frame in range(30):
            await omni.kit.app.get_app().next_update_async()
        position_1 = np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2]
        for frame in range(30):
            await omni.kit.app.get_app().next_update_async()
        position_2 = np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2]
        joint_force_2 = np.abs(position_2 - position_1)
        self.assertAlmostEqual(joint_force_2, joint_force_1 * 3, delta=0.1)
        self._timeline.stop()
        await update_stage_async()  # reset

        ## test 3
        ## mode = acceleration; expected behavior: same as test 1
        drive.GetTypeAttr().Set("acceleration")
        drive.GetMaxForceAttr().Set(1.0)  # set max force small
        drive.GetTargetPositionAttr().Set(-10)
        await omni.kit.app.get_app().next_update_async()

        from omni.isaac.core_nodes.bindings import _omni_isaac_core_nodes

        self._core_nodes_interface = _omni_isaac_core_nodes.acquire_interface()

        self._timeline.play()
        # run a fixed number of steps, check acceleration
        for frame in range(30):
            await omni.kit.app.get_app().next_update_async()
            # print(
            #     [
            #         np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2],
            #         self._core_nodes_interface.get_sim_time(),
            #     ]
            # )
        position_1 = np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2]
        for frame in range(30):
            await omni.kit.app.get_app().next_update_async()
            # print(
            #     [
            #         np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2],
            #         self._core_nodes_interface.get_sim_time(),
            #     ]
            # )
        position_2 = np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2]
        joint_force_3 = np.abs(position_2 - position_1)
        self.assertAlmostEqual(joint_force_3, joint_force_1, delta=0.1)
        self._timeline.stop()
        await update_stage_async()  # reset

        # # test 4: turn it into an articulation chain
        # articulation_root = UsdPhysics.ArticulationRootAPI.Apply(cube1Prim)
        # PhysxSchema.PhysxArticulationAPI.Get(stage, "/World/Cube1").CreateSolverPositionIterationCountAttr(128)
        # PhysxSchema.PhysxArticulationAPI.Get(stage, "/World/Cube1").CreateSolverVelocityIterationCountAttr(128)
        # await omni.kit.app.get_app().next_update_async()

        # self._timeline.play()
        # # run a fixed number of steps, check acceleration
        # for frame in range(30):
        #     await omni.kit.app.get_app().next_update_async()
        #     print([np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2], self._core_nodes_interface.get_sim_time()])
        # position_1 = np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2]
        # for frame in range(30):
        #     await omni.kit.app.get_app().next_update_async()
        #     print([np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2], self._core_nodes_interface.get_sim_time()])
        # position_2 = np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2]
        # joint_force_4 = np.abs(position_2 - position_1)
        # self.assertAlmostEqual(joint_force_4, joint_force_3, delta=0.1)
        # self._timeline.stop()
        # await update_stage_async()  # reset

    async def test_drive_mode_frame_rate(self):
        stage = omni.usd.get_context().get_stage()

        # no gravity (no external force except for joint drives)
        scene = UsdPhysics.Scene.Define(stage, "/physics")
        scene.GetGravityMagnitudeAttr().Set(0)

        # add two rigid body cube with phyiscs, mass, but no collision
        cube1Geom = UsdGeom.Cube.Define(stage, "/World/Cube1")
        cube1Prim = stage.GetPrimAtPath("/World/Cube1")
        cube1Geom.CreateSizeAttr(1.0)
        rigid_api_1 = UsdPhysics.RigidBodyAPI.Apply(cube1Prim)
        rigid_api_1.CreateRigidBodyEnabledAttr(True)
        mass_api_1 = UsdPhysics.MassAPI.Apply(cube1Prim)
        mass_api_1.CreateMassAttr(1.0)
        await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors

        cube2Geom = UsdGeom.Cube.Define(stage, "/World/Cube2")
        cube2Prim = stage.GetPrimAtPath("/World/Cube2")
        cube2Geom.CreateSizeAttr(1.0)
        cube2Geom.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 5.00))  # move it up 1 meter
        rigid_api_2 = UsdPhysics.RigidBodyAPI.Apply(cube2Prim)
        rigid_api_2.CreateRigidBodyEnabledAttr(True)
        mass_api_2 = UsdPhysics.MassAPI.Apply(cube2Prim)
        mass_api_2.CreateMassAttr(1.0)
        await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors

        # fix joint on Cube 1
        joint_0 = UsdPhysics.FixedJoint.Define(stage, "/World/Cube1/joint_0")
        joint_0.GetBody0Rel().SetTargets(["/World"])
        joint_0.GetBody1Rel().SetTargets(["/World/Cube1"])

        # prismatic joint between two cubes
        joint = UsdPhysics.PrismaticJoint.Define(stage, "/World/Cube2/joint_1")
        joint.CreateAxisAttr("Z")
        joint.GetBody0Rel().SetTargets(["/World/Cube1"])
        joint.GetBody1Rel().SetTargets(["/World/Cube2"])
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 5.0))
        PhysxSchema.JointStateAPI.Apply(joint.GetPrim(), "linear")

        # add drive and set stiffness, damping
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.GetStiffnessAttr().Set(1e8)
        drive.GetDampingAttr().Set(1)

        ## test 1:
        # mode=force; physics step rate = 60
        scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        scene_api.GetTimeStepsPerSecondAttr().Set(60)
        drive.GetTypeAttr().Set("force")
        drive.GetMaxForceAttr().Set(1.0)
        drive.GetTargetPositionAttr().Set(-10)
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        # run a fixed number of steps, check acceleration
        for frame in range(30):
            await omni.kit.app.get_app().next_update_async()
        position_1 = np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2]
        for frame in range(30):
            await omni.kit.app.get_app().next_update_async()
        position_2 = np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2]
        joint_force_1 = np.abs(position_2 - position_1)
        self.assertAlmostEqual(joint_force_1, 1, delta=0.1)
        self._timeline.stop()
        await update_stage_async()  # reset

        ## test 2
        ## mode = acceleration; physics step rate = 20, expected behavior: same as test 1
        scene_api.GetTimeStepsPerSecondAttr().Set(20)
        drive.GetTypeAttr().Set("acceleration")
        drive.GetMaxForceAttr().Set(1.0)  # set max force small
        drive.GetTargetPositionAttr().Set(-10)
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        # run a fixed number of steps, check acceleration
        for frame in range(30):
            await omni.kit.app.get_app().next_update_async()
        position_1 = np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2]
        for frame in range(30):
            await omni.kit.app.get_app().next_update_async()
        position_2 = np.array(omni.usd.utils.get_world_transform_matrix(cube2Prim).ExtractTranslation())[2]
        joint_force_2 = np.abs(position_2 - position_1)
        self.assertAlmostEqual(joint_force_2, joint_force_1, delta=0.1)
        self._timeline.stop()
