# Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


from re import I
import omni.kit.test
import carb

import omni.graph.core.tests as ogts
import omni.graph.core as og
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.robots import Robot
from pxr import UsdGeom, Gf, UsdPhysics, Usd


async def add_cube(stage, path, size, offset, physics=True, mass=0.0) -> Usd.Prim:
    cube_geom = UsdGeom.Cube.Define(stage, path)
    cube_prim = stage.GetPrimAtPath(path)
    cube_geom.CreateSizeAttr(size)
    cube_geom.AddTranslateOp().Set(offset)
    await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
    if physics:
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(cube_prim)
        await omni.kit.app.get_app().next_update_async()
        rigid_api.CreateRigidBodyEnabledAttr(True)
        await omni.kit.app.get_app().next_update_async()
        if mass > 0:
            mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
            await omni.kit.app.get_app().next_update_async()
            mass_api.CreateMassAttr(mass)
            await omni.kit.app.get_app().next_update_async()
    UsdPhysics.CollisionAPI.Apply(cube_prim)
    await omni.kit.app.get_app().next_update_async()
    return cube_prim


# class TestComputeOdometry(ogts.OmniGraphTestCase):
#     async def setUp(self):
#         """Set up  test environment, to be torn down when done"""
#         await ogts.setup_test_environment()
#         await omni.kit.stage_templates.new_stage_async()
#         self._stage = omni.usd.get_context().get_stage()
#         self._timeline = omni.timeline.get_timeline_interface()

#     # ----------------------------------------------------------------------
#     async def tearDown(self):
#         """Get rid of temporary data used by the test"""
#         await omni.kit.stage_templates.new_stage_async()

#     # ----------------------------------------------------------------------
#     async def test_odometry(self):
#         await add_cube(self._stage, "/Cube", 1, (0, 0, 0), physics=True, mass=1)

#         (test_graph, new_nodes, _, _) = og.Controller.edit(
#             {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
#             {
#                 og.Controller.Keys.CREATE_NODES: [
#                     ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
#                     ("Joint1Name", "omni.graph.nodes.ConstantToken"),
#                     ("Joint2Name", "omni.graph.nodes.ConstantToken"),
#                     ("JointNameArray", "omni.graph.nodes.MakeArray"),
#                     ("Joint1Position", "omni.graph.nodes.ConstantDouble"),
#                     ("Joint2Position", "omni.graph.nodes.ConstantDouble"),
#                     ("JointCommandArray", "omni.graph.nodes.MakeArray"),
#                     ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
#                 ],
#                 og.Controller.Keys.SET_VALUES: [
#                     ("Joint1Name.inputs:value", "panda_joint2"),
#                     ("Joint2Name.inputs:value", "panda_joint3"),
#                     ("Joint1Position.inputs:value", -1.0),
#                     ("Joint2Position.inputs:value", 1.2),
#                     ("JointNameArray.inputs:arraySize", 2),
#                     ("JointCommandArray.inputs:arraySize", 2),
#                     ("ArticulationController.inputs:robotPath", "/panda"),
#                 ],
#                 og.Controller.Keys.CONNECT: [
#                     ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
#                     ("Joint1Name.inputs:value", "JointNameArray.inputs:a"),
#                     ("Joint2Name.inputs:value", "JointNameArray.inputs:b"),
#                     ("JointNameArray.outputs:array", "ArticulationController.inputs:jointNames"),
#                     ("Joint1Position.inputs:value", "JointCommandArray.inputs:a"),
#                     ("Joint2Position.inputs:value", "JointCommandArray.inputs:b"),
#                     ("JointCommandArray.outputs:array", "ArticulationController.inputs:positionCommand"),
#                 ],
#             },
#         )

#         await og.Controller.evaluate(test_graph)

#         # check where the joints are after evaluate
#         robot = Robot(prim_path="/panda", name="franka")
#         self._timeline.play()
#         await simulate_async(2)
#         robot.initialize()

#         self.assertAlmostEqual(robot.get_joint_positions()[1], -1.0, delta=0.001)
#         self.assertAlmostEqual(robot.get_joint_positions()[2], 1.2, delta=0.001)
