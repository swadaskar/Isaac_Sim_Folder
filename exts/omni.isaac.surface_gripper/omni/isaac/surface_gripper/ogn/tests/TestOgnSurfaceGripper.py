import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.isaac.surface_gripper.ogn.OgnSurfaceGripperDatabase import OgnSurfaceGripperDatabase
        test_file_name = "OgnSurfaceGripperTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_surface_gripper_SurfaceGripper")
        database = OgnSurfaceGripperDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:BendAngle"))
        attribute = test_node.get_attribute("inputs:BendAngle")
        db_value = database.inputs.BendAngle
        expected_value = 7.5
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:Close"))
        attribute = test_node.get_attribute("inputs:Close")
        db_value = database.inputs.Close

        self.assertTrue(test_node.get_attribute_exists("inputs:Damping"))
        attribute = test_node.get_attribute("inputs:Damping")
        db_value = database.inputs.Damping
        expected_value = 1000.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:Delta"))
        attribute = test_node.get_attribute("inputs:Delta")
        db_value = database.inputs.Delta
        expected_value = 0.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:DisableGravity"))
        attribute = test_node.get_attribute("inputs:DisableGravity")
        db_value = database.inputs.DisableGravity
        expected_value = True
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:ForceLimit"))
        attribute = test_node.get_attribute("inputs:ForceLimit")
        db_value = database.inputs.ForceLimit
        expected_value = 1000000.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:GripPosition"))
        attribute = test_node.get_attribute("inputs:GripPosition")
        db_value = database.inputs.GripPosition

        self.assertTrue(test_node.get_attribute_exists("inputs:GripThreshold"))
        attribute = test_node.get_attribute("inputs:GripThreshold")
        db_value = database.inputs.GripThreshold
        expected_value = 0.01
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:Open"))
        attribute = test_node.get_attribute("inputs:Open")
        db_value = database.inputs.Open

        self.assertTrue(test_node.get_attribute_exists("inputs:ParentRigidBody"))
        attribute = test_node.get_attribute("inputs:ParentRigidBody")
        db_value = database.inputs.ParentRigidBody

        self.assertTrue(test_node.get_attribute_exists("inputs:RetryClose"))
        attribute = test_node.get_attribute("inputs:RetryClose")
        db_value = database.inputs.RetryClose
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:Stiffness"))
        attribute = test_node.get_attribute("inputs:Stiffness")
        db_value = database.inputs.Stiffness
        expected_value = 10000.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:TorqueLimit"))
        attribute = test_node.get_attribute("inputs:TorqueLimit")
        db_value = database.inputs.TorqueLimit
        expected_value = 1000000.0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:enabled"))
        attribute = test_node.get_attribute("inputs:enabled")
        db_value = database.inputs.enabled
        expected_value = True
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:onStep"))
        attribute = test_node.get_attribute("inputs:onStep")
        db_value = database.inputs.onStep

        self.assertTrue(test_node.get_attribute_exists("outputs:Closed"))
        attribute = test_node.get_attribute("outputs:Closed")
        db_value = database.outputs.Closed

        self.assertTrue(test_node.get_attribute_exists("outputs:GripBroken"))
        attribute = test_node.get_attribute("outputs:GripBroken")
        db_value = database.outputs.GripBroken

        self.assertTrue(test_node.get_attribute_exists("state:Close"))
        attribute = test_node.get_attribute("state:Close")
        db_value = database.state.Close

        self.assertTrue(test_node.get_attribute_exists("state:Open"))
        attribute = test_node.get_attribute("state:Open")
        db_value = database.state.Open
