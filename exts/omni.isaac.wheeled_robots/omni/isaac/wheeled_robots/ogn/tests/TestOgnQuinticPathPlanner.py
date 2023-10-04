import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.isaac.wheeled_robots.ogn.OgnQuinticPathPlannerDatabase import OgnQuinticPathPlannerDatabase
        test_file_name = "OgnQuinticPathPlannerTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_wheeled_robots_QuinticPathPlanner")
        database = OgnQuinticPathPlannerDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:currentOrientation"))
        attribute = test_node.get_attribute("inputs:currentOrientation")
        db_value = database.inputs.currentOrientation
        expected_value = [0.0, 0.0, 0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:currentPosition"))
        attribute = test_node.get_attribute("inputs:currentPosition")
        db_value = database.inputs.currentPosition
        expected_value = [0.0, 0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))
        attribute = test_node.get_attribute("inputs:execIn")
        db_value = database.inputs.execIn

        self.assertTrue(test_node.get_attribute_exists("inputs:goalAccel"))
        attribute = test_node.get_attribute("inputs:goalAccel")
        db_value = database.inputs.goalAccel
        expected_value = 0.02
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:goalVelocity"))
        attribute = test_node.get_attribute("inputs:goalVelocity")
        db_value = database.inputs.goalVelocity
        expected_value = 0.5
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:initialAccel"))
        attribute = test_node.get_attribute("inputs:initialAccel")
        db_value = database.inputs.initialAccel
        expected_value = 0.02
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:initialVelocity"))
        attribute = test_node.get_attribute("inputs:initialVelocity")
        db_value = database.inputs.initialVelocity
        expected_value = 0.5
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:maxAccel"))
        attribute = test_node.get_attribute("inputs:maxAccel")
        db_value = database.inputs.maxAccel
        expected_value = 1.5
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:maxJerk"))
        attribute = test_node.get_attribute("inputs:maxJerk")
        db_value = database.inputs.maxJerk
        expected_value = 0.3
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:step"))
        attribute = test_node.get_attribute("inputs:step")
        db_value = database.inputs.step
        expected_value = 0.16666666667
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:targetOrientation"))
        attribute = test_node.get_attribute("inputs:targetOrientation")
        db_value = database.inputs.targetOrientation
        expected_value = [0.0, 0.0, 0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:targetPosition"))
        attribute = test_node.get_attribute("inputs:targetPosition")
        db_value = database.inputs.targetPosition
        expected_value = [0.0, 0.0, 0.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:execOut"))
        attribute = test_node.get_attribute("outputs:execOut")
        db_value = database.outputs.execOut

        self.assertTrue(test_node.get_attribute_exists("outputs:pathArrays"))
        attribute = test_node.get_attribute("outputs:pathArrays")
        db_value = database.outputs.pathArrays

        self.assertTrue(test_node.get_attribute_exists("outputs:target"))
        attribute = test_node.get_attribute("outputs:target")
        db_value = database.outputs.target

        self.assertTrue(test_node.get_attribute_exists("outputs:targetChanged"))
        attribute = test_node.get_attribute("outputs:targetChanged")
        db_value = database.outputs.targetChanged
