import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.isaac.wheeled_robots.ogn.OgnHolonomicRobotUsdSetupDatabase import OgnHolonomicRobotUsdSetupDatabase
        test_file_name = "OgnHolonomicRobotUsdSetupTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_wheeled_robots_HolonomicRobotUsdSetup")
        database = OgnHolonomicRobotUsdSetupDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:comPrim"))
        attribute = test_node.get_attribute("inputs:comPrim")
        db_value = database.inputs.comPrim

        self.assertTrue(test_node.get_attribute_exists("inputs:comPrimPath"))
        attribute = test_node.get_attribute("inputs:comPrimPath")
        db_value = database.inputs.comPrimPath
        expected_value = ""
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:robotPrim"))
        attribute = test_node.get_attribute("inputs:robotPrim")
        db_value = database.inputs.robotPrim

        self.assertTrue(test_node.get_attribute_exists("inputs:robotPrimPath"))
        attribute = test_node.get_attribute("inputs:robotPrimPath")
        db_value = database.inputs.robotPrimPath
        expected_value = ""
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:usePath"))
        attribute = test_node.get_attribute("inputs:usePath")
        db_value = database.inputs.usePath
        expected_value = False
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:mecanumAngles"))
        attribute = test_node.get_attribute("outputs:mecanumAngles")
        db_value = database.outputs.mecanumAngles

        self.assertTrue(test_node.get_attribute_exists("outputs:upAxis"))
        attribute = test_node.get_attribute("outputs:upAxis")
        db_value = database.outputs.upAxis

        self.assertTrue(test_node.get_attribute_exists("outputs:wheelAxis"))
        attribute = test_node.get_attribute("outputs:wheelAxis")
        db_value = database.outputs.wheelAxis

        self.assertTrue(test_node.get_attribute_exists("outputs:wheelDofNames"))
        attribute = test_node.get_attribute("outputs:wheelDofNames")
        db_value = database.outputs.wheelDofNames

        self.assertTrue(test_node.get_attribute_exists("outputs:wheelOrientations"))
        attribute = test_node.get_attribute("outputs:wheelOrientations")
        db_value = database.outputs.wheelOrientations

        self.assertTrue(test_node.get_attribute_exists("outputs:wheelPositions"))
        attribute = test_node.get_attribute("outputs:wheelPositions")
        db_value = database.outputs.wheelPositions

        self.assertTrue(test_node.get_attribute_exists("outputs:wheelRadius"))
        attribute = test_node.get_attribute("outputs:wheelRadius")
        db_value = database.outputs.wheelRadius
