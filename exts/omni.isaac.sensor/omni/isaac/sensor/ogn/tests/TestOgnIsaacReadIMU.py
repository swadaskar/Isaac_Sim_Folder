import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.isaac.sensor.ogn.OgnIsaacReadIMUDatabase import OgnIsaacReadIMUDatabase
        test_file_name = "OgnIsaacReadIMUTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_sensor_IsaacReadIMU")
        database = OgnIsaacReadIMUDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))
        attribute = test_node.get_attribute("inputs:execIn")
        db_value = database.inputs.execIn

        self.assertTrue(test_node.get_attribute_exists("inputs:imuPrim"))
        attribute = test_node.get_attribute("inputs:imuPrim")
        db_value = database.inputs.imuPrim

        self.assertTrue(test_node.get_attribute_exists("outputs:angVel"))
        attribute = test_node.get_attribute("outputs:angVel")
        db_value = database.outputs.angVel

        self.assertTrue(test_node.get_attribute_exists("outputs:execOut"))
        attribute = test_node.get_attribute("outputs:execOut")
        db_value = database.outputs.execOut

        self.assertTrue(test_node.get_attribute_exists("outputs:linAcc"))
        attribute = test_node.get_attribute("outputs:linAcc")
        db_value = database.outputs.linAcc

        self.assertTrue(test_node.get_attribute_exists("outputs:orientation"))
        attribute = test_node.get_attribute("outputs:orientation")
        db_value = database.outputs.orientation
