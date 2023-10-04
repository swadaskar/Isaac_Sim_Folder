import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.isaac.ros_bridge.ogn.OgnROS1MasterDatabase import OgnROS1MasterDatabase
        test_file_name = "OgnROS1MasterTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_ros_bridge_ROS1Master")
        database = OgnROS1MasterDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))
        attribute = test_node.get_attribute("inputs:execIn")
        db_value = database.inputs.execIn

        self.assertTrue(test_node.get_attribute_exists("outputs:host"))
        attribute = test_node.get_attribute("outputs:host")
        db_value = database.outputs.host

        self.assertTrue(test_node.get_attribute_exists("outputs:port"))
        attribute = test_node.get_attribute("outputs:port")
        db_value = database.outputs.port

        self.assertTrue(test_node.get_attribute_exists("outputs:status"))
        attribute = test_node.get_attribute("outputs:status")
        db_value = database.outputs.status

        self.assertTrue(test_node.get_attribute_exists("outputs:uri"))
        attribute = test_node.get_attribute("outputs:uri")
        db_value = database.outputs.uri
