import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.isaac.core_nodes.ogn.OgnIsaacComputeOdometryDatabase import OgnIsaacComputeOdometryDatabase
        test_file_name = "OgnIsaacComputeOdometryTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_core_nodes_IsaacComputeOdometry")
        database = OgnIsaacComputeOdometryDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:chassisPrim"))
        attribute = test_node.get_attribute("inputs:chassisPrim")
        db_value = database.inputs.chassisPrim

        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))
        attribute = test_node.get_attribute("inputs:execIn")
        db_value = database.inputs.execIn

        self.assertTrue(test_node.get_attribute_exists("outputs:angularAcceleration"))
        attribute = test_node.get_attribute("outputs:angularAcceleration")
        db_value = database.outputs.angularAcceleration

        self.assertTrue(test_node.get_attribute_exists("outputs:angularVelocity"))
        attribute = test_node.get_attribute("outputs:angularVelocity")
        db_value = database.outputs.angularVelocity

        self.assertTrue(test_node.get_attribute_exists("outputs:execOut"))
        attribute = test_node.get_attribute("outputs:execOut")
        db_value = database.outputs.execOut

        self.assertTrue(test_node.get_attribute_exists("outputs:linearAcceleration"))
        attribute = test_node.get_attribute("outputs:linearAcceleration")
        db_value = database.outputs.linearAcceleration

        self.assertTrue(test_node.get_attribute_exists("outputs:linearVelocity"))
        attribute = test_node.get_attribute("outputs:linearVelocity")
        db_value = database.outputs.linearVelocity

        self.assertTrue(test_node.get_attribute_exists("outputs:orientation"))
        attribute = test_node.get_attribute("outputs:orientation")
        db_value = database.outputs.orientation

        self.assertTrue(test_node.get_attribute_exists("outputs:position"))
        attribute = test_node.get_attribute("outputs:position")
        db_value = database.outputs.position
