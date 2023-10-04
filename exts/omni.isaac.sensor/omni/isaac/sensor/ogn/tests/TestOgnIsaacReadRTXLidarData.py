import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.isaac.sensor.ogn.OgnIsaacReadRTXLidarDataDatabase import OgnIsaacReadRTXLidarDataDatabase
        test_file_name = "OgnIsaacReadRTXLidarDataTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_sensor_IsaacReadRTXLidarData")
        database = OgnIsaacReadRTXLidarDataDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:cpuPointer"))
        attribute = test_node.get_attribute("inputs:cpuPointer")
        db_value = database.inputs.cpuPointer
        expected_value = 0
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))
        attribute = test_node.get_attribute("inputs:execIn")
        db_value = database.inputs.execIn

        self.assertTrue(test_node.get_attribute_exists("inputs:keepOnlyPositiveDistance"))
        attribute = test_node.get_attribute("inputs:keepOnlyPositiveDistance")
        db_value = database.inputs.keepOnlyPositiveDistance
        expected_value = True
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:azimuth"))
        attribute = test_node.get_attribute("outputs:azimuth")
        db_value = database.outputs.azimuth

        self.assertTrue(test_node.get_attribute_exists("outputs:beamId"))
        attribute = test_node.get_attribute("outputs:beamId")
        db_value = database.outputs.beamId

        self.assertTrue(test_node.get_attribute_exists("outputs:distance"))
        attribute = test_node.get_attribute("outputs:distance")
        db_value = database.outputs.distance

        self.assertTrue(test_node.get_attribute_exists("outputs:echoId"))
        attribute = test_node.get_attribute("outputs:echoId")
        db_value = database.outputs.echoId

        self.assertTrue(test_node.get_attribute_exists("outputs:elevation"))
        attribute = test_node.get_attribute("outputs:elevation")
        db_value = database.outputs.elevation

        self.assertTrue(test_node.get_attribute_exists("outputs:emitterId"))
        attribute = test_node.get_attribute("outputs:emitterId")
        db_value = database.outputs.emitterId

        self.assertTrue(test_node.get_attribute_exists("outputs:execOut"))
        attribute = test_node.get_attribute("outputs:execOut")
        db_value = database.outputs.execOut

        self.assertTrue(test_node.get_attribute_exists("outputs:hitPointNormal"))
        attribute = test_node.get_attribute("outputs:hitPointNormal")
        db_value = database.outputs.hitPointNormal

        self.assertTrue(test_node.get_attribute_exists("outputs:intensity"))
        attribute = test_node.get_attribute("outputs:intensity")
        db_value = database.outputs.intensity

        self.assertTrue(test_node.get_attribute_exists("outputs:materialId"))
        attribute = test_node.get_attribute("outputs:materialId")
        db_value = database.outputs.materialId

        self.assertTrue(test_node.get_attribute_exists("outputs:objectId"))
        attribute = test_node.get_attribute("outputs:objectId")
        db_value = database.outputs.objectId

        self.assertTrue(test_node.get_attribute_exists("outputs:tick"))
        attribute = test_node.get_attribute("outputs:tick")
        db_value = database.outputs.tick

        self.assertTrue(test_node.get_attribute_exists("outputs:timeStampNs"))
        attribute = test_node.get_attribute("outputs:timeStampNs")
        db_value = database.outputs.timeStampNs

        self.assertTrue(test_node.get_attribute_exists("outputs:velocityMs"))
        attribute = test_node.get_attribute("outputs:velocityMs")
        db_value = database.outputs.velocityMs
