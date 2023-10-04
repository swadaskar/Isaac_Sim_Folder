import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_data_access(self):
        from omni.isaac.sensor.ogn.OgnIsaacComputeRTXRadarPointCloudDatabase import OgnIsaacComputeRTXRadarPointCloudDatabase
        test_file_name = "OgnIsaacComputeRTXRadarPointCloudTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_sensor_IsaacComputeRTXRadarPointCloud")
        database = OgnIsaacComputeRTXRadarPointCloudDatabase(test_node)
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

        self.assertTrue(test_node.get_attribute_exists("inputs:transform"))
        attribute = test_node.get_attribute("inputs:transform")
        db_value = database.inputs.transform
        expected_value = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:azimuth"))
        attribute = test_node.get_attribute("outputs:azimuth")
        db_value = database.outputs.azimuth

        self.assertTrue(test_node.get_attribute_exists("outputs:cycleCnt"))
        attribute = test_node.get_attribute("outputs:cycleCnt")
        db_value = database.outputs.cycleCnt

        self.assertTrue(test_node.get_attribute_exists("outputs:elevation"))
        attribute = test_node.get_attribute("outputs:elevation")
        db_value = database.outputs.elevation

        self.assertTrue(test_node.get_attribute_exists("outputs:execOut"))
        attribute = test_node.get_attribute("outputs:execOut")
        db_value = database.outputs.execOut

        self.assertTrue(test_node.get_attribute_exists("outputs:materialId"))
        attribute = test_node.get_attribute("outputs:materialId")
        db_value = database.outputs.materialId

        self.assertTrue(test_node.get_attribute_exists("outputs:maxAzRad"))
        attribute = test_node.get_attribute("outputs:maxAzRad")
        db_value = database.outputs.maxAzRad

        self.assertTrue(test_node.get_attribute_exists("outputs:maxElRad"))
        attribute = test_node.get_attribute("outputs:maxElRad")
        db_value = database.outputs.maxElRad

        self.assertTrue(test_node.get_attribute_exists("outputs:maxRangeM"))
        attribute = test_node.get_attribute("outputs:maxRangeM")
        db_value = database.outputs.maxRangeM

        self.assertTrue(test_node.get_attribute_exists("outputs:maxVelMps"))
        attribute = test_node.get_attribute("outputs:maxVelMps")
        db_value = database.outputs.maxVelMps

        self.assertTrue(test_node.get_attribute_exists("outputs:minAzRad"))
        attribute = test_node.get_attribute("outputs:minAzRad")
        db_value = database.outputs.minAzRad

        self.assertTrue(test_node.get_attribute_exists("outputs:minElRad"))
        attribute = test_node.get_attribute("outputs:minElRad")
        db_value = database.outputs.minElRad

        self.assertTrue(test_node.get_attribute_exists("outputs:minVelMps"))
        attribute = test_node.get_attribute("outputs:minVelMps")
        db_value = database.outputs.minVelMps

        self.assertTrue(test_node.get_attribute_exists("outputs:numDetections"))
        attribute = test_node.get_attribute("outputs:numDetections")
        db_value = database.outputs.numDetections

        self.assertTrue(test_node.get_attribute_exists("outputs:objectId"))
        attribute = test_node.get_attribute("outputs:objectId")
        db_value = database.outputs.objectId

        self.assertTrue(test_node.get_attribute_exists("outputs:pointCloudData"))
        attribute = test_node.get_attribute("outputs:pointCloudData")
        db_value = database.outputs.pointCloudData

        self.assertTrue(test_node.get_attribute_exists("outputs:radialDistance"))
        attribute = test_node.get_attribute("outputs:radialDistance")
        db_value = database.outputs.radialDistance

        self.assertTrue(test_node.get_attribute_exists("outputs:radialVelocity"))
        attribute = test_node.get_attribute("outputs:radialVelocity")
        db_value = database.outputs.radialVelocity

        self.assertTrue(test_node.get_attribute_exists("outputs:rcs"))
        attribute = test_node.get_attribute("outputs:rcs")
        db_value = database.outputs.rcs

        self.assertTrue(test_node.get_attribute_exists("outputs:scanIdx"))
        attribute = test_node.get_attribute("outputs:scanIdx")
        db_value = database.outputs.scanIdx

        self.assertTrue(test_node.get_attribute_exists("outputs:semanticId"))
        attribute = test_node.get_attribute("outputs:semanticId")
        db_value = database.outputs.semanticId

        self.assertTrue(test_node.get_attribute_exists("outputs:sensorID"))
        attribute = test_node.get_attribute("outputs:sensorID")
        db_value = database.outputs.sensorID

        self.assertTrue(test_node.get_attribute_exists("outputs:syncData"))
        attribute = test_node.get_attribute("outputs:syncData")
        db_value = database.outputs.syncData

        self.assertTrue(test_node.get_attribute_exists("outputs:timeStampNS"))
        attribute = test_node.get_attribute("outputs:timeStampNS")
        db_value = database.outputs.timeStampNS

        self.assertTrue(test_node.get_attribute_exists("outputs:transform"))
        attribute = test_node.get_attribute("outputs:transform")
        db_value = database.outputs.transform
