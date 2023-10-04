import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os


class TestOgn(ogts.OmniGraphTestCase):

    async def test_generated(self):
        test_data = [{'outputs': [['outputs:renderProductPath', '/Render/RenderProduct_Isaac', False]], 'setup': {'create_nodes': [['TestNode', 'omni.isaac.core_nodes.IsaacCreateRenderProduct']], 'expose_prims': [['bundle', '/OmniverseKit_Persp', 'PrimBundle']], 'connect': [['PrimBundle.inputs:prim', 'TestNode.inputs:cameraPrim']]}}]
        test_node = None
        test_graph = None
        for i, test_run in enumerate(test_data):
            inputs = test_run.get('inputs', [])
            outputs = test_run.get('outputs', [])
            state_set = test_run.get('state_set', [])
            state_get = test_run.get('state_get', [])
            setup = test_run.get('setup', None)
            if setup is None or setup:
                await omni.usd.get_context().new_stage_async()
                test_graph = None
            elif not setup:
                self.assertTrue(test_graph is not None and test_graph.is_valid(), "Test is misconfigured - empty setup cannot be in the first test")
            if setup:
                (test_graph, test_nodes, _, _) = og.Controller.edit("/TestGraph", setup)
                self.assertTrue(test_nodes)
                test_node = test_nodes[0]
            elif setup is None:
                if test_graph is None:
                    test_graph = og.Controller.create_graph("/TestGraph")
                    self.assertTrue(test_graph is not None and test_graph.is_valid())
                test_node = og.Controller.create_node(
                    ("TestNode_omni_isaac_core_nodes_IsaacCreateRenderProduct", test_graph), "omni.isaac.core_nodes.IsaacCreateRenderProduct"
                )
            self.assertTrue(test_graph is not None and test_graph.is_valid(), "Test graph invalid")
            self.assertTrue(test_node is not None and test_node.is_valid(), "Test node invalid")
            await og.Controller.evaluate(test_graph)
            values_to_set = inputs + state_set
            if values_to_set:
                for attribute_name, attribute_value, _ in inputs + state_set:
                    og.Controller((attribute_name, test_node)).set(attribute_value)
            await og.Controller.evaluate(test_graph)
            for attribute_name, expected_value, _ in outputs + state_get:
                attribute = og.Controller.attribute(attribute_name, test_node)
                actual_output = og.Controller.get(attribute)
                expected_type = None
                if isinstance(expected_value, dict):
                    expected_type = expected_value["type"]
                    expected_value = expected_value["value"]
                ogts.verify_values(expected_value, actual_output, f"omni.isaac.core_nodes.IsaacCreateRenderProduct User test case #{i+1}: {attribute_name} attribute value error")
                if expected_type:
                    tp = og.AttributeType.type_from_ogn_type_name(expected_type)
                    actual_type = attribute.get_resolved_type()
                    if tp != actual_type:
                        raise ValueError(f"omni.isaac.core_nodes.IsaacCreateRenderProduct User tests - {attribute_name}: Expected {expected_type}, saw {actual_type.get_ogn_type_name()}")

    async def test_data_access(self):
        from omni.isaac.core_nodes.ogn.OgnIsaacCreateRenderProductDatabase import OgnIsaacCreateRenderProductDatabase
        test_file_name = "OgnIsaacCreateRenderProductTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_omni_isaac_core_nodes_IsaacCreateRenderProduct")
        database = OgnIsaacCreateRenderProductDatabase(test_node)
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 2)

        def _attr_error(attribute: og.Attribute, usd_test: bool) -> str:
            test_type = "USD Load" if usd_test else "Database Access"
            return f"{node_type_name} {test_type} Test - {attribute.get_name()} value error"


        self.assertTrue(test_node.get_attribute_exists("inputs:cameraPrim"))
        attribute = test_node.get_attribute("inputs:cameraPrim")
        db_value = database.inputs.cameraPrim

        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))
        attribute = test_node.get_attribute("inputs:execIn")
        db_value = database.inputs.execIn

        self.assertTrue(test_node.get_attribute_exists("inputs:height"))
        attribute = test_node.get_attribute("inputs:height")
        db_value = database.inputs.height
        expected_value = 720
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("inputs:width"))
        attribute = test_node.get_attribute("inputs:width")
        db_value = database.inputs.width
        expected_value = 1280
        actual_value = og.Controller.get(attribute)
        ogts.verify_values(expected_value, actual_value, _attr_error(attribute, True))
        ogts.verify_values(expected_value, db_value, _attr_error(attribute, False))

        self.assertTrue(test_node.get_attribute_exists("outputs:execOut"))
        attribute = test_node.get_attribute("outputs:execOut")
        db_value = database.outputs.execOut

        self.assertTrue(test_node.get_attribute_exists("outputs:renderProductPath"))
        attribute = test_node.get_attribute("outputs:renderProductPath")
        db_value = database.outputs.renderProductPath
