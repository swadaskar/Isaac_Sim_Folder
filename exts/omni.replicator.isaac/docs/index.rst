Replicator Isaac Extension [omni.replicator.isaac]
######################################################

This extension builds upon omni.replicator to provide additional functionalities 
relevant to robotics.


Reinforcement Learning Domain Randomization 
============================================

The following methods provide randomization functionalities of various parameters 
pertaining to omni.isaac.core.prims.RigidPrimView, omni.isaac.core.articulations.ArticulationView,
and omni.isaac.core.SimulationContext. These methods are designed to perform randomizations in 
simulations where *update to USD* is disabled for faster simulation speed. These methods directly
set randomized values to Physx as opposed to existing methods in omni.replicator.core, 
such as omni.replicator.core.modify.pose or omni.replicator.core.physics.physics_material, which
utilizes USD APIs to set values and hence cannot be used when *update to USD* is disabled. 
Therefore, the following methods provided in omni.replicator.isaac are particularly useful 
for domain randomization in reinforcement learning (RL) as RL usecases benefit immensely from 
disabling *update to USD* to achieve faster simulation speed.

The following is a simple demo that showcases the workflow and the various features of
``omni.replicator.isaac`` for domain randomization. This demo script can be launched as
a standalone example, which can be found by going to Isaac Sim's root directory and
go to ``standalone_examples/api/omni.replicator.isaac/randomization_demo.py``.

The ``omni.replicator.isaac`` extension for domain randomization functions by constructing
an OmniGraph action graph, which consists of nodes that generate random values, regulate
frequency intervals of various randomizaiton properties, and write the random values to Physx.
This action graph gets executed according to the way in which the triggers are set up. Note 
that it is necessesary to register the views to be randomized before constructing this action graph.

The first step is to create an entry point of the action graph using ``with dr.trigger.on_rl_frame(num_envs=num_envs):``.
It is worth noting that all views to be used with this extension must have the same number of 
encapsulated prims, equaling ``num_envs`` that is passed as an argument to the ``on_rl_frame`` trigger.

After creating this entry point, there are two types of gates that determine when the nodes can write
to Physx: ``on_interval(interval)`` and ``on_env_reset()``; these gates work in conjunction with 
``omni.replicator.isaac.physics_view.step_randomization(reset_inds)``. There exists an internal
step counter for every environment in the views. Everytime the ``step_randomization`` method is called, 
it resets the counter to zero for every environment listed in the ``reset_inds`` argument while
incrementing the counter for every other environment. The ``on_interval(interval)`` gate then ensures 
the nodes to write to Physx whenever the counter for an environment is a multiple of the ``interval``
argument. The ``on_env_reset()`` gate writes to Physx whenever an environment's counter gets reset by
the ``reset_inds`` argument passed in the ``step_randomization`` method. 

Within each gate, the following methods can be used to randomize view properties: ``randomize_simulation_context``, 
``randomize_rigid_prim_view`` and ``randomize_articulation_view``. For each of these methods, there 
exists an ``operation`` argument, which can be set to ``direct``, ``additive``, or ``scaling``. ``direct`` 
sets the random values directly to Physx, while the behaviour of ``additive`` and ``scaling`` depends on 
the gate it is controlled by. Under ``on_env_reset()``, ``additive`` adds the random values to the default
values of a given attribute of the view, and ``scaling`` multiplies the random values to the default values.
Under ``on_interval(interval)``, ``additive`` and ``scaling`` adds or multiplies the random values to the
last values set during ``on_env_reset()``. In essence, ``on_env_reset()`` randomization should be treated
as *correlated noise* that lasts until the next reset, while ``on_interval(interval)`` randomization should 
be treated as *uncorrelated noise*.

After setting up this action graph, it is necessesary to run ``omni.replicator.core.orchestrator.run()``.

.. code-block:: python

    from omni.isaac.kit import SimulationApp
    simulation_app = SimulationApp({"headless": False})

    import numpy as np
    from omni.isaac.core import World
    from omni.isaac.core.prims import RigidPrimView
    from omni.isaac.core.articulations import ArticulationView
    from omni.isaac.core.utils.prims import get_prim_at_path, define_prim
    from omni.isaac.core.utils.stage import get_current_stage, add_reference_to_stage
    from omni.isaac.core.utils.nucleus import get_assets_root_path
    from omni.isaac.core.objects import DynamicSphere
    from omni.isaac.cloner import GridCloner

    # create the world
    world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene", backend="numpy")
    world.scene.add_default_ground_plane()

    # set up grid cloner
    cloner = GridCloner(spacing=1.5)
    cloner.define_base_env("/World/envs")
    define_prim("/World/envs/env_0")

    # set up the first environment
    DynamicSphere(prim_path="/World/envs/env_0/object", radius=0.1, position=np.array([0.75, 0.0, 0.2]))
    add_reference_to_stage(
        usd_path=get_assets_root_path()+ "/Isaac/Robots/Franka/franka.usd", 
        prim_path="/World/envs/env_0/franka",
    )

    # clone environments
    num_envs = 4
    prim_paths = cloner.generate_paths("/World/envs/env", num_envs)
    env_pos = cloner.clone(source_prim_path="/World/envs/env_0", prim_paths=prim_paths)

    # creates the views and set up world
    object_view = RigidPrimView(prim_paths_expr="/World/envs/*/object", name="object_view")
    franka_view = ArticulationView(prim_paths_expr="/World/envs/*/franka", name="franka_view")
    world.scene.add(object_view)
    world.scene.add(franka_view)
    world.reset()

    num_dof = franka_view.num_dof

    # set up randomization with omni.replicator.isaac, imported as dr
    import omni.replicator.isaac as dr
    import omni.replicator.core as rep

    dr.physics_view.register_simulation_context(world)
    dr.physics_view.register_rigid_prim_view(object_view)
    dr.physics_view.register_articulation_view(franka_view)

    with dr.trigger.on_rl_frame(num_envs=num_envs):
        with dr.gate.on_interval(interval=20):
            dr.physics_view.randomize_simulation_context(
                operation="scaling",
                gravity=rep.distribution.uniform((1, 1, 0.0), (1, 1, 2.0)),
            )
        with dr.gate.on_interval(interval=50):
            dr.physics_view.randomize_rigid_prim_view(
                view_name=object_view.name,
                operation="direct",
                force=rep.distribution.uniform((0, 0, 2.5), (0, 0, 5.0)),
            )
        with dr.gate.on_interval(interval=10):
            dr.physics_view.randomize_articulation_view(
                view_name=franka_view.name,
                operation="direct",
                joint_velocities=rep.distribution.uniform(tuple([-2]*num_dof), tuple([2]*num_dof)),
            )
        with dr.gate.on_env_reset():
            dr.physics_view.randomize_rigid_prim_view(
                view_name=object_view.name,
                operation="additive",
                position=rep.distribution.normal((0.0, 0.0, 0.0), (0.2, 0.2, 0.0)),
                velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
            dr.physics_view.randomize_articulation_view(
                view_name=franka_view.name,
                operation="additive",
                joint_positions=rep.distribution.uniform(tuple([-0.5]*num_dof), tuple([0.5]*num_dof)),
                position=rep.distribution.normal((0.0, 0.0, 0.0), (0.2, 0.2, 0.0)),
            )
    rep.orchestrator.run()


    frame_idx = 0
    while simulation_app.is_running():
        if world.is_playing():
            reset_inds = list()
            if frame_idx % 200 == 0:
                # triggers reset every 200 steps
                reset_inds = np.arange(num_envs)
            dr.physics_view.step_randomization(reset_inds)
            world.step(render=True)
            frame_idx += 1



.. automethod:: omni.replicator.isaac.trigger.on_rl_frame      
.. automethod:: omni.replicator.isaac.gate.on_env_reset      
.. automethod:: omni.replicator.isaac.gate.on_interval      

.. automethod:: omni.replicator.isaac.physics_view.register_simulation_context      
.. automethod:: omni.replicator.isaac.physics_view.register_rigid_prim_view
.. automethod:: omni.replicator.isaac.physics_view.register_articulation_view
.. automethod:: omni.replicator.isaac.physics_view.randomize_simulation_context
.. automethod:: omni.replicator.isaac.physics_view.randomize_rigid_prim_view
.. automethod:: omni.replicator.isaac.physics_view.randomize_articulation_view
.. automethod:: omni.replicator.isaac.physics_view.step_randomization

.. automethod:: omni.replicator.isaac.utils.set_distribution_params
.. automethod:: omni.replicator.isaac.utils.get_distribution_params

Pytorch Online Writer and Listener
==================================

The PytorchWriter and PytorchListener are APIs for using omni.replicator's writer API to retrieve 
various data such as RGB from the specified cameras (supports multiple cameras) and provides them to 
the user in both default format (eg. PNG for RGB data) and batched pytorch tensors. The PytorchListener 
provides an API to directly retrieve data sent to the PytorchWriter without the need to access the stored 
by omni.replicator's BackendDispatch. 

.. automodule:: omni.replicator.isaac.scripts.writers.pytorch_writer
    :members:
    :undoc-members:
    :exclude-members:

.. automodule:: omni.replicator.isaac.scripts.writers.pytorch_listener
    :members:
    :undoc-members:
    :exclude-members:




Omnigraph Nodes
=======================

.. include::  ogn.rst