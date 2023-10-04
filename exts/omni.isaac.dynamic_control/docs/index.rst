Dynamic Control [omni.isaac.dynamic_control]
######################################################

The Dynamic Control extension provides a set of utilities to control physics objects. 
It provides opaque handles for different physics objects that remain valid between PhysX scene resets, which occur whenever play or stop is pressed.

Basic Usage
===========

Start physics simulation, at least one frame of simulation must occur before the Dynamic Control interface will become fully active. 

.. code-block:: python
    :linenos:

    import omni
    omni.timeline.get_timeline_interface().play()

Acquire the Dynamic Control interface and interact with an articulation. 
The code block below assumes a Franka Emika Panda robot is in the stage with a base path of /Franka

.. code-block:: python
    :linenos:

    from omni.isaac.dynamic_control import _dynamic_control
    dc = _dynamic_control.acquire_dynamic_control_interface()
    
    # Get a handle to the Franka articulation
    # This handle will automatically update if simulation is stopped and restarted
    art = dc.get_articulation("/Franka")
    
    # Get information about the structure of the articulation
    num_joints = dc.get_articulation_joint_count(art)
    num_dofs = dc.get_articulation_dof_count(art)
    num_bodies = dc.get_articulation_body_count(art)
    
    # Get a specific degree of freedom on an articulation
    dof_ptr = dc.find_articulation_dof(art, "panda_joint2")

    dof_state = dc.get_dof_state(dof_ptr)
    # print position for the degree of freedom
    print(dof_state.pos)

    # This should be called each frame of simulation if state on the articulation is being changed.
    dc.wake_up_articulation(art)
    dc.set_dof_position_target(dof_ptr, -1.5)




Acquiring Extension Interface
==============================

.. automethod:: omni.isaac.dynamic_control._dynamic_control.acquire_dynamic_control_interface
.. automethod:: omni.isaac.dynamic_control._dynamic_control.release_dynamic_control_interface

Dynamic Control API
====================

.. autoclass:: omni.isaac.dynamic_control._dynamic_control.DynamicControl
    :members:
    :undoc-members:
    :exclude-members: 

Transform and Velocity
======================

.. autoclass:: omni.isaac.dynamic_control._dynamic_control.Transform
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: omni.isaac.dynamic_control._dynamic_control.Velocity
    :members:
    :undoc-members:
    :show-inheritance:



Types
=====

.. autoclass:: omni.isaac.dynamic_control._dynamic_control.ObjectType
    :members:
    :show-inheritance:
    :exclude-members: name

.. autoclass:: omni.isaac.dynamic_control._dynamic_control.DofType
    :members:
    :show-inheritance:
    :exclude-members: name

.. autoclass:: omni.isaac.dynamic_control._dynamic_control.JointType
    :members:
    :show-inheritance:
    :exclude-members: name

.. autoclass:: omni.isaac.dynamic_control._dynamic_control.DriveMode
    :members:
    :show-inheritance:
    :exclude-members: name


Properties
==========

.. autoclass:: omni.isaac.dynamic_control._dynamic_control.ArticulationProperties
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: omni.isaac.dynamic_control._dynamic_control.RigidBodyProperties
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: omni.isaac.dynamic_control._dynamic_control.DofProperties
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: omni.isaac.dynamic_control._dynamic_control.AttractorProperties
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: omni.isaac.dynamic_control._dynamic_control.D6JointProperties
    :members:
    :undoc-members:
    :show-inheritance:




States
==========

.. autoclass:: omni.isaac.dynamic_control._dynamic_control.RigidBodyState
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: omni.isaac.dynamic_control._dynamic_control.DofState
    :members:
    :undoc-members:
    :show-inheritance:

Constants
=========

Object handles
--------------

.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.INVALID_HANDLE


State Flags
-----------

.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.STATE_NONE
.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.STATE_POS
.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.STATE_VEL
.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.STATE_EFFORT
.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.STATE_ALL


Axis Flags
----------

.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.AXIS_NONE
.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.AXIS_X
.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.AXIS_Y
.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.AXIS_Z
.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.AXIS_TWIST
.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.AXIS_SWING_1
.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.AXIS_SWING_2
.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.AXIS_ALL_TRANSLATION
.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.AXIS_ALL_ROTATION
.. autoattribute:: omni.isaac.dynamic_control._dynamic_control.AXIS_ALL
    