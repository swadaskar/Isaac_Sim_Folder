Lula [omni.isaac.lula]
######################

Overview
========

.. automodule:: lula

Logging
=======

..
  autodoc does not provide a mechanism for controlling the ordering of class members derived
  from bound C++ (as opposed to python source).  Default ordering is alphabetical, so we have
  to document the log levels manually to ensure that they appear in the correct order.
.. autoclass:: lula.LogLevel

  .. py:data:: FATAL

    Logging level for nonrecoverable errors (minimum level, so always enabled).

  .. py:data:: ERROR

    Logging level for recoverable errors.

  .. py:data:: WARNING

    Logging level for warnings, indicating possible cause for concern.

  .. py:data:: INFO

    Logging level for informational messages.

  .. py:data:: VERBOSE

    Logging level for highly verbose informational messages.

.. autofunction:: lula.set_log_level

Rotations and Poses
===================

.. autoclass:: lula.Rotation3
  :members:
  :undoc-members:

.. autoclass:: lula.Pose3
  :members:
  :undoc-members:

Robot Specification
===================

.. autoclass:: lula.RobotDescription
  :members:
  :undoc-members:

.. autofunction:: lula.load_robot
.. autofunction:: lula.load_robot_from_memory

World Specification
===================

.. autoclass:: lula.Obstacle
  :members:
  :undoc-members:

.. autofunction:: lula.create_obstacle

.. autoclass:: lula.World
  :members:
  :undoc-members:

.. autofunction:: lula.create_world

.. autoclass:: lula.WorldView
  :members:
  :undoc-members:

Kinematics
==========

.. autoclass:: lula.Kinematics
  :members:
  :undoc-members:

Inverse Kinematics
==================

.. autoclass:: lula.CyclicCoordDescentIkConfig
  :members:
  :undoc-members:

.. autoclass:: lula.CyclicCoordDescentIkResults
  :members:
  :undoc-members:

.. autofunction:: lula.compute_ik_ccd

Path Specification
==================

.. autoclass:: lula.CSpacePathSpec
  :members:
  :undoc-members:

.. autofunction:: lula.create_c_space_path_spec

.. autoclass:: lula.TaskSpacePathSpec
  :members:
  :undoc-members:

.. autofunction:: lula.create_task_space_path_spec

.. autoclass:: lula.CompositePathSpec
  :members:
  :undoc-members:

.. autofunction:: lula.create_composite_path_spec

.. autofunction:: lula.load_c_space_path_spec_from_file
.. autofunction:: lula.load_c_space_path_spec_from_memory
.. autofunction:: lula.export_c_space_path_spec_to_memory
.. autofunction:: lula.load_task_space_path_spec_from_file
.. autofunction:: lula.load_task_space_path_spec_from_memory
.. autofunction:: lula.export_task_space_path_spec_to_memory
.. autofunction:: lula.load_composite_path_spec_from_file
.. autofunction:: lula.load_composite_path_spec_from_memory
.. autofunction:: lula.export_composite_path_spec_to_memory

Path Generation
===============

.. autoclass:: lula.CSpacePath
  :members:
  :undoc-members:

.. autoclass:: lula.LinearCSpacePath
  :members:
  :undoc-members:

.. autofunction:: lula.create_linear_c_space_path

.. autoclass:: lula.TaskSpacePath
  :members:
  :undoc-members:

.. autoclass:: lula.TaskSpacePathConversionConfig
  :members:
  :undoc-members:

.. autofunction:: lula.convert_composite_path_spec_to_c_space
.. autofunction:: lula.convert_task_space_path_spec_to_c_space

Trajectory Generation
=====================

.. autoclass:: lula.Trajectory
  :members:
  :undoc-members:

.. autoclass:: lula.CSpaceTrajectoryGenerator
  :members:
  :undoc-members:

.. autofunction:: lula.create_c_space_trajectory_generator

Collision Sphere Generation
===========================

.. autoclass:: lula.CollisionSphereGenerator
  :members:
  :undoc-members:

.. autofunction:: lula.create_collision_sphere_generator

Motion Planning
===============

.. autoclass:: lula.MotionPlanner
  :members:
  :undoc-members:

.. autofunction:: lula.create_motion_planner

RMPflow
=======

.. autoclass:: lula.RmpFlowConfig
  :members:
  :undoc-members:

.. autofunction:: lula.create_rmpflow_config
.. autofunction:: lula.create_rmpflow_config_from_memory

.. autoclass:: lula.RmpFlow
  :members:
  :undoc-members:

.. autofunction:: lula.create_rmpflow

Geometric Fabrics
=================

.. note::
  Lula's support for motion policies based on `geometric fabrics <https://arxiv.org/abs/2109.10443>`_
  is under active development and will be exposed in a future release.

..
  Suppress generation of geometric fabrics docs for now:
  .. autoclass:: lula.FabricConfig
  .. autofunction:: lula.create_fabric_config
  .. autofunction:: lula.create_fabric_config_from_memory

  .. autoclass:: lula.FabricState
  .. autofunction:: lula.create_fabric_state

  .. autofunction:: lula.create_fabric
