Range Based Sensor Simulation [omni.isaac.range_sensor]
#######################################################

This extension provides a set simulated range based sensors like lidar, ultrasonic, generic and interfaces to access them in the simulator.

Lidar Sensor
============

This submodule provides an interface to a simulated lidar sensor. A simplified command is provided to create a lidar sensor in the stage:

.. automethod:: omni.isaac.range_sensor.scripts.commands.RangeSensorCreateLidar        

Once a lidar sensor is in the stage you can use this interface to interact with the simulated lidar data. 
You must first call the acquire interface function.
It is also recommended to use the `is_lidar_sensor` function to check if a lidar sensor exists at the given USD path

.. code-block:: python

    from omni.isaac.range_sensor._range_sensor import acquire_lidar_sensor_interface
    lidar_sensor_interface = acquire_lidar_sensor_interface()
    if lidar_sensor_interface.is_lidar_sensor("/World/Lidar"):
        print("lidar sensor is valid")

.. automethod:: omni.isaac.range_sensor._range_sensor.acquire_lidar_sensor_interface
.. automethod:: omni.isaac.range_sensor._range_sensor.release_lidar_sensor_interface

.. autoclass:: omni.isaac.range_sensor._range_sensor.LidarSensorInterface
    :members:
    :undoc-members:
    :exclude-members:

Ultrasonic Sensor
=================

This submodule provides an interface to a simulated ultrasonic sensor.

.. automethod:: omni.isaac.range_sensor.scripts.commands.RangeSensorCreateUltrasonicArray
.. automethod:: omni.isaac.range_sensor.scripts.commands.RangeSensorCreateUltrasonicEmitter    
.. automethod:: omni.isaac.range_sensor.scripts.commands.RangeSensorCreateUltrasonicFiringGroup  

**Example**

To use this interface, you must first call the acquire interface function.
It is also recommended to use the `is_ultrasonic_sensor` function to check if a ultrasonic sensor exists at the given USD path

.. code-block:: python

    from omni.isaac.range_sensor._range_sensor import acquire_ultrasonic_sensor_interface
    ultrasonic_sensor_interface = acquire_ultrasonic_sensor_interface()
    if ultrasonic_sensor_interface.is_ultrasonic_sensor("/World/UltrasonicArray"):
        print("ultrasonic sensor is valid")

.. automethod:: omni.isaac.range_sensor._range_sensor.acquire_ultrasonic_sensor_interface
.. automethod:: omni.isaac.range_sensor._range_sensor.release_ultrasonic_sensor_interface

.. autoclass:: omni.isaac.range_sensor._range_sensor.UltrasonicSensorInterface
    :members:
    :undoc-members:
    :exclude-members:

Generic Sensor
==============

This submodule provides an interface to a simulated generic sensor.

.. automethod:: omni.isaac.range_sensor.scripts.commands.RangeSensorCreateGeneric    

**Example**

To use this interface, you must first call the acquire interface function.
It is also recommended to use the `is_generic_sensor` function to check if a generic sensor exists at the given USD path

.. code-block:: python

    from omni.isaac.range_sensor._range_sensor import acquire_generic_sensor_interface
    generic_sensor_interface = acquire_generic_sensor_interface()
    if generic_sensor_interface.is_generic_sensor("/World/GenericSensor"):
        print("generic sensor is valid")

.. automethod:: omni.isaac.range_sensor._range_sensor.acquire_generic_sensor_interface
.. automethod:: omni.isaac.range_sensor._range_sensor.release_generic_sensor_interface

.. autoclass:: omni.isaac.range_sensor._range_sensor.GenericSensorInterface
    :members:
    :undoc-members:
    :exclude-members: 


Generated USD Schema API
========================

The following USD Schema API was automatically generated, it is provided here as a reference

.. automodule:: omni.isaac.RangeSensorSchema
    :platform: Windows-x86_64, Linux-x86_64
    :members:
    :undoc-members:
    :show-inheritance:
    :imported-members:

Omnigraph Nodes
=======================

.. include::  ogn.rst