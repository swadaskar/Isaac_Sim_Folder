Isaac Sensor Extension [omni.isaac.sensor]
######################################################


The Isaac Sensor Extension provides a set of simulated physics based sensors like contact sensor, inertial measurement unit (IMU) sensor and interfaces to access them in the simulator


Contact Sensor
==============

.. automodule:: omni.isaac.sensor.scripts.contact_sensor
    :inherited-members:
    :members:
    :undoc-members:
    :exclude-members:

IMU sensor
============

.. automodule:: omni.isaac.sensor.scripts.imu_sensor
    :inherited-members:
    :members:
    :undoc-members:
    :exclude-members:

Lidar RTX sensor
================

.. automodule:: omni.isaac.sensor.scripts.lidar_rtx
    :inherited-members:
    :members:
    :undoc-members:
    :exclude-members:

Rotating Lidar PhysX sensor
============================

.. automodule:: omni.isaac.sensor.scripts.rotating_lidar_physX
    :inherited-members:
    :members:
    :undoc-members:
    :exclude-members:

Camera sensor
===============

.. automodule:: omni.isaac.sensor.scripts.camera
    :inherited-members:
    :members:
    :undoc-members:
    :exclude-members:

Contact Sensor Interface
========================

This submodule  provides an interface to a simulated contact sensor. A simplified command is provided to create a contact sensor in the stage:

.. automethod:: omni.isaac.sensor.scripts.commands.IsaacSensorCreateContactSensor        

Once the contact sensor is created, you must first acquire this interface and then you can use this interface to access the contact sensor

Also, the offset of the contact sensor is also affect by the parent's transformations.

.. code-block:: python
    :linenos:

    from omni.isaac.sensor import _sensor
    _cs = _sensor.acquire_contact_sensor_interface()

Please note: if the contact sensor is not initially created under a valid rigid body parent, the contact sensor will not output any valid data even if the contact sensor is later attached to a valid rigid body parent.

Acquiring Extension Interface
-------------------------------

.. automethod:: omni.isaac.sensor._sensor.acquire_contact_sensor_interface
.. automethod:: omni.isaac.sensor._sensor.release_contact_sensor_interface

To collect the most recent reading, call the interface `get_sensor_sim_reading(/path/to/sensor)`. The result will be most recent sensor reading.

.. code-block:: python

    reading = _cs.get_sensor_sim_reading("/World/Cube/Contact_Sensor")

To collect the readings, call the interface `get_sensor_readings(/path/to/sensor)`. The result will be the accumulated readings since last frame of the simulator. Each reading is timestamped, and contains a boolean flag to tell if the sensor is triggered.

.. code-block:: python

    readings = _cs.get_sensor_readings("/World/Cube/Contact_Sensor")

To collect raw reading, call the interface `get_contact_sensor_raw_data(/path/to/sensor)`. The result will return a list of raw contact data for that body.

.. code-block:: python

    raw_Contact = _cs.get_contact_sensor_raw_data("/World/Cube/Contact_Sensor")


Output Types
-------------
.. autoclass:: omni.isaac.sensor._sensor.CsSensorReading
    :members:
    :undoc-members:
    :exclude-members: 



.. autoclass:: omni.isaac.sensor._sensor.CsRawData
    :members:
    :undoc-members:
    :exclude-members: 


Interface Methods
-------------------

.. autoclass:: omni.isaac.sensor._sensor.ContactSensorInterface
    :members:
    :undoc-members:


IMU sensor Interface
====================

This submodule provides an interface to a simulate IMU sensor, which provides linear acceleration and angular velocity data.

A simplified command is provided to create an IMU sensor:

.. automethod:: omni.isaac.sensor.scripts.commands.IsaacSensorCreateImuSensor        

Similiarly, once an IMU sensor is created, you can use this interface to interact with the simulated IMU sensor. 
You must first call the acquire_imu_sensor_interface. 

.. code-block:: python
    :linenos:

    from omni.isaac.sensor import _sensor
    _is = _sensor.acquire_imu_sensor_interface()

Please note: if the IMU sensor is not initially created under a valid rigid body parent, the IMU sensor will not output any valid data even if the IMU sensor is later attached to a valid rigid body parent.
Also, the offset and orientation of the IMU sensor is also affect by the parent's transformations.


Acquiring Extension Interface
-------------------------------

.. automethod:: omni.isaac.sensor._sensor.acquire_imu_sensor_interface
.. automethod:: omni.isaac.sensor._sensor.release_imu_sensor_interface

To collect the most recent reading, call the interface `get_sensor_sim_reading(/path/to/sensor)`. The result will be most recent sensor reading.

.. code-block:: python

    reading = _is.get_sensor_sim_reading("/World/Cube/Imu_Sensor")

To collect the readings, call the interface `get_sensor_readings(/path/to/sensor)`. The result will be the accumulated readings since last frame of the simulator. Each reading is timestamped, and contains a boolean flag to tell if the sensor is triggered.

.. code-block:: python

    readings = _is.get_sensor_readings("/World/Cube/Imu_Sensor")

Output Types
--------------
.. autoclass:: omni.isaac.sensor._sensor.IsSensorReading
    :members:
    :undoc-members:
    :exclude-members: 

Interface Methods
------------------

.. autoclass:: omni.isaac.sensor._sensor.ImuSensorInterface
    :members:
    :undoc-members:

.. .. automodule:: omni.isaac.sensor._contact_sensor
..    :platform: Windows-x86_64, Linux-x86_64
..    :members:
..    :undoc-members:
..    :show-inheritance:
..    :imported-members:
..    :exclude-members:

.. .. automodule:: omni.isaac.sensor._imu_sensor
..    :platform: Windows-x86_64, Linux-x86_64
..    :members:
..    :undoc-members:
..    :show-inheritance:
..    :imported-members:
..    :exclude-members:

Omnigraph Nodes
=======================

.. include::  ogn.rst