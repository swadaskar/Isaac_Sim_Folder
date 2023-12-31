.. _GENERATED - Documentation _ognomni.isaac.wheeled_robots.StanleyControlPID:


OmniGraph Node omni.isaac.wheeled_robots.StanleyControlPID
==========================================================

omni.isaac.wheeled_robots.StanleyControlPID Properties
------------------------------------------------------
+---------------------------+-----------------------------------------------+
| Name                      | Value                                         |
+===========================+===============================================+
| Version                   | 1                                             |
+---------------------------+-----------------------------------------------+
| Extension                 | omni.isaac.wheeled_robots                     |
+---------------------------+-----------------------------------------------+
| Has State?                | False                                         |
+---------------------------+-----------------------------------------------+
| Implementation Language   | Python                                        |
+---------------------------+-----------------------------------------------+
| Default Memory Type       | cpu                                           |
+---------------------------+-----------------------------------------------+
| Generated Code Exclusions | None                                          |
+---------------------------+-----------------------------------------------+
| uiName                    | Stanley Control PID                           |
+---------------------------+-----------------------------------------------+
| __categories              | isaacSim                                      |
+---------------------------+-----------------------------------------------+
| __categoryDescriptions    | isaacSim,robot path planning inside Isaac Sim |
+---------------------------+-----------------------------------------------+
| __language                | Python                                        |
+---------------------------+-----------------------------------------------+
| Generated Class Name      | OgnStanleyControlPIDDatabase                  |
+---------------------------+-----------------------------------------------+
| Python Module             | omni.isaac.wheeled_robots                     |
+---------------------------+-----------------------------------------------+


omni.isaac.wheeled_robots.StanleyControlPID Description
-------------------------------------------------------
Drive to Target Steering

omni.isaac.wheeled_robots.StanleyControlPID Inputs
--------------------------------------------------
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| Name                      | Type       | Default               | Required? | Descripton                                                                                                |
+===========================+============+=======================+===========+===========================================================================================================+
| inputs:currentOrientation | quatd[4]   | [0.0, 0.0, 0.0, 0.0]  | **Y**     | Current rotation of the robot as a quaternion (recommended to use Get Prim Local to World Transform node) |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:currentPosition    | vectord[3] | [0.0, 0.0, 0.0]       | **Y**     | Current position of the robot (recommended to use Get Prim Local to World Transform node)                 |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:currentSpeed       | vectord[3] | [0.0, 0.0, 0.0]       | **Y**     | Current linear velocity of the robot                                                                      |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:drawPath           | bool       | False                 | **Y**     | Draw the provided path curve onto the stage                                                               |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | false                 |           |                                                                                                           |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:execIn             | execution  | None                  | **Y**     | The input execution                                                                                       |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:gains              | double[3]  | [0.5, 0.1, 0.0872665] | **Y**     | control, velocity and steering gains                                                                      |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | [0.5, 0.1, 0.0872665] |           |                                                                                                           |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:maxVelocity        | double     | 1.5                   | **Y**     | Maximum linear velocity of the robot                                                                      |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | 1.5                   |           |                                                                                                           |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:pathArrays         | double[]   | []                    | **Y**     | The path v, x, y, and yaw arrays                                                                          |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:reachedGoal        | bool[]     | [False, False]        | **Y**     | Position and orientation thresholds at target                                                             |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | [false, false]        |           |                                                                                                           |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:step               | double     | 0.16666666667         | **Y**     | Step                                                                                                      |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | 0.16666666667         |           |                                                                                                           |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:target             | double[3]  | [0, 0, 0]             | **Y**     | Target position and orientation                                                                           |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | [0, 0, 0]             |           |                                                                                                           |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:targetChanged      | bool       | False                 | **Y**     | Target position/orientation has changed                                                                   |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | false                 |           |                                                                                                           |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:thresholds         | double[2]  | [0.1, 0.1]            | **Y**     | Position and orientation thresholds at target                                                             |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | [0.1, 0.1]            |           |                                                                                                           |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:wheelBase          | double     | 0.4132                | **Y**     | Distance between the centers of the front and rear wheels                                                 |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | 0.4132                |           |                                                                                                           |
+---------------------------+------------+-----------------------+-----------+-----------------------------------------------------------------------------------------------------------+


omni.isaac.wheeled_robots.StanleyControlPID Outputs
---------------------------------------------------
+-------------------------+-----------+---------+-----------+---------------------------------------+
| Name                    | Type      | Default | Required? | Descripton                            |
+=========================+===========+=========+===========+=======================================+
| outputs:angularVelocity | double    | None    | **Y**     | Current angular speed for robot drive |
+-------------------------+-----------+---------+-----------+---------------------------------------+
| outputs:execOut         | execution | None    | **Y**     | The output execution                  |
+-------------------------+-----------+---------+-----------+---------------------------------------+
| outputs:linearVelocity  | double    | None    | **Y**     | Current forward speed for robot drive |
+-------------------------+-----------+---------+-----------+---------------------------------------+

