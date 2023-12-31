.. _GENERATED - Documentation _ognomni.isaac.wheeled_robots.QuinticPathPlanner:


OmniGraph Node omni.isaac.wheeled_robots.QuinticPathPlanner
===========================================================

omni.isaac.wheeled_robots.QuinticPathPlanner Properties
-------------------------------------------------------
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
| uiName                    | Quintic Path Planner                          |
+---------------------------+-----------------------------------------------+
| __categories              | isaacSim                                      |
+---------------------------+-----------------------------------------------+
| __categoryDescriptions    | isaacSim,robot path planning inside Isaac Sim |
+---------------------------+-----------------------------------------------+
| __language                | Python                                        |
+---------------------------+-----------------------------------------------+
| Generated Class Name      | OgnQuinticPathPlannerDatabase                 |
+---------------------------+-----------------------------------------------+
| Python Module             | omni.isaac.wheeled_robots                     |
+---------------------------+-----------------------------------------------+


omni.isaac.wheeled_robots.QuinticPathPlanner Description
--------------------------------------------------------
Quintic Path Planner For Wheeled robots

omni.isaac.wheeled_robots.QuinticPathPlanner Inputs
---------------------------------------------------
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| Name                      | Type       | Default              | Required? | Descripton                                                                                                |
+===========================+============+======================+===========+===========================================================================================================+
| inputs:currentOrientation | quatd[4]   | [0.0, 0.0, 0.0, 0.0] | **Y**     | Current rotation of the robot as a quaternion (recommended to use Get Prim Local to World Transform node) |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:currentPosition    | vectord[3] | [0.0, 0.0, 0.0]      | **Y**     | Current position of the robot (recommended to use Get Prim Local to World Transform node)                 |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:execIn             | execution  | None                 | **Y**     | The input execution                                                                                       |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:goalAccel          | double     | 0.02                 | **Y**     | Goal acceleration                                                                                         |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | 0.02                 |           |                                                                                                           |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:goalVelocity       | double     | 0.5                  | **Y**     | Goal velocity                                                                                             |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | 0.5                  |           |                                                                                                           |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:initialAccel       | double     | 0.02                 | **Y**     | Initial acceleration                                                                                      |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | 0.02                 |           |                                                                                                           |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:initialVelocity    | double     | 0.5                  | **Y**     | Initial velocity                                                                                          |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | 0.5                  |           |                                                                                                           |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:maxAccel           | double     | 1.5                  | **Y**     | Max acceleration                                                                                          |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | 1.5                  |           |                                                                                                           |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:maxJerk            | double     | 0.3                  | **Y**     | Max jerk                                                                                                  |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | 0.3                  |           |                                                                                                           |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:step               | double     | 0.16666666667        | **Y**     | Step                                                                                                      |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
|                           | __default  | 0.16666666667        |           |                                                                                                           |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:targetOrientation  | quatd[4]   | [0.0, 0.0, 0.0, 0.0] | **Y**     | Target orientation (used if no targetPrim provided)                                                       |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:targetPosition     | vectord[3] | [0.0, 0.0, 0.0]      | **Y**     | Target position (used if no targetPrim provided)                                                          |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+
| inputs:targetPrim         | bundle     | None                 |           | USD prim reference to the target position/orientation prim                                                |
+---------------------------+------------+----------------------+-----------+-----------------------------------------------------------------------------------------------------------+


omni.isaac.wheeled_robots.QuinticPathPlanner Outputs
----------------------------------------------------
+-----------------------+-----------+---------+-----------+-----------------------------------------+
| Name                  | Type      | Default | Required? | Descripton                              |
+=======================+===========+=========+===========+=========================================+
| outputs:execOut       | execution | None    | **Y**     | The output execution                    |
+-----------------------+-----------+---------+-----------+-----------------------------------------+
| outputs:pathArrays    | double[]  | None    | **Y**     | The path v, x, y, and yaw arrays        |
+-----------------------+-----------+---------+-----------+-----------------------------------------+
| outputs:target        | double[3] | None    | **Y**     | Target position and orientation         |
+-----------------------+-----------+---------+-----------+-----------------------------------------+
| outputs:targetChanged | bool      | None    | **Y**     | Target position/orientation has changed |
+-----------------------+-----------+---------+-----------+-----------------------------------------+

