


StanleyControlPID
-----------------
    Drive to Target Steering

    

**Inputs**
    - **execIn** (*execution*): The input execution.
    - **currentPosition** (*vectord[3]*): Current position of the robot (recommended to use Get Prim Local to World Transform node).
    - **currentOrientation** (*quatd[4]*): Current rotation of the robot as a quaternion (recommended to use Get Prim Local to World Transform node).
    - **currentSpeed** (*vectord[3]*): Current linear velocity of the robot.
    - **maxVelocity** (*double*): Maximum linear velocity of the robot. Default to 1.5.
    - **reachedGoal** (*bool[]*): Position and orientation thresholds at target. Default to [False, False].
    - **pathArrays** (*double[]*): The path v, x, y, and yaw arrays.
    - **target** (*double[3]*): Target position and orientation. Default to [0, 0, 0].
    - **targetChanged** (*bool*): Target position/orientation has changed. Default to False.
    - **wheelBase** (*double*): Distance between the centers of the front and rear wheels. Default to 0.4132.
    - **thresholds** (*double[2]*): Position and orientation thresholds at target. Default to [0.1, 0.1].
    - **drawPath** (*bool*): Draw the provided path curve onto the stage. Default to False.
    - **step** (*double*): Step. Default to 0.16666666667.
    - **gains** (*double[3]*): control, velocity and steering gains. Default to [0.5, 0.1, 0.0872665].

**Outputs**
    - **execOut** (*execution*): The output execution.
    - **linearVelocity** (*double*): Current forward speed for robot drive.
    - **angularVelocity** (*double*): Current angular speed for robot drive.


HolonomicRobotUsdSetup
----------------------
    setup any robot to be ready to be used by the holonomic controller by extract attributes from USD

    Use this node to extract all the holonomic drive information from USD if the listed information are stored in the USD file already. If they are not in USD, you can manually set those values in the HolonomicController node

**Inputs**
    - **robotPrim** (*bundle*): prim for the robot's articulation root.
    - **comPrim** (*bundle*): prim for the center of mass xform.
    - **usePath** (*bool*): use prim path instead of prim bundles. Default to False.
    - **robotPrimPath** (*token*): prim path to the robot's articulation root link when usdPath is true.
    - **comPrimPath** (*token*): prim path to the robot's center of mass xform.

**Outputs**
    - **wheelRadius** (*double[]*): an array of wheel radius.
    - **wheelPositions** (*double[3][]*): position of the wheel with respect to chassis' center of mass.
    - **wheelOrientations** (*double[4][]*): orientation of the wheel with respect to chassis' center of mass frame.
    - **mecanumAngles** (*double[]*): angles of the mechanum wheels with respect to wheel's rotation axis.
    - **wheelAxis** (*double[3]*): the rotation axis of the wheels, assuming all wheels have the same.
    - **upAxis** (*double[3]*): the rotation axis of the vehicle.
    - **wheelDofNames** (*token[]*): name of the left wheel joint.


DifferentialController
----------------------
    Differential Controller

    Use the wheel radius and the distance between the wheels to calculate the desired wheels speed when given a desired vehicle speed.

**Inputs**
    - **execIn** (*execution*): The input execution.
    - **wheelRadius** (*double*): radius of the wheels.
    - **wheelDistance** (*double*): distance between the two wheels.
    - **maxLinearSpeed** (*double*): max linear speed allowed for vehicle.
    - **maxAngularSpeed** (*double*): max angular speed allowed for vehicle.
    - **maxWheelSpeed** (*double*): max wheel speed allowed.
    - **linearVelocity** (*double*): desired linear velocity.
    - **angularVelocity** (*double*): desired rotation velocity.

**Outputs**
    - **positionCommand** (*double[]*): position commands.
    - **velocityCommand** (*double[]*): velocity commands.
    - **effortCommand** (*double[]*): effort commands.


QuinticPathPlanner
------------------
    Quintic Path Planner For Wheeled robots

    Use odometry from a robot and a target position/prim to calculate a route from the robot's starting position to the target position.

**Inputs**
    - **execIn** (*execution*): The input execution.
    - **currentPosition** (*vectord[3]*): Current position of the robot (recommended to use Get Prim Local to World Transform node).
    - **currentOrientation** (*quatd[4]*): Current rotation of the robot as a quaternion (recommended to use Get Prim Local to World Transform node).
    - **targetPrim** (*bundle, optional*): USD prim reference to the target position/orientation prim.
    - **targetPosition** (*vectord[3]*): Target position (used if no targetPrim provided).
    - **targetOrientation** (*quatd[4]*): Target orientation (used if no targetPrim provided).
    - **initialVelocity** (*double*): Initial velocity. Default to 0.5.
    - **initialAccel** (*double*): Initial acceleration. Default to 0.02.
    - **goalVelocity** (*double*): Goal velocity. Default to 0.5.
    - **goalAccel** (*double*): Goal acceleration. Default to 0.02.
    - **maxAccel** (*double*): Max acceleration. Default to 1.5.
    - **maxJerk** (*double*): Max jerk. Default to 0.3.
    - **step** (*double*): Step. Default to 0.16666666667.

**Outputs**
    - **execOut** (*execution*): The output execution.
    - **pathArrays** (*double[]*): The path v, x, y, and yaw arrays.
    - **target** (*double[3]*): Target position and orientation.
    - **targetChanged** (*bool*): Target position/orientation has changed.


HolonomicController
-------------------
    Holonomic Controller

    Calculating the desired wheel speeds when given a desired vehicle speed.

**Inputs**
    - **execIn** (*execution*): The input execution.
    - **wheelRadius** (*double[]*): an array of wheel radius.
    - **wheelPositions** (*double[3][]*): position of the wheel with respect to chassis' center of mass.
    - **wheelOrientations** (*double[4][]*): orientation of the wheel with respect to chassis' center of mass frame.
    - **mecanumAngles** (*double[]*): angles of the mecanum wheels with respect to wheel's rotation axis.
    - **wheelAxis** (*double[3]*): the rotation axis of the wheels.
    - **upAxis** (*double[3]*): the rotation axis of the vehicle.
    - **velocityCommands** (*['float[3]', 'double[3]']*): velocity in x and y and rotation.
    - **maxLinearSpeed** (*double, optional*): maximum speed allowed for the vehicle.
    - **maxAngularSpeed** (*double, optional*): maximum angular rotation speed allowed for the vehicle.
    - **maxWheelSpeed** (*double, optional*): maximum rotation speed allowed for the wheel joints.
    - **linearGain** (*double*): linear gain. Default to 1.
    - **angularGain** (*double*): angular gain. Default to 1.

**Outputs**
    - **jointPositionCommand** (*double[]*): position commands for the wheel joints.
    - **jointVelocityCommand** (*double[]*): velocity commands for the wheels joints.
    - **jointEffortCommand** (*double[]*): effort commands for the wheels joints.


CheckGoal2D
-----------
    Check if wheeled robot has reached goal

    

**Inputs**
    - **execIn** (*execution*): The input execution.
    - **currentPosition** (*vectord[3]*): Current position of the robot (recommended to use Get Prim Local to World Transform node).
    - **currentOrientation** (*quatd[4]*): Current rotation of the robot as a quaternion (recommended to use Get Prim Local to World Transform node).
    - **target** (*double[3]*): Target position and orientation. Default to [0, 0, 0].
    - **targetChanged** (*bool*): Target position/orientation has changed. Default to False.
    - **thresholds** (*double[2]*): Position and orientation thresholds at target. Default to [0.1, 0.1].

**Outputs**
    - **execOut** (*execution*): The output execution.
    - **reachedGoal** (*bool[]*): Reached position and orientation goals.