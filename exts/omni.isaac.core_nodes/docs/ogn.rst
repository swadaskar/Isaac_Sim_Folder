


IsaacGenerate32FC1
------------------
    Isaac Sim Node that generates a constant 32FC1 buffer


**Inputs**
    - **value** (*float*): Value for output image.
    - **width** (*uint*): Width for output image. Default to 100.
    - **height** (*uint*): Height for output image. Default to 100.

**Outputs**
    - **data** (*uchar[]*): Buffer 32FC1 array data.
    - **width** (*uint*): Width for output image.
    - **height** (*uint*): Height for output image.
    - **encoding** (*token*): Encoding as a token.


IsaacConvertDepthToPointCloud
-----------------------------
    Converts a 32FC1 image buffer into Point Cloud data


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **data** (*uchar[]*): Buffer 32FC1 image array data. Default to [].
    - **format** (*uint64*): Format. Default to 33.
    - **width** (*uint*): Buffer array width, same as input.
    - **height** (*uint*): Buffer array height, same as input.
    - **focalLength** (*float*)
    - **horizontalAperture** (*float*)
    - **verticalAperture** (*float*)

**Outputs**
    - **execOut** (*execution*): Output execution triggers when lidar sensor has completed a full scan.
    - **pointCloudData** (*pointf[3][]*): Buffer of 3d points containing point cloud data.


IsaacReadSimulationTime
-----------------------
    Holds values related to simulation timestamps


**Inputs**
    - **resetOnStop** (*bool*): If True the simulation time will reset when stop is pressed, False means time increases monotonically. Default to False.
    - **swhFrameNumber** (*int64*): Optional fabric frame number, leave as zero to get the latest simulation frame time. Default to 0.

**Outputs**
    - **simulationTime** (*double*): Current Simulation Time in Seconds.


IsaacConvertRGBAToRGB
---------------------
    Converts a RGBA image buffer into RGB


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **data** (*uchar[]*): Buffer rgba array data. Default to [].
    - **width** (*uint*): Buffer array width.
    - **height** (*uint*): Buffer array height.
    - **encoding** (*token*): Encoding as a token. Default to rgba8.
    - **bufferSize** (*uint*): Size (in bytes) of the buffer (0 if the input is a texture).
    - **swhFrameNumber** (*uint64*): Frame number.

**Outputs**
    - **execOut** (*execution*): Output execution triggers when conversion complete.
    - **data** (*uchar[]*): Buffer rgb array data.
    - **encoding** (*token*): Encoding as a token.
    - **width** (*uint*): Buffer array width, same as input.
    - **height** (*uint*): Buffer array height, same as input.
    - **bufferSize** (*uint*): Size (in bytes) of the buffer (0 if the input is a texture).
    - **swhFrameNumber** (*uint64*): Frame number.


IsaacReadSystemTime
-------------------
    Holds values related to system timestamps


**Inputs**
    - **swhFrameNumber** (*int64*): Optional fabric frame number, leave as zero to get the latest system frame time. Default to 0.

**Outputs**
    - **systemTime** (*double*): Current System Time in Seconds.


IsaacComputeOdometry
--------------------
    Holds values related to odometry, this node is not a replcement for the IMU sensor and the associated Read IMU node


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **chassisPrim** (*bundle*): Usd prim reference to the articulation root or rigid body prim.

**Outputs**
    - **execOut** (*execution*): The output execution port.
    - **position** (*vectord[3]*): Position vector in meters.
    - **orientation** (*quatd[4]*): Rotation as a quaternion (IJKR).
    - **linearVelocity** (*vectord[3]*): Linear velocity vector in m/s.
    - **angularVelocity** (*vectord[3]*): Angular velocity vector in rad/s.
    - **linearAcceleration** (*vectord[3]*): Linear acceleration vector in m/s^2.
    - **angularAcceleration** (*vectord[3]*): Angular acceleration vector in rad/s^2.


IsaacReadCameraInfo
-------------------
    Isaac Sim node that reads camera info for a viewport


**Inputs**
    - **viewport** (*token*): Name of the viewport, or empty for the default viewport.
    - **renderProductPath** (*token*): Path of the render product.

**Outputs**
    - **width** (*uint*): Width for output image.
    - **height** (*uint*): Height for output image.
    - **focalLength** (*float*)
    - **horizontalAperture** (*float*)
    - **verticalAperture** (*float*)
    - **horizontalOffset** (*float*)
    - **verticalOffset** (*float*)
    - **projectionType** (*token*)
    - **cameraFisheyeParams** (*float[]*): Camera fisheye projection parameters.


IsaacTestNode
-------------
    Isaac Sim Test Node


**Inputs**
    - **execIn** (*execution*): The input execution.
    - **input** (*string*): string passed here is returned on the output of this node.

**Outputs**
    - **output** (*string*): return the value of input.


IsaacCreateRenderProduct
------------------------
    Isaac Sim node that creates a render product for use with offscreen rendering


**Inputs**
    - **execIn** (*execution*): Input execution trigger.
    - **width** (*uint*): Width of the render product, in pixels. Default to 1280.
    - **height** (*uint*): Height of the render product, in pixels. Default to 720.
    - **cameraPrim** (*bundle*): Usd prim reference to the camera associated with this render product.

**Outputs**
    - **renderProductPath** (*token*): Render product path for the created hydra texture.
    - **execOut** (*execution*): Output execution trigger.


IsaacGetViewportRenderProduct
-----------------------------
    Isaac Sim node that returns the render product for a given viewport


**Inputs**
    - **execIn** (*execution*): Input execution trigger.
    - **viewport** (*token*): Name of the viewport to get renderproduct for.

**Outputs**
    - **renderProductPath** (*token*): Render product path for the created hydra texture.
    - **execOut** (*execution*): Output execution trigger.


IsaacGenerateRGBA
-----------------
    Isaac Sim Node that generates a constant rgba buffer


**Inputs**
    - **color** (*colorf[4]*): Color for output image.
    - **width** (*uint*): Width for output image. Default to 100.
    - **height** (*uint*): Height for output image. Default to 100.

**Outputs**
    - **data** (*uchar[]*): Buffer rgba array data.
    - **width** (*uint*): Width for output image.
    - **height** (*uint*): Height for output image.
    - **encoding** (*token*): Encoding as a token.


OgnIsaacScaleToFromStageUnit
----------------------------
    ['This node converts meters to/from stage units']


**Inputs**
    - **conversion** (*token*): Convert meters to/from stage units. Default to Convert to stage units.
    - **value** (*['numerics']*): The input value.

**Outputs**
    - **result** (*['numerics']*): The output value.


IsaacCreateViewport
-------------------
    Isaac Sim node that creates a unique viewport


**Inputs**
    - **execIn** (*execution*): Input execution trigger.
    - **name** (*token*): Name of the viewport window.
    - **viewportId** (*uint*): If name is empty, ID is used as the name, ID == 0 is the default viewport.

**Outputs**
    - **viewport** (*token*): Name of the created viewport.
    - **execOut** (*execution*): Input execution trigger.


IsaacReadFilePath
-----------------
    Loads contents of file when given path, if file exists


**Inputs**
    - **path** (*path*): Input path to file.

**Outputs**
    - **fileContents** (*string*): Output contents of file at path, returns empty string if file is not found.


IsaacSetViewportResolution
--------------------------
    Isaac Sim node that sets the resolution on a viewport


**Inputs**
    - **execIn** (*execution*): Input execution trigger.
    - **viewport** (*token*): Name of viewport to set resolution of.
    - **width** (*uint*): Width of the viewport, in pixels. Default to 1280.
    - **height** (*uint*): Height of the viewport, in pixels. Default to 720.

**Outputs**
    - **execOut** (*execution*): Input execution trigger.


IsaacArticulationController
---------------------------
    Controller for articulated robots

    The controller takes either joint names or joint indices, and move them by the given position/velocity/effort commands

**Inputs**
    - **execIn** (*execution*): The input execution.
    - **targetPrim** (*bundle*): The target robot prim.
    - **usePath** (*bool*): use robot and com path instead of selecting them from stage tree. Default to True.
    - **robotPath** (*string*): path to the robot articulation root.
    - **jointNames** (*token[]*): commanded joint names. Use either Joint Names or Joint Indices, if neither is given, default to all joints.
    - **jointIndices** (*int[]*): commanded joint indices. Use either Joint Names or Joint Indices, if neither is given, default to all joints.
    - **positionCommand** (*double[]*): position commands.
    - **velocityCommand** (*double[]*): velocity commands.
    - **effortCommand** (*double[]*): effort commands.


IsaacSetCameraOnRenderProduct
-----------------------------
    Isaac Sim node that sets the camera prim of an existing render product


**Inputs**
    - **execIn** (*execution*): Input execution trigger.
    - **renderProductPath** (*token*): Path of the render product.
    - **cameraPrim** (*bundle*): Usd prim reference to the camera associated with this render product.

**Outputs**
    - **execOut** (*execution*): Output execution trigger.


IsaacSimulationGate
-------------------
    Gate node that only passes through execution if simulation is playing


**Inputs**
    - **execIn** (*execution*): The input execution.
    - **step** (*uint*): Number of ticks per execution output, default is 1, set to zero to disable execution of connected nodes. Default to 1.

**Outputs**
    - **execOut** (*execution*): The output execution.


IsaacReadEnvVar
---------------
    Loads in environment variable if present


**Inputs**
    - **envVar** (*string*): Input OS environment variable name as string.

**Outputs**
    - **value** (*string*): Output OS environment variable value, returns empty string if variable is not found.