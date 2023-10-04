


ROS1PublishBbox2D
-----------------
    ['This node publishes ROS1 Bbox2d messages']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **frameId** (*string*): FrameId for ROS1 message. Default to sim_camera.
    - **topicName** (*string*): Name of ROS1 Topic. Default to bbox2d.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Default to 10.
    - **timeStamp** (*double*): Time in seconds to use when publishing the message. Default to 0.0.
    - **data** (*uchar[]*): Buffer array data. Default to [].


ROS1PublishImage
----------------
    ['This node publishes ROS1 Image messages']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **frameId** (*string*): FrameId for ROS1 message. Default to sim_camera.
    - **topicName** (*string*): Name of ROS1 Topic. Default to rgb.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Default to 10.
    - **timeStamp** (*double*): Time in seconds to use when publishing the message. Default to 0.0.
    - **data** (*uchar[]*): Buffer array data. Default to [].
    - **width** (*uint*): Buffer array width. Default to 0.
    - **height** (*uint*): Buffer array height. Default to 0.
    - **encoding** (*token*): ROS encoding format for the input data, taken from the list of strings in include/sensor_msgs/image_encodings.h. Input data is expected to already be in this format, no conversions are performed. Default to rgb8.


ROS1PublishJointState
---------------------
    ['This node publishes joint states of a robot in ROS1 JointState message']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **targetPrim** (*bundle*): USD reference to the robot prim.
    - **nodeNamespace** (*string*): Name of ROS1 Node, prepends any topic published/subscribed by the node name.
    - **topicName** (*string*): Name of ROS1 Topic. Default to joint_states.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Default to 10.
    - **timeStamp** (*double*): ROS1 Timestamp in seconds. Default to 0.0.


ROS1PublishClock
----------------
    ['This node publishes the given time as a ROS1 Clock message']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **topicName** (*string*): Name of ROS1 Topic. Default to clock.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Default to 10.
    - **timeStamp** (*double*): Time in seconds to use when publishing the message. Default to 0.


ROS1PublishCameraInfo
---------------------
    ['This node publishes camera info as a ROS1 CameraInfo message']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **frameId** (*string*): FrameId for ROS1 message. Default to sim_camera.
    - **topicName** (*string*): Name of ROS1 Topic. Default to camera_info.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Default to 10.
    - **timeStamp** (*double*): ROS1 Timestamp in seconds. Default to 0.0.
    - **width** (*uint*): Width for output image.
    - **height** (*uint*): Height for output image.
    - **focalLength** (*float*)
    - **horizontalAperture** (*float*)
    - **verticalAperture** (*float*)
    - **horizontalOffset** (*float*)
    - **verticalOffset** (*float*)
    - **projectionType** (*token*)
    - **stereoOffset** (*float[2]*): Stereo offset (Tx, Ty) used when publishing the camera info topic. Default to [0.0, 0.0].


ROS1Master
----------
    ['This node provides the current status of the ROS master node']


**Inputs**
    - **execIn** (*execution*): The input execution port.

**Outputs**
    - **status** (*bool*): Check whether the master is up.
    - **host** (*string*): The master's hostname, as a string.
    - **port** (*uint*): The master's port.
    - **uri** (*string*): RGet the full URI to the master (eg. http://host:port/).


ROS1PublishRawTransformTree
---------------------------
    ['This node publishes a user-defined transformation between any two coordinate frames as a ROS1 Transform Tree']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **parentFrameId** (*string*): Parent frameId for ROS1 TF message. Default to odom.
    - **childFrameId** (*string*): Child frameId for ROS1 TF message. Default to base_link.
    - **topicName** (*string*): Name of ROS1 Topic. Default to /tf.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Default to 10.
    - **timeStamp** (*double*): ROS1 Timestamp in seconds. Default to 0.0.
    - **translation** (*vectord[3]*): Translation vector in meters. Default to [0.0, 0.0, 0.0].
    - **rotation** (*quatd[4]*): Rotation as a quaternion (IJKR). Default to [0.0, 0.0, 0.0, 1.0].


ROS1PublishSemanticLabels
-------------------------
    ['This node publishes ROS1 semantic label messages']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **topicName** (*string*): Name of ROS1 Topic. Default to labels.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Default to 10.
    - **timeStamp** (*double*): Time in seconds to use when publishing the message. Default to 0.0.
    - **idToLabels** (*string*): Mapping from id to semantic labels.


ROS1PublishImu
--------------
    ['This node publishes IMU data as a ROS1 IMU message']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **frameId** (*string*): FrameId for ROS1 message. Default to sim_imu.
    - **topicName** (*string*): Name of ROS1 Topic. Default to imu.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Default to 10.
    - **timeStamp** (*double*): ROS1 Timestamp in seconds. Default to 0.0.
    - **publishOrientation** (*bool*): Include orientation in msg. Default to True.
    - **publishLinearAcceleration** (*bool*): Include Linear acceleration in msg. Default to True.
    - **publishAngularVelocity** (*bool*): Include Angular velocity in msg. Default to True.
    - **orientation** (*quatd[4], optional*): Orientation as a quaternion (IJKR). Default to [0.0, 0.0, 0.0, 1.0].
    - **linearAcceleration** (*vectord[3], optional*): Linear acceleration vector in m/s^2. Default to [0.0, 0.0, 0.0].
    - **angularVelocity** (*vectord[3], optional*): Angular velocity vector in rad/s. Default to [0.0, 0.0, 0.0].


ROS1SubscribeJointState
-----------------------
    ['This node subscribes to a joint state command of a robot in a ROS1 JointState message']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **topicName** (*string*): Name of ROS1 Topic. Default to joint_command.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Default to 10.

**Outputs**
    - **execOut** (*execution*): Output execution triggers when a new message is received.
    - **timeStamp** (*double*): ROS1 Timestamp in seconds.
    - **jointNames** (*token[]*): Commanded joint names.
    - **positionCommand** (*double[]*): Position commands.
    - **velocityCommand** (*double[]*): Velocity commands.
    - **effortCommand** (*double[]*): Effort commands.


ROS1ServiceTeleport
-------------------
    ['This node provides the service to teleport a robot to the commanded pose']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **serviceName** (*string*): Name of ROS1 service. Default to teleport.


ROS1SubscribeClock
------------------
    ['This node subscribes to a ROS1 Clock message']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **topicName** (*string*): Name of ROS1 Topic. Default to clock.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be processed. Default to 10.

**Outputs**
    - **execOut** (*execution*): Output execution triggers when a new message is received.
    - **timeStamp** (*double*): Time in seconds.


ROS1PublishOdometry
-------------------
    ['This node publishes odometry as a ROS1 Odometry message']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **odomFrameId** (*string*): FrameId for ROS1 odometry message. Default to odom.
    - **chassisFrameId** (*string*): FrameId for robot chassis frame. Default to base_link.
    - **topicName** (*string*): Name of ROS1 Topic. Default to odom.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Default to 10.
    - **timeStamp** (*double*): ROS1 Timestamp in seconds. Default to 0.0.
    - **position** (*vectord[3]*): Position vector in meters. Default to [0.0, 0.0, 0.0].
    - **orientation** (*quatd[4]*): Orientation as a quaternion (IJKR). Default to [0.0, 0.0, 0.0, 1.0].
    - **linearVelocity** (*vectord[3]*): Linear velocity vector in m/s. Default to [0.0, 0.0, 0.0].
    - **angularVelocity** (*vectord[3]*): Angular velocity vector in rad/s. Default to [0.0, 0.0, 0.0].
    - **robotFront** (*vectord[3]*): The front of the robot. Default to [1.0, 0.0, 0.0].


ROS1PublishTransformTree
------------------------
    ['This node publishes the pose of prims as a ROS1 Transform Tree']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **parentPrim** (*bundle, optional*): Prim used as parent frame for poses, leave blank to use World.
    - **targetPrims** (*bundle*): Target prims to publish poses for, if prim is an articulation, the entire articulation tree will be published.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **topicName** (*string*): Name of ROS1 Topic. Default to /tf.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Default to 10.
    - **timeStamp** (*double*): ROS1 Timestamp in seconds. Default to 0.0.


ROS1PublishLaserScan
--------------------
    ['This node publishes LiDAR scans as a ROS1 LaserScan message']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **frameId** (*string*): FrameId for ROS1 message. Default to sim_lidar.
    - **topicName** (*string*): Name of ROS1 Topic. Default to scan.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Default to 10.
    - **timeStamp** (*double*): ROS1 Timestamp in seconds. Default to 0.0.
    - **horizontalFov** (*float*): Horizontal Field of View in degrees. Default to 0.
    - **horizontalResolution** (*float*): Degrees in between rays for horizontal axis. Default to 0.
    - **depthRange** (*float[2]*): The min and max range for sensor to detect a hit [min, max]. Default to [0, 0].
    - **rotationRate** (*float*): Rotation rate of sensor in Hz. Default to 0.
    - **linearDepthData** (*float[]*): Buffer array containing linear depth data. Default to [].
    - **intensitiesData** (*uchar[]*): Buffer array containing intensities data. Default to [].
    - **numRows** (*int*): Number of rows in buffers. Default to 0.
    - **numCols** (*int*): Number of columns in buffers. Default to 0.
    - **azimuthRange** (*float[2]*): The azimuth range [min, max]. Default to [0.0, 0.0].


ROS1SubscribeTwist
------------------
    ['This node subscribes to a ROS1 Twist message']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **topicName** (*string*): Name of ROS1 Topic. Default to cmd_vel.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be processed. Default to 10.

**Outputs**
    - **execOut** (*execution*): Output execution triggers when a new message is received.
    - **linearVelocity** (*vectord[3]*): Linear velocity vector in m/s.
    - **angularVelocity** (*vectord[3]*): Angular velocity vector in rad/s.


ROS1PublishBbox3D
-----------------
    ['This node publishes ROS1 Bbox3d messages']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **frameId** (*string*): FrameId for ROS1 message. Default to sim_camera.
    - **topicName** (*string*): Name of ROS1 Topic. Default to bbox3d.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Default to 10.
    - **timeStamp** (*double*): Time in seconds to use when publishing the message. Default to 0.0.
    - **data** (*uchar[]*): Buffer array data. Default to [].


ROS1PublishPointCloud
---------------------
    ['This node publishes LiDAR scans as a ROS1 PointCloud2 message']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **frameId** (*string*): FrameId for ROS1 message. Default to sim_lidar.
    - **topicName** (*string*): Name of ROS1 Topic. Default to point_cloud.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Default to 10.
    - **timeStamp** (*double*): ROS1 Timestamp in seconds. Default to 0.0.
    - **pointCloudData** (*pointf[3][]*): Buffer of 3d points containing point cloud data. Default to [].


ROS1RtxLidarHelper
------------------
    Handles automation of Lidar Sensor pipeline


**Inputs**
    - **execIn** (*execution*): Triggering this causes the sesnor pipeline to be generated.
    - **context** (*uint64*): ROS context handle, default of zero will use the global context. Default to 0.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends and published/subscribed topic by the node namespace.
    - **frameId** (*string*): FrameID for the ROS1 message. Default to sim_lidar.
    - **topicName** (*string*): Topic name for sensor data. Default to scan.
    - **queueSize** (*uint64*): Number of message to queue up before throwing away, in case messages are collected faster than they can be sent. Default to 10.
    - **renderProductPath** (*token*): Name of the render product path to publish lidar data.
    - **type** (*token*): Data to publsih from node. Default to laser_scan.
    - **resetSimulationTimeOnStop** (*bool*): If True the simulation time will reset when stop is pressed, False means time increases monotonically. Default to False.


ROS1CameraHelper
----------------
    This node handles automation of the camera sensor pipeline


**Inputs**
    - **execIn** (*execution*): Triggering this causes the sensor pipeline to be generated.
    - **nodeNamespace** (*string*): Namespace of ROS1 Node, prepends any published/subscribed topic by the node namespace.
    - **frameId** (*string*): FrameId for ROS1 message. Default to sim_camera.
    - **topicName** (*string*): Topic name for sensor data. Default to rgb.
    - **queueSize** (*uint64*): The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Default to 10.
    - **viewport** (*token*): DEPRECATED, use renderProductPath. Name of the desired viewport to publish.
    - **renderProductPath** (*token*): Path of the render product used for capturing data.
    - **type** (*token*): . Default to rgb.
    - **enableSemanticLabels** (*bool*): Enable publishing of semantic labels, applies only to instance_segmentation, semantic_segmentation, bbox_2d_tight, bbox_2d_loose, bbox_3d.
    - **semanticLabelsTopicName** (*string*): Topic name used for publishing semantic labels, applies only to instance_segmentation, semantic_segmentation, bbox_2d_tight, bbox_2d_loose, bbox_3d. Default to semantic_labels.
    - **stereoOffset** (*float[2]*): Stereo offset (Tx, Ty) used when publishing the camera info topic. Default to [0, 0].
    - **resetSimulationTimeOnStop** (*bool*): If True the simulation time will reset when stop is pressed, False means time increases monotonically. Default to False.