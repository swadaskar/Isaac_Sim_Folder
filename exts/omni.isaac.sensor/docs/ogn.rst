


IsaacComputeRTXLidarPointCloud
------------------------------
    ['This node reads from the an RTX Lidar sensor and holds point cloud data buffers']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **cpuPointer** (*uint64*): CPU Pointer to LiDAR render result.
    - **keepOnlyPositiveDistance** (*bool*): Keep points only if the return distance is > 0. Default to True.
    - **accuracyErrorAzimuthDeg** (*float*): Accuracy error of azimuth in degrees applied to all points equally.
    - **accuracyErrorElevationDeg** (*float*): Accuracy error of elevation in degrees applied to all points equally.
    - **accuracyErrorPosition** (*float[3]*): Position offset applied to all points equally.
    - **renderProductPath** (*token*): Path of the renderProduct to wait for being rendered.

**Outputs**
    - **execOut** (*execution*): Output execution triggers when lidar sensor has data.
    - **toWorldMatrix** (*matrixd[4]*): The transform matrix from lidar to world coordinates.
    - **pointCloudData** (*pointf[3][]*): Buffer of 3d points containing point cloud data in Lidar coordinates.
    - **intensity** (*float[]*): intensity [0,1].
    - **range** (*float[]*): range in m.
    - **azimuth** (*float[]*): azimuth in rad [-pi,pi].
    - **elevation** (*float[]*): elevation in rad [-pi/2, pi/2].


IsaacRenderVarToCpuPointer
--------------------------
    ['This node gets a pointer to a render var that is stored on the CPU']


**Inputs**
    - **renderVar** (*token*): Name of the renderVar.
    - **exec** (*execution*): Trigger.
    - **renderResults** (*uint64*): Render results pointer.

**Outputs**
    - **cpuPointer** (*uint64*): Pointer to render var on CPU.
    - **bufferSize** (*uint64*): Size (in bytes) of the buffer.
    - **exec** (*execution*): Executes when the event is received.


IsaacComputeRTXRadarPointCloud
------------------------------
    ['This node reads from the an RTX Radar sensor and holds point cloud data buffers']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **cpuPointer** (*uint64*): CPU Pointer to Radar render result.
    - **transform** (*matrixd[4]*): The matrix to transform the points by.

**Outputs**
    - **execOut** (*execution*): Output execution triggers when Radar sensor has data.
    - **transform** (*matrixd[4]*): The input matrix transformed from Radar to World.
    - **syncData** (*uint64*): Pointer to SyncData Sync primitives for syncing with model.
    - **sensorID** (*uchar*): Sensor Id for sensor that generated the scan.
    - **scanIdx** (*uchar*): Scan index for sensors with multi scan support.
    - **timeStampNS** (*uint64*): Scan timestamp in nanoseconds.
    - **cycleCnt** (*uint64*): Scan cycle count.
    - **maxRangeM** (*float*): The max unambiguous range for the scan.
    - **minVelMps** (*float*): The min unambiguous velocity for the scan.
    - **maxVelMps** (*float*): The max unambiguous velocity for the scan.
    - **minAzRad** (*float*): The min unambiguous azimuth for the scan.
    - **maxAzRad** (*float*): The max unambiguous azimuth for the scan.
    - **minElRad** (*float*): The min unambiguous elevation for the scan.
    - **maxElRad** (*float*): The max unambiguous elevation for the scan.
    - **numDetections** (*uint*): The number of valid detections in the array.
    - **pointCloudData** (*pointf[3][]*): Buffer of 3d points containing point cloud data in Radar coordinates.
    - **radialDistance** (*float[]*): Radial distance (m).
    - **radialVelocity** (*float[]*): Radial velocity (m/s).
    - **azimuth** (*float[]*): Azimuth angle (radians).
    - **elevation** (*float[]*): Angle of elevation (radians).
    - **rcs** (*float[]*): Radar cross section in decibels referenced to a square meter (dBsm).
    - **semanticId** (*uint[]*): semantic ID.
    - **materialId** (*uint[]*): material ID.
    - **objectId** (*uint[]*): object ID.


IsaacComputeRTXLidarFlatScan
----------------------------
    ['This node reads from the an RTX Lidar sensor and holds flat scan buffers']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **cpuPointer** (*uint64*): CPU Pointer to LiDAR render result.

**Outputs**
    - **execOut** (*execution*): Output execution triggers when lidar sensor has data.
    - **horizontalFov** (*float*): Horizontal Field of View in degrees.
    - **horizontalResolution** (*float*): Degrees in between rays for horizontal axis.
    - **depthRange** (*float[2]*): The min and max range for sensor to detect a hit [min, max].
    - **rotationRate** (*float*): Rotation rate of sensor in Hz.
    - **linearDepthData** (*float[]*): Buffer array containing linear depth data.
    - **intensitiesData** (*uchar[]*): Buffer array containing intensities data.
    - **numRows** (*int*): Number of rows in buffers.
    - **numCols** (*int*): Number of columns in buffers.
    - **azimuthRange** (*float[2]*): The azimuth range [min, max].


IsaacReadRTXLidarData
---------------------
    ['This node reads the data straight from the an RTX Lidar sensor.']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **cpuPointer** (*uint64*): CPU Pointer to LiDAR render result.
    - **keepOnlyPositiveDistance** (*bool*): Keep points only if the return distance is > 0. Default to True.

**Outputs**
    - **execOut** (*execution*): Output execution triggers when lidar sensor has data.
    - **intensity** (*float[]*): intensity [0,1].
    - **distance** (*float[]*): distance in m.
    - **azimuth** (*float[]*): azimuth in rad [-pi,pi].
    - **elevation** (*float[]*): elevation in rad [-pi/2, pi/2].
    - **velocityMs** (*pointf[3][]*): velocity at hit point in sensor coordinates [m/s].
    - **echoId** (*uint[]*): echo id in ascending order.
    - **emitterId** (*uint[]*): beam/laser detector id.
    - **beamId** (*uint[]*): beam/laser detector id.
    - **materialId** (*uint[]*): hit point material id.
    - **hitPointNormal** (*pointf[3][]*): hit point Normal.
    - **tick** (*uint[]*): tick of point.
    - **objectId** (*uint64[]*): hit point object id.
    - **timeStampNs** (*uint64[]*): absolute timeStamp in nano seconds.


IsaacReadIMU
------------
    Node that reads out IMU linear acceleration, angular velocity and orientation data


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **imuPrim** (*bundle*): Usd prim reference to the IMU prim.

**Outputs**
    - **execOut** (*execution*): Output execution triggers when sensor has data.
    - **linAcc** (*vectord[3]*): Linear acceleration IMU reading.
    - **angVel** (*vectord[3]*): Angular velocity IMU reading.
    - **orientation** (*quatd[4]*): Relative orientation as quaternion.


IsaacReadContactSensor
----------------------
    Node that reads out contact sensor data


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **csPrim** (*bundle*): USD prim reference to contact sensor prim.

**Outputs**
    - **execOut** (*execution*): Output execution triggers when sensor has data.
    - **inContact** (*bool*): Bool that registers current sensor contact.
    - **value** (*float*): Contact force value reading (N).


IsaacPrintRTXLidarInfo
----------------------
    ['process and print the raw RTX lidar data']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **cpuPointer** (*uint64*): Pointer to LiDAR render result.


IsaacPrintRTXRadarInfo
----------------------
    ['process and print the raw RTX Radar data']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **cpuPointer** (*uint64*): Pointer to Radar render result.