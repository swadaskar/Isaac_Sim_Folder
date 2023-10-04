


IsaacReadLidarBeams
-------------------
    ['This node reads from the lidar sensor and holds data buffers for a full scan']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **lidarPrim** (*bundle*): Usd prim reference to the lidar prim.

**Outputs**
    - **execOut** (*execution*): Output execution triggers when lidar sensor has completed a full scan.
    - **horizontalFov** (*float*): Horizontal Field of View in degrees.
    - **verticalFov** (*float*): Vertical Field of View in degrees.
    - **horizontalResolution** (*float*): Degrees in between rays for horizontal axis.
    - **verticalResolution** (*float*): Degrees in between rays for vertical axis.
    - **depthRange** (*float[2]*): The min and max range for sensor to detect a hit [min, max].
    - **rotationRate** (*float*): Rotation rate of sensor in Hz.
    - **linearDepthData** (*float[]*): Buffer array containing linear depth data.
    - **intensitiesData** (*uchar[]*): Buffer array containing intensities data.
    - **numRows** (*int*): Number of rows in buffers.
    - **numCols** (*int*): Number of columns in buffers.
    - **azimuthRange** (*float[2]*): The azimuth range [min, max].
    - **zenithRange** (*float[2]*): The zenith range [min, max].


IsaacReadLidarPointCloud
------------------------
    ['This node reads from the lidar sensor and holds point cloud data buffers for a full scan']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **lidarPrim** (*bundle*): Usd prim reference to the lidar prim.

**Outputs**
    - **execOut** (*execution*): Output execution triggers when lidar sensor has completed a full scan.
    - **pointCloudData** (*pointf[3][]*): Buffer of 3d points containing point cloud data.