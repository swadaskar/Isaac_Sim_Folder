


DebugDrawPointCloud
-------------------
    ['Take a point cloud as input and display it in the scene.']


**Inputs**
    - **execIn** (*execution*): The input execution port.
    - **depthTest** (*bool*): If true, the points will not render when behind other objects. Default to True.
    - **transform** (*matrixd[4]*): The matrix to transform the points by.
    - **pointCloudData** (*pointf[3][]*): Buffer of 3d points containing point cloud data. Default to [].
    - **color** (*colorf[4]*): Color of points. Default to [0.75, 0.75, 1, 1].
    - **width** (*float*): Size of points. Default to 0.02.