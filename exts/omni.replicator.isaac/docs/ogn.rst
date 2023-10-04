


OgnCountIndices
---------------
    ['Get length of array']


**Inputs**
    - **indices** (*int[]*): indices to count.

**Outputs**
    - **count** (*uint*): Count.


Pose
----
    This node outputs the poses of assets with semantic labels


**Inputs**
    - **exec** (*execution*): Trigger.
    - **includeOccludedPrims** (*bool*): Set to True if poses (and if enabled, centers) of fully occluded/out-of-frame semantic entities should be output.
    - **getCenters** (*bool*): Set to True if producing center coordinates of every semantic entity projected in the image space.
    - **cameraRotation** (*float[]*): Rotation of the desired camera frame from the default camera frame, as XYZ Euler angles.
    - **imageWidth** (*uint*): Width of the viewport.
    - **imageHeight** (*uint*): Height of the viewport.
    - **cameraViewTransform** (*matrixd[4]*): Camera view matrix.
    - **cameraProjection** (*matrixd[4]*): Camera projection matrix.
    - **sdIMNumSemantics** (*uint*): Number of semantic entities in the semantic arrays.
    - **sdIMNumSemanticTokens** (*uint*): Number of semantics token including the semantic entity path, the semantic entity types and if the number of semantic types is greater than one.
    - **sdIMInstanceSemanticMap** (*uchar[]*): Raw array of uint16_t of size sdIMNumInstances*sdIMMaxSemanticHierarchyDepth containing the mapping from the instances index to their inherited semantic entities.
    - **sdIMSemanticTokenMap** (*token[]*): Semantic array of token of size numSemantics * numSemanticTypes containing the mapping from the semantic entities to the semantic entity path and semantic types.
    - **sdIMMinSemanticIndex** (*uint*): Semantic id of the first instance in the instance arrays.
    - **sdIMMaxSemanticHierarchyDepth** (*uint*): Maximal number of semantic entities inherited by an instance.
    - **sdIMSemanticWorldTransform** (*float[]*): Semantic array of 4x4 float matrices containing the transform from local to world space for every semantic entity.
    - **data** (*uchar[]*): Buffer array data. Default to [].
    - **bufferSize** (*uint*): Size (in bytes) of the buffer (0 if the input is a texture).
    - **swhFrameNumber** (*uint64*): Frame number.
    - **semanticTypes** (*token[]*): Semantic Types to consider. Default to ['class'].

**Outputs**
    - **exec** (*execution*): Trigger.
    - **data** (*uchar[]*): Semantic array of 4x4 float matrices containing the transform from local to view space for every semantic entity. Additionally, an optional semantic array of float[2] vectors containing the center coordinates of every semantic entity projected in the image space.
    - **idToLabels** (*string*): Mapping from id to semantic labels.
    - **primPaths** (*token[]*): Prim paths corresponding to each pose.
    - **swhFrameNumber** (*uint64*): Frame number.
    - **bufferSize** (*uint*): Size (in bytes) of the buffer (0 if the input is a texture).
    - **height** (*uint*): Shape of the data.
    - **width** (*uint*): Shape of the data.


Dope
----
    Gets poses of assets as required by ground truth file for DOPE training


**Inputs**
    - **exec** (*execution*): Trigger.
    - **cameraModel** (*token*): Camera model (pinhole or fisheye models).
    - **cameraViewTransform** (*matrixd[4]*): Camera view matrix.
    - **cameraProjection** (*matrixd[4]*): Camera projection matrix.
    - **cameraFisheyeNominalWidth** (*int*): Camera fisheye nominal width.
    - **cameraFisheyeNominalHeight** (*int*): Camera fisheye nominal height.
    - **cameraFisheyeOpticalCentre** (*float[2]*): Camera fisheye optical centre.
    - **cameraFisheyeMaxFOV** (*float*): Camera fisheye maximum field of view.
    - **cameraFisheyePolynomial** (*float[]*): Camera fisheye polynomial.
    - **cameraNearFar** (*float[2]*): Camera near/far clipping range.
    - **horizontalAperture** (*float*): Horizontal aperture of camera.
    - **focalLength** (*float*): Camera fisheye maximum field of view.
    - **cameraRotation** (*float[3]*): Camera rotation in euler angles.
    - **width** (*uint*): Width of the viewport.
    - **height** (*uint*): Height of the viewport.
    - **boundingBox3d** (*uchar[]*): 3d bounding box data.
    - **occlusion** (*uchar[]*): Occlusion data.
    - **swhFrameNumber** (*uint64*): Frame number.
    - **sdIMNumSemantics** (*uint*): Number of semantic entities in the semantic arrays.
    - **sdIMNumSemanticTokens** (*uint*): Number of semantics token including the semantic entity path, the semantic entity types and if the number of semantic types is greater than one.
    - **sdIMInstanceSemanticMap** (*uchar[]*): Raw array of uint16_t of size sdIMNumInstances*sdIMMaxSemanticHierarchyDepth containing the mapping from the instances index to their inherited semantic entities.
    - **sdIMSemanticTokenMap** (*token[]*): Semantic array of token of size numSemantics * numSemanticTypes containing the mapping from the semantic entities to the semantic entity path and semantic types.
    - **sdIMMinSemanticIndex** (*uint*): Semantic id of the first instance in the instance arrays.
    - **sdIMMaxSemanticHierarchyDepth** (*uint*): Maximal number of semantic entities inherited by an instance.
    - **semanticTypes** (*token[]*): Semantic Types to consider. Default to ['class'].

**Outputs**
    - **exec** (*execution*): Trigger.
    - **data** (*uchar[]*): Semantic array of 4x4 float matrices containing the transform from local to view space for every semantic entity. Additionally, an optional semantic array of float[2] vectors containing the center coordinates of every semantic entity projected in the image space.
    - **idToLabels** (*string*): Mapping from id to semantic labels.
    - **swhFrameNumber** (*uint64*): Frame number.
    - **bufferSize** (*uint*): Size (in bytes) of the buffer (0 if the input is a texture).
    - **height** (*uint*): Shape of the data.
    - **width** (*uint*): Shape of the data.


OgnWritePhysicsSimulationContext
--------------------------------
    This node writes physics attributes to TensorAPI views


**Inputs**
    - **prims** (*string*): Name of registered view to randomize.
    - **attribute** (*string*): Name of attribute that is to be written.
    - **operation** (*string*): Type of randomization operation to be applied.
    - **distribution** (*string*): Type of distribution used to sample values.
    - **dist_param_1** (*float[]*): Distribution parameter 1.
    - **dist_param_2** (*float[]*): Distribution parameter 2.
    - **num_buckets** (*int*): Number of buckets to randomize from.
    - **values** (*float[]*): Values to be assigned to the physics attribute.
    - **indices** (*int[]*): Indices of the environments to assign the physics attribute.
    - **on_reset** (*bool*): indicates whether an on_reset context triggered the execution.
    - **execIn** (*execution*): exec.

**Outputs**
    - **execOut** (*execution*): exec.


OgnWritePhysicsRigidPrimView
----------------------------
    This node writes physics attributes to TensorAPI views


**Inputs**
    - **prims** (*string*): Name of registered view to randomize.
    - **attribute** (*string*): Name of attribute that is to be written.
    - **operation** (*string*): Type of randomization operation to be applied.
    - **distribution** (*string*): Type of distribution used to sample values.
    - **dist_param_1** (*float[]*): Distribution parameter 1.
    - **dist_param_2** (*float[]*): Distribution parameter 2.
    - **num_buckets** (*int*): Number of buckets to randomize from.
    - **values** (*float[]*): Values to be assigned to the physics attribute.
    - **indices** (*int[]*): Indices of the environments to assign the physics attribute.
    - **on_reset** (*bool*): indicates whether an on_reset context triggered the execution.
    - **execIn** (*execution*): exec.

**Outputs**
    - **execOut** (*execution*): exec.


Random3f
--------
    This node outputs the poses of assets with semantic labels


**Inputs**
    - **minimum** (*float[3]*): minimum range for randomization.
    - **maximum** (*float[3]*): minimum range for randomization.

**Outputs**
    - **output** (*float[3]*): Random float 3.


OgnOnRLFrame
------------
    ['Triggered every frame in an Rl setting']


**Inputs**
    - **run** (*bool*): Run.
    - **num_envs** (*int*): number of RL environments.

**Outputs**
    - **execOut** (*execution*): Output Execution.
    - **resetInds** (*int[]*): indices of environments to be reset.
    - **frameNum** (*int[]*): frame number for every environment.


OgnIntervalFiltering
--------------------
    Outputs indices if their frame count is a multiple of the interval


**Inputs**
    - **interval** (*int*): randomization will take place once every `interval` frames.
    - **frameCounts** (*int[]*): the current frame number for every environment.
    - **indices** (*int[]*): a list of indices to use in case of ignoring interval.
    - **ignoreInterval** (*bool*): if true, will just pass indices to downstream node. Default to False.
    - **execIn** (*execution*): exec.

**Outputs**
    - **execOut** (*execution*): exec.
    - **indices** (*int[]*): the indices that are at the interval and need to be randomized.
    - **on_reset** (*bool*): indicates whether an on_reset context triggered the execution.


OgnWritePhysicsArticulationView
-------------------------------
    This node writes physics attributes to TensorAPI views


**Inputs**
    - **prims** (*string*): Name of registered view to randomize.
    - **attribute** (*string*): Name of attribute that is to be written.
    - **operation** (*string*): Type of randomization operation to be applied.
    - **distribution** (*string*): Type of distribution used to sample values.
    - **dist_param_1** (*float[]*): Distribution parameter 1.
    - **dist_param_2** (*float[]*): Distribution parameter 2.
    - **num_buckets** (*int*): Number of buckets to randomize from.
    - **values** (*float[]*): Values to be assigned to the physics attribute.
    - **indices** (*int[]*): Indices of the environments to assign the physics attribute.
    - **on_reset** (*bool*): indicates whether an on_reset context triggered the execution.
    - **execIn** (*execution*): exec.

**Outputs**
    - **execOut** (*execution*): exec.