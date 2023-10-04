;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::ICPAlignWithBox)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'ICPAlignWithBox (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::ICPALIGNWITHBOX")
  (make-package "JSK_RECOGNITION_MSGS::ICPALIGNWITHBOX"))
(unless (find-package "JSK_RECOGNITION_MSGS::ICPALIGNWITHBOXREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::ICPALIGNWITHBOXREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::ICPALIGNWITHBOXRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::ICPALIGNWITHBOXRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))




(defclass jsk_recognition_msgs::ICPAlignWithBoxRequest
  :super ros::object
  :slots (_target_cloud _target_box ))

(defmethod jsk_recognition_msgs::ICPAlignWithBoxRequest
  (:init
   (&key
    ((:target_cloud __target_cloud) (instance sensor_msgs::PointCloud2 :init))
    ((:target_box __target_box) (instance jsk_recognition_msgs::BoundingBox :init))
    )
   (send-super :init)
   (setq _target_cloud __target_cloud)
   (setq _target_box __target_box)
   self)
  (:target_cloud
   (&rest __target_cloud)
   (if (keywordp (car __target_cloud))
       (send* _target_cloud __target_cloud)
     (progn
       (if __target_cloud (setq _target_cloud (car __target_cloud)))
       _target_cloud)))
  (:target_box
   (&rest __target_box)
   (if (keywordp (car __target_box))
       (send* _target_box __target_box)
     (progn
       (if __target_box (setq _target_box (car __target_box)))
       _target_box)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _target_cloud
    (send _target_cloud :serialization-length)
    ;; jsk_recognition_msgs/BoundingBox _target_box
    (send _target_box :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _target_cloud
       (send _target_cloud :serialize s)
     ;; jsk_recognition_msgs/BoundingBox _target_box
       (send _target_box :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _target_cloud
     (send _target_cloud :deserialize buf ptr-) (incf ptr- (send _target_cloud :serialization-length))
   ;; jsk_recognition_msgs/BoundingBox _target_box
     (send _target_box :deserialize buf ptr-) (incf ptr- (send _target_box :serialization-length))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::ICPAlignWithBoxResponse
  :super ros::object
  :slots (_result ))

(defmethod jsk_recognition_msgs::ICPAlignWithBoxResponse
  (:init
   (&key
    ((:result __result) (instance jsk_recognition_msgs::ICPResult :init))
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&rest __result)
   (if (keywordp (car __result))
       (send* _result __result)
     (progn
       (if __result (setq _result (car __result)))
       _result)))
  (:serialization-length
   ()
   (+
    ;; jsk_recognition_msgs/ICPResult _result
    (send _result :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; jsk_recognition_msgs/ICPResult _result
       (send _result :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; jsk_recognition_msgs/ICPResult _result
     (send _result :deserialize buf ptr-) (incf ptr- (send _result :serialization-length))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::ICPAlignWithBox
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::ICPAlignWithBox :md5sum-) "d23c790c7c4a5f0e959eba05efc9d29a")
(setf (get jsk_recognition_msgs::ICPAlignWithBox :datatype-) "jsk_recognition_msgs/ICPAlignWithBox")
(setf (get jsk_recognition_msgs::ICPAlignWithBox :request) jsk_recognition_msgs::ICPAlignWithBoxRequest)
(setf (get jsk_recognition_msgs::ICPAlignWithBox :response) jsk_recognition_msgs::ICPAlignWithBoxResponse)

(defmethod jsk_recognition_msgs::ICPAlignWithBoxRequest
  (:response () (instance jsk_recognition_msgs::ICPAlignWithBoxResponse :init)))

(setf (get jsk_recognition_msgs::ICPAlignWithBoxRequest :md5sum-) "d23c790c7c4a5f0e959eba05efc9d29a")
(setf (get jsk_recognition_msgs::ICPAlignWithBoxRequest :datatype-) "jsk_recognition_msgs/ICPAlignWithBoxRequest")
(setf (get jsk_recognition_msgs::ICPAlignWithBoxRequest :definition-)
      "sensor_msgs/PointCloud2 target_cloud
jsk_recognition_msgs/BoundingBox target_box

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

================================================================================
MSG: jsk_recognition_msgs/BoundingBox
# BoundingBox represents a oriented bounding box.
Header header
geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)
# You can use this field to hold value such as likelihood
float32 value
uint32 label

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
jsk_recognition_msgs/ICPResult result


================================================================================
MSG: jsk_recognition_msgs/ICPResult
Header header
geometry_msgs/Pose pose
string name
float64 score
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get jsk_recognition_msgs::ICPAlignWithBoxResponse :md5sum-) "d23c790c7c4a5f0e959eba05efc9d29a")
(setf (get jsk_recognition_msgs::ICPAlignWithBoxResponse :datatype-) "jsk_recognition_msgs/ICPAlignWithBoxResponse")
(setf (get jsk_recognition_msgs::ICPAlignWithBoxResponse :definition-)
      "sensor_msgs/PointCloud2 target_cloud
jsk_recognition_msgs/BoundingBox target_box

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

================================================================================
MSG: jsk_recognition_msgs/BoundingBox
# BoundingBox represents a oriented bounding box.
Header header
geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)
# You can use this field to hold value such as likelihood
float32 value
uint32 label

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
jsk_recognition_msgs/ICPResult result


================================================================================
MSG: jsk_recognition_msgs/ICPResult
Header header
geometry_msgs/Pose pose
string name
float64 score
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :jsk_recognition_msgs/ICPAlignWithBox "d23c790c7c4a5f0e959eba05efc9d29a")


