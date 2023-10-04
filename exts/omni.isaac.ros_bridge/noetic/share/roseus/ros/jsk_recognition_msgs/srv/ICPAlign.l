;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::ICPAlign)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'ICPAlign (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::ICPALIGN")
  (make-package "JSK_RECOGNITION_MSGS::ICPALIGN"))
(unless (find-package "JSK_RECOGNITION_MSGS::ICPALIGNREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::ICPALIGNREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::ICPALIGNRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::ICPALIGNRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))




(defclass jsk_recognition_msgs::ICPAlignRequest
  :super ros::object
  :slots (_reference_cloud _target_cloud ))

(defmethod jsk_recognition_msgs::ICPAlignRequest
  (:init
   (&key
    ((:reference_cloud __reference_cloud) (instance sensor_msgs::PointCloud2 :init))
    ((:target_cloud __target_cloud) (instance sensor_msgs::PointCloud2 :init))
    )
   (send-super :init)
   (setq _reference_cloud __reference_cloud)
   (setq _target_cloud __target_cloud)
   self)
  (:reference_cloud
   (&rest __reference_cloud)
   (if (keywordp (car __reference_cloud))
       (send* _reference_cloud __reference_cloud)
     (progn
       (if __reference_cloud (setq _reference_cloud (car __reference_cloud)))
       _reference_cloud)))
  (:target_cloud
   (&rest __target_cloud)
   (if (keywordp (car __target_cloud))
       (send* _target_cloud __target_cloud)
     (progn
       (if __target_cloud (setq _target_cloud (car __target_cloud)))
       _target_cloud)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _reference_cloud
    (send _reference_cloud :serialization-length)
    ;; sensor_msgs/PointCloud2 _target_cloud
    (send _target_cloud :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _reference_cloud
       (send _reference_cloud :serialize s)
     ;; sensor_msgs/PointCloud2 _target_cloud
       (send _target_cloud :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _reference_cloud
     (send _reference_cloud :deserialize buf ptr-) (incf ptr- (send _reference_cloud :serialization-length))
   ;; sensor_msgs/PointCloud2 _target_cloud
     (send _target_cloud :deserialize buf ptr-) (incf ptr- (send _target_cloud :serialization-length))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::ICPAlignResponse
  :super ros::object
  :slots (_result ))

(defmethod jsk_recognition_msgs::ICPAlignResponse
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

(defclass jsk_recognition_msgs::ICPAlign
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::ICPAlign :md5sum-) "356a3a6001b7560173247ee06893ddba")
(setf (get jsk_recognition_msgs::ICPAlign :datatype-) "jsk_recognition_msgs/ICPAlign")
(setf (get jsk_recognition_msgs::ICPAlign :request) jsk_recognition_msgs::ICPAlignRequest)
(setf (get jsk_recognition_msgs::ICPAlign :response) jsk_recognition_msgs::ICPAlignResponse)

(defmethod jsk_recognition_msgs::ICPAlignRequest
  (:response () (instance jsk_recognition_msgs::ICPAlignResponse :init)))

(setf (get jsk_recognition_msgs::ICPAlignRequest :md5sum-) "356a3a6001b7560173247ee06893ddba")
(setf (get jsk_recognition_msgs::ICPAlignRequest :datatype-) "jsk_recognition_msgs/ICPAlignRequest")
(setf (get jsk_recognition_msgs::ICPAlignRequest :definition-)
      "sensor_msgs/PointCloud2 reference_cloud
sensor_msgs/PointCloud2 target_cloud

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

(setf (get jsk_recognition_msgs::ICPAlignResponse :md5sum-) "356a3a6001b7560173247ee06893ddba")
(setf (get jsk_recognition_msgs::ICPAlignResponse :datatype-) "jsk_recognition_msgs/ICPAlignResponse")
(setf (get jsk_recognition_msgs::ICPAlignResponse :definition-)
      "sensor_msgs/PointCloud2 reference_cloud
sensor_msgs/PointCloud2 target_cloud

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



(provide :jsk_recognition_msgs/ICPAlign "356a3a6001b7560173247ee06893ddba")


