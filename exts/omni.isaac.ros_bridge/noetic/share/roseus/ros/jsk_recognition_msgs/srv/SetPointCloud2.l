;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SetPointCloud2)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SetPointCloud2 (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SETPOINTCLOUD2")
  (make-package "JSK_RECOGNITION_MSGS::SETPOINTCLOUD2"))
(unless (find-package "JSK_RECOGNITION_MSGS::SETPOINTCLOUD2REQUEST")
  (make-package "JSK_RECOGNITION_MSGS::SETPOINTCLOUD2REQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::SETPOINTCLOUD2RESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::SETPOINTCLOUD2RESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))




(defclass jsk_recognition_msgs::SetPointCloud2Request
  :super ros::object
  :slots (_cloud _name ))

(defmethod jsk_recognition_msgs::SetPointCloud2Request
  (:init
   (&key
    ((:cloud __cloud) (instance sensor_msgs::PointCloud2 :init))
    ((:name __name) "")
    )
   (send-super :init)
   (setq _cloud __cloud)
   (setq _name (string __name))
   self)
  (:cloud
   (&rest __cloud)
   (if (keywordp (car __cloud))
       (send* _cloud __cloud)
     (progn
       (if __cloud (setq _cloud (car __cloud)))
       _cloud)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _cloud
    (send _cloud :serialization-length)
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _cloud
       (send _cloud :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _cloud
     (send _cloud :deserialize buf ptr-) (incf ptr- (send _cloud :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::SetPointCloud2Response
  :super ros::object
  :slots (_output ))

(defmethod jsk_recognition_msgs::SetPointCloud2Response
  (:init
   (&key
    ((:output __output) "")
    )
   (send-super :init)
   (setq _output (string __output))
   self)
  (:output
   (&optional __output)
   (if __output (setq _output __output)) _output)
  (:serialization-length
   ()
   (+
    ;; string _output
    4 (length _output)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _output
       (write-long (length _output) s) (princ _output s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _output
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _output (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::SetPointCloud2
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::SetPointCloud2 :md5sum-) "f978291b9350d081a884434370be18a5")
(setf (get jsk_recognition_msgs::SetPointCloud2 :datatype-) "jsk_recognition_msgs/SetPointCloud2")
(setf (get jsk_recognition_msgs::SetPointCloud2 :request) jsk_recognition_msgs::SetPointCloud2Request)
(setf (get jsk_recognition_msgs::SetPointCloud2 :response) jsk_recognition_msgs::SetPointCloud2Response)

(defmethod jsk_recognition_msgs::SetPointCloud2Request
  (:response () (instance jsk_recognition_msgs::SetPointCloud2Response :init)))

(setf (get jsk_recognition_msgs::SetPointCloud2Request :md5sum-) "f978291b9350d081a884434370be18a5")
(setf (get jsk_recognition_msgs::SetPointCloud2Request :datatype-) "jsk_recognition_msgs/SetPointCloud2Request")
(setf (get jsk_recognition_msgs::SetPointCloud2Request :definition-)
      "sensor_msgs/PointCloud2 cloud
string name

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
string output

")

(setf (get jsk_recognition_msgs::SetPointCloud2Response :md5sum-) "f978291b9350d081a884434370be18a5")
(setf (get jsk_recognition_msgs::SetPointCloud2Response :datatype-) "jsk_recognition_msgs/SetPointCloud2Response")
(setf (get jsk_recognition_msgs::SetPointCloud2Response :definition-)
      "sensor_msgs/PointCloud2 cloud
string name

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
string output

")



(provide :jsk_recognition_msgs/SetPointCloud2 "f978291b9350d081a884434370be18a5")


