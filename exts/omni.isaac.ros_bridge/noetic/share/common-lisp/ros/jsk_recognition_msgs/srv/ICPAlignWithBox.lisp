; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude ICPAlignWithBox-request.msg.html

(cl:defclass <ICPAlignWithBox-request> (roslisp-msg-protocol:ros-message)
  ((target_cloud
    :reader target_cloud
    :initarg :target_cloud
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (target_box
    :reader target_box
    :initarg :target_box
    :type jsk_recognition_msgs-msg:BoundingBox
    :initform (cl:make-instance 'jsk_recognition_msgs-msg:BoundingBox)))
)

(cl:defclass ICPAlignWithBox-request (<ICPAlignWithBox-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ICPAlignWithBox-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ICPAlignWithBox-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<ICPAlignWithBox-request> is deprecated: use jsk_recognition_msgs-srv:ICPAlignWithBox-request instead.")))

(cl:ensure-generic-function 'target_cloud-val :lambda-list '(m))
(cl:defmethod target_cloud-val ((m <ICPAlignWithBox-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:target_cloud-val is deprecated.  Use jsk_recognition_msgs-srv:target_cloud instead.")
  (target_cloud m))

(cl:ensure-generic-function 'target_box-val :lambda-list '(m))
(cl:defmethod target_box-val ((m <ICPAlignWithBox-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:target_box-val is deprecated.  Use jsk_recognition_msgs-srv:target_box instead.")
  (target_box m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ICPAlignWithBox-request>) ostream)
  "Serializes a message object of type '<ICPAlignWithBox-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_cloud) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_box) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ICPAlignWithBox-request>) istream)
  "Deserializes a message object of type '<ICPAlignWithBox-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_cloud) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_box) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ICPAlignWithBox-request>)))
  "Returns string type for a service object of type '<ICPAlignWithBox-request>"
  "jsk_recognition_msgs/ICPAlignWithBoxRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ICPAlignWithBox-request)))
  "Returns string type for a service object of type 'ICPAlignWithBox-request"
  "jsk_recognition_msgs/ICPAlignWithBoxRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ICPAlignWithBox-request>)))
  "Returns md5sum for a message object of type '<ICPAlignWithBox-request>"
  "d23c790c7c4a5f0e959eba05efc9d29a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ICPAlignWithBox-request)))
  "Returns md5sum for a message object of type 'ICPAlignWithBox-request"
  "d23c790c7c4a5f0e959eba05efc9d29a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ICPAlignWithBox-request>)))
  "Returns full string definition for message of type '<ICPAlignWithBox-request>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 target_cloud~%jsk_recognition_msgs/BoundingBox target_box~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: jsk_recognition_msgs/BoundingBox~%# BoundingBox represents a oriented bounding box.~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)~%# You can use this field to hold value such as likelihood~%float32 value~%uint32 label~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ICPAlignWithBox-request)))
  "Returns full string definition for message of type 'ICPAlignWithBox-request"
  (cl:format cl:nil "sensor_msgs/PointCloud2 target_cloud~%jsk_recognition_msgs/BoundingBox target_box~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: jsk_recognition_msgs/BoundingBox~%# BoundingBox represents a oriented bounding box.~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)~%# You can use this field to hold value such as likelihood~%float32 value~%uint32 label~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ICPAlignWithBox-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_cloud))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_box))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ICPAlignWithBox-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ICPAlignWithBox-request
    (cl:cons ':target_cloud (target_cloud msg))
    (cl:cons ':target_box (target_box msg))
))
;//! \htmlinclude ICPAlignWithBox-response.msg.html

(cl:defclass <ICPAlignWithBox-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type jsk_recognition_msgs-msg:ICPResult
    :initform (cl:make-instance 'jsk_recognition_msgs-msg:ICPResult)))
)

(cl:defclass ICPAlignWithBox-response (<ICPAlignWithBox-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ICPAlignWithBox-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ICPAlignWithBox-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<ICPAlignWithBox-response> is deprecated: use jsk_recognition_msgs-srv:ICPAlignWithBox-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <ICPAlignWithBox-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:result-val is deprecated.  Use jsk_recognition_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ICPAlignWithBox-response>) ostream)
  "Serializes a message object of type '<ICPAlignWithBox-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'result) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ICPAlignWithBox-response>) istream)
  "Deserializes a message object of type '<ICPAlignWithBox-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'result) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ICPAlignWithBox-response>)))
  "Returns string type for a service object of type '<ICPAlignWithBox-response>"
  "jsk_recognition_msgs/ICPAlignWithBoxResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ICPAlignWithBox-response)))
  "Returns string type for a service object of type 'ICPAlignWithBox-response"
  "jsk_recognition_msgs/ICPAlignWithBoxResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ICPAlignWithBox-response>)))
  "Returns md5sum for a message object of type '<ICPAlignWithBox-response>"
  "d23c790c7c4a5f0e959eba05efc9d29a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ICPAlignWithBox-response)))
  "Returns md5sum for a message object of type 'ICPAlignWithBox-response"
  "d23c790c7c4a5f0e959eba05efc9d29a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ICPAlignWithBox-response>)))
  "Returns full string definition for message of type '<ICPAlignWithBox-response>"
  (cl:format cl:nil "jsk_recognition_msgs/ICPResult result~%~%~%================================================================================~%MSG: jsk_recognition_msgs/ICPResult~%Header header~%geometry_msgs/Pose pose~%string name~%float64 score~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ICPAlignWithBox-response)))
  "Returns full string definition for message of type 'ICPAlignWithBox-response"
  (cl:format cl:nil "jsk_recognition_msgs/ICPResult result~%~%~%================================================================================~%MSG: jsk_recognition_msgs/ICPResult~%Header header~%geometry_msgs/Pose pose~%string name~%float64 score~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ICPAlignWithBox-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ICPAlignWithBox-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ICPAlignWithBox-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ICPAlignWithBox)))
  'ICPAlignWithBox-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ICPAlignWithBox)))
  'ICPAlignWithBox-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ICPAlignWithBox)))
  "Returns string type for a service object of type '<ICPAlignWithBox>"
  "jsk_recognition_msgs/ICPAlignWithBox")