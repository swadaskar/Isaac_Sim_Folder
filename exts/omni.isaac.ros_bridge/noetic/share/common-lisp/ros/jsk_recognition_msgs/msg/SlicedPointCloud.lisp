; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude SlicedPointCloud.msg.html

(cl:defclass <SlicedPointCloud> (roslisp-msg-protocol:ros-message)
  ((point_cloud
    :reader point_cloud
    :initarg :point_cloud
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (slice_index
    :reader slice_index
    :initarg :slice_index
    :type cl:fixnum
    :initform 0)
   (sequence_id
    :reader sequence_id
    :initarg :sequence_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SlicedPointCloud (<SlicedPointCloud>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SlicedPointCloud>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SlicedPointCloud)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<SlicedPointCloud> is deprecated: use jsk_recognition_msgs-msg:SlicedPointCloud instead.")))

(cl:ensure-generic-function 'point_cloud-val :lambda-list '(m))
(cl:defmethod point_cloud-val ((m <SlicedPointCloud>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:point_cloud-val is deprecated.  Use jsk_recognition_msgs-msg:point_cloud instead.")
  (point_cloud m))

(cl:ensure-generic-function 'slice_index-val :lambda-list '(m))
(cl:defmethod slice_index-val ((m <SlicedPointCloud>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:slice_index-val is deprecated.  Use jsk_recognition_msgs-msg:slice_index instead.")
  (slice_index m))

(cl:ensure-generic-function 'sequence_id-val :lambda-list '(m))
(cl:defmethod sequence_id-val ((m <SlicedPointCloud>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:sequence_id-val is deprecated.  Use jsk_recognition_msgs-msg:sequence_id instead.")
  (sequence_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SlicedPointCloud>) ostream)
  "Serializes a message object of type '<SlicedPointCloud>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point_cloud) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'slice_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sequence_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SlicedPointCloud>) istream)
  "Deserializes a message object of type '<SlicedPointCloud>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point_cloud) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'slice_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sequence_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SlicedPointCloud>)))
  "Returns string type for a message object of type '<SlicedPointCloud>"
  "jsk_recognition_msgs/SlicedPointCloud")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SlicedPointCloud)))
  "Returns string type for a message object of type 'SlicedPointCloud"
  "jsk_recognition_msgs/SlicedPointCloud")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SlicedPointCloud>)))
  "Returns md5sum for a message object of type '<SlicedPointCloud>"
  "37376a48b034bf543f8cfd86844487c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SlicedPointCloud)))
  "Returns md5sum for a message object of type 'SlicedPointCloud"
  "37376a48b034bf543f8cfd86844487c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SlicedPointCloud>)))
  "Returns full string definition for message of type '<SlicedPointCloud>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 point_cloud~%uint8 slice_index~%uint8 sequence_id~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SlicedPointCloud)))
  "Returns full string definition for message of type 'SlicedPointCloud"
  (cl:format cl:nil "sensor_msgs/PointCloud2 point_cloud~%uint8 slice_index~%uint8 sequence_id~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SlicedPointCloud>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point_cloud))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SlicedPointCloud>))
  "Converts a ROS message object to a list"
  (cl:list 'SlicedPointCloud
    (cl:cons ':point_cloud (point_cloud msg))
    (cl:cons ':slice_index (slice_index msg))
    (cl:cons ':sequence_id (sequence_id msg))
))
