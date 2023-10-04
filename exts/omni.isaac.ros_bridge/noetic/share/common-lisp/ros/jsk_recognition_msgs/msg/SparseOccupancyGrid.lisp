; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude SparseOccupancyGrid.msg.html

(cl:defclass <SparseOccupancyGrid> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (origin_pose
    :reader origin_pose
    :initarg :origin_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (resolution
    :reader resolution
    :initarg :resolution
    :type cl:float
    :initform 0.0)
   (columns
    :reader columns
    :initarg :columns
    :type (cl:vector jsk_recognition_msgs-msg:SparseOccupancyGridColumn)
   :initform (cl:make-array 0 :element-type 'jsk_recognition_msgs-msg:SparseOccupancyGridColumn :initial-element (cl:make-instance 'jsk_recognition_msgs-msg:SparseOccupancyGridColumn))))
)

(cl:defclass SparseOccupancyGrid (<SparseOccupancyGrid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SparseOccupancyGrid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SparseOccupancyGrid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<SparseOccupancyGrid> is deprecated: use jsk_recognition_msgs-msg:SparseOccupancyGrid instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SparseOccupancyGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'origin_pose-val :lambda-list '(m))
(cl:defmethod origin_pose-val ((m <SparseOccupancyGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:origin_pose-val is deprecated.  Use jsk_recognition_msgs-msg:origin_pose instead.")
  (origin_pose m))

(cl:ensure-generic-function 'resolution-val :lambda-list '(m))
(cl:defmethod resolution-val ((m <SparseOccupancyGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:resolution-val is deprecated.  Use jsk_recognition_msgs-msg:resolution instead.")
  (resolution m))

(cl:ensure-generic-function 'columns-val :lambda-list '(m))
(cl:defmethod columns-val ((m <SparseOccupancyGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:columns-val is deprecated.  Use jsk_recognition_msgs-msg:columns instead.")
  (columns m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SparseOccupancyGrid>) ostream)
  "Serializes a message object of type '<SparseOccupancyGrid>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'origin_pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'resolution))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'columns))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'columns))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SparseOccupancyGrid>) istream)
  "Deserializes a message object of type '<SparseOccupancyGrid>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'origin_pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'resolution) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'columns) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'columns)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'jsk_recognition_msgs-msg:SparseOccupancyGridColumn))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SparseOccupancyGrid>)))
  "Returns string type for a message object of type '<SparseOccupancyGrid>"
  "jsk_recognition_msgs/SparseOccupancyGrid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SparseOccupancyGrid)))
  "Returns string type for a message object of type 'SparseOccupancyGrid"
  "jsk_recognition_msgs/SparseOccupancyGrid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SparseOccupancyGrid>)))
  "Returns md5sum for a message object of type '<SparseOccupancyGrid>"
  "497269ddab6058d0d4860f25dc49448f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SparseOccupancyGrid)))
  "Returns md5sum for a message object of type 'SparseOccupancyGrid"
  "497269ddab6058d0d4860f25dc49448f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SparseOccupancyGrid>)))
  "Returns full string definition for message of type '<SparseOccupancyGrid>"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose origin_pose~%float32 resolution~%~%SparseOccupancyGridColumn[] columns~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: jsk_recognition_msgs/SparseOccupancyGridColumn~%int32 column_index~%SparseOccupancyGridCell[] cells~%~%================================================================================~%MSG: jsk_recognition_msgs/SparseOccupancyGridCell~%int32 row_index~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SparseOccupancyGrid)))
  "Returns full string definition for message of type 'SparseOccupancyGrid"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose origin_pose~%float32 resolution~%~%SparseOccupancyGridColumn[] columns~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: jsk_recognition_msgs/SparseOccupancyGridColumn~%int32 column_index~%SparseOccupancyGridCell[] cells~%~%================================================================================~%MSG: jsk_recognition_msgs/SparseOccupancyGridCell~%int32 row_index~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SparseOccupancyGrid>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'origin_pose))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'columns) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SparseOccupancyGrid>))
  "Converts a ROS message object to a list"
  (cl:list 'SparseOccupancyGrid
    (cl:cons ':header (header msg))
    (cl:cons ':origin_pose (origin_pose msg))
    (cl:cons ':resolution (resolution msg))
    (cl:cons ':columns (columns msg))
))
