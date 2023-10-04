; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude ClusterPointIndices.msg.html

(cl:defclass <ClusterPointIndices> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cluster_indices
    :reader cluster_indices
    :initarg :cluster_indices
    :type (cl:vector pcl_msgs-msg:PointIndices)
   :initform (cl:make-array 0 :element-type 'pcl_msgs-msg:PointIndices :initial-element (cl:make-instance 'pcl_msgs-msg:PointIndices))))
)

(cl:defclass ClusterPointIndices (<ClusterPointIndices>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ClusterPointIndices>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ClusterPointIndices)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<ClusterPointIndices> is deprecated: use jsk_recognition_msgs-msg:ClusterPointIndices instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ClusterPointIndices>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cluster_indices-val :lambda-list '(m))
(cl:defmethod cluster_indices-val ((m <ClusterPointIndices>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:cluster_indices-val is deprecated.  Use jsk_recognition_msgs-msg:cluster_indices instead.")
  (cluster_indices m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ClusterPointIndices>) ostream)
  "Serializes a message object of type '<ClusterPointIndices>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cluster_indices))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cluster_indices))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ClusterPointIndices>) istream)
  "Deserializes a message object of type '<ClusterPointIndices>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cluster_indices) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cluster_indices)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pcl_msgs-msg:PointIndices))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ClusterPointIndices>)))
  "Returns string type for a message object of type '<ClusterPointIndices>"
  "jsk_recognition_msgs/ClusterPointIndices")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClusterPointIndices)))
  "Returns string type for a message object of type 'ClusterPointIndices"
  "jsk_recognition_msgs/ClusterPointIndices")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ClusterPointIndices>)))
  "Returns md5sum for a message object of type '<ClusterPointIndices>"
  "d43e94ea5e491effac7685a42b7b9d14")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ClusterPointIndices)))
  "Returns md5sum for a message object of type 'ClusterPointIndices"
  "d43e94ea5e491effac7685a42b7b9d14")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ClusterPointIndices>)))
  "Returns full string definition for message of type '<ClusterPointIndices>"
  (cl:format cl:nil "# ClusterPointIndices is used to represent segmentation result.~%# Simply put, ClusterPointIndices is a list of PointIndices.~%Header header~%pcl_msgs/PointIndices[] cluster_indices~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pcl_msgs/PointIndices~%Header header~%int32[] indices~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ClusterPointIndices)))
  "Returns full string definition for message of type 'ClusterPointIndices"
  (cl:format cl:nil "# ClusterPointIndices is used to represent segmentation result.~%# Simply put, ClusterPointIndices is a list of PointIndices.~%Header header~%pcl_msgs/PointIndices[] cluster_indices~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pcl_msgs/PointIndices~%Header header~%int32[] indices~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ClusterPointIndices>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cluster_indices) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ClusterPointIndices>))
  "Converts a ROS message object to a list"
  (cl:list 'ClusterPointIndices
    (cl:cons ':header (header msg))
    (cl:cons ':cluster_indices (cluster_indices msg))
))
