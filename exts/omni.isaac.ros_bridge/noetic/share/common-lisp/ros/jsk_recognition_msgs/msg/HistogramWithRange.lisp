; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude HistogramWithRange.msg.html

(cl:defclass <HistogramWithRange> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (bins
    :reader bins
    :initarg :bins
    :type (cl:vector jsk_recognition_msgs-msg:HistogramWithRangeBin)
   :initform (cl:make-array 0 :element-type 'jsk_recognition_msgs-msg:HistogramWithRangeBin :initial-element (cl:make-instance 'jsk_recognition_msgs-msg:HistogramWithRangeBin))))
)

(cl:defclass HistogramWithRange (<HistogramWithRange>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HistogramWithRange>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HistogramWithRange)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<HistogramWithRange> is deprecated: use jsk_recognition_msgs-msg:HistogramWithRange instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HistogramWithRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'bins-val :lambda-list '(m))
(cl:defmethod bins-val ((m <HistogramWithRange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:bins-val is deprecated.  Use jsk_recognition_msgs-msg:bins instead.")
  (bins m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HistogramWithRange>) ostream)
  "Serializes a message object of type '<HistogramWithRange>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'bins))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'bins))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HistogramWithRange>) istream)
  "Deserializes a message object of type '<HistogramWithRange>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'bins) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'bins)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'jsk_recognition_msgs-msg:HistogramWithRangeBin))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HistogramWithRange>)))
  "Returns string type for a message object of type '<HistogramWithRange>"
  "jsk_recognition_msgs/HistogramWithRange")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HistogramWithRange)))
  "Returns string type for a message object of type 'HistogramWithRange"
  "jsk_recognition_msgs/HistogramWithRange")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HistogramWithRange>)))
  "Returns md5sum for a message object of type '<HistogramWithRange>"
  "aa922a22cbdcbdcc1146f30df5b605b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HistogramWithRange)))
  "Returns md5sum for a message object of type 'HistogramWithRange"
  "aa922a22cbdcbdcc1146f30df5b605b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HistogramWithRange>)))
  "Returns full string definition for message of type '<HistogramWithRange>"
  (cl:format cl:nil "Header header~%HistogramWithRangeBin[] bins~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/HistogramWithRangeBin~%float64 min_value~%float64 max_value~%uint32 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HistogramWithRange)))
  "Returns full string definition for message of type 'HistogramWithRange"
  (cl:format cl:nil "Header header~%HistogramWithRangeBin[] bins~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/HistogramWithRangeBin~%float64 min_value~%float64 max_value~%uint32 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HistogramWithRange>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'bins) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HistogramWithRange>))
  "Converts a ROS message object to a list"
  (cl:list 'HistogramWithRange
    (cl:cons ':header (header msg))
    (cl:cons ':bins (bins msg))
))
