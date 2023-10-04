; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude Circle2DArray.msg.html

(cl:defclass <Circle2DArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (circles
    :reader circles
    :initarg :circles
    :type (cl:vector jsk_recognition_msgs-msg:Circle2D)
   :initform (cl:make-array 0 :element-type 'jsk_recognition_msgs-msg:Circle2D :initial-element (cl:make-instance 'jsk_recognition_msgs-msg:Circle2D))))
)

(cl:defclass Circle2DArray (<Circle2DArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Circle2DArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Circle2DArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<Circle2DArray> is deprecated: use jsk_recognition_msgs-msg:Circle2DArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Circle2DArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'circles-val :lambda-list '(m))
(cl:defmethod circles-val ((m <Circle2DArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:circles-val is deprecated.  Use jsk_recognition_msgs-msg:circles instead.")
  (circles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Circle2DArray>) ostream)
  "Serializes a message object of type '<Circle2DArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'circles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'circles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Circle2DArray>) istream)
  "Deserializes a message object of type '<Circle2DArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'circles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'circles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'jsk_recognition_msgs-msg:Circle2D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Circle2DArray>)))
  "Returns string type for a message object of type '<Circle2DArray>"
  "jsk_recognition_msgs/Circle2DArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Circle2DArray)))
  "Returns string type for a message object of type 'Circle2DArray"
  "jsk_recognition_msgs/Circle2DArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Circle2DArray>)))
  "Returns md5sum for a message object of type '<Circle2DArray>"
  "7c3f0f08758538a1e02ce600cf99132f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Circle2DArray)))
  "Returns md5sum for a message object of type 'Circle2DArray"
  "7c3f0f08758538a1e02ce600cf99132f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Circle2DArray>)))
  "Returns full string definition for message of type '<Circle2DArray>"
  (cl:format cl:nil "Header header~%Circle2D[] circles~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/Circle2D~%Header header~%float64 radius~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Circle2DArray)))
  "Returns full string definition for message of type 'Circle2DArray"
  (cl:format cl:nil "Header header~%Circle2D[] circles~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/Circle2D~%Header header~%float64 radius~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Circle2DArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'circles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Circle2DArray>))
  "Converts a ROS message object to a list"
  (cl:list 'Circle2DArray
    (cl:cons ':header (header msg))
    (cl:cons ':circles (circles msg))
))
