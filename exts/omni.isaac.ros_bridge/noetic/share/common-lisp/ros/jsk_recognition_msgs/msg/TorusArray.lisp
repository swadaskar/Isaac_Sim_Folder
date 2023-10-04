; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude TorusArray.msg.html

(cl:defclass <TorusArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (toruses
    :reader toruses
    :initarg :toruses
    :type (cl:vector jsk_recognition_msgs-msg:Torus)
   :initform (cl:make-array 0 :element-type 'jsk_recognition_msgs-msg:Torus :initial-element (cl:make-instance 'jsk_recognition_msgs-msg:Torus))))
)

(cl:defclass TorusArray (<TorusArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TorusArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TorusArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<TorusArray> is deprecated: use jsk_recognition_msgs-msg:TorusArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TorusArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'toruses-val :lambda-list '(m))
(cl:defmethod toruses-val ((m <TorusArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:toruses-val is deprecated.  Use jsk_recognition_msgs-msg:toruses instead.")
  (toruses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TorusArray>) ostream)
  "Serializes a message object of type '<TorusArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'toruses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'toruses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TorusArray>) istream)
  "Deserializes a message object of type '<TorusArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'toruses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'toruses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'jsk_recognition_msgs-msg:Torus))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TorusArray>)))
  "Returns string type for a message object of type '<TorusArray>"
  "jsk_recognition_msgs/TorusArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TorusArray)))
  "Returns string type for a message object of type 'TorusArray"
  "jsk_recognition_msgs/TorusArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TorusArray>)))
  "Returns md5sum for a message object of type '<TorusArray>"
  "81d0dbf46016b5714fa4ea9eca5485e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TorusArray)))
  "Returns md5sum for a message object of type 'TorusArray"
  "81d0dbf46016b5714fa4ea9eca5485e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TorusArray>)))
  "Returns full string definition for message of type '<TorusArray>"
  (cl:format cl:nil "Header header~%Torus[] toruses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/Torus~%Header header~%bool failure~%geometry_msgs/Pose pose~%float64 large_radius~%float64 small_radius~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TorusArray)))
  "Returns full string definition for message of type 'TorusArray"
  (cl:format cl:nil "Header header~%Torus[] toruses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/Torus~%Header header~%bool failure~%geometry_msgs/Pose pose~%float64 large_radius~%float64 small_radius~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TorusArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'toruses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TorusArray>))
  "Converts a ROS message object to a list"
  (cl:list 'TorusArray
    (cl:cons ':header (header msg))
    (cl:cons ':toruses (toruses msg))
))
