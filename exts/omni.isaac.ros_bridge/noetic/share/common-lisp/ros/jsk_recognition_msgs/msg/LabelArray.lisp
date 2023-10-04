; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude LabelArray.msg.html

(cl:defclass <LabelArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (labels
    :reader labels
    :initarg :labels
    :type (cl:vector jsk_recognition_msgs-msg:Label)
   :initform (cl:make-array 0 :element-type 'jsk_recognition_msgs-msg:Label :initial-element (cl:make-instance 'jsk_recognition_msgs-msg:Label))))
)

(cl:defclass LabelArray (<LabelArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LabelArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LabelArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<LabelArray> is deprecated: use jsk_recognition_msgs-msg:LabelArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LabelArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'labels-val :lambda-list '(m))
(cl:defmethod labels-val ((m <LabelArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:labels-val is deprecated.  Use jsk_recognition_msgs-msg:labels instead.")
  (labels m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LabelArray>) ostream)
  "Serializes a message object of type '<LabelArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'labels))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'labels))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LabelArray>) istream)
  "Deserializes a message object of type '<LabelArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'labels) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'labels)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'jsk_recognition_msgs-msg:Label))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LabelArray>)))
  "Returns string type for a message object of type '<LabelArray>"
  "jsk_recognition_msgs/LabelArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LabelArray)))
  "Returns string type for a message object of type 'LabelArray"
  "jsk_recognition_msgs/LabelArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LabelArray>)))
  "Returns md5sum for a message object of type '<LabelArray>"
  "8cdb9aed89bee725ff5ad76b2986927d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LabelArray)))
  "Returns md5sum for a message object of type 'LabelArray"
  "8cdb9aed89bee725ff5ad76b2986927d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LabelArray>)))
  "Returns full string definition for message of type '<LabelArray>"
  (cl:format cl:nil "Header header~%Label[] labels~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/Label~%int32 id~%string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LabelArray)))
  "Returns full string definition for message of type 'LabelArray"
  (cl:format cl:nil "Header header~%Label[] labels~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/Label~%int32 id~%string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LabelArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'labels) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LabelArray>))
  "Converts a ROS message object to a list"
  (cl:list 'LabelArray
    (cl:cons ':header (header msg))
    (cl:cons ':labels (labels msg))
))
