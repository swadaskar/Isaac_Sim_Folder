; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude ContactSensor.msg.html

(cl:defclass <ContactSensor> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (contact
    :reader contact
    :initarg :contact
    :type cl:boolean
    :initform cl:nil)
   (link_name
    :reader link_name
    :initarg :link_name
    :type cl:string
    :initform ""))
)

(cl:defclass ContactSensor (<ContactSensor>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ContactSensor>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ContactSensor)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<ContactSensor> is deprecated: use jsk_recognition_msgs-msg:ContactSensor instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ContactSensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'contact-val :lambda-list '(m))
(cl:defmethod contact-val ((m <ContactSensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:contact-val is deprecated.  Use jsk_recognition_msgs-msg:contact instead.")
  (contact m))

(cl:ensure-generic-function 'link_name-val :lambda-list '(m))
(cl:defmethod link_name-val ((m <ContactSensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:link_name-val is deprecated.  Use jsk_recognition_msgs-msg:link_name instead.")
  (link_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ContactSensor>) ostream)
  "Serializes a message object of type '<ContactSensor>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'contact) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'link_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'link_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ContactSensor>) istream)
  "Deserializes a message object of type '<ContactSensor>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'contact) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'link_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'link_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ContactSensor>)))
  "Returns string type for a message object of type '<ContactSensor>"
  "jsk_recognition_msgs/ContactSensor")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ContactSensor)))
  "Returns string type for a message object of type 'ContactSensor"
  "jsk_recognition_msgs/ContactSensor")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ContactSensor>)))
  "Returns md5sum for a message object of type '<ContactSensor>"
  "364b2b952a51d85dfa877e334264e361")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ContactSensor)))
  "Returns md5sum for a message object of type 'ContactSensor"
  "364b2b952a51d85dfa877e334264e361")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ContactSensor>)))
  "Returns full string definition for message of type '<ContactSensor>"
  (cl:format cl:nil "# Header~%Header header~%~%# Whether sensor detects contact or not~%bool contact~%~%string link_name~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ContactSensor)))
  "Returns full string definition for message of type 'ContactSensor"
  (cl:format cl:nil "# Header~%Header header~%~%# Whether sensor detects contact or not~%bool contact~%~%string link_name~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ContactSensor>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'link_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ContactSensor>))
  "Converts a ROS message object to a list"
  (cl:list 'ContactSensor
    (cl:cons ':header (header msg))
    (cl:cons ':contact (contact msg))
    (cl:cons ':link_name (link_name msg))
))
