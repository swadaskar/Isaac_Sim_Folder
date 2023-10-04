; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude ContactSensorArray.msg.html

(cl:defclass <ContactSensorArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (datas
    :reader datas
    :initarg :datas
    :type (cl:vector jsk_recognition_msgs-msg:ContactSensor)
   :initform (cl:make-array 0 :element-type 'jsk_recognition_msgs-msg:ContactSensor :initial-element (cl:make-instance 'jsk_recognition_msgs-msg:ContactSensor))))
)

(cl:defclass ContactSensorArray (<ContactSensorArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ContactSensorArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ContactSensorArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<ContactSensorArray> is deprecated: use jsk_recognition_msgs-msg:ContactSensorArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ContactSensorArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'datas-val :lambda-list '(m))
(cl:defmethod datas-val ((m <ContactSensorArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:datas-val is deprecated.  Use jsk_recognition_msgs-msg:datas instead.")
  (datas m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ContactSensorArray>) ostream)
  "Serializes a message object of type '<ContactSensorArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'datas))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'datas))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ContactSensorArray>) istream)
  "Deserializes a message object of type '<ContactSensorArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'datas) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'datas)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'jsk_recognition_msgs-msg:ContactSensor))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ContactSensorArray>)))
  "Returns string type for a message object of type '<ContactSensorArray>"
  "jsk_recognition_msgs/ContactSensorArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ContactSensorArray)))
  "Returns string type for a message object of type 'ContactSensorArray"
  "jsk_recognition_msgs/ContactSensorArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ContactSensorArray>)))
  "Returns md5sum for a message object of type '<ContactSensorArray>"
  "c65f16fb3a523c0b77d7e31330b214da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ContactSensorArray)))
  "Returns md5sum for a message object of type 'ContactSensorArray"
  "c65f16fb3a523c0b77d7e31330b214da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ContactSensorArray>)))
  "Returns full string definition for message of type '<ContactSensorArray>"
  (cl:format cl:nil "std_msgs/Header header~%jsk_recognition_msgs/ContactSensor[] datas~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/ContactSensor~%# Header~%Header header~%~%# Whether sensor detects contact or not~%bool contact~%~%string link_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ContactSensorArray)))
  "Returns full string definition for message of type 'ContactSensorArray"
  (cl:format cl:nil "std_msgs/Header header~%jsk_recognition_msgs/ContactSensor[] datas~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/ContactSensor~%# Header~%Header header~%~%# Whether sensor detects contact or not~%bool contact~%~%string link_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ContactSensorArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'datas) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ContactSensorArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ContactSensorArray
    (cl:cons ':header (header msg))
    (cl:cons ':datas (datas msg))
))
