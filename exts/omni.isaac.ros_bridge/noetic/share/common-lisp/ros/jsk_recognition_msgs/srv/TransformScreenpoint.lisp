; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude TransformScreenpoint-request.msg.html

(cl:defclass <TransformScreenpoint-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (no_update
    :reader no_update
    :initarg :no_update
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass TransformScreenpoint-request (<TransformScreenpoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransformScreenpoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransformScreenpoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<TransformScreenpoint-request> is deprecated: use jsk_recognition_msgs-srv:TransformScreenpoint-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <TransformScreenpoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:x-val is deprecated.  Use jsk_recognition_msgs-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <TransformScreenpoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:y-val is deprecated.  Use jsk_recognition_msgs-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'no_update-val :lambda-list '(m))
(cl:defmethod no_update-val ((m <TransformScreenpoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:no_update-val is deprecated.  Use jsk_recognition_msgs-srv:no_update instead.")
  (no_update m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransformScreenpoint-request>) ostream)
  "Serializes a message object of type '<TransformScreenpoint-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'no_update) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransformScreenpoint-request>) istream)
  "Deserializes a message object of type '<TransformScreenpoint-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'no_update) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransformScreenpoint-request>)))
  "Returns string type for a service object of type '<TransformScreenpoint-request>"
  "jsk_recognition_msgs/TransformScreenpointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformScreenpoint-request)))
  "Returns string type for a service object of type 'TransformScreenpoint-request"
  "jsk_recognition_msgs/TransformScreenpointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransformScreenpoint-request>)))
  "Returns md5sum for a message object of type '<TransformScreenpoint-request>"
  "ad80b7dbbce8c5ae0833e8fb24e35c06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransformScreenpoint-request)))
  "Returns md5sum for a message object of type 'TransformScreenpoint-request"
  "ad80b7dbbce8c5ae0833e8fb24e35c06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransformScreenpoint-request>)))
  "Returns full string definition for message of type '<TransformScreenpoint-request>"
  (cl:format cl:nil "# screen point~%float32 x~%float32 y~%bool no_update~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransformScreenpoint-request)))
  "Returns full string definition for message of type 'TransformScreenpoint-request"
  (cl:format cl:nil "# screen point~%float32 x~%float32 y~%bool no_update~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransformScreenpoint-request>))
  (cl:+ 0
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransformScreenpoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TransformScreenpoint-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':no_update (no_update msg))
))
;//! \htmlinclude TransformScreenpoint-response.msg.html

(cl:defclass <TransformScreenpoint-response> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (vector
    :reader vector
    :initarg :vector
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass TransformScreenpoint-response (<TransformScreenpoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransformScreenpoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransformScreenpoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<TransformScreenpoint-response> is deprecated: use jsk_recognition_msgs-srv:TransformScreenpoint-response instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TransformScreenpoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:header-val is deprecated.  Use jsk_recognition_msgs-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <TransformScreenpoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:point-val is deprecated.  Use jsk_recognition_msgs-srv:point instead.")
  (point m))

(cl:ensure-generic-function 'vector-val :lambda-list '(m))
(cl:defmethod vector-val ((m <TransformScreenpoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:vector-val is deprecated.  Use jsk_recognition_msgs-srv:vector instead.")
  (vector m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransformScreenpoint-response>) ostream)
  "Serializes a message object of type '<TransformScreenpoint-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vector) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransformScreenpoint-response>) istream)
  "Deserializes a message object of type '<TransformScreenpoint-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vector) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransformScreenpoint-response>)))
  "Returns string type for a service object of type '<TransformScreenpoint-response>"
  "jsk_recognition_msgs/TransformScreenpointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformScreenpoint-response)))
  "Returns string type for a service object of type 'TransformScreenpoint-response"
  "jsk_recognition_msgs/TransformScreenpointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransformScreenpoint-response>)))
  "Returns md5sum for a message object of type '<TransformScreenpoint-response>"
  "ad80b7dbbce8c5ae0833e8fb24e35c06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransformScreenpoint-response)))
  "Returns md5sum for a message object of type 'TransformScreenpoint-response"
  "ad80b7dbbce8c5ae0833e8fb24e35c06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransformScreenpoint-response>)))
  "Returns full string definition for message of type '<TransformScreenpoint-response>"
  (cl:format cl:nil "# position in actual world~%std_msgs/Header header~%geometry_msgs/Point point~%geometry_msgs/Vector3 vector~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransformScreenpoint-response)))
  "Returns full string definition for message of type 'TransformScreenpoint-response"
  (cl:format cl:nil "# position in actual world~%std_msgs/Header header~%geometry_msgs/Point point~%geometry_msgs/Vector3 vector~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransformScreenpoint-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vector))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransformScreenpoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TransformScreenpoint-response
    (cl:cons ':header (header msg))
    (cl:cons ':point (point msg))
    (cl:cons ':vector (vector msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TransformScreenpoint)))
  'TransformScreenpoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TransformScreenpoint)))
  'TransformScreenpoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformScreenpoint)))
  "Returns string type for a service object of type '<TransformScreenpoint>"
  "jsk_recognition_msgs/TransformScreenpoint")