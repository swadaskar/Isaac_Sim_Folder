; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude PolygonOnEnvironment-request.msg.html

(cl:defclass <PolygonOnEnvironment-request> (roslisp-msg-protocol:ros-message)
  ((environment_id
    :reader environment_id
    :initarg :environment_id
    :type cl:integer
    :initform 0)
   (plane_index
    :reader plane_index
    :initarg :plane_index
    :type cl:integer
    :initform 0)
   (polygon
    :reader polygon
    :initarg :polygon
    :type geometry_msgs-msg:PolygonStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PolygonStamped)))
)

(cl:defclass PolygonOnEnvironment-request (<PolygonOnEnvironment-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PolygonOnEnvironment-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PolygonOnEnvironment-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<PolygonOnEnvironment-request> is deprecated: use jsk_recognition_msgs-srv:PolygonOnEnvironment-request instead.")))

(cl:ensure-generic-function 'environment_id-val :lambda-list '(m))
(cl:defmethod environment_id-val ((m <PolygonOnEnvironment-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:environment_id-val is deprecated.  Use jsk_recognition_msgs-srv:environment_id instead.")
  (environment_id m))

(cl:ensure-generic-function 'plane_index-val :lambda-list '(m))
(cl:defmethod plane_index-val ((m <PolygonOnEnvironment-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:plane_index-val is deprecated.  Use jsk_recognition_msgs-srv:plane_index instead.")
  (plane_index m))

(cl:ensure-generic-function 'polygon-val :lambda-list '(m))
(cl:defmethod polygon-val ((m <PolygonOnEnvironment-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:polygon-val is deprecated.  Use jsk_recognition_msgs-srv:polygon instead.")
  (polygon m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PolygonOnEnvironment-request>) ostream)
  "Serializes a message object of type '<PolygonOnEnvironment-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'environment_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'environment_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'environment_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'environment_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'plane_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'plane_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'plane_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'plane_index)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'polygon) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PolygonOnEnvironment-request>) istream)
  "Deserializes a message object of type '<PolygonOnEnvironment-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'environment_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'environment_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'environment_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'environment_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'plane_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'plane_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'plane_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'plane_index)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'polygon) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PolygonOnEnvironment-request>)))
  "Returns string type for a service object of type '<PolygonOnEnvironment-request>"
  "jsk_recognition_msgs/PolygonOnEnvironmentRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PolygonOnEnvironment-request)))
  "Returns string type for a service object of type 'PolygonOnEnvironment-request"
  "jsk_recognition_msgs/PolygonOnEnvironmentRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PolygonOnEnvironment-request>)))
  "Returns md5sum for a message object of type '<PolygonOnEnvironment-request>"
  "a8ff091210c071091863d880820e505c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PolygonOnEnvironment-request)))
  "Returns md5sum for a message object of type 'PolygonOnEnvironment-request"
  "a8ff091210c071091863d880820e505c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PolygonOnEnvironment-request>)))
  "Returns full string definition for message of type '<PolygonOnEnvironment-request>"
  (cl:format cl:nil "uint32 environment_id~%uint32 plane_index~%geometry_msgs/PolygonStamped polygon~%~%================================================================================~%MSG: geometry_msgs/PolygonStamped~%# This represents a Polygon with reference coordinate frame and timestamp~%Header header~%Polygon polygon~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PolygonOnEnvironment-request)))
  "Returns full string definition for message of type 'PolygonOnEnvironment-request"
  (cl:format cl:nil "uint32 environment_id~%uint32 plane_index~%geometry_msgs/PolygonStamped polygon~%~%================================================================================~%MSG: geometry_msgs/PolygonStamped~%# This represents a Polygon with reference coordinate frame and timestamp~%Header header~%Polygon polygon~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PolygonOnEnvironment-request>))
  (cl:+ 0
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'polygon))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PolygonOnEnvironment-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PolygonOnEnvironment-request
    (cl:cons ':environment_id (environment_id msg))
    (cl:cons ':plane_index (plane_index msg))
    (cl:cons ':polygon (polygon msg))
))
;//! \htmlinclude PolygonOnEnvironment-response.msg.html

(cl:defclass <PolygonOnEnvironment-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil)
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform ""))
)

(cl:defclass PolygonOnEnvironment-response (<PolygonOnEnvironment-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PolygonOnEnvironment-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PolygonOnEnvironment-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<PolygonOnEnvironment-response> is deprecated: use jsk_recognition_msgs-srv:PolygonOnEnvironment-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <PolygonOnEnvironment-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:result-val is deprecated.  Use jsk_recognition_msgs-srv:result instead.")
  (result m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <PolygonOnEnvironment-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:reason-val is deprecated.  Use jsk_recognition_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PolygonOnEnvironment-response>) ostream)
  "Serializes a message object of type '<PolygonOnEnvironment-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PolygonOnEnvironment-response>) istream)
  "Deserializes a message object of type '<PolygonOnEnvironment-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reason) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reason) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PolygonOnEnvironment-response>)))
  "Returns string type for a service object of type '<PolygonOnEnvironment-response>"
  "jsk_recognition_msgs/PolygonOnEnvironmentResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PolygonOnEnvironment-response)))
  "Returns string type for a service object of type 'PolygonOnEnvironment-response"
  "jsk_recognition_msgs/PolygonOnEnvironmentResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PolygonOnEnvironment-response>)))
  "Returns md5sum for a message object of type '<PolygonOnEnvironment-response>"
  "a8ff091210c071091863d880820e505c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PolygonOnEnvironment-response)))
  "Returns md5sum for a message object of type 'PolygonOnEnvironment-response"
  "a8ff091210c071091863d880820e505c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PolygonOnEnvironment-response>)))
  "Returns full string definition for message of type '<PolygonOnEnvironment-response>"
  (cl:format cl:nil "bool result~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PolygonOnEnvironment-response)))
  "Returns full string definition for message of type 'PolygonOnEnvironment-response"
  (cl:format cl:nil "bool result~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PolygonOnEnvironment-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PolygonOnEnvironment-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PolygonOnEnvironment-response
    (cl:cons ':result (result msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PolygonOnEnvironment)))
  'PolygonOnEnvironment-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PolygonOnEnvironment)))
  'PolygonOnEnvironment-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PolygonOnEnvironment)))
  "Returns string type for a service object of type '<PolygonOnEnvironment>"
  "jsk_recognition_msgs/PolygonOnEnvironment")