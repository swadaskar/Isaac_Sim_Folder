; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude CallPolygon-request.msg.html

(cl:defclass <CallPolygon-request> (roslisp-msg-protocol:ros-message)
  ((filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform ""))
)

(cl:defclass CallPolygon-request (<CallPolygon-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CallPolygon-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CallPolygon-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<CallPolygon-request> is deprecated: use jsk_recognition_msgs-srv:CallPolygon-request instead.")))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <CallPolygon-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:filename-val is deprecated.  Use jsk_recognition_msgs-srv:filename instead.")
  (filename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CallPolygon-request>) ostream)
  "Serializes a message object of type '<CallPolygon-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CallPolygon-request>) istream)
  "Deserializes a message object of type '<CallPolygon-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CallPolygon-request>)))
  "Returns string type for a service object of type '<CallPolygon-request>"
  "jsk_recognition_msgs/CallPolygonRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CallPolygon-request)))
  "Returns string type for a service object of type 'CallPolygon-request"
  "jsk_recognition_msgs/CallPolygonRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CallPolygon-request>)))
  "Returns md5sum for a message object of type '<CallPolygon-request>"
  "b28bb04ea4e0c4a8f9c4d715ba76b7ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CallPolygon-request)))
  "Returns md5sum for a message object of type 'CallPolygon-request"
  "b28bb04ea4e0c4a8f9c4d715ba76b7ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CallPolygon-request>)))
  "Returns full string definition for message of type '<CallPolygon-request>"
  (cl:format cl:nil "string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CallPolygon-request)))
  "Returns full string definition for message of type 'CallPolygon-request"
  (cl:format cl:nil "string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CallPolygon-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CallPolygon-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CallPolygon-request
    (cl:cons ':filename (filename msg))
))
;//! \htmlinclude CallPolygon-response.msg.html

(cl:defclass <CallPolygon-response> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type geometry_msgs-msg:PolygonStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PolygonStamped)))
)

(cl:defclass CallPolygon-response (<CallPolygon-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CallPolygon-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CallPolygon-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<CallPolygon-response> is deprecated: use jsk_recognition_msgs-srv:CallPolygon-response instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <CallPolygon-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:points-val is deprecated.  Use jsk_recognition_msgs-srv:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CallPolygon-response>) ostream)
  "Serializes a message object of type '<CallPolygon-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'points) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CallPolygon-response>) istream)
  "Deserializes a message object of type '<CallPolygon-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'points) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CallPolygon-response>)))
  "Returns string type for a service object of type '<CallPolygon-response>"
  "jsk_recognition_msgs/CallPolygonResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CallPolygon-response)))
  "Returns string type for a service object of type 'CallPolygon-response"
  "jsk_recognition_msgs/CallPolygonResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CallPolygon-response>)))
  "Returns md5sum for a message object of type '<CallPolygon-response>"
  "b28bb04ea4e0c4a8f9c4d715ba76b7ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CallPolygon-response)))
  "Returns md5sum for a message object of type 'CallPolygon-response"
  "b28bb04ea4e0c4a8f9c4d715ba76b7ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CallPolygon-response>)))
  "Returns full string definition for message of type '<CallPolygon-response>"
  (cl:format cl:nil "geometry_msgs/PolygonStamped points~%~%~%================================================================================~%MSG: geometry_msgs/PolygonStamped~%# This represents a Polygon with reference coordinate frame and timestamp~%Header header~%Polygon polygon~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CallPolygon-response)))
  "Returns full string definition for message of type 'CallPolygon-response"
  (cl:format cl:nil "geometry_msgs/PolygonStamped points~%~%~%================================================================================~%MSG: geometry_msgs/PolygonStamped~%# This represents a Polygon with reference coordinate frame and timestamp~%Header header~%Polygon polygon~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CallPolygon-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'points))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CallPolygon-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CallPolygon-response
    (cl:cons ':points (points msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CallPolygon)))
  'CallPolygon-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CallPolygon)))
  'CallPolygon-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CallPolygon)))
  "Returns string type for a service object of type '<CallPolygon>"
  "jsk_recognition_msgs/CallPolygon")