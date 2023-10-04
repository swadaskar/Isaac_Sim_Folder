; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude RobotPickupReleasePoint-request.msg.html

(cl:defclass <RobotPickupReleasePoint-request> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (target_point
    :reader target_point
    :initarg :target_point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (pick_or_release
    :reader pick_or_release
    :initarg :pick_or_release
    :type cl:integer
    :initform 0))
)

(cl:defclass RobotPickupReleasePoint-request (<RobotPickupReleasePoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotPickupReleasePoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotPickupReleasePoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<RobotPickupReleasePoint-request> is deprecated: use jsk_recognition_msgs-srv:RobotPickupReleasePoint-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RobotPickupReleasePoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:header-val is deprecated.  Use jsk_recognition_msgs-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'target_point-val :lambda-list '(m))
(cl:defmethod target_point-val ((m <RobotPickupReleasePoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:target_point-val is deprecated.  Use jsk_recognition_msgs-srv:target_point instead.")
  (target_point m))

(cl:ensure-generic-function 'pick_or_release-val :lambda-list '(m))
(cl:defmethod pick_or_release-val ((m <RobotPickupReleasePoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:pick_or_release-val is deprecated.  Use jsk_recognition_msgs-srv:pick_or_release instead.")
  (pick_or_release m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotPickupReleasePoint-request>) ostream)
  "Serializes a message object of type '<RobotPickupReleasePoint-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_point) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pick_or_release)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotPickupReleasePoint-request>) istream)
  "Deserializes a message object of type '<RobotPickupReleasePoint-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_point) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pick_or_release)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotPickupReleasePoint-request>)))
  "Returns string type for a service object of type '<RobotPickupReleasePoint-request>"
  "jsk_recognition_msgs/RobotPickupReleasePointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotPickupReleasePoint-request)))
  "Returns string type for a service object of type 'RobotPickupReleasePoint-request"
  "jsk_recognition_msgs/RobotPickupReleasePointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotPickupReleasePoint-request>)))
  "Returns md5sum for a message object of type '<RobotPickupReleasePoint-request>"
  "b6469871ddb5ec099685daff904ecc2f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotPickupReleasePoint-request)))
  "Returns md5sum for a message object of type 'RobotPickupReleasePoint-request"
  "b6469871ddb5ec099685daff904ecc2f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotPickupReleasePoint-request>)))
  "Returns full string definition for message of type '<RobotPickupReleasePoint-request>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point target_point~%byte pick_or_release  # 0 -> pick, 1 -> release~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotPickupReleasePoint-request)))
  "Returns full string definition for message of type 'RobotPickupReleasePoint-request"
  (cl:format cl:nil "Header header~%geometry_msgs/Point target_point~%byte pick_or_release  # 0 -> pick, 1 -> release~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotPickupReleasePoint-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_point))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotPickupReleasePoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotPickupReleasePoint-request
    (cl:cons ':header (header msg))
    (cl:cons ':target_point (target_point msg))
    (cl:cons ':pick_or_release (pick_or_release msg))
))
;//! \htmlinclude RobotPickupReleasePoint-response.msg.html

(cl:defclass <RobotPickupReleasePoint-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RobotPickupReleasePoint-response (<RobotPickupReleasePoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotPickupReleasePoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotPickupReleasePoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<RobotPickupReleasePoint-response> is deprecated: use jsk_recognition_msgs-srv:RobotPickupReleasePoint-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RobotPickupReleasePoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:success-val is deprecated.  Use jsk_recognition_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotPickupReleasePoint-response>) ostream)
  "Serializes a message object of type '<RobotPickupReleasePoint-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotPickupReleasePoint-response>) istream)
  "Deserializes a message object of type '<RobotPickupReleasePoint-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotPickupReleasePoint-response>)))
  "Returns string type for a service object of type '<RobotPickupReleasePoint-response>"
  "jsk_recognition_msgs/RobotPickupReleasePointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotPickupReleasePoint-response)))
  "Returns string type for a service object of type 'RobotPickupReleasePoint-response"
  "jsk_recognition_msgs/RobotPickupReleasePointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotPickupReleasePoint-response>)))
  "Returns md5sum for a message object of type '<RobotPickupReleasePoint-response>"
  "b6469871ddb5ec099685daff904ecc2f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotPickupReleasePoint-response)))
  "Returns md5sum for a message object of type 'RobotPickupReleasePoint-response"
  "b6469871ddb5ec099685daff904ecc2f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotPickupReleasePoint-response>)))
  "Returns full string definition for message of type '<RobotPickupReleasePoint-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotPickupReleasePoint-response)))
  "Returns full string definition for message of type 'RobotPickupReleasePoint-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotPickupReleasePoint-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotPickupReleasePoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotPickupReleasePoint-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RobotPickupReleasePoint)))
  'RobotPickupReleasePoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RobotPickupReleasePoint)))
  'RobotPickupReleasePoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotPickupReleasePoint)))
  "Returns string type for a service object of type '<RobotPickupReleasePoint>"
  "jsk_recognition_msgs/RobotPickupReleasePoint")