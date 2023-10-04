; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude EnvironmentLock-request.msg.html

(cl:defclass <EnvironmentLock-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass EnvironmentLock-request (<EnvironmentLock-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EnvironmentLock-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EnvironmentLock-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<EnvironmentLock-request> is deprecated: use jsk_recognition_msgs-srv:EnvironmentLock-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EnvironmentLock-request>) ostream)
  "Serializes a message object of type '<EnvironmentLock-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EnvironmentLock-request>) istream)
  "Deserializes a message object of type '<EnvironmentLock-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EnvironmentLock-request>)))
  "Returns string type for a service object of type '<EnvironmentLock-request>"
  "jsk_recognition_msgs/EnvironmentLockRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnvironmentLock-request)))
  "Returns string type for a service object of type 'EnvironmentLock-request"
  "jsk_recognition_msgs/EnvironmentLockRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EnvironmentLock-request>)))
  "Returns md5sum for a message object of type '<EnvironmentLock-request>"
  "109afc0d3bd22aa461d45c8ef5ab6d75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EnvironmentLock-request)))
  "Returns md5sum for a message object of type 'EnvironmentLock-request"
  "109afc0d3bd22aa461d45c8ef5ab6d75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EnvironmentLock-request>)))
  "Returns full string definition for message of type '<EnvironmentLock-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EnvironmentLock-request)))
  "Returns full string definition for message of type 'EnvironmentLock-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EnvironmentLock-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EnvironmentLock-request>))
  "Converts a ROS message object to a list"
  (cl:list 'EnvironmentLock-request
))
;//! \htmlinclude EnvironmentLock-response.msg.html

(cl:defclass <EnvironmentLock-response> (roslisp-msg-protocol:ros-message)
  ((environment_id
    :reader environment_id
    :initarg :environment_id
    :type cl:integer
    :initform 0))
)

(cl:defclass EnvironmentLock-response (<EnvironmentLock-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EnvironmentLock-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EnvironmentLock-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<EnvironmentLock-response> is deprecated: use jsk_recognition_msgs-srv:EnvironmentLock-response instead.")))

(cl:ensure-generic-function 'environment_id-val :lambda-list '(m))
(cl:defmethod environment_id-val ((m <EnvironmentLock-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:environment_id-val is deprecated.  Use jsk_recognition_msgs-srv:environment_id instead.")
  (environment_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EnvironmentLock-response>) ostream)
  "Serializes a message object of type '<EnvironmentLock-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'environment_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'environment_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'environment_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'environment_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EnvironmentLock-response>) istream)
  "Deserializes a message object of type '<EnvironmentLock-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'environment_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'environment_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'environment_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'environment_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EnvironmentLock-response>)))
  "Returns string type for a service object of type '<EnvironmentLock-response>"
  "jsk_recognition_msgs/EnvironmentLockResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnvironmentLock-response)))
  "Returns string type for a service object of type 'EnvironmentLock-response"
  "jsk_recognition_msgs/EnvironmentLockResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EnvironmentLock-response>)))
  "Returns md5sum for a message object of type '<EnvironmentLock-response>"
  "109afc0d3bd22aa461d45c8ef5ab6d75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EnvironmentLock-response)))
  "Returns md5sum for a message object of type 'EnvironmentLock-response"
  "109afc0d3bd22aa461d45c8ef5ab6d75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EnvironmentLock-response>)))
  "Returns full string definition for message of type '<EnvironmentLock-response>"
  (cl:format cl:nil "uint32 environment_id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EnvironmentLock-response)))
  "Returns full string definition for message of type 'EnvironmentLock-response"
  (cl:format cl:nil "uint32 environment_id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EnvironmentLock-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EnvironmentLock-response>))
  "Converts a ROS message object to a list"
  (cl:list 'EnvironmentLock-response
    (cl:cons ':environment_id (environment_id msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'EnvironmentLock)))
  'EnvironmentLock-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'EnvironmentLock)))
  'EnvironmentLock-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnvironmentLock)))
  "Returns string type for a service object of type '<EnvironmentLock>"
  "jsk_recognition_msgs/EnvironmentLock")