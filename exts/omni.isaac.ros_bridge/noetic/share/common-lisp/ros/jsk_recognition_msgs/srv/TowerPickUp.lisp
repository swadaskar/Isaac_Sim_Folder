; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude TowerPickUp-request.msg.html

(cl:defclass <TowerPickUp-request> (roslisp-msg-protocol:ros-message)
  ((tower_index
    :reader tower_index
    :initarg :tower_index
    :type cl:integer
    :initform 0))
)

(cl:defclass TowerPickUp-request (<TowerPickUp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TowerPickUp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TowerPickUp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<TowerPickUp-request> is deprecated: use jsk_recognition_msgs-srv:TowerPickUp-request instead.")))

(cl:ensure-generic-function 'tower_index-val :lambda-list '(m))
(cl:defmethod tower_index-val ((m <TowerPickUp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:tower_index-val is deprecated.  Use jsk_recognition_msgs-srv:tower_index instead.")
  (tower_index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TowerPickUp-request>) ostream)
  "Serializes a message object of type '<TowerPickUp-request>"
  (cl:let* ((signed (cl:slot-value msg 'tower_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TowerPickUp-request>) istream)
  "Deserializes a message object of type '<TowerPickUp-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tower_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TowerPickUp-request>)))
  "Returns string type for a service object of type '<TowerPickUp-request>"
  "jsk_recognition_msgs/TowerPickUpRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TowerPickUp-request)))
  "Returns string type for a service object of type 'TowerPickUp-request"
  "jsk_recognition_msgs/TowerPickUpRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TowerPickUp-request>)))
  "Returns md5sum for a message object of type '<TowerPickUp-request>"
  "e8bd24109f26b6d833bc4570d67d71c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TowerPickUp-request)))
  "Returns md5sum for a message object of type 'TowerPickUp-request"
  "e8bd24109f26b6d833bc4570d67d71c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TowerPickUp-request>)))
  "Returns full string definition for message of type '<TowerPickUp-request>"
  (cl:format cl:nil "int32 tower_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TowerPickUp-request)))
  "Returns full string definition for message of type 'TowerPickUp-request"
  (cl:format cl:nil "int32 tower_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TowerPickUp-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TowerPickUp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TowerPickUp-request
    (cl:cons ':tower_index (tower_index msg))
))
;//! \htmlinclude TowerPickUp-response.msg.html

(cl:defclass <TowerPickUp-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TowerPickUp-response (<TowerPickUp-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TowerPickUp-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TowerPickUp-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<TowerPickUp-response> is deprecated: use jsk_recognition_msgs-srv:TowerPickUp-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TowerPickUp-response>) ostream)
  "Serializes a message object of type '<TowerPickUp-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TowerPickUp-response>) istream)
  "Deserializes a message object of type '<TowerPickUp-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TowerPickUp-response>)))
  "Returns string type for a service object of type '<TowerPickUp-response>"
  "jsk_recognition_msgs/TowerPickUpResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TowerPickUp-response)))
  "Returns string type for a service object of type 'TowerPickUp-response"
  "jsk_recognition_msgs/TowerPickUpResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TowerPickUp-response>)))
  "Returns md5sum for a message object of type '<TowerPickUp-response>"
  "e8bd24109f26b6d833bc4570d67d71c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TowerPickUp-response)))
  "Returns md5sum for a message object of type 'TowerPickUp-response"
  "e8bd24109f26b6d833bc4570d67d71c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TowerPickUp-response>)))
  "Returns full string definition for message of type '<TowerPickUp-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TowerPickUp-response)))
  "Returns full string definition for message of type 'TowerPickUp-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TowerPickUp-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TowerPickUp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TowerPickUp-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TowerPickUp)))
  'TowerPickUp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TowerPickUp)))
  'TowerPickUp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TowerPickUp)))
  "Returns string type for a service object of type '<TowerPickUp>"
  "jsk_recognition_msgs/TowerPickUp")