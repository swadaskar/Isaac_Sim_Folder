; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude SwitchTopic-request.msg.html

(cl:defclass <SwitchTopic-request> (roslisp-msg-protocol:ros-message)
  ((camera_info
    :reader camera_info
    :initarg :camera_info
    :type cl:string
    :initform "")
   (points
    :reader points
    :initarg :points
    :type cl:string
    :initform ""))
)

(cl:defclass SwitchTopic-request (<SwitchTopic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwitchTopic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwitchTopic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<SwitchTopic-request> is deprecated: use jsk_recognition_msgs-srv:SwitchTopic-request instead.")))

(cl:ensure-generic-function 'camera_info-val :lambda-list '(m))
(cl:defmethod camera_info-val ((m <SwitchTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:camera_info-val is deprecated.  Use jsk_recognition_msgs-srv:camera_info instead.")
  (camera_info m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <SwitchTopic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:points-val is deprecated.  Use jsk_recognition_msgs-srv:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwitchTopic-request>) ostream)
  "Serializes a message object of type '<SwitchTopic-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'camera_info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'camera_info))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwitchTopic-request>) istream)
  "Deserializes a message object of type '<SwitchTopic-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'camera_info) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'camera_info) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'points) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'points) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwitchTopic-request>)))
  "Returns string type for a service object of type '<SwitchTopic-request>"
  "jsk_recognition_msgs/SwitchTopicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchTopic-request)))
  "Returns string type for a service object of type 'SwitchTopic-request"
  "jsk_recognition_msgs/SwitchTopicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwitchTopic-request>)))
  "Returns md5sum for a message object of type '<SwitchTopic-request>"
  "e4a276b5a9b7b8fd97441d0fd991bdb9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwitchTopic-request)))
  "Returns md5sum for a message object of type 'SwitchTopic-request"
  "e4a276b5a9b7b8fd97441d0fd991bdb9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwitchTopic-request>)))
  "Returns full string definition for message of type '<SwitchTopic-request>"
  (cl:format cl:nil "# new topics~%string camera_info~%string points~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwitchTopic-request)))
  "Returns full string definition for message of type 'SwitchTopic-request"
  (cl:format cl:nil "# new topics~%string camera_info~%string points~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwitchTopic-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'camera_info))
     4 (cl:length (cl:slot-value msg 'points))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwitchTopic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SwitchTopic-request
    (cl:cons ':camera_info (camera_info msg))
    (cl:cons ':points (points msg))
))
;//! \htmlinclude SwitchTopic-response.msg.html

(cl:defclass <SwitchTopic-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SwitchTopic-response (<SwitchTopic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwitchTopic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwitchTopic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<SwitchTopic-response> is deprecated: use jsk_recognition_msgs-srv:SwitchTopic-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwitchTopic-response>) ostream)
  "Serializes a message object of type '<SwitchTopic-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwitchTopic-response>) istream)
  "Deserializes a message object of type '<SwitchTopic-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwitchTopic-response>)))
  "Returns string type for a service object of type '<SwitchTopic-response>"
  "jsk_recognition_msgs/SwitchTopicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchTopic-response)))
  "Returns string type for a service object of type 'SwitchTopic-response"
  "jsk_recognition_msgs/SwitchTopicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwitchTopic-response>)))
  "Returns md5sum for a message object of type '<SwitchTopic-response>"
  "e4a276b5a9b7b8fd97441d0fd991bdb9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwitchTopic-response)))
  "Returns md5sum for a message object of type 'SwitchTopic-response"
  "e4a276b5a9b7b8fd97441d0fd991bdb9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwitchTopic-response>)))
  "Returns full string definition for message of type '<SwitchTopic-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwitchTopic-response)))
  "Returns full string definition for message of type 'SwitchTopic-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwitchTopic-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwitchTopic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SwitchTopic-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SwitchTopic)))
  'SwitchTopic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SwitchTopic)))
  'SwitchTopic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchTopic)))
  "Returns string type for a service object of type '<SwitchTopic>"
  "jsk_recognition_msgs/SwitchTopic")