; Auto-generated. Do not edit!


(cl:in-package cortex_control-srv)


;//! \htmlinclude MsgService-request.msg.html

(cl:defclass <MsgService-request> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass MsgService-request (<MsgService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MsgService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MsgService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cortex_control-srv:<MsgService-request> is deprecated: use cortex_control-srv:MsgService-request instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <MsgService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cortex_control-srv:msg-val is deprecated.  Use cortex_control-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MsgService-request>) ostream)
  "Serializes a message object of type '<MsgService-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MsgService-request>) istream)
  "Deserializes a message object of type '<MsgService-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MsgService-request>)))
  "Returns string type for a service object of type '<MsgService-request>"
  "cortex_control/MsgServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MsgService-request)))
  "Returns string type for a service object of type 'MsgService-request"
  "cortex_control/MsgServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MsgService-request>)))
  "Returns md5sum for a message object of type '<MsgService-request>"
  "38a3da27a00f36b8fc53764490266dcb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MsgService-request)))
  "Returns md5sum for a message object of type 'MsgService-request"
  "38a3da27a00f36b8fc53764490266dcb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MsgService-request>)))
  "Returns full string definition for message of type '<MsgService-request>"
  (cl:format cl:nil "# commands sent in json format~%string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MsgService-request)))
  "Returns full string definition for message of type 'MsgService-request"
  (cl:format cl:nil "# commands sent in json format~%string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MsgService-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MsgService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MsgService-request
    (cl:cons ':msg (msg msg))
))
;//! \htmlinclude MsgService-response.msg.html

(cl:defclass <MsgService-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:fixnum
    :initform 0)
   (error_str
    :reader error_str
    :initarg :error_str
    :type cl:string
    :initform ""))
)

(cl:defclass MsgService-response (<MsgService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MsgService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MsgService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cortex_control-srv:<MsgService-response> is deprecated: use cortex_control-srv:MsgService-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <MsgService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cortex_control-srv:result-val is deprecated.  Use cortex_control-srv:result instead.")
  (result m))

(cl:ensure-generic-function 'error_str-val :lambda-list '(m))
(cl:defmethod error_str-val ((m <MsgService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cortex_control-srv:error_str-val is deprecated.  Use cortex_control-srv:error_str instead.")
  (error_str m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<MsgService-response>)))
    "Constants for message type '<MsgService-response>"
  '((:SUCCESS . 0)
    (:FAILURE . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'MsgService-response)))
    "Constants for message type 'MsgService-response"
  '((:SUCCESS . 0)
    (:FAILURE . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MsgService-response>) ostream)
  "Serializes a message object of type '<MsgService-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'error_str))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'error_str))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MsgService-response>) istream)
  "Deserializes a message object of type '<MsgService-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'error_str) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'error_str) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MsgService-response>)))
  "Returns string type for a service object of type '<MsgService-response>"
  "cortex_control/MsgServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MsgService-response)))
  "Returns string type for a service object of type 'MsgService-response"
  "cortex_control/MsgServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MsgService-response>)))
  "Returns md5sum for a message object of type '<MsgService-response>"
  "38a3da27a00f36b8fc53764490266dcb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MsgService-response)))
  "Returns md5sum for a message object of type 'MsgService-response"
  "38a3da27a00f36b8fc53764490266dcb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MsgService-response>)))
  "Returns full string definition for message of type '<MsgService-response>"
  (cl:format cl:nil "uint8 SUCCESS=0~%uint8 FAILURE=1~%uint8 result~%string error_str~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MsgService-response)))
  "Returns full string definition for message of type 'MsgService-response"
  (cl:format cl:nil "uint8 SUCCESS=0~%uint8 FAILURE=1~%uint8 result~%string error_str~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MsgService-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'error_str))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MsgService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MsgService-response
    (cl:cons ':result (result msg))
    (cl:cons ':error_str (error_str msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MsgService)))
  'MsgService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MsgService)))
  'MsgService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MsgService)))
  "Returns string type for a service object of type '<MsgService>"
  "cortex_control/MsgService")