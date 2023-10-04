; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude CheckCircle-request.msg.html

(cl:defclass <CheckCircle-request> (roslisp-msg-protocol:ros-message)
  ((point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass CheckCircle-request (<CheckCircle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckCircle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckCircle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<CheckCircle-request> is deprecated: use jsk_recognition_msgs-srv:CheckCircle-request instead.")))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <CheckCircle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:point-val is deprecated.  Use jsk_recognition_msgs-srv:point instead.")
  (point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckCircle-request>) ostream)
  "Serializes a message object of type '<CheckCircle-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckCircle-request>) istream)
  "Deserializes a message object of type '<CheckCircle-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckCircle-request>)))
  "Returns string type for a service object of type '<CheckCircle-request>"
  "jsk_recognition_msgs/CheckCircleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckCircle-request)))
  "Returns string type for a service object of type 'CheckCircle-request"
  "jsk_recognition_msgs/CheckCircleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckCircle-request>)))
  "Returns md5sum for a message object of type '<CheckCircle-request>"
  "36549e2f705ec32b8c873cf22d68b357")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckCircle-request)))
  "Returns md5sum for a message object of type 'CheckCircle-request"
  "36549e2f705ec32b8c873cf22d68b357")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckCircle-request>)))
  "Returns full string definition for message of type '<CheckCircle-request>"
  (cl:format cl:nil "geometry_msgs/Point point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckCircle-request)))
  "Returns full string definition for message of type 'CheckCircle-request"
  (cl:format cl:nil "geometry_msgs/Point point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckCircle-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckCircle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckCircle-request
    (cl:cons ':point (point msg))
))
;//! \htmlinclude CheckCircle-response.msg.html

(cl:defclass <CheckCircle-response> (roslisp-msg-protocol:ros-message)
  ((clicked
    :reader clicked
    :initarg :clicked
    :type cl:boolean
    :initform cl:nil)
   (index
    :reader index
    :initarg :index
    :type cl:integer
    :initform 0)
   (msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass CheckCircle-response (<CheckCircle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckCircle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckCircle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<CheckCircle-response> is deprecated: use jsk_recognition_msgs-srv:CheckCircle-response instead.")))

(cl:ensure-generic-function 'clicked-val :lambda-list '(m))
(cl:defmethod clicked-val ((m <CheckCircle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:clicked-val is deprecated.  Use jsk_recognition_msgs-srv:clicked instead.")
  (clicked m))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <CheckCircle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:index-val is deprecated.  Use jsk_recognition_msgs-srv:index instead.")
  (index m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <CheckCircle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:msg-val is deprecated.  Use jsk_recognition_msgs-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckCircle-response>) ostream)
  "Serializes a message object of type '<CheckCircle-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clicked) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckCircle-response>) istream)
  "Deserializes a message object of type '<CheckCircle-response>"
    (cl:setf (cl:slot-value msg 'clicked) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckCircle-response>)))
  "Returns string type for a service object of type '<CheckCircle-response>"
  "jsk_recognition_msgs/CheckCircleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckCircle-response)))
  "Returns string type for a service object of type 'CheckCircle-response"
  "jsk_recognition_msgs/CheckCircleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckCircle-response>)))
  "Returns md5sum for a message object of type '<CheckCircle-response>"
  "36549e2f705ec32b8c873cf22d68b357")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckCircle-response)))
  "Returns md5sum for a message object of type 'CheckCircle-response"
  "36549e2f705ec32b8c873cf22d68b357")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckCircle-response>)))
  "Returns full string definition for message of type '<CheckCircle-response>"
  (cl:format cl:nil "bool clicked~%int32 index~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckCircle-response)))
  "Returns full string definition for message of type 'CheckCircle-response"
  (cl:format cl:nil "bool clicked~%int32 index~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckCircle-response>))
  (cl:+ 0
     1
     4
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckCircle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckCircle-response
    (cl:cons ':clicked (clicked msg))
    (cl:cons ':index (index msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CheckCircle)))
  'CheckCircle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CheckCircle)))
  'CheckCircle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckCircle)))
  "Returns string type for a service object of type '<CheckCircle>"
  "jsk_recognition_msgs/CheckCircle")