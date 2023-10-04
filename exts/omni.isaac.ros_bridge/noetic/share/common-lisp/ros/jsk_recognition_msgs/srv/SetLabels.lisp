; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude SetLabels-request.msg.html

(cl:defclass <SetLabels-request> (roslisp-msg-protocol:ros-message)
  ((labels
    :reader labels
    :initarg :labels
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass SetLabels-request (<SetLabels-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetLabels-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetLabels-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<SetLabels-request> is deprecated: use jsk_recognition_msgs-srv:SetLabels-request instead.")))

(cl:ensure-generic-function 'labels-val :lambda-list '(m))
(cl:defmethod labels-val ((m <SetLabels-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:labels-val is deprecated.  Use jsk_recognition_msgs-srv:labels instead.")
  (labels m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetLabels-request>) ostream)
  "Serializes a message object of type '<SetLabels-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'labels))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    ))
   (cl:slot-value msg 'labels))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetLabels-request>) istream)
  "Deserializes a message object of type '<SetLabels-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'labels) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'labels)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetLabels-request>)))
  "Returns string type for a service object of type '<SetLabels-request>"
  "jsk_recognition_msgs/SetLabelsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLabels-request)))
  "Returns string type for a service object of type 'SetLabels-request"
  "jsk_recognition_msgs/SetLabelsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetLabels-request>)))
  "Returns md5sum for a message object of type '<SetLabels-request>"
  "f23605aaff6e30abc41a406ed99d7e0a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetLabels-request)))
  "Returns md5sum for a message object of type 'SetLabels-request"
  "f23605aaff6e30abc41a406ed99d7e0a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetLabels-request>)))
  "Returns full string definition for message of type '<SetLabels-request>"
  (cl:format cl:nil "int64[] labels~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetLabels-request)))
  "Returns full string definition for message of type 'SetLabels-request"
  (cl:format cl:nil "int64[] labels~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetLabels-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'labels) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetLabels-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetLabels-request
    (cl:cons ':labels (labels msg))
))
;//! \htmlinclude SetLabels-response.msg.html

(cl:defclass <SetLabels-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetLabels-response (<SetLabels-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetLabels-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetLabels-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<SetLabels-response> is deprecated: use jsk_recognition_msgs-srv:SetLabels-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetLabels-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:success-val is deprecated.  Use jsk_recognition_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetLabels-response>) ostream)
  "Serializes a message object of type '<SetLabels-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetLabels-response>) istream)
  "Deserializes a message object of type '<SetLabels-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetLabels-response>)))
  "Returns string type for a service object of type '<SetLabels-response>"
  "jsk_recognition_msgs/SetLabelsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLabels-response)))
  "Returns string type for a service object of type 'SetLabels-response"
  "jsk_recognition_msgs/SetLabelsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetLabels-response>)))
  "Returns md5sum for a message object of type '<SetLabels-response>"
  "f23605aaff6e30abc41a406ed99d7e0a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetLabels-response)))
  "Returns md5sum for a message object of type 'SetLabels-response"
  "f23605aaff6e30abc41a406ed99d7e0a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetLabels-response>)))
  "Returns full string definition for message of type '<SetLabels-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetLabels-response)))
  "Returns full string definition for message of type 'SetLabels-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetLabels-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetLabels-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetLabels-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetLabels)))
  'SetLabels-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetLabels)))
  'SetLabels-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLabels)))
  "Returns string type for a service object of type '<SetLabels>"
  "jsk_recognition_msgs/SetLabels")