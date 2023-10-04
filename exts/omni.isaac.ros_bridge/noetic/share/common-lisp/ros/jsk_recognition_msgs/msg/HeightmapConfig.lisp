; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude HeightmapConfig.msg.html

(cl:defclass <HeightmapConfig> (roslisp-msg-protocol:ros-message)
  ((min_x
    :reader min_x
    :initarg :min_x
    :type cl:float
    :initform 0.0)
   (max_x
    :reader max_x
    :initarg :max_x
    :type cl:float
    :initform 0.0)
   (min_y
    :reader min_y
    :initarg :min_y
    :type cl:float
    :initform 0.0)
   (max_y
    :reader max_y
    :initarg :max_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass HeightmapConfig (<HeightmapConfig>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HeightmapConfig>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HeightmapConfig)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<HeightmapConfig> is deprecated: use jsk_recognition_msgs-msg:HeightmapConfig instead.")))

(cl:ensure-generic-function 'min_x-val :lambda-list '(m))
(cl:defmethod min_x-val ((m <HeightmapConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:min_x-val is deprecated.  Use jsk_recognition_msgs-msg:min_x instead.")
  (min_x m))

(cl:ensure-generic-function 'max_x-val :lambda-list '(m))
(cl:defmethod max_x-val ((m <HeightmapConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:max_x-val is deprecated.  Use jsk_recognition_msgs-msg:max_x instead.")
  (max_x m))

(cl:ensure-generic-function 'min_y-val :lambda-list '(m))
(cl:defmethod min_y-val ((m <HeightmapConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:min_y-val is deprecated.  Use jsk_recognition_msgs-msg:min_y instead.")
  (min_y m))

(cl:ensure-generic-function 'max_y-val :lambda-list '(m))
(cl:defmethod max_y-val ((m <HeightmapConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:max_y-val is deprecated.  Use jsk_recognition_msgs-msg:max_y instead.")
  (max_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HeightmapConfig>) ostream)
  "Serializes a message object of type '<HeightmapConfig>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HeightmapConfig>) istream)
  "Deserializes a message object of type '<HeightmapConfig>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HeightmapConfig>)))
  "Returns string type for a message object of type '<HeightmapConfig>"
  "jsk_recognition_msgs/HeightmapConfig")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HeightmapConfig)))
  "Returns string type for a message object of type 'HeightmapConfig"
  "jsk_recognition_msgs/HeightmapConfig")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HeightmapConfig>)))
  "Returns md5sum for a message object of type '<HeightmapConfig>"
  "f2f90f6dd5aeedef48b062e1a4aabb89")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HeightmapConfig)))
  "Returns md5sum for a message object of type 'HeightmapConfig"
  "f2f90f6dd5aeedef48b062e1a4aabb89")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HeightmapConfig>)))
  "Returns full string definition for message of type '<HeightmapConfig>"
  (cl:format cl:nil "float32 min_x~%float32 max_x~%float32 min_y~%float32 max_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HeightmapConfig)))
  "Returns full string definition for message of type 'HeightmapConfig"
  (cl:format cl:nil "float32 min_x~%float32 max_x~%float32 min_y~%float32 max_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HeightmapConfig>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HeightmapConfig>))
  "Converts a ROS message object to a list"
  (cl:list 'HeightmapConfig
    (cl:cons ':min_x (min_x msg))
    (cl:cons ':max_x (max_x msg))
    (cl:cons ':min_y (min_y msg))
    (cl:cons ':max_y (max_y msg))
))
