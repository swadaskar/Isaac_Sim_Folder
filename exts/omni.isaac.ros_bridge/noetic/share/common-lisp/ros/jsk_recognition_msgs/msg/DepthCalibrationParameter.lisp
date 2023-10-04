; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude DepthCalibrationParameter.msg.html

(cl:defclass <DepthCalibrationParameter> (roslisp-msg-protocol:ros-message)
  ((coefficients2
    :reader coefficients2
    :initarg :coefficients2
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (coefficients1
    :reader coefficients1
    :initarg :coefficients1
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (coefficients0
    :reader coefficients0
    :initarg :coefficients0
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (use_abs
    :reader use_abs
    :initarg :use_abs
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DepthCalibrationParameter (<DepthCalibrationParameter>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DepthCalibrationParameter>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DepthCalibrationParameter)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<DepthCalibrationParameter> is deprecated: use jsk_recognition_msgs-msg:DepthCalibrationParameter instead.")))

(cl:ensure-generic-function 'coefficients2-val :lambda-list '(m))
(cl:defmethod coefficients2-val ((m <DepthCalibrationParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:coefficients2-val is deprecated.  Use jsk_recognition_msgs-msg:coefficients2 instead.")
  (coefficients2 m))

(cl:ensure-generic-function 'coefficients1-val :lambda-list '(m))
(cl:defmethod coefficients1-val ((m <DepthCalibrationParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:coefficients1-val is deprecated.  Use jsk_recognition_msgs-msg:coefficients1 instead.")
  (coefficients1 m))

(cl:ensure-generic-function 'coefficients0-val :lambda-list '(m))
(cl:defmethod coefficients0-val ((m <DepthCalibrationParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:coefficients0-val is deprecated.  Use jsk_recognition_msgs-msg:coefficients0 instead.")
  (coefficients0 m))

(cl:ensure-generic-function 'use_abs-val :lambda-list '(m))
(cl:defmethod use_abs-val ((m <DepthCalibrationParameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:use_abs-val is deprecated.  Use jsk_recognition_msgs-msg:use_abs instead.")
  (use_abs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DepthCalibrationParameter>) ostream)
  "Serializes a message object of type '<DepthCalibrationParameter>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'coefficients2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'coefficients2))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'coefficients1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'coefficients1))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'coefficients0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'coefficients0))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_abs) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DepthCalibrationParameter>) istream)
  "Deserializes a message object of type '<DepthCalibrationParameter>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'coefficients2) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'coefficients2)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'coefficients1) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'coefficients1)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'coefficients0) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'coefficients0)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'use_abs) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DepthCalibrationParameter>)))
  "Returns string type for a message object of type '<DepthCalibrationParameter>"
  "jsk_recognition_msgs/DepthCalibrationParameter")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DepthCalibrationParameter)))
  "Returns string type for a message object of type 'DepthCalibrationParameter"
  "jsk_recognition_msgs/DepthCalibrationParameter")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DepthCalibrationParameter>)))
  "Returns md5sum for a message object of type '<DepthCalibrationParameter>"
  "d8318983ee0a76ad66ecf4b504350888")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DepthCalibrationParameter)))
  "Returns md5sum for a message object of type 'DepthCalibrationParameter"
  "d8318983ee0a76ad66ecf4b504350888")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DepthCalibrationParameter>)))
  "Returns full string definition for message of type '<DepthCalibrationParameter>"
  (cl:format cl:nil "# each vector stands for C(u, v)~%# C(u, v) = a_0 * u^2 + a_1 * u + a_2 * v^2 + a_3 * v + a_4~%float64[] coefficients2~%float64[] coefficients1~%float64[] coefficients0~%bool use_abs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DepthCalibrationParameter)))
  "Returns full string definition for message of type 'DepthCalibrationParameter"
  (cl:format cl:nil "# each vector stands for C(u, v)~%# C(u, v) = a_0 * u^2 + a_1 * u + a_2 * v^2 + a_3 * v + a_4~%float64[] coefficients2~%float64[] coefficients1~%float64[] coefficients0~%bool use_abs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DepthCalibrationParameter>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'coefficients2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'coefficients1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'coefficients0) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DepthCalibrationParameter>))
  "Converts a ROS message object to a list"
  (cl:list 'DepthCalibrationParameter
    (cl:cons ':coefficients2 (coefficients2 msg))
    (cl:cons ':coefficients1 (coefficients1 msg))
    (cl:cons ':coefficients0 (coefficients0 msg))
    (cl:cons ':use_abs (use_abs msg))
))
