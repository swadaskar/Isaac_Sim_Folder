; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude HistogramWithRangeBin.msg.html

(cl:defclass <HistogramWithRangeBin> (roslisp-msg-protocol:ros-message)
  ((min_value
    :reader min_value
    :initarg :min_value
    :type cl:float
    :initform 0.0)
   (max_value
    :reader max_value
    :initarg :max_value
    :type cl:float
    :initform 0.0)
   (count
    :reader count
    :initarg :count
    :type cl:integer
    :initform 0))
)

(cl:defclass HistogramWithRangeBin (<HistogramWithRangeBin>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HistogramWithRangeBin>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HistogramWithRangeBin)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<HistogramWithRangeBin> is deprecated: use jsk_recognition_msgs-msg:HistogramWithRangeBin instead.")))

(cl:ensure-generic-function 'min_value-val :lambda-list '(m))
(cl:defmethod min_value-val ((m <HistogramWithRangeBin>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:min_value-val is deprecated.  Use jsk_recognition_msgs-msg:min_value instead.")
  (min_value m))

(cl:ensure-generic-function 'max_value-val :lambda-list '(m))
(cl:defmethod max_value-val ((m <HistogramWithRangeBin>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:max_value-val is deprecated.  Use jsk_recognition_msgs-msg:max_value instead.")
  (max_value m))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <HistogramWithRangeBin>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:count-val is deprecated.  Use jsk_recognition_msgs-msg:count instead.")
  (count m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HistogramWithRangeBin>) ostream)
  "Serializes a message object of type '<HistogramWithRangeBin>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'min_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'count)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HistogramWithRangeBin>) istream)
  "Deserializes a message object of type '<HistogramWithRangeBin>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_value) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_value) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'count)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HistogramWithRangeBin>)))
  "Returns string type for a message object of type '<HistogramWithRangeBin>"
  "jsk_recognition_msgs/HistogramWithRangeBin")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HistogramWithRangeBin)))
  "Returns string type for a message object of type 'HistogramWithRangeBin"
  "jsk_recognition_msgs/HistogramWithRangeBin")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HistogramWithRangeBin>)))
  "Returns md5sum for a message object of type '<HistogramWithRangeBin>"
  "a7fe6c3021fcba2c6357f3db21601551")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HistogramWithRangeBin)))
  "Returns md5sum for a message object of type 'HistogramWithRangeBin"
  "a7fe6c3021fcba2c6357f3db21601551")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HistogramWithRangeBin>)))
  "Returns full string definition for message of type '<HistogramWithRangeBin>"
  (cl:format cl:nil "float64 min_value~%float64 max_value~%uint32 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HistogramWithRangeBin)))
  "Returns full string definition for message of type 'HistogramWithRangeBin"
  (cl:format cl:nil "float64 min_value~%float64 max_value~%uint32 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HistogramWithRangeBin>))
  (cl:+ 0
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HistogramWithRangeBin>))
  "Converts a ROS message object to a list"
  (cl:list 'HistogramWithRangeBin
    (cl:cons ':min_value (min_value msg))
    (cl:cons ':max_value (max_value msg))
    (cl:cons ':count (count msg))
))
