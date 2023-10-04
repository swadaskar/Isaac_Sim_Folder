; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude NonMaximumSuppression-request.msg.html

(cl:defclass <NonMaximumSuppression-request> (roslisp-msg-protocol:ros-message)
  ((rect
    :reader rect
    :initarg :rect
    :type (cl:vector jsk_recognition_msgs-msg:Rect)
   :initform (cl:make-array 0 :element-type 'jsk_recognition_msgs-msg:Rect :initial-element (cl:make-instance 'jsk_recognition_msgs-msg:Rect)))
   (threshold
    :reader threshold
    :initarg :threshold
    :type cl:float
    :initform 0.0))
)

(cl:defclass NonMaximumSuppression-request (<NonMaximumSuppression-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NonMaximumSuppression-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NonMaximumSuppression-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<NonMaximumSuppression-request> is deprecated: use jsk_recognition_msgs-srv:NonMaximumSuppression-request instead.")))

(cl:ensure-generic-function 'rect-val :lambda-list '(m))
(cl:defmethod rect-val ((m <NonMaximumSuppression-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:rect-val is deprecated.  Use jsk_recognition_msgs-srv:rect instead.")
  (rect m))

(cl:ensure-generic-function 'threshold-val :lambda-list '(m))
(cl:defmethod threshold-val ((m <NonMaximumSuppression-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:threshold-val is deprecated.  Use jsk_recognition_msgs-srv:threshold instead.")
  (threshold m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NonMaximumSuppression-request>) ostream)
  "Serializes a message object of type '<NonMaximumSuppression-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rect))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'rect))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'threshold))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NonMaximumSuppression-request>) istream)
  "Deserializes a message object of type '<NonMaximumSuppression-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rect) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rect)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'jsk_recognition_msgs-msg:Rect))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'threshold) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NonMaximumSuppression-request>)))
  "Returns string type for a service object of type '<NonMaximumSuppression-request>"
  "jsk_recognition_msgs/NonMaximumSuppressionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NonMaximumSuppression-request)))
  "Returns string type for a service object of type 'NonMaximumSuppression-request"
  "jsk_recognition_msgs/NonMaximumSuppressionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NonMaximumSuppression-request>)))
  "Returns md5sum for a message object of type '<NonMaximumSuppression-request>"
  "b5f1e6a38f5b631194f786d010dab9fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NonMaximumSuppression-request)))
  "Returns md5sum for a message object of type 'NonMaximumSuppression-request"
  "b5f1e6a38f5b631194f786d010dab9fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NonMaximumSuppression-request>)))
  "Returns full string definition for message of type '<NonMaximumSuppression-request>"
  (cl:format cl:nil "jsk_recognition_msgs/Rect[] rect~%float32 threshold~%~%================================================================================~%MSG: jsk_recognition_msgs/Rect~%int32 x~%int32 y~%int32 width~%int32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NonMaximumSuppression-request)))
  "Returns full string definition for message of type 'NonMaximumSuppression-request"
  (cl:format cl:nil "jsk_recognition_msgs/Rect[] rect~%float32 threshold~%~%================================================================================~%MSG: jsk_recognition_msgs/Rect~%int32 x~%int32 y~%int32 width~%int32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NonMaximumSuppression-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rect) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NonMaximumSuppression-request>))
  "Converts a ROS message object to a list"
  (cl:list 'NonMaximumSuppression-request
    (cl:cons ':rect (rect msg))
    (cl:cons ':threshold (threshold msg))
))
;//! \htmlinclude NonMaximumSuppression-response.msg.html

(cl:defclass <NonMaximumSuppression-response> (roslisp-msg-protocol:ros-message)
  ((bbox
    :reader bbox
    :initarg :bbox
    :type (cl:vector jsk_recognition_msgs-msg:Rect)
   :initform (cl:make-array 0 :element-type 'jsk_recognition_msgs-msg:Rect :initial-element (cl:make-instance 'jsk_recognition_msgs-msg:Rect)))
   (bbox_count
    :reader bbox_count
    :initarg :bbox_count
    :type cl:integer
    :initform 0))
)

(cl:defclass NonMaximumSuppression-response (<NonMaximumSuppression-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NonMaximumSuppression-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NonMaximumSuppression-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<NonMaximumSuppression-response> is deprecated: use jsk_recognition_msgs-srv:NonMaximumSuppression-response instead.")))

(cl:ensure-generic-function 'bbox-val :lambda-list '(m))
(cl:defmethod bbox-val ((m <NonMaximumSuppression-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:bbox-val is deprecated.  Use jsk_recognition_msgs-srv:bbox instead.")
  (bbox m))

(cl:ensure-generic-function 'bbox_count-val :lambda-list '(m))
(cl:defmethod bbox_count-val ((m <NonMaximumSuppression-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:bbox_count-val is deprecated.  Use jsk_recognition_msgs-srv:bbox_count instead.")
  (bbox_count m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NonMaximumSuppression-response>) ostream)
  "Serializes a message object of type '<NonMaximumSuppression-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'bbox))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'bbox))
  (cl:let* ((signed (cl:slot-value msg 'bbox_count)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NonMaximumSuppression-response>) istream)
  "Deserializes a message object of type '<NonMaximumSuppression-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'bbox) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'bbox)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'jsk_recognition_msgs-msg:Rect))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bbox_count) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NonMaximumSuppression-response>)))
  "Returns string type for a service object of type '<NonMaximumSuppression-response>"
  "jsk_recognition_msgs/NonMaximumSuppressionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NonMaximumSuppression-response)))
  "Returns string type for a service object of type 'NonMaximumSuppression-response"
  "jsk_recognition_msgs/NonMaximumSuppressionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NonMaximumSuppression-response>)))
  "Returns md5sum for a message object of type '<NonMaximumSuppression-response>"
  "b5f1e6a38f5b631194f786d010dab9fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NonMaximumSuppression-response)))
  "Returns md5sum for a message object of type 'NonMaximumSuppression-response"
  "b5f1e6a38f5b631194f786d010dab9fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NonMaximumSuppression-response>)))
  "Returns full string definition for message of type '<NonMaximumSuppression-response>"
  (cl:format cl:nil "jsk_recognition_msgs/Rect[] bbox~%int64 bbox_count~%~%~%================================================================================~%MSG: jsk_recognition_msgs/Rect~%int32 x~%int32 y~%int32 width~%int32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NonMaximumSuppression-response)))
  "Returns full string definition for message of type 'NonMaximumSuppression-response"
  (cl:format cl:nil "jsk_recognition_msgs/Rect[] bbox~%int64 bbox_count~%~%~%================================================================================~%MSG: jsk_recognition_msgs/Rect~%int32 x~%int32 y~%int32 width~%int32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NonMaximumSuppression-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'bbox) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NonMaximumSuppression-response>))
  "Converts a ROS message object to a list"
  (cl:list 'NonMaximumSuppression-response
    (cl:cons ':bbox (bbox msg))
    (cl:cons ':bbox_count (bbox_count msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'NonMaximumSuppression)))
  'NonMaximumSuppression-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'NonMaximumSuppression)))
  'NonMaximumSuppression-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NonMaximumSuppression)))
  "Returns string type for a service object of type '<NonMaximumSuppression>"
  "jsk_recognition_msgs/NonMaximumSuppression")