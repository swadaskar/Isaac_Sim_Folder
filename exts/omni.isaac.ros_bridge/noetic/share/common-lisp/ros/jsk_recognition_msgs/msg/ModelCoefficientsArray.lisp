; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude ModelCoefficientsArray.msg.html

(cl:defclass <ModelCoefficientsArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (coefficients
    :reader coefficients
    :initarg :coefficients
    :type (cl:vector pcl_msgs-msg:ModelCoefficients)
   :initform (cl:make-array 0 :element-type 'pcl_msgs-msg:ModelCoefficients :initial-element (cl:make-instance 'pcl_msgs-msg:ModelCoefficients))))
)

(cl:defclass ModelCoefficientsArray (<ModelCoefficientsArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModelCoefficientsArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModelCoefficientsArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<ModelCoefficientsArray> is deprecated: use jsk_recognition_msgs-msg:ModelCoefficientsArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ModelCoefficientsArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'coefficients-val :lambda-list '(m))
(cl:defmethod coefficients-val ((m <ModelCoefficientsArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:coefficients-val is deprecated.  Use jsk_recognition_msgs-msg:coefficients instead.")
  (coefficients m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModelCoefficientsArray>) ostream)
  "Serializes a message object of type '<ModelCoefficientsArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'coefficients))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'coefficients))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModelCoefficientsArray>) istream)
  "Deserializes a message object of type '<ModelCoefficientsArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'coefficients) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'coefficients)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pcl_msgs-msg:ModelCoefficients))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModelCoefficientsArray>)))
  "Returns string type for a message object of type '<ModelCoefficientsArray>"
  "jsk_recognition_msgs/ModelCoefficientsArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModelCoefficientsArray)))
  "Returns string type for a message object of type 'ModelCoefficientsArray"
  "jsk_recognition_msgs/ModelCoefficientsArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModelCoefficientsArray>)))
  "Returns md5sum for a message object of type '<ModelCoefficientsArray>"
  "059efee897c3f4ae027a493e30c4c26b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModelCoefficientsArray)))
  "Returns md5sum for a message object of type 'ModelCoefficientsArray"
  "059efee897c3f4ae027a493e30c4c26b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModelCoefficientsArray>)))
  "Returns full string definition for message of type '<ModelCoefficientsArray>"
  (cl:format cl:nil "# ModelCoefficientsArray is used to represent coefficients of model~%# for each segmented clusters.~%# Simply put, ModelCoefficientsArray is a list of ModelCoefficients.~%Header header~%pcl_msgs/ModelCoefficients[] coefficients~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pcl_msgs/ModelCoefficients~%Header header~%float32[] values~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModelCoefficientsArray)))
  "Returns full string definition for message of type 'ModelCoefficientsArray"
  (cl:format cl:nil "# ModelCoefficientsArray is used to represent coefficients of model~%# for each segmented clusters.~%# Simply put, ModelCoefficientsArray is a list of ModelCoefficients.~%Header header~%pcl_msgs/ModelCoefficients[] coefficients~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pcl_msgs/ModelCoefficients~%Header header~%float32[] values~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModelCoefficientsArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'coefficients) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModelCoefficientsArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ModelCoefficientsArray
    (cl:cons ':header (header msg))
    (cl:cons ':coefficients (coefficients msg))
))
