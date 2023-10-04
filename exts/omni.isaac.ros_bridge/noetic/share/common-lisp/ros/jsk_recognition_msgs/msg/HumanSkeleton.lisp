; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude HumanSkeleton.msg.html

(cl:defclass <HumanSkeleton> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (bone_names
    :reader bone_names
    :initarg :bone_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (bones
    :reader bones
    :initarg :bones
    :type (cl:vector jsk_recognition_msgs-msg:Segment)
   :initform (cl:make-array 0 :element-type 'jsk_recognition_msgs-msg:Segment :initial-element (cl:make-instance 'jsk_recognition_msgs-msg:Segment))))
)

(cl:defclass HumanSkeleton (<HumanSkeleton>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HumanSkeleton>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HumanSkeleton)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<HumanSkeleton> is deprecated: use jsk_recognition_msgs-msg:HumanSkeleton instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HumanSkeleton>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'bone_names-val :lambda-list '(m))
(cl:defmethod bone_names-val ((m <HumanSkeleton>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:bone_names-val is deprecated.  Use jsk_recognition_msgs-msg:bone_names instead.")
  (bone_names m))

(cl:ensure-generic-function 'bones-val :lambda-list '(m))
(cl:defmethod bones-val ((m <HumanSkeleton>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:bones-val is deprecated.  Use jsk_recognition_msgs-msg:bones instead.")
  (bones m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HumanSkeleton>) ostream)
  "Serializes a message object of type '<HumanSkeleton>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'bone_names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'bone_names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'bones))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'bones))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HumanSkeleton>) istream)
  "Deserializes a message object of type '<HumanSkeleton>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'bone_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'bone_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'bones) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'bones)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'jsk_recognition_msgs-msg:Segment))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HumanSkeleton>)))
  "Returns string type for a message object of type '<HumanSkeleton>"
  "jsk_recognition_msgs/HumanSkeleton")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HumanSkeleton)))
  "Returns string type for a message object of type 'HumanSkeleton"
  "jsk_recognition_msgs/HumanSkeleton")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HumanSkeleton>)))
  "Returns md5sum for a message object of type '<HumanSkeleton>"
  "b8098e195ee1734e91bc2e38c969a23f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HumanSkeleton)))
  "Returns md5sum for a message object of type 'HumanSkeleton"
  "b8098e195ee1734e91bc2e38c969a23f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HumanSkeleton>)))
  "Returns full string definition for message of type '<HumanSkeleton>"
  (cl:format cl:nil "std_msgs/Header header~%string[] bone_names~%jsk_recognition_msgs/Segment[] bones~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/Segment~%geometry_msgs/Point start_point~%geometry_msgs/Point end_point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HumanSkeleton)))
  "Returns full string definition for message of type 'HumanSkeleton"
  (cl:format cl:nil "std_msgs/Header header~%string[] bone_names~%jsk_recognition_msgs/Segment[] bones~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/Segment~%geometry_msgs/Point start_point~%geometry_msgs/Point end_point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HumanSkeleton>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'bone_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'bones) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HumanSkeleton>))
  "Converts a ROS message object to a list"
  (cl:list 'HumanSkeleton
    (cl:cons ':header (header msg))
    (cl:cons ':bone_names (bone_names msg))
    (cl:cons ':bones (bones msg))
))
