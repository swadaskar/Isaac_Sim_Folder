; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude HumanSkeletonArray.msg.html

(cl:defclass <HumanSkeletonArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (human_ids
    :reader human_ids
    :initarg :human_ids
    :type (cl:vector std_msgs-msg:Int32)
   :initform (cl:make-array 0 :element-type 'std_msgs-msg:Int32 :initial-element (cl:make-instance 'std_msgs-msg:Int32)))
   (skeletons
    :reader skeletons
    :initarg :skeletons
    :type (cl:vector jsk_recognition_msgs-msg:HumanSkeleton)
   :initform (cl:make-array 0 :element-type 'jsk_recognition_msgs-msg:HumanSkeleton :initial-element (cl:make-instance 'jsk_recognition_msgs-msg:HumanSkeleton))))
)

(cl:defclass HumanSkeletonArray (<HumanSkeletonArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HumanSkeletonArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HumanSkeletonArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<HumanSkeletonArray> is deprecated: use jsk_recognition_msgs-msg:HumanSkeletonArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HumanSkeletonArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'human_ids-val :lambda-list '(m))
(cl:defmethod human_ids-val ((m <HumanSkeletonArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:human_ids-val is deprecated.  Use jsk_recognition_msgs-msg:human_ids instead.")
  (human_ids m))

(cl:ensure-generic-function 'skeletons-val :lambda-list '(m))
(cl:defmethod skeletons-val ((m <HumanSkeletonArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:skeletons-val is deprecated.  Use jsk_recognition_msgs-msg:skeletons instead.")
  (skeletons m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HumanSkeletonArray>) ostream)
  "Serializes a message object of type '<HumanSkeletonArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'human_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'human_ids))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'skeletons))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'skeletons))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HumanSkeletonArray>) istream)
  "Deserializes a message object of type '<HumanSkeletonArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'human_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'human_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'std_msgs-msg:Int32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'skeletons) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'skeletons)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'jsk_recognition_msgs-msg:HumanSkeleton))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HumanSkeletonArray>)))
  "Returns string type for a message object of type '<HumanSkeletonArray>"
  "jsk_recognition_msgs/HumanSkeletonArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HumanSkeletonArray)))
  "Returns string type for a message object of type 'HumanSkeletonArray"
  "jsk_recognition_msgs/HumanSkeletonArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HumanSkeletonArray>)))
  "Returns md5sum for a message object of type '<HumanSkeletonArray>"
  "4013ce44abe9aafa9055e095797a9d50")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HumanSkeletonArray)))
  "Returns md5sum for a message object of type 'HumanSkeletonArray"
  "4013ce44abe9aafa9055e095797a9d50")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HumanSkeletonArray>)))
  "Returns full string definition for message of type '<HumanSkeletonArray>"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/Int32[] human_ids~%jsk_recognition_msgs/HumanSkeleton[] skeletons~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: jsk_recognition_msgs/HumanSkeleton~%std_msgs/Header header~%string[] bone_names~%jsk_recognition_msgs/Segment[] bones~%~%================================================================================~%MSG: jsk_recognition_msgs/Segment~%geometry_msgs/Point start_point~%geometry_msgs/Point end_point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HumanSkeletonArray)))
  "Returns full string definition for message of type 'HumanSkeletonArray"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/Int32[] human_ids~%jsk_recognition_msgs/HumanSkeleton[] skeletons~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: jsk_recognition_msgs/HumanSkeleton~%std_msgs/Header header~%string[] bone_names~%jsk_recognition_msgs/Segment[] bones~%~%================================================================================~%MSG: jsk_recognition_msgs/Segment~%geometry_msgs/Point start_point~%geometry_msgs/Point end_point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HumanSkeletonArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'human_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'skeletons) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HumanSkeletonArray>))
  "Converts a ROS message object to a list"
  (cl:list 'HumanSkeletonArray
    (cl:cons ':header (header msg))
    (cl:cons ':human_ids (human_ids msg))
    (cl:cons ':skeletons (skeletons msg))
))
