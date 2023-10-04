; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude TrackerStatus.msg.html

(cl:defclass <TrackerStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (is_tracking
    :reader is_tracking
    :initarg :is_tracking
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass TrackerStatus (<TrackerStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackerStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackerStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<TrackerStatus> is deprecated: use jsk_recognition_msgs-msg:TrackerStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrackerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'is_tracking-val :lambda-list '(m))
(cl:defmethod is_tracking-val ((m <TrackerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:is_tracking-val is deprecated.  Use jsk_recognition_msgs-msg:is_tracking instead.")
  (is_tracking m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackerStatus>) ostream)
  "Serializes a message object of type '<TrackerStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_tracking) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackerStatus>) istream)
  "Deserializes a message object of type '<TrackerStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'is_tracking) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackerStatus>)))
  "Returns string type for a message object of type '<TrackerStatus>"
  "jsk_recognition_msgs/TrackerStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackerStatus)))
  "Returns string type for a message object of type 'TrackerStatus"
  "jsk_recognition_msgs/TrackerStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackerStatus>)))
  "Returns md5sum for a message object of type '<TrackerStatus>"
  "78e8f06a6b728df052dd181bfcf6c26e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackerStatus)))
  "Returns md5sum for a message object of type 'TrackerStatus"
  "78e8f06a6b728df052dd181bfcf6c26e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackerStatus>)))
  "Returns full string definition for message of type '<TrackerStatus>"
  (cl:format cl:nil "Header header~%# if true, it means the tracker is trying to track object,~%# else, the tracker think scene is stable and no need to track~%# to save computational resource.~%bool is_tracking~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackerStatus)))
  "Returns full string definition for message of type 'TrackerStatus"
  (cl:format cl:nil "Header header~%# if true, it means the tracker is trying to track object,~%# else, the tracker think scene is stable and no need to track~%# to save computational resource.~%bool is_tracking~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackerStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackerStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackerStatus
    (cl:cons ':header (header msg))
    (cl:cons ':is_tracking (is_tracking msg))
))
