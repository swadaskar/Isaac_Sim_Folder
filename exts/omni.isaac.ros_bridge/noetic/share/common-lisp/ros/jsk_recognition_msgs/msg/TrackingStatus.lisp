; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude TrackingStatus.msg.html

(cl:defclass <TrackingStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (is_lost
    :reader is_lost
    :initarg :is_lost
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass TrackingStatus (<TrackingStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackingStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackingStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<TrackingStatus> is deprecated: use jsk_recognition_msgs-msg:TrackingStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrackingStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'is_lost-val :lambda-list '(m))
(cl:defmethod is_lost-val ((m <TrackingStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:is_lost-val is deprecated.  Use jsk_recognition_msgs-msg:is_lost instead.")
  (is_lost m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackingStatus>) ostream)
  "Serializes a message object of type '<TrackingStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_lost) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackingStatus>) istream)
  "Deserializes a message object of type '<TrackingStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'is_lost) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackingStatus>)))
  "Returns string type for a message object of type '<TrackingStatus>"
  "jsk_recognition_msgs/TrackingStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackingStatus)))
  "Returns string type for a message object of type 'TrackingStatus"
  "jsk_recognition_msgs/TrackingStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackingStatus>)))
  "Returns md5sum for a message object of type '<TrackingStatus>"
  "2e374cd736cf06cc3e69b0c76b8cb117")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackingStatus)))
  "Returns md5sum for a message object of type 'TrackingStatus"
  "2e374cd736cf06cc3e69b0c76b8cb117")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackingStatus>)))
  "Returns full string definition for message of type '<TrackingStatus>"
  (cl:format cl:nil "Header header~%~%# if true, it means the tracker abandon to track the object.~%bool is_lost~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackingStatus)))
  "Returns full string definition for message of type 'TrackingStatus"
  (cl:format cl:nil "Header header~%~%# if true, it means the tracker abandon to track the object.~%bool is_lost~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackingStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackingStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackingStatus
    (cl:cons ':header (header msg))
    (cl:cons ':is_lost (is_lost msg))
))
