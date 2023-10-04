; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude SegmentStamped.msg.html

(cl:defclass <SegmentStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (segment
    :reader segment
    :initarg :segment
    :type jsk_recognition_msgs-msg:Segment
    :initform (cl:make-instance 'jsk_recognition_msgs-msg:Segment)))
)

(cl:defclass SegmentStamped (<SegmentStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SegmentStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SegmentStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<SegmentStamped> is deprecated: use jsk_recognition_msgs-msg:SegmentStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SegmentStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'segment-val :lambda-list '(m))
(cl:defmethod segment-val ((m <SegmentStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:segment-val is deprecated.  Use jsk_recognition_msgs-msg:segment instead.")
  (segment m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SegmentStamped>) ostream)
  "Serializes a message object of type '<SegmentStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'segment) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SegmentStamped>) istream)
  "Deserializes a message object of type '<SegmentStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'segment) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SegmentStamped>)))
  "Returns string type for a message object of type '<SegmentStamped>"
  "jsk_recognition_msgs/SegmentStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SegmentStamped)))
  "Returns string type for a message object of type 'SegmentStamped"
  "jsk_recognition_msgs/SegmentStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SegmentStamped>)))
  "Returns md5sum for a message object of type '<SegmentStamped>"
  "1f2fbdf9b9a242110bee5312e7718d1f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SegmentStamped)))
  "Returns md5sum for a message object of type 'SegmentStamped"
  "1f2fbdf9b9a242110bee5312e7718d1f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SegmentStamped>)))
  "Returns full string definition for message of type '<SegmentStamped>"
  (cl:format cl:nil "Header header~%Segment segment~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/Segment~%geometry_msgs/Point start_point~%geometry_msgs/Point end_point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SegmentStamped)))
  "Returns full string definition for message of type 'SegmentStamped"
  (cl:format cl:nil "Header header~%Segment segment~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/Segment~%geometry_msgs/Point start_point~%geometry_msgs/Point end_point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SegmentStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'segment))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SegmentStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'SegmentStamped
    (cl:cons ':header (header msg))
    (cl:cons ':segment (segment msg))
))
