; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude BoundingBoxMovement.msg.html

(cl:defclass <BoundingBoxMovement> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (box
    :reader box
    :initarg :box
    :type jsk_recognition_msgs-msg:BoundingBox
    :initform (cl:make-instance 'jsk_recognition_msgs-msg:BoundingBox))
   (handle_pose
    :reader handle_pose
    :initarg :handle_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (destination
    :reader destination
    :initarg :destination
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass BoundingBoxMovement (<BoundingBoxMovement>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BoundingBoxMovement>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BoundingBoxMovement)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<BoundingBoxMovement> is deprecated: use jsk_recognition_msgs-msg:BoundingBoxMovement instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BoundingBoxMovement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'box-val :lambda-list '(m))
(cl:defmethod box-val ((m <BoundingBoxMovement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:box-val is deprecated.  Use jsk_recognition_msgs-msg:box instead.")
  (box m))

(cl:ensure-generic-function 'handle_pose-val :lambda-list '(m))
(cl:defmethod handle_pose-val ((m <BoundingBoxMovement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:handle_pose-val is deprecated.  Use jsk_recognition_msgs-msg:handle_pose instead.")
  (handle_pose m))

(cl:ensure-generic-function 'destination-val :lambda-list '(m))
(cl:defmethod destination-val ((m <BoundingBoxMovement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:destination-val is deprecated.  Use jsk_recognition_msgs-msg:destination instead.")
  (destination m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BoundingBoxMovement>) ostream)
  "Serializes a message object of type '<BoundingBoxMovement>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'box) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'destination) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BoundingBoxMovement>) istream)
  "Deserializes a message object of type '<BoundingBoxMovement>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'box) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'destination) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BoundingBoxMovement>)))
  "Returns string type for a message object of type '<BoundingBoxMovement>"
  "jsk_recognition_msgs/BoundingBoxMovement")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BoundingBoxMovement)))
  "Returns string type for a message object of type 'BoundingBoxMovement"
  "jsk_recognition_msgs/BoundingBoxMovement")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BoundingBoxMovement>)))
  "Returns md5sum for a message object of type '<BoundingBoxMovement>"
  "60aeb288c030d1b8a67ddeedc91812f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BoundingBoxMovement)))
  "Returns md5sum for a message object of type 'BoundingBoxMovement"
  "60aeb288c030d1b8a67ddeedc91812f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BoundingBoxMovement>)))
  "Returns full string definition for message of type '<BoundingBoxMovement>"
  (cl:format cl:nil "Header header~%BoundingBox box~%geometry_msgs/Pose handle_pose~%geometry_msgs/PoseStamped destination~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/BoundingBox~%# BoundingBox represents a oriented bounding box.~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)~%# You can use this field to hold value such as likelihood~%float32 value~%uint32 label~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BoundingBoxMovement)))
  "Returns full string definition for message of type 'BoundingBoxMovement"
  (cl:format cl:nil "Header header~%BoundingBox box~%geometry_msgs/Pose handle_pose~%geometry_msgs/PoseStamped destination~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/BoundingBox~%# BoundingBox represents a oriented bounding box.~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)~%# You can use this field to hold value such as likelihood~%float32 value~%uint32 label~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BoundingBoxMovement>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'box))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'destination))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BoundingBoxMovement>))
  "Converts a ROS message object to a list"
  (cl:list 'BoundingBoxMovement
    (cl:cons ':header (header msg))
    (cl:cons ':box (box msg))
    (cl:cons ':handle_pose (handle_pose msg))
    (cl:cons ':destination (destination msg))
))
