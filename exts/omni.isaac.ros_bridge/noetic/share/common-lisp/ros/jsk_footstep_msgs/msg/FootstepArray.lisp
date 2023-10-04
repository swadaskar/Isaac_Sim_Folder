; Auto-generated. Do not edit!


(cl:in-package jsk_footstep_msgs-msg)


;//! \htmlinclude FootstepArray.msg.html

(cl:defclass <FootstepArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (footsteps
    :reader footsteps
    :initarg :footsteps
    :type (cl:vector jsk_footstep_msgs-msg:Footstep)
   :initform (cl:make-array 0 :element-type 'jsk_footstep_msgs-msg:Footstep :initial-element (cl:make-instance 'jsk_footstep_msgs-msg:Footstep))))
)

(cl:defclass FootstepArray (<FootstepArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FootstepArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FootstepArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_footstep_msgs-msg:<FootstepArray> is deprecated: use jsk_footstep_msgs-msg:FootstepArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FootstepArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_footstep_msgs-msg:header-val is deprecated.  Use jsk_footstep_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'footsteps-val :lambda-list '(m))
(cl:defmethod footsteps-val ((m <FootstepArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_footstep_msgs-msg:footsteps-val is deprecated.  Use jsk_footstep_msgs-msg:footsteps instead.")
  (footsteps m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FootstepArray>) ostream)
  "Serializes a message object of type '<FootstepArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'footsteps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'footsteps))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FootstepArray>) istream)
  "Deserializes a message object of type '<FootstepArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'footsteps) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'footsteps)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'jsk_footstep_msgs-msg:Footstep))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FootstepArray>)))
  "Returns string type for a message object of type '<FootstepArray>"
  "jsk_footstep_msgs/FootstepArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FootstepArray)))
  "Returns string type for a message object of type 'FootstepArray"
  "jsk_footstep_msgs/FootstepArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FootstepArray>)))
  "Returns md5sum for a message object of type '<FootstepArray>"
  "385bc396845a4680214262a4679d83b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FootstepArray)))
  "Returns md5sum for a message object of type 'FootstepArray"
  "385bc396845a4680214262a4679d83b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FootstepArray>)))
  "Returns full string definition for message of type '<FootstepArray>"
  (cl:format cl:nil "Header header~%Footstep[] footsteps~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_footstep_msgs/Footstep~%uint8 RIGHT=2~%uint8 LEFT=1~%~%# Constants to visualize progress~%uint8 REJECTED=3~%uint8 APPROVED=4~%~%## limb_indicator values~%uint8 LLEG=1~%uint8 RLEG=2~%uint8 LARM=5~%uint8 RARM=6~%~%uint8 leg ## value should be one of limb_indicator values.~%geometry_msgs/Pose pose ## 'pose' represents nominal pose. It may be an end-effector of limb.~%duration duration~%# optional parameters~%uint32 footstep_group~%geometry_msgs/Vector3 dimensions ## cube [length(x), width(y), height(z)]~%geometry_msgs/Vector3 offset     ## offset from pose to center of cube~%float32 swing_height~%float32 cost~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FootstepArray)))
  "Returns full string definition for message of type 'FootstepArray"
  (cl:format cl:nil "Header header~%Footstep[] footsteps~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_footstep_msgs/Footstep~%uint8 RIGHT=2~%uint8 LEFT=1~%~%# Constants to visualize progress~%uint8 REJECTED=3~%uint8 APPROVED=4~%~%## limb_indicator values~%uint8 LLEG=1~%uint8 RLEG=2~%uint8 LARM=5~%uint8 RARM=6~%~%uint8 leg ## value should be one of limb_indicator values.~%geometry_msgs/Pose pose ## 'pose' represents nominal pose. It may be an end-effector of limb.~%duration duration~%# optional parameters~%uint32 footstep_group~%geometry_msgs/Vector3 dimensions ## cube [length(x), width(y), height(z)]~%geometry_msgs/Vector3 offset     ## offset from pose to center of cube~%float32 swing_height~%float32 cost~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FootstepArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'footsteps) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FootstepArray>))
  "Converts a ROS message object to a list"
  (cl:list 'FootstepArray
    (cl:cons ':header (header msg))
    (cl:cons ':footsteps (footsteps msg))
))
