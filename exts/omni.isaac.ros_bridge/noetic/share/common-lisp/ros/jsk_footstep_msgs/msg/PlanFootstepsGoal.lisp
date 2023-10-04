; Auto-generated. Do not edit!


(cl:in-package jsk_footstep_msgs-msg)


;//! \htmlinclude PlanFootstepsGoal.msg.html

(cl:defclass <PlanFootstepsGoal> (roslisp-msg-protocol:ros-message)
  ((goal_footstep
    :reader goal_footstep
    :initarg :goal_footstep
    :type jsk_footstep_msgs-msg:FootstepArray
    :initform (cl:make-instance 'jsk_footstep_msgs-msg:FootstepArray))
   (initial_footstep
    :reader initial_footstep
    :initarg :initial_footstep
    :type jsk_footstep_msgs-msg:FootstepArray
    :initform (cl:make-instance 'jsk_footstep_msgs-msg:FootstepArray))
   (timeout
    :reader timeout
    :initarg :timeout
    :type cl:real
    :initform 0))
)

(cl:defclass PlanFootstepsGoal (<PlanFootstepsGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanFootstepsGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanFootstepsGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_footstep_msgs-msg:<PlanFootstepsGoal> is deprecated: use jsk_footstep_msgs-msg:PlanFootstepsGoal instead.")))

(cl:ensure-generic-function 'goal_footstep-val :lambda-list '(m))
(cl:defmethod goal_footstep-val ((m <PlanFootstepsGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_footstep_msgs-msg:goal_footstep-val is deprecated.  Use jsk_footstep_msgs-msg:goal_footstep instead.")
  (goal_footstep m))

(cl:ensure-generic-function 'initial_footstep-val :lambda-list '(m))
(cl:defmethod initial_footstep-val ((m <PlanFootstepsGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_footstep_msgs-msg:initial_footstep-val is deprecated.  Use jsk_footstep_msgs-msg:initial_footstep instead.")
  (initial_footstep m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <PlanFootstepsGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_footstep_msgs-msg:timeout-val is deprecated.  Use jsk_footstep_msgs-msg:timeout instead.")
  (timeout m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanFootstepsGoal>) ostream)
  "Serializes a message object of type '<PlanFootstepsGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_footstep) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'initial_footstep) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'timeout)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'timeout) (cl:floor (cl:slot-value msg 'timeout)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanFootstepsGoal>) istream)
  "Deserializes a message object of type '<PlanFootstepsGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_footstep) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'initial_footstep) istream)
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timeout) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanFootstepsGoal>)))
  "Returns string type for a message object of type '<PlanFootstepsGoal>"
  "jsk_footstep_msgs/PlanFootstepsGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanFootstepsGoal)))
  "Returns string type for a message object of type 'PlanFootstepsGoal"
  "jsk_footstep_msgs/PlanFootstepsGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanFootstepsGoal>)))
  "Returns md5sum for a message object of type '<PlanFootstepsGoal>"
  "68011023a311776030c53b8ca437fae1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanFootstepsGoal)))
  "Returns md5sum for a message object of type 'PlanFootstepsGoal"
  "68011023a311776030c53b8ca437fae1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanFootstepsGoal>)))
  "Returns full string definition for message of type '<PlanFootstepsGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# goal~%FootstepArray goal_footstep~%FootstepArray initial_footstep~%duration timeout~%~%================================================================================~%MSG: jsk_footstep_msgs/FootstepArray~%Header header~%Footstep[] footsteps~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_footstep_msgs/Footstep~%uint8 RIGHT=2~%uint8 LEFT=1~%~%# Constants to visualize progress~%uint8 REJECTED=3~%uint8 APPROVED=4~%~%## limb_indicator values~%uint8 LLEG=1~%uint8 RLEG=2~%uint8 LARM=5~%uint8 RARM=6~%~%uint8 leg ## value should be one of limb_indicator values.~%geometry_msgs/Pose pose ## 'pose' represents nominal pose. It may be an end-effector of limb.~%duration duration~%# optional parameters~%uint32 footstep_group~%geometry_msgs/Vector3 dimensions ## cube [length(x), width(y), height(z)]~%geometry_msgs/Vector3 offset     ## offset from pose to center of cube~%float32 swing_height~%float32 cost~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanFootstepsGoal)))
  "Returns full string definition for message of type 'PlanFootstepsGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# goal~%FootstepArray goal_footstep~%FootstepArray initial_footstep~%duration timeout~%~%================================================================================~%MSG: jsk_footstep_msgs/FootstepArray~%Header header~%Footstep[] footsteps~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_footstep_msgs/Footstep~%uint8 RIGHT=2~%uint8 LEFT=1~%~%# Constants to visualize progress~%uint8 REJECTED=3~%uint8 APPROVED=4~%~%## limb_indicator values~%uint8 LLEG=1~%uint8 RLEG=2~%uint8 LARM=5~%uint8 RARM=6~%~%uint8 leg ## value should be one of limb_indicator values.~%geometry_msgs/Pose pose ## 'pose' represents nominal pose. It may be an end-effector of limb.~%duration duration~%# optional parameters~%uint32 footstep_group~%geometry_msgs/Vector3 dimensions ## cube [length(x), width(y), height(z)]~%geometry_msgs/Vector3 offset     ## offset from pose to center of cube~%float32 swing_height~%float32 cost~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanFootstepsGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_footstep))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'initial_footstep))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanFootstepsGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanFootstepsGoal
    (cl:cons ':goal_footstep (goal_footstep msg))
    (cl:cons ':initial_footstep (initial_footstep msg))
    (cl:cons ':timeout (timeout msg))
))
