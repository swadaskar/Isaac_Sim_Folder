; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude SnapFootstep-request.msg.html

(cl:defclass <SnapFootstep-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type jsk_footstep_msgs-msg:FootstepArray
    :initform (cl:make-instance 'jsk_footstep_msgs-msg:FootstepArray)))
)

(cl:defclass SnapFootstep-request (<SnapFootstep-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SnapFootstep-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SnapFootstep-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<SnapFootstep-request> is deprecated: use jsk_recognition_msgs-srv:SnapFootstep-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SnapFootstep-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:input-val is deprecated.  Use jsk_recognition_msgs-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SnapFootstep-request>) ostream)
  "Serializes a message object of type '<SnapFootstep-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SnapFootstep-request>) istream)
  "Deserializes a message object of type '<SnapFootstep-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SnapFootstep-request>)))
  "Returns string type for a service object of type '<SnapFootstep-request>"
  "jsk_recognition_msgs/SnapFootstepRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SnapFootstep-request)))
  "Returns string type for a service object of type 'SnapFootstep-request"
  "jsk_recognition_msgs/SnapFootstepRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SnapFootstep-request>)))
  "Returns md5sum for a message object of type '<SnapFootstep-request>"
  "ea63ea1160bdd6c252d3a568e9f2ee4f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SnapFootstep-request)))
  "Returns md5sum for a message object of type 'SnapFootstep-request"
  "ea63ea1160bdd6c252d3a568e9f2ee4f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SnapFootstep-request>)))
  "Returns full string definition for message of type '<SnapFootstep-request>"
  (cl:format cl:nil "jsk_footstep_msgs/FootstepArray input~%~%================================================================================~%MSG: jsk_footstep_msgs/FootstepArray~%Header header~%Footstep[] footsteps~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_footstep_msgs/Footstep~%uint8 RIGHT=2~%uint8 LEFT=1~%~%# Constants to visualize progress~%uint8 REJECTED=3~%uint8 APPROVED=4~%~%## limb_indicator values~%uint8 LLEG=1~%uint8 RLEG=2~%uint8 LARM=5~%uint8 RARM=6~%~%uint8 leg ## value should be one of limb_indicator values.~%geometry_msgs/Pose pose ## 'pose' represents nominal pose. It may be an end-effector of limb.~%duration duration~%# optional parameters~%uint32 footstep_group~%geometry_msgs/Vector3 dimensions ## cube [length(x), width(y), height(z)]~%geometry_msgs/Vector3 offset     ## offset from pose to center of cube~%float32 swing_height~%float32 cost~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SnapFootstep-request)))
  "Returns full string definition for message of type 'SnapFootstep-request"
  (cl:format cl:nil "jsk_footstep_msgs/FootstepArray input~%~%================================================================================~%MSG: jsk_footstep_msgs/FootstepArray~%Header header~%Footstep[] footsteps~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_footstep_msgs/Footstep~%uint8 RIGHT=2~%uint8 LEFT=1~%~%# Constants to visualize progress~%uint8 REJECTED=3~%uint8 APPROVED=4~%~%## limb_indicator values~%uint8 LLEG=1~%uint8 RLEG=2~%uint8 LARM=5~%uint8 RARM=6~%~%uint8 leg ## value should be one of limb_indicator values.~%geometry_msgs/Pose pose ## 'pose' represents nominal pose. It may be an end-effector of limb.~%duration duration~%# optional parameters~%uint32 footstep_group~%geometry_msgs/Vector3 dimensions ## cube [length(x), width(y), height(z)]~%geometry_msgs/Vector3 offset     ## offset from pose to center of cube~%float32 swing_height~%float32 cost~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SnapFootstep-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SnapFootstep-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SnapFootstep-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SnapFootstep-response.msg.html

(cl:defclass <SnapFootstep-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type jsk_footstep_msgs-msg:FootstepArray
    :initform (cl:make-instance 'jsk_footstep_msgs-msg:FootstepArray)))
)

(cl:defclass SnapFootstep-response (<SnapFootstep-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SnapFootstep-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SnapFootstep-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<SnapFootstep-response> is deprecated: use jsk_recognition_msgs-srv:SnapFootstep-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SnapFootstep-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:output-val is deprecated.  Use jsk_recognition_msgs-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SnapFootstep-response>) ostream)
  "Serializes a message object of type '<SnapFootstep-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SnapFootstep-response>) istream)
  "Deserializes a message object of type '<SnapFootstep-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SnapFootstep-response>)))
  "Returns string type for a service object of type '<SnapFootstep-response>"
  "jsk_recognition_msgs/SnapFootstepResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SnapFootstep-response)))
  "Returns string type for a service object of type 'SnapFootstep-response"
  "jsk_recognition_msgs/SnapFootstepResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SnapFootstep-response>)))
  "Returns md5sum for a message object of type '<SnapFootstep-response>"
  "ea63ea1160bdd6c252d3a568e9f2ee4f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SnapFootstep-response)))
  "Returns md5sum for a message object of type 'SnapFootstep-response"
  "ea63ea1160bdd6c252d3a568e9f2ee4f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SnapFootstep-response>)))
  "Returns full string definition for message of type '<SnapFootstep-response>"
  (cl:format cl:nil "jsk_footstep_msgs/FootstepArray output~%~%~%================================================================================~%MSG: jsk_footstep_msgs/FootstepArray~%Header header~%Footstep[] footsteps~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_footstep_msgs/Footstep~%uint8 RIGHT=2~%uint8 LEFT=1~%~%# Constants to visualize progress~%uint8 REJECTED=3~%uint8 APPROVED=4~%~%## limb_indicator values~%uint8 LLEG=1~%uint8 RLEG=2~%uint8 LARM=5~%uint8 RARM=6~%~%uint8 leg ## value should be one of limb_indicator values.~%geometry_msgs/Pose pose ## 'pose' represents nominal pose. It may be an end-effector of limb.~%duration duration~%# optional parameters~%uint32 footstep_group~%geometry_msgs/Vector3 dimensions ## cube [length(x), width(y), height(z)]~%geometry_msgs/Vector3 offset     ## offset from pose to center of cube~%float32 swing_height~%float32 cost~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SnapFootstep-response)))
  "Returns full string definition for message of type 'SnapFootstep-response"
  (cl:format cl:nil "jsk_footstep_msgs/FootstepArray output~%~%~%================================================================================~%MSG: jsk_footstep_msgs/FootstepArray~%Header header~%Footstep[] footsteps~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_footstep_msgs/Footstep~%uint8 RIGHT=2~%uint8 LEFT=1~%~%# Constants to visualize progress~%uint8 REJECTED=3~%uint8 APPROVED=4~%~%## limb_indicator values~%uint8 LLEG=1~%uint8 RLEG=2~%uint8 LARM=5~%uint8 RARM=6~%~%uint8 leg ## value should be one of limb_indicator values.~%geometry_msgs/Pose pose ## 'pose' represents nominal pose. It may be an end-effector of limb.~%duration duration~%# optional parameters~%uint32 footstep_group~%geometry_msgs/Vector3 dimensions ## cube [length(x), width(y), height(z)]~%geometry_msgs/Vector3 offset     ## offset from pose to center of cube~%float32 swing_height~%float32 cost~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SnapFootstep-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SnapFootstep-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SnapFootstep-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SnapFootstep)))
  'SnapFootstep-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SnapFootstep)))
  'SnapFootstep-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SnapFootstep)))
  "Returns string type for a service object of type '<SnapFootstep>"
  "jsk_recognition_msgs/SnapFootstep")