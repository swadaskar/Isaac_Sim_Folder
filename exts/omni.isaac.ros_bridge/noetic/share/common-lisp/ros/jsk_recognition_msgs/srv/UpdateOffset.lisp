; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude UpdateOffset-request.msg.html

(cl:defclass <UpdateOffset-request> (roslisp-msg-protocol:ros-message)
  ((transformation
    :reader transformation
    :initarg :transformation
    :type geometry_msgs-msg:TransformStamped
    :initform (cl:make-instance 'geometry_msgs-msg:TransformStamped)))
)

(cl:defclass UpdateOffset-request (<UpdateOffset-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateOffset-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateOffset-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<UpdateOffset-request> is deprecated: use jsk_recognition_msgs-srv:UpdateOffset-request instead.")))

(cl:ensure-generic-function 'transformation-val :lambda-list '(m))
(cl:defmethod transformation-val ((m <UpdateOffset-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:transformation-val is deprecated.  Use jsk_recognition_msgs-srv:transformation instead.")
  (transformation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateOffset-request>) ostream)
  "Serializes a message object of type '<UpdateOffset-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'transformation) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateOffset-request>) istream)
  "Deserializes a message object of type '<UpdateOffset-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'transformation) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateOffset-request>)))
  "Returns string type for a service object of type '<UpdateOffset-request>"
  "jsk_recognition_msgs/UpdateOffsetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateOffset-request)))
  "Returns string type for a service object of type 'UpdateOffset-request"
  "jsk_recognition_msgs/UpdateOffsetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateOffset-request>)))
  "Returns md5sum for a message object of type '<UpdateOffset-request>"
  "72c98a75ad1f2a3dcf256e7c072420a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateOffset-request)))
  "Returns md5sum for a message object of type 'UpdateOffset-request"
  "72c98a75ad1f2a3dcf256e7c072420a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateOffset-request>)))
  "Returns full string definition for message of type '<UpdateOffset-request>"
  (cl:format cl:nil "# UpdatePose.srv~%# This service is designed to specify localization~%# transformation manually~%geometry_msgs/TransformStamped transformation~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://wiki.ros.org/tf\">tf</a> package. ~%# See its documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateOffset-request)))
  "Returns full string definition for message of type 'UpdateOffset-request"
  (cl:format cl:nil "# UpdatePose.srv~%# This service is designed to specify localization~%# transformation manually~%geometry_msgs/TransformStamped transformation~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://wiki.ros.org/tf\">tf</a> package. ~%# See its documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateOffset-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'transformation))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateOffset-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateOffset-request
    (cl:cons ':transformation (transformation msg))
))
;//! \htmlinclude UpdateOffset-response.msg.html

(cl:defclass <UpdateOffset-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass UpdateOffset-response (<UpdateOffset-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateOffset-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateOffset-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<UpdateOffset-response> is deprecated: use jsk_recognition_msgs-srv:UpdateOffset-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateOffset-response>) ostream)
  "Serializes a message object of type '<UpdateOffset-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateOffset-response>) istream)
  "Deserializes a message object of type '<UpdateOffset-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateOffset-response>)))
  "Returns string type for a service object of type '<UpdateOffset-response>"
  "jsk_recognition_msgs/UpdateOffsetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateOffset-response)))
  "Returns string type for a service object of type 'UpdateOffset-response"
  "jsk_recognition_msgs/UpdateOffsetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateOffset-response>)))
  "Returns md5sum for a message object of type '<UpdateOffset-response>"
  "72c98a75ad1f2a3dcf256e7c072420a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateOffset-response)))
  "Returns md5sum for a message object of type 'UpdateOffset-response"
  "72c98a75ad1f2a3dcf256e7c072420a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateOffset-response>)))
  "Returns full string definition for message of type '<UpdateOffset-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateOffset-response)))
  "Returns full string definition for message of type 'UpdateOffset-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateOffset-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateOffset-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateOffset-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UpdateOffset)))
  'UpdateOffset-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UpdateOffset)))
  'UpdateOffset-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateOffset)))
  "Returns string type for a service object of type '<UpdateOffset>"
  "jsk_recognition_msgs/UpdateOffset")