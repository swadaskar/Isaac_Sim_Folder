; Auto-generated. Do not edit!


(cl:in-package jsk_footstep_msgs-msg)


;//! \htmlinclude ExecFootstepsFeedback.msg.html

(cl:defclass <ExecFootstepsFeedback> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ExecFootstepsFeedback (<ExecFootstepsFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecFootstepsFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecFootstepsFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_footstep_msgs-msg:<ExecFootstepsFeedback> is deprecated: use jsk_footstep_msgs-msg:ExecFootstepsFeedback instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecFootstepsFeedback>) ostream)
  "Serializes a message object of type '<ExecFootstepsFeedback>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecFootstepsFeedback>) istream)
  "Deserializes a message object of type '<ExecFootstepsFeedback>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecFootstepsFeedback>)))
  "Returns string type for a message object of type '<ExecFootstepsFeedback>"
  "jsk_footstep_msgs/ExecFootstepsFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecFootstepsFeedback)))
  "Returns string type for a message object of type 'ExecFootstepsFeedback"
  "jsk_footstep_msgs/ExecFootstepsFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecFootstepsFeedback>)))
  "Returns md5sum for a message object of type '<ExecFootstepsFeedback>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecFootstepsFeedback)))
  "Returns md5sum for a message object of type 'ExecFootstepsFeedback"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecFootstepsFeedback>)))
  "Returns full string definition for message of type '<ExecFootstepsFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# feedback~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecFootstepsFeedback)))
  "Returns full string definition for message of type 'ExecFootstepsFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# feedback~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecFootstepsFeedback>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecFootstepsFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecFootstepsFeedback
))
