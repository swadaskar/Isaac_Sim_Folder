; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude SaveMesh-request.msg.html

(cl:defclass <SaveMesh-request> (roslisp-msg-protocol:ros-message)
  ((ground_frame_id
    :reader ground_frame_id
    :initarg :ground_frame_id
    :type cl:string
    :initform "")
   (box
    :reader box
    :initarg :box
    :type jsk_recognition_msgs-msg:BoundingBox
    :initform (cl:make-instance 'jsk_recognition_msgs-msg:BoundingBox)))
)

(cl:defclass SaveMesh-request (<SaveMesh-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SaveMesh-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SaveMesh-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<SaveMesh-request> is deprecated: use jsk_recognition_msgs-srv:SaveMesh-request instead.")))

(cl:ensure-generic-function 'ground_frame_id-val :lambda-list '(m))
(cl:defmethod ground_frame_id-val ((m <SaveMesh-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:ground_frame_id-val is deprecated.  Use jsk_recognition_msgs-srv:ground_frame_id instead.")
  (ground_frame_id m))

(cl:ensure-generic-function 'box-val :lambda-list '(m))
(cl:defmethod box-val ((m <SaveMesh-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:box-val is deprecated.  Use jsk_recognition_msgs-srv:box instead.")
  (box m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SaveMesh-request>) ostream)
  "Serializes a message object of type '<SaveMesh-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ground_frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ground_frame_id))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'box) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SaveMesh-request>) istream)
  "Deserializes a message object of type '<SaveMesh-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ground_frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ground_frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'box) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SaveMesh-request>)))
  "Returns string type for a service object of type '<SaveMesh-request>"
  "jsk_recognition_msgs/SaveMeshRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaveMesh-request)))
  "Returns string type for a service object of type 'SaveMesh-request"
  "jsk_recognition_msgs/SaveMeshRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SaveMesh-request>)))
  "Returns md5sum for a message object of type '<SaveMesh-request>"
  "bac972d77fa349fd55cd1ee98f47a7b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SaveMesh-request)))
  "Returns md5sum for a message object of type 'SaveMesh-request"
  "bac972d77fa349fd55cd1ee98f47a7b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SaveMesh-request>)))
  "Returns full string definition for message of type '<SaveMesh-request>"
  (cl:format cl:nil "string ground_frame_id  # to create solid model~%jsk_recognition_msgs/BoundingBox box  # to crop mesh~%~%================================================================================~%MSG: jsk_recognition_msgs/BoundingBox~%# BoundingBox represents a oriented bounding box.~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)~%# You can use this field to hold value such as likelihood~%float32 value~%uint32 label~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SaveMesh-request)))
  "Returns full string definition for message of type 'SaveMesh-request"
  (cl:format cl:nil "string ground_frame_id  # to create solid model~%jsk_recognition_msgs/BoundingBox box  # to crop mesh~%~%================================================================================~%MSG: jsk_recognition_msgs/BoundingBox~%# BoundingBox represents a oriented bounding box.~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)~%# You can use this field to hold value such as likelihood~%float32 value~%uint32 label~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SaveMesh-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'ground_frame_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'box))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SaveMesh-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SaveMesh-request
    (cl:cons ':ground_frame_id (ground_frame_id msg))
    (cl:cons ':box (box msg))
))
;//! \htmlinclude SaveMesh-response.msg.html

(cl:defclass <SaveMesh-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SaveMesh-response (<SaveMesh-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SaveMesh-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SaveMesh-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<SaveMesh-response> is deprecated: use jsk_recognition_msgs-srv:SaveMesh-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SaveMesh-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:success-val is deprecated.  Use jsk_recognition_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SaveMesh-response>) ostream)
  "Serializes a message object of type '<SaveMesh-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SaveMesh-response>) istream)
  "Deserializes a message object of type '<SaveMesh-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SaveMesh-response>)))
  "Returns string type for a service object of type '<SaveMesh-response>"
  "jsk_recognition_msgs/SaveMeshResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaveMesh-response)))
  "Returns string type for a service object of type 'SaveMesh-response"
  "jsk_recognition_msgs/SaveMeshResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SaveMesh-response>)))
  "Returns md5sum for a message object of type '<SaveMesh-response>"
  "bac972d77fa349fd55cd1ee98f47a7b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SaveMesh-response)))
  "Returns md5sum for a message object of type 'SaveMesh-response"
  "bac972d77fa349fd55cd1ee98f47a7b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SaveMesh-response>)))
  "Returns full string definition for message of type '<SaveMesh-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SaveMesh-response)))
  "Returns full string definition for message of type 'SaveMesh-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SaveMesh-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SaveMesh-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SaveMesh-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SaveMesh)))
  'SaveMesh-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SaveMesh)))
  'SaveMesh-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaveMesh)))
  "Returns string type for a service object of type '<SaveMesh>"
  "jsk_recognition_msgs/SaveMesh")