; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude SetTemplate-request.msg.html

(cl:defclass <SetTemplate-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (dimx
    :reader dimx
    :initarg :dimx
    :type cl:float
    :initform 0.0)
   (dimy
    :reader dimy
    :initarg :dimy
    :type cl:float
    :initform 0.0)
   (relativepose
    :reader relativepose
    :initarg :relativepose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (savefilename
    :reader savefilename
    :initarg :savefilename
    :type cl:string
    :initform ""))
)

(cl:defclass SetTemplate-request (<SetTemplate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTemplate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTemplate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<SetTemplate-request> is deprecated: use jsk_recognition_msgs-srv:SetTemplate-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <SetTemplate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:type-val is deprecated.  Use jsk_recognition_msgs-srv:type instead.")
  (type m))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <SetTemplate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:image-val is deprecated.  Use jsk_recognition_msgs-srv:image instead.")
  (image m))

(cl:ensure-generic-function 'dimx-val :lambda-list '(m))
(cl:defmethod dimx-val ((m <SetTemplate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:dimx-val is deprecated.  Use jsk_recognition_msgs-srv:dimx instead.")
  (dimx m))

(cl:ensure-generic-function 'dimy-val :lambda-list '(m))
(cl:defmethod dimy-val ((m <SetTemplate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:dimy-val is deprecated.  Use jsk_recognition_msgs-srv:dimy instead.")
  (dimy m))

(cl:ensure-generic-function 'relativepose-val :lambda-list '(m))
(cl:defmethod relativepose-val ((m <SetTemplate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:relativepose-val is deprecated.  Use jsk_recognition_msgs-srv:relativepose instead.")
  (relativepose m))

(cl:ensure-generic-function 'savefilename-val :lambda-list '(m))
(cl:defmethod savefilename-val ((m <SetTemplate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:savefilename-val is deprecated.  Use jsk_recognition_msgs-srv:savefilename instead.")
  (savefilename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTemplate-request>) ostream)
  "Serializes a message object of type '<SetTemplate-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dimx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dimy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'relativepose) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'savefilename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'savefilename))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTemplate-request>) istream)
  "Deserializes a message object of type '<SetTemplate-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dimx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dimy) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'relativepose) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'savefilename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'savefilename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTemplate-request>)))
  "Returns string type for a service object of type '<SetTemplate-request>"
  "jsk_recognition_msgs/SetTemplateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTemplate-request)))
  "Returns string type for a service object of type 'SetTemplate-request"
  "jsk_recognition_msgs/SetTemplateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTemplate-request>)))
  "Returns md5sum for a message object of type '<SetTemplate-request>"
  "116fa80f27cbdfcd76d0b57a30ef79ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTemplate-request)))
  "Returns md5sum for a message object of type 'SetTemplate-request"
  "116fa80f27cbdfcd76d0b57a30ef79ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTemplate-request>)))
  "Returns full string definition for message of type '<SetTemplate-request>"
  (cl:format cl:nil "string type~%sensor_msgs/Image image~%# dimensions of the texture in the real world (in meters)~%float32 dimx~%float32 dimy~%geometry_msgs/Pose relativepose # used to set the coordinate system of the object relative to the texture~%string savefilename # if not empty, will save the resulting pp file to here~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTemplate-request)))
  "Returns full string definition for message of type 'SetTemplate-request"
  (cl:format cl:nil "string type~%sensor_msgs/Image image~%# dimensions of the texture in the real world (in meters)~%float32 dimx~%float32 dimy~%geometry_msgs/Pose relativepose # used to set the coordinate system of the object relative to the texture~%string savefilename # if not empty, will save the resulting pp file to here~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTemplate-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'type))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'relativepose))
     4 (cl:length (cl:slot-value msg 'savefilename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTemplate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTemplate-request
    (cl:cons ':type (type msg))
    (cl:cons ':image (image msg))
    (cl:cons ':dimx (dimx msg))
    (cl:cons ':dimy (dimy msg))
    (cl:cons ':relativepose (relativepose msg))
    (cl:cons ':savefilename (savefilename msg))
))
;//! \htmlinclude SetTemplate-response.msg.html

(cl:defclass <SetTemplate-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetTemplate-response (<SetTemplate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTemplate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTemplate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<SetTemplate-response> is deprecated: use jsk_recognition_msgs-srv:SetTemplate-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTemplate-response>) ostream)
  "Serializes a message object of type '<SetTemplate-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTemplate-response>) istream)
  "Deserializes a message object of type '<SetTemplate-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTemplate-response>)))
  "Returns string type for a service object of type '<SetTemplate-response>"
  "jsk_recognition_msgs/SetTemplateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTemplate-response)))
  "Returns string type for a service object of type 'SetTemplate-response"
  "jsk_recognition_msgs/SetTemplateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTemplate-response>)))
  "Returns md5sum for a message object of type '<SetTemplate-response>"
  "116fa80f27cbdfcd76d0b57a30ef79ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTemplate-response)))
  "Returns md5sum for a message object of type 'SetTemplate-response"
  "116fa80f27cbdfcd76d0b57a30ef79ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTemplate-response>)))
  "Returns full string definition for message of type '<SetTemplate-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTemplate-response)))
  "Returns full string definition for message of type 'SetTemplate-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTemplate-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTemplate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTemplate-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetTemplate)))
  'SetTemplate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetTemplate)))
  'SetTemplate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTemplate)))
  "Returns string type for a service object of type '<SetTemplate>"
  "jsk_recognition_msgs/SetTemplate")