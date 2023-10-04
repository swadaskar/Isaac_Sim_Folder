; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude CallSnapIt-request.msg.html

(cl:defclass <CallSnapIt-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type jsk_recognition_msgs-msg:SnapItRequest
    :initform (cl:make-instance 'jsk_recognition_msgs-msg:SnapItRequest)))
)

(cl:defclass CallSnapIt-request (<CallSnapIt-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CallSnapIt-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CallSnapIt-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<CallSnapIt-request> is deprecated: use jsk_recognition_msgs-srv:CallSnapIt-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <CallSnapIt-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:request-val is deprecated.  Use jsk_recognition_msgs-srv:request instead.")
  (request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CallSnapIt-request>) ostream)
  "Serializes a message object of type '<CallSnapIt-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'request) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CallSnapIt-request>) istream)
  "Deserializes a message object of type '<CallSnapIt-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'request) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CallSnapIt-request>)))
  "Returns string type for a service object of type '<CallSnapIt-request>"
  "jsk_recognition_msgs/CallSnapItRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CallSnapIt-request)))
  "Returns string type for a service object of type 'CallSnapIt-request"
  "jsk_recognition_msgs/CallSnapItRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CallSnapIt-request>)))
  "Returns md5sum for a message object of type '<CallSnapIt-request>"
  "00529f1ccb0137f3a14b6d5b281f9a3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CallSnapIt-request)))
  "Returns md5sum for a message object of type 'CallSnapIt-request"
  "00529f1ccb0137f3a14b6d5b281f9a3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CallSnapIt-request>)))
  "Returns full string definition for message of type '<CallSnapIt-request>"
  (cl:format cl:nil "jsk_recognition_msgs/SnapItRequest request~%~%================================================================================~%MSG: jsk_recognition_msgs/SnapItRequest~%Header header~%uint8 MODEL_PLANE=0~%uint8 MODEL_CYLINDER=1~%uint8 model_type~%~%geometry_msgs/PolygonStamped target_plane~%~%geometry_msgs/PointStamped center~%geometry_msgs/Vector3Stamped direction~%float64 radius~%float64 height~%# parameters, 0 means ~%float64 max_distance~%float64 eps_angle~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PolygonStamped~%# This represents a Polygon with reference coordinate frame and timestamp~%Header header~%Polygon polygon~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3Stamped~%# This represents a Vector3 with reference coordinate frame and timestamp~%Header header~%Vector3 vector~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CallSnapIt-request)))
  "Returns full string definition for message of type 'CallSnapIt-request"
  (cl:format cl:nil "jsk_recognition_msgs/SnapItRequest request~%~%================================================================================~%MSG: jsk_recognition_msgs/SnapItRequest~%Header header~%uint8 MODEL_PLANE=0~%uint8 MODEL_CYLINDER=1~%uint8 model_type~%~%geometry_msgs/PolygonStamped target_plane~%~%geometry_msgs/PointStamped center~%geometry_msgs/Vector3Stamped direction~%float64 radius~%float64 height~%# parameters, 0 means ~%float64 max_distance~%float64 eps_angle~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PolygonStamped~%# This represents a Polygon with reference coordinate frame and timestamp~%Header header~%Polygon polygon~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3Stamped~%# This represents a Vector3 with reference coordinate frame and timestamp~%Header header~%Vector3 vector~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CallSnapIt-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'request))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CallSnapIt-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CallSnapIt-request
    (cl:cons ':request (request msg))
))
;//! \htmlinclude CallSnapIt-response.msg.html

(cl:defclass <CallSnapIt-response> (roslisp-msg-protocol:ros-message)
  ((transformation
    :reader transformation
    :initarg :transformation
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass CallSnapIt-response (<CallSnapIt-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CallSnapIt-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CallSnapIt-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<CallSnapIt-response> is deprecated: use jsk_recognition_msgs-srv:CallSnapIt-response instead.")))

(cl:ensure-generic-function 'transformation-val :lambda-list '(m))
(cl:defmethod transformation-val ((m <CallSnapIt-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:transformation-val is deprecated.  Use jsk_recognition_msgs-srv:transformation instead.")
  (transformation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CallSnapIt-response>) ostream)
  "Serializes a message object of type '<CallSnapIt-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'transformation) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CallSnapIt-response>) istream)
  "Deserializes a message object of type '<CallSnapIt-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'transformation) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CallSnapIt-response>)))
  "Returns string type for a service object of type '<CallSnapIt-response>"
  "jsk_recognition_msgs/CallSnapItResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CallSnapIt-response)))
  "Returns string type for a service object of type 'CallSnapIt-response"
  "jsk_recognition_msgs/CallSnapItResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CallSnapIt-response>)))
  "Returns md5sum for a message object of type '<CallSnapIt-response>"
  "00529f1ccb0137f3a14b6d5b281f9a3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CallSnapIt-response)))
  "Returns md5sum for a message object of type 'CallSnapIt-response"
  "00529f1ccb0137f3a14b6d5b281f9a3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CallSnapIt-response>)))
  "Returns full string definition for message of type '<CallSnapIt-response>"
  (cl:format cl:nil "geometry_msgs/Pose transformation~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CallSnapIt-response)))
  "Returns full string definition for message of type 'CallSnapIt-response"
  (cl:format cl:nil "geometry_msgs/Pose transformation~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CallSnapIt-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'transformation))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CallSnapIt-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CallSnapIt-response
    (cl:cons ':transformation (transformation msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CallSnapIt)))
  'CallSnapIt-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CallSnapIt)))
  'CallSnapIt-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CallSnapIt)))
  "Returns string type for a service object of type '<CallSnapIt>"
  "jsk_recognition_msgs/CallSnapIt")