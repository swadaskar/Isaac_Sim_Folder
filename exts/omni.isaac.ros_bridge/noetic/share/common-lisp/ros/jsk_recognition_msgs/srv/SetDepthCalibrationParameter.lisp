; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude SetDepthCalibrationParameter-request.msg.html

(cl:defclass <SetDepthCalibrationParameter-request> (roslisp-msg-protocol:ros-message)
  ((parameter
    :reader parameter
    :initarg :parameter
    :type jsk_recognition_msgs-msg:DepthCalibrationParameter
    :initform (cl:make-instance 'jsk_recognition_msgs-msg:DepthCalibrationParameter)))
)

(cl:defclass SetDepthCalibrationParameter-request (<SetDepthCalibrationParameter-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDepthCalibrationParameter-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDepthCalibrationParameter-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<SetDepthCalibrationParameter-request> is deprecated: use jsk_recognition_msgs-srv:SetDepthCalibrationParameter-request instead.")))

(cl:ensure-generic-function 'parameter-val :lambda-list '(m))
(cl:defmethod parameter-val ((m <SetDepthCalibrationParameter-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:parameter-val is deprecated.  Use jsk_recognition_msgs-srv:parameter instead.")
  (parameter m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDepthCalibrationParameter-request>) ostream)
  "Serializes a message object of type '<SetDepthCalibrationParameter-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'parameter) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDepthCalibrationParameter-request>) istream)
  "Deserializes a message object of type '<SetDepthCalibrationParameter-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'parameter) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDepthCalibrationParameter-request>)))
  "Returns string type for a service object of type '<SetDepthCalibrationParameter-request>"
  "jsk_recognition_msgs/SetDepthCalibrationParameterRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDepthCalibrationParameter-request)))
  "Returns string type for a service object of type 'SetDepthCalibrationParameter-request"
  "jsk_recognition_msgs/SetDepthCalibrationParameterRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDepthCalibrationParameter-request>)))
  "Returns md5sum for a message object of type '<SetDepthCalibrationParameter-request>"
  "46b1552cd9512adcf9bbec97800b0e0d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDepthCalibrationParameter-request)))
  "Returns md5sum for a message object of type 'SetDepthCalibrationParameter-request"
  "46b1552cd9512adcf9bbec97800b0e0d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDepthCalibrationParameter-request>)))
  "Returns full string definition for message of type '<SetDepthCalibrationParameter-request>"
  (cl:format cl:nil "jsk_recognition_msgs/DepthCalibrationParameter parameter~%~%================================================================================~%MSG: jsk_recognition_msgs/DepthCalibrationParameter~%# each vector stands for C(u, v)~%# C(u, v) = a_0 * u^2 + a_1 * u + a_2 * v^2 + a_3 * v + a_4~%float64[] coefficients2~%float64[] coefficients1~%float64[] coefficients0~%bool use_abs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDepthCalibrationParameter-request)))
  "Returns full string definition for message of type 'SetDepthCalibrationParameter-request"
  (cl:format cl:nil "jsk_recognition_msgs/DepthCalibrationParameter parameter~%~%================================================================================~%MSG: jsk_recognition_msgs/DepthCalibrationParameter~%# each vector stands for C(u, v)~%# C(u, v) = a_0 * u^2 + a_1 * u + a_2 * v^2 + a_3 * v + a_4~%float64[] coefficients2~%float64[] coefficients1~%float64[] coefficients0~%bool use_abs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDepthCalibrationParameter-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'parameter))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDepthCalibrationParameter-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDepthCalibrationParameter-request
    (cl:cons ':parameter (parameter msg))
))
;//! \htmlinclude SetDepthCalibrationParameter-response.msg.html

(cl:defclass <SetDepthCalibrationParameter-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetDepthCalibrationParameter-response (<SetDepthCalibrationParameter-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDepthCalibrationParameter-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDepthCalibrationParameter-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<SetDepthCalibrationParameter-response> is deprecated: use jsk_recognition_msgs-srv:SetDepthCalibrationParameter-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDepthCalibrationParameter-response>) ostream)
  "Serializes a message object of type '<SetDepthCalibrationParameter-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDepthCalibrationParameter-response>) istream)
  "Deserializes a message object of type '<SetDepthCalibrationParameter-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDepthCalibrationParameter-response>)))
  "Returns string type for a service object of type '<SetDepthCalibrationParameter-response>"
  "jsk_recognition_msgs/SetDepthCalibrationParameterResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDepthCalibrationParameter-response)))
  "Returns string type for a service object of type 'SetDepthCalibrationParameter-response"
  "jsk_recognition_msgs/SetDepthCalibrationParameterResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDepthCalibrationParameter-response>)))
  "Returns md5sum for a message object of type '<SetDepthCalibrationParameter-response>"
  "46b1552cd9512adcf9bbec97800b0e0d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDepthCalibrationParameter-response)))
  "Returns md5sum for a message object of type 'SetDepthCalibrationParameter-response"
  "46b1552cd9512adcf9bbec97800b0e0d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDepthCalibrationParameter-response>)))
  "Returns full string definition for message of type '<SetDepthCalibrationParameter-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDepthCalibrationParameter-response)))
  "Returns full string definition for message of type 'SetDepthCalibrationParameter-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDepthCalibrationParameter-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDepthCalibrationParameter-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDepthCalibrationParameter-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetDepthCalibrationParameter)))
  'SetDepthCalibrationParameter-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetDepthCalibrationParameter)))
  'SetDepthCalibrationParameter-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDepthCalibrationParameter)))
  "Returns string type for a service object of type '<SetDepthCalibrationParameter>"
  "jsk_recognition_msgs/SetDepthCalibrationParameter")