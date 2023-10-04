; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-srv)


;//! \htmlinclude TowerRobotMoveCommand-request.msg.html

(cl:defclass <TowerRobotMoveCommand-request> (roslisp-msg-protocol:ros-message)
  ((robot_index
    :reader robot_index
    :initarg :robot_index
    :type cl:integer
    :initform 0)
   (plate_index
    :reader plate_index
    :initarg :plate_index
    :type cl:integer
    :initform 0)
   (from_tower
    :reader from_tower
    :initarg :from_tower
    :type cl:integer
    :initform 0)
   (to_tower
    :reader to_tower
    :initarg :to_tower
    :type cl:integer
    :initform 0)
   (option_command
    :reader option_command
    :initarg :option_command
    :type cl:integer
    :initform 0))
)

(cl:defclass TowerRobotMoveCommand-request (<TowerRobotMoveCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TowerRobotMoveCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TowerRobotMoveCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<TowerRobotMoveCommand-request> is deprecated: use jsk_recognition_msgs-srv:TowerRobotMoveCommand-request instead.")))

(cl:ensure-generic-function 'robot_index-val :lambda-list '(m))
(cl:defmethod robot_index-val ((m <TowerRobotMoveCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:robot_index-val is deprecated.  Use jsk_recognition_msgs-srv:robot_index instead.")
  (robot_index m))

(cl:ensure-generic-function 'plate_index-val :lambda-list '(m))
(cl:defmethod plate_index-val ((m <TowerRobotMoveCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:plate_index-val is deprecated.  Use jsk_recognition_msgs-srv:plate_index instead.")
  (plate_index m))

(cl:ensure-generic-function 'from_tower-val :lambda-list '(m))
(cl:defmethod from_tower-val ((m <TowerRobotMoveCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:from_tower-val is deprecated.  Use jsk_recognition_msgs-srv:from_tower instead.")
  (from_tower m))

(cl:ensure-generic-function 'to_tower-val :lambda-list '(m))
(cl:defmethod to_tower-val ((m <TowerRobotMoveCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:to_tower-val is deprecated.  Use jsk_recognition_msgs-srv:to_tower instead.")
  (to_tower m))

(cl:ensure-generic-function 'option_command-val :lambda-list '(m))
(cl:defmethod option_command-val ((m <TowerRobotMoveCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-srv:option_command-val is deprecated.  Use jsk_recognition_msgs-srv:option_command instead.")
  (option_command m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TowerRobotMoveCommand-request>)))
    "Constants for message type '<TowerRobotMoveCommand-request>"
  '((:ROBOT1 . 1)
    (:ROBOT2 . 2)
    (:ROBOT3 . 3)
    (:PLATE_SMALL . 1)
    (:PLATE_MIDDLE . 2)
    (:PLATE_LARGE . 3)
    (:TOWER_LOWEST . 1)
    (:TOWER_MIDDLE . 2)
    (:TOWER_HIGHEST . 3)
    (:TOWER_LOWEST2 . 1)
    (:OPTION_NONE . 0)
    (:OPTION_MOVE_INITIAL . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TowerRobotMoveCommand-request)))
    "Constants for message type 'TowerRobotMoveCommand-request"
  '((:ROBOT1 . 1)
    (:ROBOT2 . 2)
    (:ROBOT3 . 3)
    (:PLATE_SMALL . 1)
    (:PLATE_MIDDLE . 2)
    (:PLATE_LARGE . 3)
    (:TOWER_LOWEST . 1)
    (:TOWER_MIDDLE . 2)
    (:TOWER_HIGHEST . 3)
    (:TOWER_LOWEST2 . 1)
    (:OPTION_NONE . 0)
    (:OPTION_MOVE_INITIAL . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TowerRobotMoveCommand-request>) ostream)
  "Serializes a message object of type '<TowerRobotMoveCommand-request>"
  (cl:let* ((signed (cl:slot-value msg 'robot_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'plate_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'from_tower)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'to_tower)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'option_command)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TowerRobotMoveCommand-request>) istream)
  "Deserializes a message object of type '<TowerRobotMoveCommand-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'plate_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'from_tower) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'to_tower) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'option_command) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TowerRobotMoveCommand-request>)))
  "Returns string type for a service object of type '<TowerRobotMoveCommand-request>"
  "jsk_recognition_msgs/TowerRobotMoveCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TowerRobotMoveCommand-request)))
  "Returns string type for a service object of type 'TowerRobotMoveCommand-request"
  "jsk_recognition_msgs/TowerRobotMoveCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TowerRobotMoveCommand-request>)))
  "Returns md5sum for a message object of type '<TowerRobotMoveCommand-request>"
  "aadba056bdce0494569ab50ecd2ec90c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TowerRobotMoveCommand-request)))
  "Returns md5sum for a message object of type 'TowerRobotMoveCommand-request"
  "aadba056bdce0494569ab50ecd2ec90c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TowerRobotMoveCommand-request>)))
  "Returns full string definition for message of type '<TowerRobotMoveCommand-request>"
  (cl:format cl:nil "# a service to move robot with tower index~%byte ROBOT1=1~%byte ROBOT2=2~%byte ROBOT3=3~%~%byte PLATE_SMALL=1~%byte PLATE_MIDDLE=2~%byte PLATE_LARGE=3~%~%byte TOWER_LOWEST=1~%byte TOWER_MIDDLE=2~%byte TOWER_HIGHEST=3~%byte TOWER_LOWEST2=1~%~%byte OPTION_NONE=0~%byte OPTION_MOVE_INITIAL=1~%~%int32 robot_index~%int32 plate_index~%int32 from_tower~%int32 to_tower~%int32 option_command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TowerRobotMoveCommand-request)))
  "Returns full string definition for message of type 'TowerRobotMoveCommand-request"
  (cl:format cl:nil "# a service to move robot with tower index~%byte ROBOT1=1~%byte ROBOT2=2~%byte ROBOT3=3~%~%byte PLATE_SMALL=1~%byte PLATE_MIDDLE=2~%byte PLATE_LARGE=3~%~%byte TOWER_LOWEST=1~%byte TOWER_MIDDLE=2~%byte TOWER_HIGHEST=3~%byte TOWER_LOWEST2=1~%~%byte OPTION_NONE=0~%byte OPTION_MOVE_INITIAL=1~%~%int32 robot_index~%int32 plate_index~%int32 from_tower~%int32 to_tower~%int32 option_command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TowerRobotMoveCommand-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TowerRobotMoveCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TowerRobotMoveCommand-request
    (cl:cons ':robot_index (robot_index msg))
    (cl:cons ':plate_index (plate_index msg))
    (cl:cons ':from_tower (from_tower msg))
    (cl:cons ':to_tower (to_tower msg))
    (cl:cons ':option_command (option_command msg))
))
;//! \htmlinclude TowerRobotMoveCommand-response.msg.html

(cl:defclass <TowerRobotMoveCommand-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TowerRobotMoveCommand-response (<TowerRobotMoveCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TowerRobotMoveCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TowerRobotMoveCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-srv:<TowerRobotMoveCommand-response> is deprecated: use jsk_recognition_msgs-srv:TowerRobotMoveCommand-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TowerRobotMoveCommand-response>) ostream)
  "Serializes a message object of type '<TowerRobotMoveCommand-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TowerRobotMoveCommand-response>) istream)
  "Deserializes a message object of type '<TowerRobotMoveCommand-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TowerRobotMoveCommand-response>)))
  "Returns string type for a service object of type '<TowerRobotMoveCommand-response>"
  "jsk_recognition_msgs/TowerRobotMoveCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TowerRobotMoveCommand-response)))
  "Returns string type for a service object of type 'TowerRobotMoveCommand-response"
  "jsk_recognition_msgs/TowerRobotMoveCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TowerRobotMoveCommand-response>)))
  "Returns md5sum for a message object of type '<TowerRobotMoveCommand-response>"
  "aadba056bdce0494569ab50ecd2ec90c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TowerRobotMoveCommand-response)))
  "Returns md5sum for a message object of type 'TowerRobotMoveCommand-response"
  "aadba056bdce0494569ab50ecd2ec90c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TowerRobotMoveCommand-response>)))
  "Returns full string definition for message of type '<TowerRobotMoveCommand-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TowerRobotMoveCommand-response)))
  "Returns full string definition for message of type 'TowerRobotMoveCommand-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TowerRobotMoveCommand-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TowerRobotMoveCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TowerRobotMoveCommand-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TowerRobotMoveCommand)))
  'TowerRobotMoveCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TowerRobotMoveCommand)))
  'TowerRobotMoveCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TowerRobotMoveCommand)))
  "Returns string type for a service object of type '<TowerRobotMoveCommand>"
  "jsk_recognition_msgs/TowerRobotMoveCommand")