; Auto-generated. Do not edit!


(cl:in-package cortex_control-msg)


;//! \htmlinclude CortexCommandAck.msg.html

(cl:defclass <CortexCommandAck> (roslisp-msg-protocol:ros-message)
  ((cortex_command_time
    :reader cortex_command_time
    :initarg :cortex_command_time
    :type cl:real
    :initform 0)
   (cortex_command_id
    :reader cortex_command_id
    :initarg :cortex_command_id
    :type cl:integer
    :initform 0)
   (time_offset
    :reader time_offset
    :initarg :time_offset
    :type cl:real
    :initform 0))
)

(cl:defclass CortexCommandAck (<CortexCommandAck>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CortexCommandAck>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CortexCommandAck)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cortex_control-msg:<CortexCommandAck> is deprecated: use cortex_control-msg:CortexCommandAck instead.")))

(cl:ensure-generic-function 'cortex_command_time-val :lambda-list '(m))
(cl:defmethod cortex_command_time-val ((m <CortexCommandAck>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cortex_control-msg:cortex_command_time-val is deprecated.  Use cortex_control-msg:cortex_command_time instead.")
  (cortex_command_time m))

(cl:ensure-generic-function 'cortex_command_id-val :lambda-list '(m))
(cl:defmethod cortex_command_id-val ((m <CortexCommandAck>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cortex_control-msg:cortex_command_id-val is deprecated.  Use cortex_control-msg:cortex_command_id instead.")
  (cortex_command_id m))

(cl:ensure-generic-function 'time_offset-val :lambda-list '(m))
(cl:defmethod time_offset-val ((m <CortexCommandAck>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cortex_control-msg:time_offset-val is deprecated.  Use cortex_control-msg:time_offset instead.")
  (time_offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CortexCommandAck>) ostream)
  "Serializes a message object of type '<CortexCommandAck>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'cortex_command_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'cortex_command_time) (cl:floor (cl:slot-value msg 'cortex_command_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let* ((signed (cl:slot-value msg 'cortex_command_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time_offset)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time_offset) (cl:floor (cl:slot-value msg 'time_offset)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CortexCommandAck>) istream)
  "Deserializes a message object of type '<CortexCommandAck>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cortex_command_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cortex_command_id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time_offset) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CortexCommandAck>)))
  "Returns string type for a message object of type '<CortexCommandAck>"
  "cortex_control/CortexCommandAck")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CortexCommandAck)))
  "Returns string type for a message object of type 'CortexCommandAck"
  "cortex_control/CortexCommandAck")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CortexCommandAck>)))
  "Returns md5sum for a message object of type '<CortexCommandAck>"
  "81e929ba8529002a2be583a8c6600952")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CortexCommandAck)))
  "Returns md5sum for a message object of type 'CortexCommandAck"
  "81e929ba8529002a2be583a8c6600952")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CortexCommandAck>)))
  "Returns full string definition for message of type '<CortexCommandAck>"
  (cl:format cl:nil "# Command time stamp of the latest Cortex command received.~%time cortex_command_time~%~%# ID of the latest command received.~%int64 cortex_command_id~%~%# If there is a slight accumulative clock rate difference between the Cortex~%# commander and the low-level controller, the time offset gives how much~%# further ahead the controller's clock is from the Cortex commander's clock (note~%# it can be negative). So synchronizing the clocks would entail~%#~%#    <cortex_commander_time_synced> = <cortex_commander_time> + time_offset~%duration time_offset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CortexCommandAck)))
  "Returns full string definition for message of type 'CortexCommandAck"
  (cl:format cl:nil "# Command time stamp of the latest Cortex command received.~%time cortex_command_time~%~%# ID of the latest command received.~%int64 cortex_command_id~%~%# If there is a slight accumulative clock rate difference between the Cortex~%# commander and the low-level controller, the time offset gives how much~%# further ahead the controller's clock is from the Cortex commander's clock (note~%# it can be negative). So synchronizing the clocks would entail~%#~%#    <cortex_commander_time_synced> = <cortex_commander_time> + time_offset~%duration time_offset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CortexCommandAck>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CortexCommandAck>))
  "Converts a ROS message object to a list"
  (cl:list 'CortexCommandAck
    (cl:cons ':cortex_command_time (cortex_command_time msg))
    (cl:cons ':cortex_command_id (cortex_command_id msg))
    (cl:cons ':time_offset (time_offset msg))
))
