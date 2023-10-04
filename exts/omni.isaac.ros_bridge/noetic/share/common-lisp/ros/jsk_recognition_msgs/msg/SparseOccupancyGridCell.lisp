; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude SparseOccupancyGridCell.msg.html

(cl:defclass <SparseOccupancyGridCell> (roslisp-msg-protocol:ros-message)
  ((row_index
    :reader row_index
    :initarg :row_index
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass SparseOccupancyGridCell (<SparseOccupancyGridCell>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SparseOccupancyGridCell>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SparseOccupancyGridCell)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<SparseOccupancyGridCell> is deprecated: use jsk_recognition_msgs-msg:SparseOccupancyGridCell instead.")))

(cl:ensure-generic-function 'row_index-val :lambda-list '(m))
(cl:defmethod row_index-val ((m <SparseOccupancyGridCell>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:row_index-val is deprecated.  Use jsk_recognition_msgs-msg:row_index instead.")
  (row_index m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SparseOccupancyGridCell>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:value-val is deprecated.  Use jsk_recognition_msgs-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SparseOccupancyGridCell>) ostream)
  "Serializes a message object of type '<SparseOccupancyGridCell>"
  (cl:let* ((signed (cl:slot-value msg 'row_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SparseOccupancyGridCell>) istream)
  "Deserializes a message object of type '<SparseOccupancyGridCell>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'row_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SparseOccupancyGridCell>)))
  "Returns string type for a message object of type '<SparseOccupancyGridCell>"
  "jsk_recognition_msgs/SparseOccupancyGridCell")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SparseOccupancyGridCell)))
  "Returns string type for a message object of type 'SparseOccupancyGridCell"
  "jsk_recognition_msgs/SparseOccupancyGridCell")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SparseOccupancyGridCell>)))
  "Returns md5sum for a message object of type '<SparseOccupancyGridCell>"
  "a5179e922852f82ee6322db1f097ab64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SparseOccupancyGridCell)))
  "Returns md5sum for a message object of type 'SparseOccupancyGridCell"
  "a5179e922852f82ee6322db1f097ab64")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SparseOccupancyGridCell>)))
  "Returns full string definition for message of type '<SparseOccupancyGridCell>"
  (cl:format cl:nil "int32 row_index~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SparseOccupancyGridCell)))
  "Returns full string definition for message of type 'SparseOccupancyGridCell"
  (cl:format cl:nil "int32 row_index~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SparseOccupancyGridCell>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SparseOccupancyGridCell>))
  "Converts a ROS message object to a list"
  (cl:list 'SparseOccupancyGridCell
    (cl:cons ':row_index (row_index msg))
    (cl:cons ':value (value msg))
))
