; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude SparseOccupancyGridColumn.msg.html

(cl:defclass <SparseOccupancyGridColumn> (roslisp-msg-protocol:ros-message)
  ((column_index
    :reader column_index
    :initarg :column_index
    :type cl:integer
    :initform 0)
   (cells
    :reader cells
    :initarg :cells
    :type (cl:vector jsk_recognition_msgs-msg:SparseOccupancyGridCell)
   :initform (cl:make-array 0 :element-type 'jsk_recognition_msgs-msg:SparseOccupancyGridCell :initial-element (cl:make-instance 'jsk_recognition_msgs-msg:SparseOccupancyGridCell))))
)

(cl:defclass SparseOccupancyGridColumn (<SparseOccupancyGridColumn>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SparseOccupancyGridColumn>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SparseOccupancyGridColumn)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<SparseOccupancyGridColumn> is deprecated: use jsk_recognition_msgs-msg:SparseOccupancyGridColumn instead.")))

(cl:ensure-generic-function 'column_index-val :lambda-list '(m))
(cl:defmethod column_index-val ((m <SparseOccupancyGridColumn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:column_index-val is deprecated.  Use jsk_recognition_msgs-msg:column_index instead.")
  (column_index m))

(cl:ensure-generic-function 'cells-val :lambda-list '(m))
(cl:defmethod cells-val ((m <SparseOccupancyGridColumn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:cells-val is deprecated.  Use jsk_recognition_msgs-msg:cells instead.")
  (cells m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SparseOccupancyGridColumn>) ostream)
  "Serializes a message object of type '<SparseOccupancyGridColumn>"
  (cl:let* ((signed (cl:slot-value msg 'column_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cells))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cells))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SparseOccupancyGridColumn>) istream)
  "Deserializes a message object of type '<SparseOccupancyGridColumn>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'column_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cells) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cells)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'jsk_recognition_msgs-msg:SparseOccupancyGridCell))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SparseOccupancyGridColumn>)))
  "Returns string type for a message object of type '<SparseOccupancyGridColumn>"
  "jsk_recognition_msgs/SparseOccupancyGridColumn")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SparseOccupancyGridColumn)))
  "Returns string type for a message object of type 'SparseOccupancyGridColumn"
  "jsk_recognition_msgs/SparseOccupancyGridColumn")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SparseOccupancyGridColumn>)))
  "Returns md5sum for a message object of type '<SparseOccupancyGridColumn>"
  "55074b193e722d5ead092ffe27f06522")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SparseOccupancyGridColumn)))
  "Returns md5sum for a message object of type 'SparseOccupancyGridColumn"
  "55074b193e722d5ead092ffe27f06522")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SparseOccupancyGridColumn>)))
  "Returns full string definition for message of type '<SparseOccupancyGridColumn>"
  (cl:format cl:nil "int32 column_index~%SparseOccupancyGridCell[] cells~%~%================================================================================~%MSG: jsk_recognition_msgs/SparseOccupancyGridCell~%int32 row_index~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SparseOccupancyGridColumn)))
  "Returns full string definition for message of type 'SparseOccupancyGridColumn"
  (cl:format cl:nil "int32 column_index~%SparseOccupancyGridCell[] cells~%~%================================================================================~%MSG: jsk_recognition_msgs/SparseOccupancyGridCell~%int32 row_index~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SparseOccupancyGridColumn>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cells) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SparseOccupancyGridColumn>))
  "Converts a ROS message object to a list"
  (cl:list 'SparseOccupancyGridColumn
    (cl:cons ':column_index (column_index msg))
    (cl:cons ':cells (cells msg))
))
