; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude PlotDataArray.msg.html

(cl:defclass <PlotDataArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type (cl:vector jsk_recognition_msgs-msg:PlotData)
   :initform (cl:make-array 0 :element-type 'jsk_recognition_msgs-msg:PlotData :initial-element (cl:make-instance 'jsk_recognition_msgs-msg:PlotData)))
   (no_legend
    :reader no_legend
    :initarg :no_legend
    :type cl:boolean
    :initform cl:nil)
   (legend_font_size
    :reader legend_font_size
    :initarg :legend_font_size
    :type cl:float
    :initform 0.0)
   (max_x
    :reader max_x
    :initarg :max_x
    :type cl:float
    :initform 0.0)
   (min_x
    :reader min_x
    :initarg :min_x
    :type cl:float
    :initform 0.0)
   (min_y
    :reader min_y
    :initarg :min_y
    :type cl:float
    :initform 0.0)
   (max_y
    :reader max_y
    :initarg :max_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass PlotDataArray (<PlotDataArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlotDataArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlotDataArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<PlotDataArray> is deprecated: use jsk_recognition_msgs-msg:PlotDataArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PlotDataArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <PlotDataArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:data-val is deprecated.  Use jsk_recognition_msgs-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'no_legend-val :lambda-list '(m))
(cl:defmethod no_legend-val ((m <PlotDataArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:no_legend-val is deprecated.  Use jsk_recognition_msgs-msg:no_legend instead.")
  (no_legend m))

(cl:ensure-generic-function 'legend_font_size-val :lambda-list '(m))
(cl:defmethod legend_font_size-val ((m <PlotDataArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:legend_font_size-val is deprecated.  Use jsk_recognition_msgs-msg:legend_font_size instead.")
  (legend_font_size m))

(cl:ensure-generic-function 'max_x-val :lambda-list '(m))
(cl:defmethod max_x-val ((m <PlotDataArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:max_x-val is deprecated.  Use jsk_recognition_msgs-msg:max_x instead.")
  (max_x m))

(cl:ensure-generic-function 'min_x-val :lambda-list '(m))
(cl:defmethod min_x-val ((m <PlotDataArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:min_x-val is deprecated.  Use jsk_recognition_msgs-msg:min_x instead.")
  (min_x m))

(cl:ensure-generic-function 'min_y-val :lambda-list '(m))
(cl:defmethod min_y-val ((m <PlotDataArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:min_y-val is deprecated.  Use jsk_recognition_msgs-msg:min_y instead.")
  (min_y m))

(cl:ensure-generic-function 'max_y-val :lambda-list '(m))
(cl:defmethod max_y-val ((m <PlotDataArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:max_y-val is deprecated.  Use jsk_recognition_msgs-msg:max_y instead.")
  (max_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlotDataArray>) ostream)
  "Serializes a message object of type '<PlotDataArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'no_legend) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'legend_font_size))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlotDataArray>) istream)
  "Deserializes a message object of type '<PlotDataArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'jsk_recognition_msgs-msg:PlotData))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:slot-value msg 'no_legend) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'legend_font_size) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlotDataArray>)))
  "Returns string type for a message object of type '<PlotDataArray>"
  "jsk_recognition_msgs/PlotDataArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlotDataArray)))
  "Returns string type for a message object of type 'PlotDataArray"
  "jsk_recognition_msgs/PlotDataArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlotDataArray>)))
  "Returns md5sum for a message object of type '<PlotDataArray>"
  "e83e9378b374b8436f955a6cd212770a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlotDataArray)))
  "Returns md5sum for a message object of type 'PlotDataArray"
  "e83e9378b374b8436f955a6cd212770a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlotDataArray>)))
  "Returns full string definition for message of type '<PlotDataArray>"
  (cl:format cl:nil "Header header~%jsk_recognition_msgs/PlotData[] data~%bool no_legend~%float32 legend_font_size~%float32 max_x~%float32 min_x~%float32 min_y~%float32 max_y~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/PlotData~%uint32 SCATTER=1~%uint32 LINE=2~%~%Header header~%float32[] xs~%float32[] ys~%uint32 type                     #SCATTER or LINE~%string label~%bool fit_line~%bool fit_line_ransac~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlotDataArray)))
  "Returns full string definition for message of type 'PlotDataArray"
  (cl:format cl:nil "Header header~%jsk_recognition_msgs/PlotData[] data~%bool no_legend~%float32 legend_font_size~%float32 max_x~%float32 min_x~%float32 min_y~%float32 max_y~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/PlotData~%uint32 SCATTER=1~%uint32 LINE=2~%~%Header header~%float32[] xs~%float32[] ys~%uint32 type                     #SCATTER or LINE~%string label~%bool fit_line~%bool fit_line_ransac~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlotDataArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlotDataArray>))
  "Converts a ROS message object to a list"
  (cl:list 'PlotDataArray
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
    (cl:cons ':no_legend (no_legend msg))
    (cl:cons ':legend_font_size (legend_font_size msg))
    (cl:cons ':max_x (max_x msg))
    (cl:cons ':min_x (min_x msg))
    (cl:cons ':min_y (min_y msg))
    (cl:cons ':max_y (max_y msg))
))
