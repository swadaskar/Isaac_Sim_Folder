; Auto-generated. Do not edit!


(cl:in-package jsk_recognition_msgs-msg)


;//! \htmlinclude HistogramWithRangeArray.msg.html

(cl:defclass <HistogramWithRangeArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (histograms
    :reader histograms
    :initarg :histograms
    :type (cl:vector jsk_recognition_msgs-msg:HistogramWithRange)
   :initform (cl:make-array 0 :element-type 'jsk_recognition_msgs-msg:HistogramWithRange :initial-element (cl:make-instance 'jsk_recognition_msgs-msg:HistogramWithRange))))
)

(cl:defclass HistogramWithRangeArray (<HistogramWithRangeArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HistogramWithRangeArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HistogramWithRangeArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jsk_recognition_msgs-msg:<HistogramWithRangeArray> is deprecated: use jsk_recognition_msgs-msg:HistogramWithRangeArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HistogramWithRangeArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:header-val is deprecated.  Use jsk_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'histograms-val :lambda-list '(m))
(cl:defmethod histograms-val ((m <HistogramWithRangeArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jsk_recognition_msgs-msg:histograms-val is deprecated.  Use jsk_recognition_msgs-msg:histograms instead.")
  (histograms m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HistogramWithRangeArray>) ostream)
  "Serializes a message object of type '<HistogramWithRangeArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'histograms))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'histograms))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HistogramWithRangeArray>) istream)
  "Deserializes a message object of type '<HistogramWithRangeArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'histograms) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'histograms)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'jsk_recognition_msgs-msg:HistogramWithRange))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HistogramWithRangeArray>)))
  "Returns string type for a message object of type '<HistogramWithRangeArray>"
  "jsk_recognition_msgs/HistogramWithRangeArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HistogramWithRangeArray)))
  "Returns string type for a message object of type 'HistogramWithRangeArray"
  "jsk_recognition_msgs/HistogramWithRangeArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HistogramWithRangeArray>)))
  "Returns md5sum for a message object of type '<HistogramWithRangeArray>"
  "2622b1b7ae84f154677cf4c0d3a398c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HistogramWithRangeArray)))
  "Returns md5sum for a message object of type 'HistogramWithRangeArray"
  "2622b1b7ae84f154677cf4c0d3a398c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HistogramWithRangeArray>)))
  "Returns full string definition for message of type '<HistogramWithRangeArray>"
  (cl:format cl:nil "Header header~%HistogramWithRange[] histograms~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/HistogramWithRange~%Header header~%HistogramWithRangeBin[] bins~%~%~%================================================================================~%MSG: jsk_recognition_msgs/HistogramWithRangeBin~%float64 min_value~%float64 max_value~%uint32 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HistogramWithRangeArray)))
  "Returns full string definition for message of type 'HistogramWithRangeArray"
  (cl:format cl:nil "Header header~%HistogramWithRange[] histograms~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: jsk_recognition_msgs/HistogramWithRange~%Header header~%HistogramWithRangeBin[] bins~%~%~%================================================================================~%MSG: jsk_recognition_msgs/HistogramWithRangeBin~%float64 min_value~%float64 max_value~%uint32 count~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HistogramWithRangeArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'histograms) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HistogramWithRangeArray>))
  "Converts a ROS message object to a list"
  (cl:list 'HistogramWithRangeArray
    (cl:cons ':header (header msg))
    (cl:cons ':histograms (histograms msg))
))
