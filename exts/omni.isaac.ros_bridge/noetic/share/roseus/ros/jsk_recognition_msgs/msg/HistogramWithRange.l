;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::HistogramWithRange)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'HistogramWithRange (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::HISTOGRAMWITHRANGE")
  (make-package "JSK_RECOGNITION_MSGS::HISTOGRAMWITHRANGE"))

(in-package "ROS")
;;//! \htmlinclude HistogramWithRange.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::HistogramWithRange
  :super ros::object
  :slots (_header _bins ))

(defmethod jsk_recognition_msgs::HistogramWithRange
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:bins __bins) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _bins __bins)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:bins
   (&rest __bins)
   (if (keywordp (car __bins))
       (send* _bins __bins)
     (progn
       (if __bins (setq _bins (car __bins)))
       _bins)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; jsk_recognition_msgs/HistogramWithRangeBin[] _bins
    (apply #'+ (send-all _bins :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jsk_recognition_msgs/HistogramWithRangeBin[] _bins
     (write-long (length _bins) s)
     (dolist (elem _bins)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; jsk_recognition_msgs/HistogramWithRangeBin[] _bins
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bins (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::HistogramWithRangeBin :init) r)) r))
     (dolist (elem- _bins)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::HistogramWithRange :md5sum-) "aa922a22cbdcbdcc1146f30df5b605b8")
(setf (get jsk_recognition_msgs::HistogramWithRange :datatype-) "jsk_recognition_msgs/HistogramWithRange")
(setf (get jsk_recognition_msgs::HistogramWithRange :definition-)
      "Header header
HistogramWithRangeBin[] bins


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: jsk_recognition_msgs/HistogramWithRangeBin
float64 min_value
float64 max_value
uint32 count

")



(provide :jsk_recognition_msgs/HistogramWithRange "aa922a22cbdcbdcc1146f30df5b605b8")


