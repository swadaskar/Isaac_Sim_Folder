;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::ColorHistogramArray)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'ColorHistogramArray (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::COLORHISTOGRAMARRAY")
  (make-package "JSK_RECOGNITION_MSGS::COLORHISTOGRAMARRAY"))

(in-package "ROS")
;;//! \htmlinclude ColorHistogramArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::ColorHistogramArray
  :super ros::object
  :slots (_header _histograms ))

(defmethod jsk_recognition_msgs::ColorHistogramArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:histograms __histograms) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _histograms __histograms)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:histograms
   (&rest __histograms)
   (if (keywordp (car __histograms))
       (send* _histograms __histograms)
     (progn
       (if __histograms (setq _histograms (car __histograms)))
       _histograms)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; jsk_recognition_msgs/ColorHistogram[] _histograms
    (apply #'+ (send-all _histograms :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jsk_recognition_msgs/ColorHistogram[] _histograms
     (write-long (length _histograms) s)
     (dolist (elem _histograms)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; jsk_recognition_msgs/ColorHistogram[] _histograms
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _histograms (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::ColorHistogram :init) r)) r))
     (dolist (elem- _histograms)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::ColorHistogramArray :md5sum-) "3bcc7f05c5520f311047096d5530e715")
(setf (get jsk_recognition_msgs::ColorHistogramArray :datatype-) "jsk_recognition_msgs/ColorHistogramArray")
(setf (get jsk_recognition_msgs::ColorHistogramArray :definition-)
      "Header header
ColorHistogram[] histograms

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
MSG: jsk_recognition_msgs/ColorHistogram
Header header
float32[] histogram

")



(provide :jsk_recognition_msgs/ColorHistogramArray "3bcc7f05c5520f311047096d5530e715")


