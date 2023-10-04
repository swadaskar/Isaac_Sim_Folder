;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::Histogram)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'Histogram (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::HISTOGRAM")
  (make-package "JSK_RECOGNITION_MSGS::HISTOGRAM"))

(in-package "ROS")
;;//! \htmlinclude Histogram.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::Histogram
  :super ros::object
  :slots (_header _histogram ))

(defmethod jsk_recognition_msgs::Histogram
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:histogram __histogram) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _histogram __histogram)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:histogram
   (&optional __histogram)
   (if __histogram (setq _histogram __histogram)) _histogram)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[] _histogram
    (* 8    (length _histogram)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[] _histogram
     (write-long (length _histogram) s)
     (dotimes (i (length _histogram))
       (sys::poke (elt _histogram i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[] _histogram
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _histogram (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _histogram i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::Histogram :md5sum-) "376372f016c22c39ab7ee6b6bdd0f10a")
(setf (get jsk_recognition_msgs::Histogram :datatype-) "jsk_recognition_msgs/Histogram")
(setf (get jsk_recognition_msgs::Histogram :definition-)
      "Header header
float64[] histogram

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

")



(provide :jsk_recognition_msgs/Histogram "376372f016c22c39ab7ee6b6bdd0f10a")


