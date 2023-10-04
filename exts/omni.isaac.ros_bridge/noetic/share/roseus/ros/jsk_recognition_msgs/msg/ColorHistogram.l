;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::ColorHistogram)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'ColorHistogram (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::COLORHISTOGRAM")
  (make-package "JSK_RECOGNITION_MSGS::COLORHISTOGRAM"))

(in-package "ROS")
;;//! \htmlinclude ColorHistogram.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::ColorHistogram
  :super ros::object
  :slots (_header _histogram ))

(defmethod jsk_recognition_msgs::ColorHistogram
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
    ;; float32[] _histogram
    (* 4    (length _histogram)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[] _histogram
     (write-long (length _histogram) s)
     (dotimes (i (length _histogram))
       (sys::poke (elt _histogram i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[] _histogram
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _histogram (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _histogram i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::ColorHistogram :md5sum-) "5b08641478fdecd8720ba08b36fce2aa")
(setf (get jsk_recognition_msgs::ColorHistogram :datatype-) "jsk_recognition_msgs/ColorHistogram")
(setf (get jsk_recognition_msgs::ColorHistogram :definition-)
      "Header header
float32[] histogram

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



(provide :jsk_recognition_msgs/ColorHistogram "5b08641478fdecd8720ba08b36fce2aa")


