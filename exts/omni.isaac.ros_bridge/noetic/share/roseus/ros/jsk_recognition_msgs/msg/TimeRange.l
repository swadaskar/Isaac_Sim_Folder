;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::TimeRange)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'TimeRange (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::TIMERANGE")
  (make-package "JSK_RECOGNITION_MSGS::TIMERANGE"))

(in-package "ROS")
;;//! \htmlinclude TimeRange.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::TimeRange
  :super ros::object
  :slots (_header _start _end ))

(defmethod jsk_recognition_msgs::TimeRange
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:start __start) (instance ros::time :init))
    ((:end __end) (instance ros::time :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _start __start)
   (setq _end __end)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:start
   (&optional __start)
   (if __start (setq _start __start)) _start)
  (:end
   (&optional __end)
   (if __end (setq _end __end)) _end)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; time _start
    8
    ;; time _end
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; time _start
       (write-long (send _start :sec) s) (write-long (send _start :nsec) s)
     ;; time _end
       (write-long (send _end :sec) s) (write-long (send _end :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; time _start
     (send _start :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _end
     (send _end :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _end :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::TimeRange :md5sum-) "a8a69e1e51e3731790c8e4120a725398")
(setf (get jsk_recognition_msgs::TimeRange :datatype-) "jsk_recognition_msgs/TimeRange")
(setf (get jsk_recognition_msgs::TimeRange :definition-)
      "# Represents range of time.
std_msgs/Header header
time start
time end

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



(provide :jsk_recognition_msgs/TimeRange "a8a69e1e51e3731790c8e4120a725398")


