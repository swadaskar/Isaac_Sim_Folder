;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SegmentStamped)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SegmentStamped (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SEGMENTSTAMPED")
  (make-package "JSK_RECOGNITION_MSGS::SEGMENTSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude SegmentStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::SegmentStamped
  :super ros::object
  :slots (_header _segment ))

(defmethod jsk_recognition_msgs::SegmentStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:segment __segment) (instance jsk_recognition_msgs::Segment :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _segment __segment)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:segment
   (&rest __segment)
   (if (keywordp (car __segment))
       (send* _segment __segment)
     (progn
       (if __segment (setq _segment (car __segment)))
       _segment)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; jsk_recognition_msgs/Segment _segment
    (send _segment :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jsk_recognition_msgs/Segment _segment
       (send _segment :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; jsk_recognition_msgs/Segment _segment
     (send _segment :deserialize buf ptr-) (incf ptr- (send _segment :serialization-length))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::SegmentStamped :md5sum-) "1f2fbdf9b9a242110bee5312e7718d1f")
(setf (get jsk_recognition_msgs::SegmentStamped :datatype-) "jsk_recognition_msgs/SegmentStamped")
(setf (get jsk_recognition_msgs::SegmentStamped :definition-)
      "Header header
Segment segment

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
MSG: jsk_recognition_msgs/Segment
geometry_msgs/Point start_point
geometry_msgs/Point end_point

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :jsk_recognition_msgs/SegmentStamped "1f2fbdf9b9a242110bee5312e7718d1f")


