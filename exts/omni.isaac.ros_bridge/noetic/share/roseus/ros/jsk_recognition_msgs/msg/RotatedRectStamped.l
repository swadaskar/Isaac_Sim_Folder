;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::RotatedRectStamped)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'RotatedRectStamped (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::ROTATEDRECTSTAMPED")
  (make-package "JSK_RECOGNITION_MSGS::ROTATEDRECTSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude RotatedRectStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::RotatedRectStamped
  :super ros::object
  :slots (_header _rect ))

(defmethod jsk_recognition_msgs::RotatedRectStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:rect __rect) (instance jsk_recognition_msgs::RotatedRect :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _rect __rect)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:rect
   (&rest __rect)
   (if (keywordp (car __rect))
       (send* _rect __rect)
     (progn
       (if __rect (setq _rect (car __rect)))
       _rect)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; jsk_recognition_msgs/RotatedRect _rect
    (send _rect :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jsk_recognition_msgs/RotatedRect _rect
       (send _rect :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; jsk_recognition_msgs/RotatedRect _rect
     (send _rect :deserialize buf ptr-) (incf ptr- (send _rect :serialization-length))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::RotatedRectStamped :md5sum-) "0260299b5425567e14c7b295b58829e9")
(setf (get jsk_recognition_msgs::RotatedRectStamped :datatype-) "jsk_recognition_msgs/RotatedRectStamped")
(setf (get jsk_recognition_msgs::RotatedRectStamped :definition-)
      "Header header
RotatedRect rect

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
MSG: jsk_recognition_msgs/RotatedRect
float64 x
float64 y
float64 width
float64 height
float64 angle # degree

")



(provide :jsk_recognition_msgs/RotatedRectStamped "0260299b5425567e14c7b295b58829e9")


