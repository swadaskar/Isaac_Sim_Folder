;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::ImageDifferenceValue)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'ImageDifferenceValue (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::IMAGEDIFFERENCEVALUE")
  (make-package "JSK_RECOGNITION_MSGS::IMAGEDIFFERENCEVALUE"))

(in-package "ROS")
;;//! \htmlinclude ImageDifferenceValue.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::ImageDifferenceValue
  :super ros::object
  :slots (_header _difference ))

(defmethod jsk_recognition_msgs::ImageDifferenceValue
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:difference __difference) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _difference (float __difference))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:difference
   (&optional __difference)
   (if __difference (setq _difference __difference)) _difference)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _difference
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _difference
       (sys::poke _difference (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _difference
     (setq _difference (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::ImageDifferenceValue :md5sum-) "80b6f355db87c1b2d8b04d2f620fe0a7")
(setf (get jsk_recognition_msgs::ImageDifferenceValue :datatype-) "jsk_recognition_msgs/ImageDifferenceValue")
(setf (get jsk_recognition_msgs::ImageDifferenceValue :definition-)
      "Header header

float32 difference

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



(provide :jsk_recognition_msgs/ImageDifferenceValue "80b6f355db87c1b2d8b04d2f620fe0a7")


