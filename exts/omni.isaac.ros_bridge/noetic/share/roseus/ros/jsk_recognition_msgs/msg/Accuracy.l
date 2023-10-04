;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::Accuracy)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'Accuracy (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::ACCURACY")
  (make-package "JSK_RECOGNITION_MSGS::ACCURACY"))

(in-package "ROS")
;;//! \htmlinclude Accuracy.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::Accuracy
  :super ros::object
  :slots (_header _accuracy ))

(defmethod jsk_recognition_msgs::Accuracy
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:accuracy __accuracy) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _accuracy (float __accuracy))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:accuracy
   (&optional __accuracy)
   (if __accuracy (setq _accuracy __accuracy)) _accuracy)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _accuracy
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _accuracy
       (sys::poke _accuracy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _accuracy
     (setq _accuracy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::Accuracy :md5sum-) "198da9dc2684aac2aa03a85a3344f5ef")
(setf (get jsk_recognition_msgs::Accuracy :datatype-) "jsk_recognition_msgs/Accuracy")
(setf (get jsk_recognition_msgs::Accuracy :definition-)
      "Header header
float32 accuracy

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



(provide :jsk_recognition_msgs/Accuracy "198da9dc2684aac2aa03a85a3344f5ef")


