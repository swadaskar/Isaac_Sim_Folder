;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::TrackingStatus)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'TrackingStatus (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::TRACKINGSTATUS")
  (make-package "JSK_RECOGNITION_MSGS::TRACKINGSTATUS"))

(in-package "ROS")
;;//! \htmlinclude TrackingStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::TrackingStatus
  :super ros::object
  :slots (_header _is_lost ))

(defmethod jsk_recognition_msgs::TrackingStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:is_lost __is_lost) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _is_lost __is_lost)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:is_lost
   (&optional (__is_lost :null))
   (if (not (eq __is_lost :null)) (setq _is_lost __is_lost)) _is_lost)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _is_lost
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _is_lost
       (if _is_lost (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _is_lost
     (setq _is_lost (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::TrackingStatus :md5sum-) "2e374cd736cf06cc3e69b0c76b8cb117")
(setf (get jsk_recognition_msgs::TrackingStatus :datatype-) "jsk_recognition_msgs/TrackingStatus")
(setf (get jsk_recognition_msgs::TrackingStatus :definition-)
      "Header header

# if true, it means the tracker abandon to track the object.
bool is_lost

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



(provide :jsk_recognition_msgs/TrackingStatus "2e374cd736cf06cc3e69b0c76b8cb117")


