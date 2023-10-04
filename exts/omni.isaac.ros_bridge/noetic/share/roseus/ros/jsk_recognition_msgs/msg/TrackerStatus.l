;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::TrackerStatus)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'TrackerStatus (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::TRACKERSTATUS")
  (make-package "JSK_RECOGNITION_MSGS::TRACKERSTATUS"))

(in-package "ROS")
;;//! \htmlinclude TrackerStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::TrackerStatus
  :super ros::object
  :slots (_header _is_tracking ))

(defmethod jsk_recognition_msgs::TrackerStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:is_tracking __is_tracking) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _is_tracking __is_tracking)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:is_tracking
   (&optional (__is_tracking :null))
   (if (not (eq __is_tracking :null)) (setq _is_tracking __is_tracking)) _is_tracking)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _is_tracking
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _is_tracking
       (if _is_tracking (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _is_tracking
     (setq _is_tracking (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::TrackerStatus :md5sum-) "78e8f06a6b728df052dd181bfcf6c26e")
(setf (get jsk_recognition_msgs::TrackerStatus :datatype-) "jsk_recognition_msgs/TrackerStatus")
(setf (get jsk_recognition_msgs::TrackerStatus :definition-)
      "Header header
# if true, it means the tracker is trying to track object,
# else, the tracker think scene is stable and no need to track
# to save computational resource.
bool is_tracking

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



(provide :jsk_recognition_msgs/TrackerStatus "78e8f06a6b728df052dd181bfcf6c26e")


