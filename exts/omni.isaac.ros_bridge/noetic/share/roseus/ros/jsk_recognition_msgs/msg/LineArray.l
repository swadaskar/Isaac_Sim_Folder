;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::LineArray)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'LineArray (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::LINEARRAY")
  (make-package "JSK_RECOGNITION_MSGS::LINEARRAY"))

(in-package "ROS")
;;//! \htmlinclude LineArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::LineArray
  :super ros::object
  :slots (_header _lines ))

(defmethod jsk_recognition_msgs::LineArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lines __lines) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _lines __lines)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lines
   (&rest __lines)
   (if (keywordp (car __lines))
       (send* _lines __lines)
     (progn
       (if __lines (setq _lines (car __lines)))
       _lines)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; jsk_recognition_msgs/Line[] _lines
    (apply #'+ (send-all _lines :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jsk_recognition_msgs/Line[] _lines
     (write-long (length _lines) s)
     (dolist (elem _lines)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; jsk_recognition_msgs/Line[] _lines
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lines (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::Line :init) r)) r))
     (dolist (elem- _lines)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::LineArray :md5sum-) "1571346a320fd11e0b7dc11a90a72ddf")
(setf (get jsk_recognition_msgs::LineArray :datatype-) "jsk_recognition_msgs/LineArray")
(setf (get jsk_recognition_msgs::LineArray :definition-)
      "Header header
Line[] lines


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
MSG: jsk_recognition_msgs/Line
float64 x1
float64 y1
float64 x2
float64 y2



")



(provide :jsk_recognition_msgs/LineArray "1571346a320fd11e0b7dc11a90a72ddf")


