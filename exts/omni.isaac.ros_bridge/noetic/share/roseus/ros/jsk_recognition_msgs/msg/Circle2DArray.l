;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::Circle2DArray)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'Circle2DArray (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::CIRCLE2DARRAY")
  (make-package "JSK_RECOGNITION_MSGS::CIRCLE2DARRAY"))

(in-package "ROS")
;;//! \htmlinclude Circle2DArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::Circle2DArray
  :super ros::object
  :slots (_header _circles ))

(defmethod jsk_recognition_msgs::Circle2DArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:circles __circles) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _circles __circles)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:circles
   (&rest __circles)
   (if (keywordp (car __circles))
       (send* _circles __circles)
     (progn
       (if __circles (setq _circles (car __circles)))
       _circles)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; jsk_recognition_msgs/Circle2D[] _circles
    (apply #'+ (send-all _circles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jsk_recognition_msgs/Circle2D[] _circles
     (write-long (length _circles) s)
     (dolist (elem _circles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; jsk_recognition_msgs/Circle2D[] _circles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _circles (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::Circle2D :init) r)) r))
     (dolist (elem- _circles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::Circle2DArray :md5sum-) "7c3f0f08758538a1e02ce600cf99132f")
(setf (get jsk_recognition_msgs::Circle2DArray :datatype-) "jsk_recognition_msgs/Circle2DArray")
(setf (get jsk_recognition_msgs::Circle2DArray :definition-)
      "Header header
Circle2D[] circles

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
MSG: jsk_recognition_msgs/Circle2D
Header header
float64 radius
float64 x
float64 y

")



(provide :jsk_recognition_msgs/Circle2DArray "7c3f0f08758538a1e02ce600cf99132f")


