;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::TorusArray)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'TorusArray (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::TORUSARRAY")
  (make-package "JSK_RECOGNITION_MSGS::TORUSARRAY"))

(in-package "ROS")
;;//! \htmlinclude TorusArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::TorusArray
  :super ros::object
  :slots (_header _toruses ))

(defmethod jsk_recognition_msgs::TorusArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:toruses __toruses) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _toruses __toruses)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:toruses
   (&rest __toruses)
   (if (keywordp (car __toruses))
       (send* _toruses __toruses)
     (progn
       (if __toruses (setq _toruses (car __toruses)))
       _toruses)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; jsk_recognition_msgs/Torus[] _toruses
    (apply #'+ (send-all _toruses :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jsk_recognition_msgs/Torus[] _toruses
     (write-long (length _toruses) s)
     (dolist (elem _toruses)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; jsk_recognition_msgs/Torus[] _toruses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _toruses (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::Torus :init) r)) r))
     (dolist (elem- _toruses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::TorusArray :md5sum-) "81d0dbf46016b5714fa4ea9eca5485e0")
(setf (get jsk_recognition_msgs::TorusArray :datatype-) "jsk_recognition_msgs/TorusArray")
(setf (get jsk_recognition_msgs::TorusArray :definition-)
      "Header header
Torus[] toruses

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
MSG: jsk_recognition_msgs/Torus
Header header
bool failure
geometry_msgs/Pose pose
float64 large_radius
float64 small_radius

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :jsk_recognition_msgs/TorusArray "81d0dbf46016b5714fa4ea9eca5485e0")


