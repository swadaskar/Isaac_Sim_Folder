;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::ContactSensorArray)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'ContactSensorArray (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::CONTACTSENSORARRAY")
  (make-package "JSK_RECOGNITION_MSGS::CONTACTSENSORARRAY"))

(in-package "ROS")
;;//! \htmlinclude ContactSensorArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::ContactSensorArray
  :super ros::object
  :slots (_header _datas ))

(defmethod jsk_recognition_msgs::ContactSensorArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:datas __datas) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _datas __datas)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:datas
   (&rest __datas)
   (if (keywordp (car __datas))
       (send* _datas __datas)
     (progn
       (if __datas (setq _datas (car __datas)))
       _datas)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; jsk_recognition_msgs/ContactSensor[] _datas
    (apply #'+ (send-all _datas :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jsk_recognition_msgs/ContactSensor[] _datas
     (write-long (length _datas) s)
     (dolist (elem _datas)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; jsk_recognition_msgs/ContactSensor[] _datas
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _datas (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::ContactSensor :init) r)) r))
     (dolist (elem- _datas)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::ContactSensorArray :md5sum-) "c65f16fb3a523c0b77d7e31330b214da")
(setf (get jsk_recognition_msgs::ContactSensorArray :datatype-) "jsk_recognition_msgs/ContactSensorArray")
(setf (get jsk_recognition_msgs::ContactSensorArray :definition-)
      "std_msgs/Header header
jsk_recognition_msgs/ContactSensor[] datas
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
MSG: jsk_recognition_msgs/ContactSensor
# Header
Header header

# Whether sensor detects contact or not
bool contact

string link_name

")



(provide :jsk_recognition_msgs/ContactSensorArray "c65f16fb3a523c0b77d7e31330b214da")


