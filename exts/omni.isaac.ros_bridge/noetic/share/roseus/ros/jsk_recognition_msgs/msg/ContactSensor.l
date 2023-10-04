;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::ContactSensor)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'ContactSensor (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::CONTACTSENSOR")
  (make-package "JSK_RECOGNITION_MSGS::CONTACTSENSOR"))

(in-package "ROS")
;;//! \htmlinclude ContactSensor.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::ContactSensor
  :super ros::object
  :slots (_header _contact _link_name ))

(defmethod jsk_recognition_msgs::ContactSensor
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:contact __contact) nil)
    ((:link_name __link_name) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _contact __contact)
   (setq _link_name (string __link_name))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:contact
   (&optional (__contact :null))
   (if (not (eq __contact :null)) (setq _contact __contact)) _contact)
  (:link_name
   (&optional __link_name)
   (if __link_name (setq _link_name __link_name)) _link_name)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _contact
    1
    ;; string _link_name
    4 (length _link_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _contact
       (if _contact (write-byte -1 s) (write-byte 0 s))
     ;; string _link_name
       (write-long (length _link_name) s) (princ _link_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _contact
     (setq _contact (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _link_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _link_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::ContactSensor :md5sum-) "364b2b952a51d85dfa877e334264e361")
(setf (get jsk_recognition_msgs::ContactSensor :datatype-) "jsk_recognition_msgs/ContactSensor")
(setf (get jsk_recognition_msgs::ContactSensor :definition-)
      "# Header
Header header

# Whether sensor detects contact or not
bool contact

string link_name

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



(provide :jsk_recognition_msgs/ContactSensor "364b2b952a51d85dfa877e334264e361")


