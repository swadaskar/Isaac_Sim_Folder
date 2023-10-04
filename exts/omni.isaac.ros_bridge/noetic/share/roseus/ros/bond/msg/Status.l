;; Auto-generated. Do not edit!


(when (boundp 'bond::Status)
  (if (not (find-package "BOND"))
    (make-package "BOND"))
  (shadow 'Status (find-package "BOND")))
(unless (find-package "BOND::STATUS")
  (make-package "BOND::STATUS"))

(in-package "ROS")
;;//! \htmlinclude Status.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass bond::Status
  :super ros::object
  :slots (_header _id _instance_id _active _heartbeat_timeout _heartbeat_period ))

(defmethod bond::Status
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) "")
    ((:instance_id __instance_id) "")
    ((:active __active) nil)
    ((:heartbeat_timeout __heartbeat_timeout) 0.0)
    ((:heartbeat_period __heartbeat_period) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (string __id))
   (setq _instance_id (string __instance_id))
   (setq _active __active)
   (setq _heartbeat_timeout (float __heartbeat_timeout))
   (setq _heartbeat_period (float __heartbeat_period))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:instance_id
   (&optional __instance_id)
   (if __instance_id (setq _instance_id __instance_id)) _instance_id)
  (:active
   (&optional (__active :null))
   (if (not (eq __active :null)) (setq _active __active)) _active)
  (:heartbeat_timeout
   (&optional __heartbeat_timeout)
   (if __heartbeat_timeout (setq _heartbeat_timeout __heartbeat_timeout)) _heartbeat_timeout)
  (:heartbeat_period
   (&optional __heartbeat_period)
   (if __heartbeat_period (setq _heartbeat_period __heartbeat_period)) _heartbeat_period)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _id
    4 (length _id)
    ;; string _instance_id
    4 (length _instance_id)
    ;; bool _active
    1
    ;; float32 _heartbeat_timeout
    4
    ;; float32 _heartbeat_period
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;; string _instance_id
       (write-long (length _instance_id) s) (princ _instance_id s)
     ;; bool _active
       (if _active (write-byte -1 s) (write-byte 0 s))
     ;; float32 _heartbeat_timeout
       (sys::poke _heartbeat_timeout (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heartbeat_period
       (sys::poke _heartbeat_period (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _instance_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _instance_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _active
     (setq _active (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _heartbeat_timeout
     (setq _heartbeat_timeout (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heartbeat_period
     (setq _heartbeat_period (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get bond::Status :md5sum-) "eacc84bf5d65b6777d4c50f463dfb9c8")
(setf (get bond::Status :datatype-) "bond/Status")
(setf (get bond::Status :definition-)
      "Header header
string id  # ID of the bond
string instance_id  # Unique ID for an individual in a bond
bool active

# Including the timeouts for the bond makes it easier to debug mis-matches
# between the two sides.
float32 heartbeat_timeout
float32 heartbeat_period
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



(provide :bond/Status "eacc84bf5d65b6777d4c50f463dfb9c8")


