;; Auto-generated. Do not edit!


(when (boundp 'control_msgs::JointJog)
  (if (not (find-package "CONTROL_MSGS"))
    (make-package "CONTROL_MSGS"))
  (shadow 'JointJog (find-package "CONTROL_MSGS")))
(unless (find-package "CONTROL_MSGS::JOINTJOG")
  (make-package "CONTROL_MSGS::JOINTJOG"))

(in-package "ROS")
;;//! \htmlinclude JointJog.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_msgs::JointJog
  :super ros::object
  :slots (_header _joint_names _displacements _velocities _duration ))

(defmethod control_msgs::JointJog
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:joint_names __joint_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:displacements __displacements) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocities __velocities) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _joint_names __joint_names)
   (setq _displacements __displacements)
   (setq _velocities __velocities)
   (setq _duration (float __duration))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:joint_names
   (&optional __joint_names)
   (if __joint_names (setq _joint_names __joint_names)) _joint_names)
  (:displacements
   (&optional __displacements)
   (if __displacements (setq _displacements __displacements)) _displacements)
  (:velocities
   (&optional __velocities)
   (if __velocities (setq _velocities __velocities)) _velocities)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _joint_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _joint_names)) 4
    ;; float64[] _displacements
    (* 8    (length _displacements)) 4
    ;; float64[] _velocities
    (* 8    (length _velocities)) 4
    ;; float64 _duration
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _joint_names
     (write-long (length _joint_names) s)
     (dolist (elem _joint_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _displacements
     (write-long (length _displacements) s)
     (dotimes (i (length _displacements))
       (sys::poke (elt _displacements i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _velocities
     (write-long (length _velocities) s)
     (dotimes (i (length _velocities))
       (sys::poke (elt _velocities i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string[] _joint_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _joint_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _displacements
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _displacements (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _displacements i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _velocities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocities (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocities i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _duration
     (setq _duration (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get control_msgs::JointJog :md5sum-) "1685da700c8c2e1254afc92a5fb89c96")
(setf (get control_msgs::JointJog :datatype-) "control_msgs/JointJog")
(setf (get control_msgs::JointJog :definition-)
      "# Used in time-stamping the message.
Header header

# Name list of the joints. You don't need to specify all joints of the
# robot. Joint names are case-sensitive.
string[] joint_names

# A position command to the joints listed in joint_names.
# The order must be identical.
# Units are meters or radians.
# If displacements and velocities are filled, a profiled motion is requested.
float64[] displacements # or position_deltas

# A velocity command to the joints listed in joint_names.
# The order must be identical.
# Units are m/s or rad/s.
# If displacements and velocities are filled, a profiled motion is requested.
float64[] velocities

float64 duration

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



(provide :control_msgs/JointJog "1685da700c8c2e1254afc92a5fb89c96")


