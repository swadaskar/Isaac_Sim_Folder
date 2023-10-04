;; Auto-generated. Do not edit!


(when (boundp 'control_msgs::PidState)
  (if (not (find-package "CONTROL_MSGS"))
    (make-package "CONTROL_MSGS"))
  (shadow 'PidState (find-package "CONTROL_MSGS")))
(unless (find-package "CONTROL_MSGS::PIDSTATE")
  (make-package "CONTROL_MSGS::PIDSTATE"))

(in-package "ROS")
;;//! \htmlinclude PidState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_msgs::PidState
  :super ros::object
  :slots (_header _timestep _error _error_dot _p_error _i_error _d_error _p_term _i_term _d_term _i_max _i_min _output ))

(defmethod control_msgs::PidState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:timestep __timestep) (instance ros::time :init))
    ((:error __error) 0.0)
    ((:error_dot __error_dot) 0.0)
    ((:p_error __p_error) 0.0)
    ((:i_error __i_error) 0.0)
    ((:d_error __d_error) 0.0)
    ((:p_term __p_term) 0.0)
    ((:i_term __i_term) 0.0)
    ((:d_term __d_term) 0.0)
    ((:i_max __i_max) 0.0)
    ((:i_min __i_min) 0.0)
    ((:output __output) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _timestep __timestep)
   (setq _error (float __error))
   (setq _error_dot (float __error_dot))
   (setq _p_error (float __p_error))
   (setq _i_error (float __i_error))
   (setq _d_error (float __d_error))
   (setq _p_term (float __p_term))
   (setq _i_term (float __i_term))
   (setq _d_term (float __d_term))
   (setq _i_max (float __i_max))
   (setq _i_min (float __i_min))
   (setq _output (float __output))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:timestep
   (&optional __timestep)
   (if __timestep (setq _timestep __timestep)) _timestep)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:error_dot
   (&optional __error_dot)
   (if __error_dot (setq _error_dot __error_dot)) _error_dot)
  (:p_error
   (&optional __p_error)
   (if __p_error (setq _p_error __p_error)) _p_error)
  (:i_error
   (&optional __i_error)
   (if __i_error (setq _i_error __i_error)) _i_error)
  (:d_error
   (&optional __d_error)
   (if __d_error (setq _d_error __d_error)) _d_error)
  (:p_term
   (&optional __p_term)
   (if __p_term (setq _p_term __p_term)) _p_term)
  (:i_term
   (&optional __i_term)
   (if __i_term (setq _i_term __i_term)) _i_term)
  (:d_term
   (&optional __d_term)
   (if __d_term (setq _d_term __d_term)) _d_term)
  (:i_max
   (&optional __i_max)
   (if __i_max (setq _i_max __i_max)) _i_max)
  (:i_min
   (&optional __i_min)
   (if __i_min (setq _i_min __i_min)) _i_min)
  (:output
   (&optional __output)
   (if __output (setq _output __output)) _output)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; duration _timestep
    8
    ;; float64 _error
    8
    ;; float64 _error_dot
    8
    ;; float64 _p_error
    8
    ;; float64 _i_error
    8
    ;; float64 _d_error
    8
    ;; float64 _p_term
    8
    ;; float64 _i_term
    8
    ;; float64 _d_term
    8
    ;; float64 _i_max
    8
    ;; float64 _i_min
    8
    ;; float64 _output
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; duration _timestep
       (write-long (send _timestep :sec) s) (write-long (send _timestep :nsec) s)
     ;; float64 _error
       (sys::poke _error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _error_dot
       (sys::poke _error_dot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _p_error
       (sys::poke _p_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _i_error
       (sys::poke _i_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d_error
       (sys::poke _d_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _p_term
       (sys::poke _p_term (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _i_term
       (sys::poke _i_term (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d_term
       (sys::poke _d_term (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _i_max
       (sys::poke _i_max (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _i_min
       (sys::poke _i_min (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _output
       (sys::poke _output (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; duration _timestep
     (send _timestep :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _timestep :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _error
     (setq _error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _error_dot
     (setq _error_dot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _p_error
     (setq _p_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _i_error
     (setq _i_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d_error
     (setq _d_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _p_term
     (setq _p_term (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _i_term
     (setq _i_term (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d_term
     (setq _d_term (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _i_max
     (setq _i_max (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _i_min
     (setq _i_min (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _output
     (setq _output (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get control_msgs::PidState :md5sum-) "b138ec00e886c10e73f27e8712252ea6")
(setf (get control_msgs::PidState :datatype-) "control_msgs/PidState")
(setf (get control_msgs::PidState :definition-)
      "Header header
duration timestep
float64 error
float64 error_dot
float64 p_error
float64 i_error
float64 d_error
float64 p_term
float64 i_term
float64 d_term
float64 i_max
float64 i_min
float64 output

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



(provide :control_msgs/PidState "b138ec00e886c10e73f27e8712252ea6")


