;; Auto-generated. Do not edit!


(when (boundp 'control_msgs::JointControllerState)
  (if (not (find-package "CONTROL_MSGS"))
    (make-package "CONTROL_MSGS"))
  (shadow 'JointControllerState (find-package "CONTROL_MSGS")))
(unless (find-package "CONTROL_MSGS::JOINTCONTROLLERSTATE")
  (make-package "CONTROL_MSGS::JOINTCONTROLLERSTATE"))

(in-package "ROS")
;;//! \htmlinclude JointControllerState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_msgs::JointControllerState
  :super ros::object
  :slots (_header _set_point _process_value _process_value_dot _error _time_step _command _p _i _d _i_clamp _antiwindup ))

(defmethod control_msgs::JointControllerState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:set_point __set_point) 0.0)
    ((:process_value __process_value) 0.0)
    ((:process_value_dot __process_value_dot) 0.0)
    ((:error __error) 0.0)
    ((:time_step __time_step) 0.0)
    ((:command __command) 0.0)
    ((:p __p) 0.0)
    ((:i __i) 0.0)
    ((:d __d) 0.0)
    ((:i_clamp __i_clamp) 0.0)
    ((:antiwindup __antiwindup) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _set_point (float __set_point))
   (setq _process_value (float __process_value))
   (setq _process_value_dot (float __process_value_dot))
   (setq _error (float __error))
   (setq _time_step (float __time_step))
   (setq _command (float __command))
   (setq _p (float __p))
   (setq _i (float __i))
   (setq _d (float __d))
   (setq _i_clamp (float __i_clamp))
   (setq _antiwindup __antiwindup)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:set_point
   (&optional __set_point)
   (if __set_point (setq _set_point __set_point)) _set_point)
  (:process_value
   (&optional __process_value)
   (if __process_value (setq _process_value __process_value)) _process_value)
  (:process_value_dot
   (&optional __process_value_dot)
   (if __process_value_dot (setq _process_value_dot __process_value_dot)) _process_value_dot)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:time_step
   (&optional __time_step)
   (if __time_step (setq _time_step __time_step)) _time_step)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:p
   (&optional __p)
   (if __p (setq _p __p)) _p)
  (:i
   (&optional __i)
   (if __i (setq _i __i)) _i)
  (:d
   (&optional __d)
   (if __d (setq _d __d)) _d)
  (:i_clamp
   (&optional __i_clamp)
   (if __i_clamp (setq _i_clamp __i_clamp)) _i_clamp)
  (:antiwindup
   (&optional (__antiwindup :null))
   (if (not (eq __antiwindup :null)) (setq _antiwindup __antiwindup)) _antiwindup)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _set_point
    8
    ;; float64 _process_value
    8
    ;; float64 _process_value_dot
    8
    ;; float64 _error
    8
    ;; float64 _time_step
    8
    ;; float64 _command
    8
    ;; float64 _p
    8
    ;; float64 _i
    8
    ;; float64 _d
    8
    ;; float64 _i_clamp
    8
    ;; bool _antiwindup
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _set_point
       (sys::poke _set_point (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _process_value
       (sys::poke _process_value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _process_value_dot
       (sys::poke _process_value_dot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _error
       (sys::poke _error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _time_step
       (sys::poke _time_step (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _command
       (sys::poke _command (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _p
       (sys::poke _p (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _i
       (sys::poke _i (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d
       (sys::poke _d (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _i_clamp
       (sys::poke _i_clamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _antiwindup
       (if _antiwindup (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _set_point
     (setq _set_point (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _process_value
     (setq _process_value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _process_value_dot
     (setq _process_value_dot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _error
     (setq _error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _time_step
     (setq _time_step (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _command
     (setq _command (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _p
     (setq _p (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _i
     (setq _i (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d
     (setq _d (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _i_clamp
     (setq _i_clamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _antiwindup
     (setq _antiwindup (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get control_msgs::JointControllerState :md5sum-) "987ad85e4756f3aef7f1e5e7fe0595d1")
(setf (get control_msgs::JointControllerState :datatype-) "control_msgs/JointControllerState")
(setf (get control_msgs::JointControllerState :definition-)
      "# This message presents current controller state of one joint.

# Header timestamp should be update time of controller state
Header header

# The set point, that is, desired state.
float64 set_point

# Current value of the process (ie: latest sensor measurement on the controlled value).
float64 process_value

# First time-derivative of the process value.
float64 process_value_dot

# The error of the controlled value, essentially process_value - set_point (for a regular PID implementation).
float64 error

# Time between two consecutive updates/execution of the control law.
float64 time_step

# Current output of the controller.
float64 command

# Current PID parameters of the controller.
float64 p
float64 i
float64 d
float64 i_clamp
bool antiwindup

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



(provide :control_msgs/JointControllerState "987ad85e4756f3aef7f1e5e7fe0595d1")


