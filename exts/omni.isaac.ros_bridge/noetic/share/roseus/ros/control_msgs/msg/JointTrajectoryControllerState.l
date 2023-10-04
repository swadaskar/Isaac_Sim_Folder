;; Auto-generated. Do not edit!


(when (boundp 'control_msgs::JointTrajectoryControllerState)
  (if (not (find-package "CONTROL_MSGS"))
    (make-package "CONTROL_MSGS"))
  (shadow 'JointTrajectoryControllerState (find-package "CONTROL_MSGS")))
(unless (find-package "CONTROL_MSGS::JOINTTRAJECTORYCONTROLLERSTATE")
  (make-package "CONTROL_MSGS::JOINTTRAJECTORYCONTROLLERSTATE"))

(in-package "ROS")
;;//! \htmlinclude JointTrajectoryControllerState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))
(if (not (find-package "TRAJECTORY_MSGS"))
  (ros::roseus-add-msgs "trajectory_msgs"))


(defclass control_msgs::JointTrajectoryControllerState
  :super ros::object
  :slots (_header _joint_names _desired _actual _error ))

(defmethod control_msgs::JointTrajectoryControllerState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:joint_names __joint_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:desired __desired) (instance trajectory_msgs::JointTrajectoryPoint :init))
    ((:actual __actual) (instance trajectory_msgs::JointTrajectoryPoint :init))
    ((:error __error) (instance trajectory_msgs::JointTrajectoryPoint :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _joint_names __joint_names)
   (setq _desired __desired)
   (setq _actual __actual)
   (setq _error __error)
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
  (:desired
   (&rest __desired)
   (if (keywordp (car __desired))
       (send* _desired __desired)
     (progn
       (if __desired (setq _desired (car __desired)))
       _desired)))
  (:actual
   (&rest __actual)
   (if (keywordp (car __actual))
       (send* _actual __actual)
     (progn
       (if __actual (setq _actual (car __actual)))
       _actual)))
  (:error
   (&rest __error)
   (if (keywordp (car __error))
       (send* _error __error)
     (progn
       (if __error (setq _error (car __error)))
       _error)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _joint_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _joint_names)) 4
    ;; trajectory_msgs/JointTrajectoryPoint _desired
    (send _desired :serialization-length)
    ;; trajectory_msgs/JointTrajectoryPoint _actual
    (send _actual :serialization-length)
    ;; trajectory_msgs/JointTrajectoryPoint _error
    (send _error :serialization-length)
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
     ;; trajectory_msgs/JointTrajectoryPoint _desired
       (send _desired :serialize s)
     ;; trajectory_msgs/JointTrajectoryPoint _actual
       (send _actual :serialize s)
     ;; trajectory_msgs/JointTrajectoryPoint _error
       (send _error :serialize s)
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
   ;; trajectory_msgs/JointTrajectoryPoint _desired
     (send _desired :deserialize buf ptr-) (incf ptr- (send _desired :serialization-length))
   ;; trajectory_msgs/JointTrajectoryPoint _actual
     (send _actual :deserialize buf ptr-) (incf ptr- (send _actual :serialization-length))
   ;; trajectory_msgs/JointTrajectoryPoint _error
     (send _error :deserialize buf ptr-) (incf ptr- (send _error :serialization-length))
   ;;
   self)
  )

(setf (get control_msgs::JointTrajectoryControllerState :md5sum-) "10817c60c2486ef6b33e97dcd87f4474")
(setf (get control_msgs::JointTrajectoryControllerState :datatype-) "control_msgs/JointTrajectoryControllerState")
(setf (get control_msgs::JointTrajectoryControllerState :definition-)
      "Header header
string[] joint_names
trajectory_msgs/JointTrajectoryPoint desired
trajectory_msgs/JointTrajectoryPoint actual
trajectory_msgs/JointTrajectoryPoint error  # Redundant, but useful

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
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start

")



(provide :control_msgs/JointTrajectoryControllerState "10817c60c2486ef6b33e97dcd87f4474")


