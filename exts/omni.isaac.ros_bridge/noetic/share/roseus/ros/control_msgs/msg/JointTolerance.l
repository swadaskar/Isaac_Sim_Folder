;; Auto-generated. Do not edit!


(when (boundp 'control_msgs::JointTolerance)
  (if (not (find-package "CONTROL_MSGS"))
    (make-package "CONTROL_MSGS"))
  (shadow 'JointTolerance (find-package "CONTROL_MSGS")))
(unless (find-package "CONTROL_MSGS::JOINTTOLERANCE")
  (make-package "CONTROL_MSGS::JOINTTOLERANCE"))

(in-package "ROS")
;;//! \htmlinclude JointTolerance.msg.html


(defclass control_msgs::JointTolerance
  :super ros::object
  :slots (_name _position _velocity _acceleration ))

(defmethod control_msgs::JointTolerance
  (:init
   (&key
    ((:name __name) "")
    ((:position __position) 0.0)
    ((:velocity __velocity) 0.0)
    ((:acceleration __acceleration) 0.0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _position (float __position))
   (setq _velocity (float __velocity))
   (setq _acceleration (float __acceleration))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:acceleration
   (&optional __acceleration)
   (if __acceleration (setq _acceleration __acceleration)) _acceleration)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float64 _position
    8
    ;; float64 _velocity
    8
    ;; float64 _acceleration
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float64 _position
       (sys::poke _position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _acceleration
       (sys::poke _acceleration (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _position
     (setq _position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _velocity
     (setq _velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _acceleration
     (setq _acceleration (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get control_msgs::JointTolerance :md5sum-) "f544fe9c16cf04547e135dd6063ff5be")
(setf (get control_msgs::JointTolerance :datatype-) "control_msgs/JointTolerance")
(setf (get control_msgs::JointTolerance :definition-)
      "# The tolerances specify the amount the position, velocity, and
# accelerations can vary from the setpoints.  For example, in the case
# of trajectory control, when the actual position varies beyond
# (desired position + position tolerance), the trajectory goal may
# abort.
# 
# There are two special values for tolerances:
#  * 0 - The tolerance is unspecified and will remain at whatever the default is
#  * -1 - The tolerance is \"erased\".  If there was a default, the joint will be
#         allowed to move without restriction.

string name
float64 position  # in radians or meters (for a revolute or prismatic joint, respectively)
float64 velocity  # in rad/sec or m/sec
float64 acceleration  # in rad/sec^2 or m/sec^2

")



(provide :control_msgs/JointTolerance "f544fe9c16cf04547e135dd6063ff5be")


