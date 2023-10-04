;; Auto-generated. Do not edit!


(when (boundp 'jsk_footstep_msgs::FootstepArray)
  (if (not (find-package "JSK_FOOTSTEP_MSGS"))
    (make-package "JSK_FOOTSTEP_MSGS"))
  (shadow 'FootstepArray (find-package "JSK_FOOTSTEP_MSGS")))
(unless (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEPARRAY")
  (make-package "JSK_FOOTSTEP_MSGS::FOOTSTEPARRAY"))

(in-package "ROS")
;;//! \htmlinclude FootstepArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_footstep_msgs::FootstepArray
  :super ros::object
  :slots (_header _footsteps ))

(defmethod jsk_footstep_msgs::FootstepArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:footsteps __footsteps) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _footsteps __footsteps)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:footsteps
   (&rest __footsteps)
   (if (keywordp (car __footsteps))
       (send* _footsteps __footsteps)
     (progn
       (if __footsteps (setq _footsteps (car __footsteps)))
       _footsteps)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; jsk_footstep_msgs/Footstep[] _footsteps
    (apply #'+ (send-all _footsteps :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jsk_footstep_msgs/Footstep[] _footsteps
     (write-long (length _footsteps) s)
     (dolist (elem _footsteps)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; jsk_footstep_msgs/Footstep[] _footsteps
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _footsteps (let (r) (dotimes (i n) (push (instance jsk_footstep_msgs::Footstep :init) r)) r))
     (dolist (elem- _footsteps)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_footstep_msgs::FootstepArray :md5sum-) "385bc396845a4680214262a4679d83b3")
(setf (get jsk_footstep_msgs::FootstepArray :datatype-) "jsk_footstep_msgs/FootstepArray")
(setf (get jsk_footstep_msgs::FootstepArray :definition-)
      "Header header
Footstep[] footsteps
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
MSG: jsk_footstep_msgs/Footstep
uint8 RIGHT=2
uint8 LEFT=1

# Constants to visualize progress
uint8 REJECTED=3
uint8 APPROVED=4

## limb_indicator values
uint8 LLEG=1
uint8 RLEG=2
uint8 LARM=5
uint8 RARM=6

uint8 leg ## value should be one of limb_indicator values.
geometry_msgs/Pose pose ## 'pose' represents nominal pose. It may be an end-effector of limb.
duration duration
# optional parameters
uint32 footstep_group
geometry_msgs/Vector3 dimensions ## cube [length(x), width(y), height(z)]
geometry_msgs/Vector3 offset     ## offset from pose to center of cube
float32 swing_height
float32 cost

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

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :jsk_footstep_msgs/FootstepArray "385bc396845a4680214262a4679d83b3")


