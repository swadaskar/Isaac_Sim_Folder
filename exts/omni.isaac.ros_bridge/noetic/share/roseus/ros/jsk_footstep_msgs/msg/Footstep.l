;; Auto-generated. Do not edit!


(when (boundp 'jsk_footstep_msgs::Footstep)
  (if (not (find-package "JSK_FOOTSTEP_MSGS"))
    (make-package "JSK_FOOTSTEP_MSGS"))
  (shadow 'Footstep (find-package "JSK_FOOTSTEP_MSGS")))
(unless (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP")
  (make-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))

(in-package "ROS")
;;//! \htmlinclude Footstep.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*RIGHT*" (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(shadow '*RIGHT* (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(defconstant jsk_footstep_msgs::Footstep::*RIGHT* 2)
(intern "*LEFT*" (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(shadow '*LEFT* (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(defconstant jsk_footstep_msgs::Footstep::*LEFT* 1)
(intern "*REJECTED*" (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(shadow '*REJECTED* (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(defconstant jsk_footstep_msgs::Footstep::*REJECTED* 3)
(intern "*APPROVED*" (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(shadow '*APPROVED* (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(defconstant jsk_footstep_msgs::Footstep::*APPROVED* 4)
(intern "*LLEG*" (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(shadow '*LLEG* (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(defconstant jsk_footstep_msgs::Footstep::*LLEG* 1)
(intern "*RLEG*" (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(shadow '*RLEG* (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(defconstant jsk_footstep_msgs::Footstep::*RLEG* 2)
(intern "*LARM*" (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(shadow '*LARM* (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(defconstant jsk_footstep_msgs::Footstep::*LARM* 5)
(intern "*RARM*" (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(shadow '*RARM* (find-package "JSK_FOOTSTEP_MSGS::FOOTSTEP"))
(defconstant jsk_footstep_msgs::Footstep::*RARM* 6)

(defun jsk_footstep_msgs::Footstep-to-symbol (const)
  (cond
        ((= const 2) 'jsk_footstep_msgs::Footstep::*RIGHT*)
        ((= const 1) 'jsk_footstep_msgs::Footstep::*LEFT*)
        ((= const 3) 'jsk_footstep_msgs::Footstep::*REJECTED*)
        ((= const 4) 'jsk_footstep_msgs::Footstep::*APPROVED*)
        ((= const 1) 'jsk_footstep_msgs::Footstep::*LLEG*)
        ((= const 2) 'jsk_footstep_msgs::Footstep::*RLEG*)
        ((= const 5) 'jsk_footstep_msgs::Footstep::*LARM*)
        ((= const 6) 'jsk_footstep_msgs::Footstep::*RARM*)
        (t nil)))

(defclass jsk_footstep_msgs::Footstep
  :super ros::object
  :slots (_leg _pose _duration _footstep_group _dimensions _offset _swing_height _cost ))

(defmethod jsk_footstep_msgs::Footstep
  (:init
   (&key
    ((:leg __leg) 0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:duration __duration) (instance ros::time :init))
    ((:footstep_group __footstep_group) 0)
    ((:dimensions __dimensions) (instance geometry_msgs::Vector3 :init))
    ((:offset __offset) (instance geometry_msgs::Vector3 :init))
    ((:swing_height __swing_height) 0.0)
    ((:cost __cost) 0.0)
    )
   (send-super :init)
   (setq _leg (round __leg))
   (setq _pose __pose)
   (setq _duration __duration)
   (setq _footstep_group (round __footstep_group))
   (setq _dimensions __dimensions)
   (setq _offset __offset)
   (setq _swing_height (float __swing_height))
   (setq _cost (float __cost))
   self)
  (:leg
   (&optional __leg)
   (if __leg (setq _leg __leg)) _leg)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:footstep_group
   (&optional __footstep_group)
   (if __footstep_group (setq _footstep_group __footstep_group)) _footstep_group)
  (:dimensions
   (&rest __dimensions)
   (if (keywordp (car __dimensions))
       (send* _dimensions __dimensions)
     (progn
       (if __dimensions (setq _dimensions (car __dimensions)))
       _dimensions)))
  (:offset
   (&rest __offset)
   (if (keywordp (car __offset))
       (send* _offset __offset)
     (progn
       (if __offset (setq _offset (car __offset)))
       _offset)))
  (:swing_height
   (&optional __swing_height)
   (if __swing_height (setq _swing_height __swing_height)) _swing_height)
  (:cost
   (&optional __cost)
   (if __cost (setq _cost __cost)) _cost)
  (:serialization-length
   ()
   (+
    ;; uint8 _leg
    1
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; duration _duration
    8
    ;; uint32 _footstep_group
    4
    ;; geometry_msgs/Vector3 _dimensions
    (send _dimensions :serialization-length)
    ;; geometry_msgs/Vector3 _offset
    (send _offset :serialization-length)
    ;; float32 _swing_height
    4
    ;; float32 _cost
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _leg
       (write-byte _leg s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; duration _duration
       (write-long (send _duration :sec) s) (write-long (send _duration :nsec) s)
     ;; uint32 _footstep_group
       (write-long _footstep_group s)
     ;; geometry_msgs/Vector3 _dimensions
       (send _dimensions :serialize s)
     ;; geometry_msgs/Vector3 _offset
       (send _offset :serialize s)
     ;; float32 _swing_height
       (sys::poke _swing_height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cost
       (sys::poke _cost (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _leg
     (setq _leg (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; duration _duration
     (send _duration :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _duration :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _footstep_group
     (setq _footstep_group (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Vector3 _dimensions
     (send _dimensions :deserialize buf ptr-) (incf ptr- (send _dimensions :serialization-length))
   ;; geometry_msgs/Vector3 _offset
     (send _offset :deserialize buf ptr-) (incf ptr- (send _offset :serialization-length))
   ;; float32 _swing_height
     (setq _swing_height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cost
     (setq _cost (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get jsk_footstep_msgs::Footstep :md5sum-) "d890b275b63a90fe5f22a21e9a879971")
(setf (get jsk_footstep_msgs::Footstep :datatype-) "jsk_footstep_msgs/Footstep")
(setf (get jsk_footstep_msgs::Footstep :definition-)
      "uint8 RIGHT=2
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



(provide :jsk_footstep_msgs/Footstep "d890b275b63a90fe5f22a21e9a879971")


