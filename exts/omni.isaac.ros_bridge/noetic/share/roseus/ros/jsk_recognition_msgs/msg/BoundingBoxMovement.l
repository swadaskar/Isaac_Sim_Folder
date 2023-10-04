;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::BoundingBoxMovement)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'BoundingBoxMovement (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::BOUNDINGBOXMOVEMENT")
  (make-package "JSK_RECOGNITION_MSGS::BOUNDINGBOXMOVEMENT"))

(in-package "ROS")
;;//! \htmlinclude BoundingBoxMovement.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::BoundingBoxMovement
  :super ros::object
  :slots (_header _box _handle_pose _destination ))

(defmethod jsk_recognition_msgs::BoundingBoxMovement
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:box __box) (instance jsk_recognition_msgs::BoundingBox :init))
    ((:handle_pose __handle_pose) (instance geometry_msgs::Pose :init))
    ((:destination __destination) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _box __box)
   (setq _handle_pose __handle_pose)
   (setq _destination __destination)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:box
   (&rest __box)
   (if (keywordp (car __box))
       (send* _box __box)
     (progn
       (if __box (setq _box (car __box)))
       _box)))
  (:handle_pose
   (&rest __handle_pose)
   (if (keywordp (car __handle_pose))
       (send* _handle_pose __handle_pose)
     (progn
       (if __handle_pose (setq _handle_pose (car __handle_pose)))
       _handle_pose)))
  (:destination
   (&rest __destination)
   (if (keywordp (car __destination))
       (send* _destination __destination)
     (progn
       (if __destination (setq _destination (car __destination)))
       _destination)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; jsk_recognition_msgs/BoundingBox _box
    (send _box :serialization-length)
    ;; geometry_msgs/Pose _handle_pose
    (send _handle_pose :serialization-length)
    ;; geometry_msgs/PoseStamped _destination
    (send _destination :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jsk_recognition_msgs/BoundingBox _box
       (send _box :serialize s)
     ;; geometry_msgs/Pose _handle_pose
       (send _handle_pose :serialize s)
     ;; geometry_msgs/PoseStamped _destination
       (send _destination :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; jsk_recognition_msgs/BoundingBox _box
     (send _box :deserialize buf ptr-) (incf ptr- (send _box :serialization-length))
   ;; geometry_msgs/Pose _handle_pose
     (send _handle_pose :deserialize buf ptr-) (incf ptr- (send _handle_pose :serialization-length))
   ;; geometry_msgs/PoseStamped _destination
     (send _destination :deserialize buf ptr-) (incf ptr- (send _destination :serialization-length))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::BoundingBoxMovement :md5sum-) "60aeb288c030d1b8a67ddeedc91812f0")
(setf (get jsk_recognition_msgs::BoundingBoxMovement :datatype-) "jsk_recognition_msgs/BoundingBoxMovement")
(setf (get jsk_recognition_msgs::BoundingBoxMovement :definition-)
      "Header header
BoundingBox box
geometry_msgs/Pose handle_pose
geometry_msgs/PoseStamped destination

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
MSG: jsk_recognition_msgs/BoundingBox
# BoundingBox represents a oriented bounding box.
Header header
geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)
# You can use this field to hold value such as likelihood
float32 value
uint32 label

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
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

")



(provide :jsk_recognition_msgs/BoundingBoxMovement "60aeb288c030d1b8a67ddeedc91812f0")


