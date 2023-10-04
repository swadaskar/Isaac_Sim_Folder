;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::WeightedPoseArray)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'WeightedPoseArray (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::WEIGHTEDPOSEARRAY")
  (make-package "JSK_RECOGNITION_MSGS::WEIGHTEDPOSEARRAY"))

(in-package "ROS")
;;//! \htmlinclude WeightedPoseArray.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::WeightedPoseArray
  :super ros::object
  :slots (_header _weights _poses ))

(defmethod jsk_recognition_msgs::WeightedPoseArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:weights __weights) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:poses __poses) (instance geometry_msgs::PoseArray :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _weights __weights)
   (setq _poses __poses)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:weights
   (&optional __weights)
   (if __weights (setq _weights __weights)) _weights)
  (:poses
   (&rest __poses)
   (if (keywordp (car __poses))
       (send* _poses __poses)
     (progn
       (if __poses (setq _poses (car __poses)))
       _poses)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[] _weights
    (* 4    (length _weights)) 4
    ;; geometry_msgs/PoseArray _poses
    (send _poses :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[] _weights
     (write-long (length _weights) s)
     (dotimes (i (length _weights))
       (sys::poke (elt _weights i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; geometry_msgs/PoseArray _poses
       (send _poses :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[] _weights
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _weights (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _weights i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; geometry_msgs/PoseArray _poses
     (send _poses :deserialize buf ptr-) (incf ptr- (send _poses :serialization-length))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::WeightedPoseArray :md5sum-) "40f180494a75a8797b1c2ba81b2cb4c0")
(setf (get jsk_recognition_msgs::WeightedPoseArray :datatype-) "jsk_recognition_msgs/WeightedPoseArray")
(setf (get jsk_recognition_msgs::WeightedPoseArray :definition-)
      "Header header
float32[] weights
geometry_msgs/PoseArray poses

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
MSG: geometry_msgs/PoseArray
# An array of poses with a header for global reference.

Header header

Pose[] poses

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

")



(provide :jsk_recognition_msgs/WeightedPoseArray "40f180494a75a8797b1c2ba81b2cb4c0")


