;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::Torus)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'Torus (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::TORUS")
  (make-package "JSK_RECOGNITION_MSGS::TORUS"))

(in-package "ROS")
;;//! \htmlinclude Torus.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::Torus
  :super ros::object
  :slots (_header _failure _pose _large_radius _small_radius ))

(defmethod jsk_recognition_msgs::Torus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:failure __failure) nil)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:large_radius __large_radius) 0.0)
    ((:small_radius __small_radius) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _failure __failure)
   (setq _pose __pose)
   (setq _large_radius (float __large_radius))
   (setq _small_radius (float __small_radius))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:failure
   (&optional (__failure :null))
   (if (not (eq __failure :null)) (setq _failure __failure)) _failure)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:large_radius
   (&optional __large_radius)
   (if __large_radius (setq _large_radius __large_radius)) _large_radius)
  (:small_radius
   (&optional __small_radius)
   (if __small_radius (setq _small_radius __small_radius)) _small_radius)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _failure
    1
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; float64 _large_radius
    8
    ;; float64 _small_radius
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _failure
       (if _failure (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; float64 _large_radius
       (sys::poke _large_radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _small_radius
       (sys::poke _small_radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _failure
     (setq _failure (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; float64 _large_radius
     (setq _large_radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _small_radius
     (setq _small_radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::Torus :md5sum-) "7172d433485e406ce56f4cf6e9ab1062")
(setf (get jsk_recognition_msgs::Torus :datatype-) "jsk_recognition_msgs/Torus")
(setf (get jsk_recognition_msgs::Torus :definition-)
      "Header header
bool failure
geometry_msgs/Pose pose
float64 large_radius
float64 small_radius

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



(provide :jsk_recognition_msgs/Torus "7172d433485e406ce56f4cf6e9ab1062")


