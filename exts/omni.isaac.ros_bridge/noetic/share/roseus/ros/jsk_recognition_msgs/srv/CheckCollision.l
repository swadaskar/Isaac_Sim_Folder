;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::CheckCollision)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'CheckCollision (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::CHECKCOLLISION")
  (make-package "JSK_RECOGNITION_MSGS::CHECKCOLLISION"))
(unless (find-package "JSK_RECOGNITION_MSGS::CHECKCOLLISIONREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::CHECKCOLLISIONREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::CHECKCOLLISIONRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::CHECKCOLLISIONRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))




(defclass jsk_recognition_msgs::CheckCollisionRequest
  :super ros::object
  :slots (_joint _pose ))

(defmethod jsk_recognition_msgs::CheckCollisionRequest
  (:init
   (&key
    ((:joint __joint) (instance sensor_msgs::JointState :init))
    ((:pose __pose) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _joint __joint)
   (setq _pose __pose)
   self)
  (:joint
   (&rest __joint)
   (if (keywordp (car __joint))
       (send* _joint __joint)
     (progn
       (if __joint (setq _joint (car __joint)))
       _joint)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/JointState _joint
    (send _joint :serialization-length)
    ;; geometry_msgs/PoseStamped _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/JointState _joint
       (send _joint :serialize s)
     ;; geometry_msgs/PoseStamped _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/JointState _joint
     (send _joint :deserialize buf ptr-) (incf ptr- (send _joint :serialization-length))
   ;; geometry_msgs/PoseStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::CheckCollisionResponse
  :super ros::object
  :slots (_result ))

(defmethod jsk_recognition_msgs::CheckCollisionResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional (__result :null))
   (if (not (eq __result :null)) (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass jsk_recognition_msgs::CheckCollision
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::CheckCollision :md5sum-) "3a30746acfb91fe161523d1e16f11286")
(setf (get jsk_recognition_msgs::CheckCollision :datatype-) "jsk_recognition_msgs/CheckCollision")
(setf (get jsk_recognition_msgs::CheckCollision :request) jsk_recognition_msgs::CheckCollisionRequest)
(setf (get jsk_recognition_msgs::CheckCollision :response) jsk_recognition_msgs::CheckCollisionResponse)

(defmethod jsk_recognition_msgs::CheckCollisionRequest
  (:response () (instance jsk_recognition_msgs::CheckCollisionResponse :init)))

(setf (get jsk_recognition_msgs::CheckCollisionRequest :md5sum-) "3a30746acfb91fe161523d1e16f11286")
(setf (get jsk_recognition_msgs::CheckCollisionRequest :datatype-) "jsk_recognition_msgs/CheckCollisionRequest")
(setf (get jsk_recognition_msgs::CheckCollisionRequest :definition-)
      "sensor_msgs/JointState joint
geometry_msgs/PoseStamped pose

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

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
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
---
bool result

")

(setf (get jsk_recognition_msgs::CheckCollisionResponse :md5sum-) "3a30746acfb91fe161523d1e16f11286")
(setf (get jsk_recognition_msgs::CheckCollisionResponse :datatype-) "jsk_recognition_msgs/CheckCollisionResponse")
(setf (get jsk_recognition_msgs::CheckCollisionResponse :definition-)
      "sensor_msgs/JointState joint
geometry_msgs/PoseStamped pose

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

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
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
---
bool result

")



(provide :jsk_recognition_msgs/CheckCollision "3a30746acfb91fe161523d1e16f11286")


