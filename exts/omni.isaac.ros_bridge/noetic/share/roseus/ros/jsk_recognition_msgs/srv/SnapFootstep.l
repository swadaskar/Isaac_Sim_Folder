;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SnapFootstep)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SnapFootstep (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SNAPFOOTSTEP")
  (make-package "JSK_RECOGNITION_MSGS::SNAPFOOTSTEP"))
(unless (find-package "JSK_RECOGNITION_MSGS::SNAPFOOTSTEPREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::SNAPFOOTSTEPREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::SNAPFOOTSTEPRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::SNAPFOOTSTEPRESPONSE"))

(in-package "ROS")

(if (not (find-package "JSK_FOOTSTEP_MSGS"))
  (ros::roseus-add-msgs "jsk_footstep_msgs"))


(if (not (find-package "JSK_FOOTSTEP_MSGS"))
  (ros::roseus-add-msgs "jsk_footstep_msgs"))


(defclass jsk_recognition_msgs::SnapFootstepRequest
  :super ros::object
  :slots (_input ))

(defmethod jsk_recognition_msgs::SnapFootstepRequest
  (:init
   (&key
    ((:input __input) (instance jsk_footstep_msgs::FootstepArray :init))
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&rest __input)
   (if (keywordp (car __input))
       (send* _input __input)
     (progn
       (if __input (setq _input (car __input)))
       _input)))
  (:serialization-length
   ()
   (+
    ;; jsk_footstep_msgs/FootstepArray _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; jsk_footstep_msgs/FootstepArray _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; jsk_footstep_msgs/FootstepArray _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::SnapFootstepResponse
  :super ros::object
  :slots (_output ))

(defmethod jsk_recognition_msgs::SnapFootstepResponse
  (:init
   (&key
    ((:output __output) (instance jsk_footstep_msgs::FootstepArray :init))
    )
   (send-super :init)
   (setq _output __output)
   self)
  (:output
   (&rest __output)
   (if (keywordp (car __output))
       (send* _output __output)
     (progn
       (if __output (setq _output (car __output)))
       _output)))
  (:serialization-length
   ()
   (+
    ;; jsk_footstep_msgs/FootstepArray _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; jsk_footstep_msgs/FootstepArray _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; jsk_footstep_msgs/FootstepArray _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::SnapFootstep
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::SnapFootstep :md5sum-) "ea63ea1160bdd6c252d3a568e9f2ee4f")
(setf (get jsk_recognition_msgs::SnapFootstep :datatype-) "jsk_recognition_msgs/SnapFootstep")
(setf (get jsk_recognition_msgs::SnapFootstep :request) jsk_recognition_msgs::SnapFootstepRequest)
(setf (get jsk_recognition_msgs::SnapFootstep :response) jsk_recognition_msgs::SnapFootstepResponse)

(defmethod jsk_recognition_msgs::SnapFootstepRequest
  (:response () (instance jsk_recognition_msgs::SnapFootstepResponse :init)))

(setf (get jsk_recognition_msgs::SnapFootstepRequest :md5sum-) "ea63ea1160bdd6c252d3a568e9f2ee4f")
(setf (get jsk_recognition_msgs::SnapFootstepRequest :datatype-) "jsk_recognition_msgs/SnapFootstepRequest")
(setf (get jsk_recognition_msgs::SnapFootstepRequest :definition-)
      "jsk_footstep_msgs/FootstepArray input

================================================================================
MSG: jsk_footstep_msgs/FootstepArray
Header header
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
---
jsk_footstep_msgs/FootstepArray output


================================================================================
MSG: jsk_footstep_msgs/FootstepArray
Header header
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
")

(setf (get jsk_recognition_msgs::SnapFootstepResponse :md5sum-) "ea63ea1160bdd6c252d3a568e9f2ee4f")
(setf (get jsk_recognition_msgs::SnapFootstepResponse :datatype-) "jsk_recognition_msgs/SnapFootstepResponse")
(setf (get jsk_recognition_msgs::SnapFootstepResponse :definition-)
      "jsk_footstep_msgs/FootstepArray input

================================================================================
MSG: jsk_footstep_msgs/FootstepArray
Header header
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
---
jsk_footstep_msgs/FootstepArray output


================================================================================
MSG: jsk_footstep_msgs/FootstepArray
Header header
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
")



(provide :jsk_recognition_msgs/SnapFootstep "ea63ea1160bdd6c252d3a568e9f2ee4f")


