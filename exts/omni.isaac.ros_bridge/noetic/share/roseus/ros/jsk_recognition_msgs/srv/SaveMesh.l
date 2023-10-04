;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SaveMesh)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SaveMesh (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SAVEMESH")
  (make-package "JSK_RECOGNITION_MSGS::SAVEMESH"))
(unless (find-package "JSK_RECOGNITION_MSGS::SAVEMESHREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::SAVEMESHREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::SAVEMESHRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::SAVEMESHRESPONSE"))

(in-package "ROS")





(defclass jsk_recognition_msgs::SaveMeshRequest
  :super ros::object
  :slots (_ground_frame_id _box ))

(defmethod jsk_recognition_msgs::SaveMeshRequest
  (:init
   (&key
    ((:ground_frame_id __ground_frame_id) "")
    ((:box __box) (instance jsk_recognition_msgs::BoundingBox :init))
    )
   (send-super :init)
   (setq _ground_frame_id (string __ground_frame_id))
   (setq _box __box)
   self)
  (:ground_frame_id
   (&optional __ground_frame_id)
   (if __ground_frame_id (setq _ground_frame_id __ground_frame_id)) _ground_frame_id)
  (:box
   (&rest __box)
   (if (keywordp (car __box))
       (send* _box __box)
     (progn
       (if __box (setq _box (car __box)))
       _box)))
  (:serialization-length
   ()
   (+
    ;; string _ground_frame_id
    4 (length _ground_frame_id)
    ;; jsk_recognition_msgs/BoundingBox _box
    (send _box :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _ground_frame_id
       (write-long (length _ground_frame_id) s) (princ _ground_frame_id s)
     ;; jsk_recognition_msgs/BoundingBox _box
       (send _box :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _ground_frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ground_frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; jsk_recognition_msgs/BoundingBox _box
     (send _box :deserialize buf ptr-) (incf ptr- (send _box :serialization-length))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::SaveMeshResponse
  :super ros::object
  :slots (_success ))

(defmethod jsk_recognition_msgs::SaveMeshResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass jsk_recognition_msgs::SaveMesh
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::SaveMesh :md5sum-) "bac972d77fa349fd55cd1ee98f47a7b0")
(setf (get jsk_recognition_msgs::SaveMesh :datatype-) "jsk_recognition_msgs/SaveMesh")
(setf (get jsk_recognition_msgs::SaveMesh :request) jsk_recognition_msgs::SaveMeshRequest)
(setf (get jsk_recognition_msgs::SaveMesh :response) jsk_recognition_msgs::SaveMeshResponse)

(defmethod jsk_recognition_msgs::SaveMeshRequest
  (:response () (instance jsk_recognition_msgs::SaveMeshResponse :init)))

(setf (get jsk_recognition_msgs::SaveMeshRequest :md5sum-) "bac972d77fa349fd55cd1ee98f47a7b0")
(setf (get jsk_recognition_msgs::SaveMeshRequest :datatype-) "jsk_recognition_msgs/SaveMeshRequest")
(setf (get jsk_recognition_msgs::SaveMeshRequest :definition-)
      "string ground_frame_id  # to create solid model
jsk_recognition_msgs/BoundingBox box  # to crop mesh

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
bool success

")

(setf (get jsk_recognition_msgs::SaveMeshResponse :md5sum-) "bac972d77fa349fd55cd1ee98f47a7b0")
(setf (get jsk_recognition_msgs::SaveMeshResponse :datatype-) "jsk_recognition_msgs/SaveMeshResponse")
(setf (get jsk_recognition_msgs::SaveMeshResponse :definition-)
      "string ground_frame_id  # to create solid model
jsk_recognition_msgs/BoundingBox box  # to crop mesh

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
bool success

")



(provide :jsk_recognition_msgs/SaveMesh "bac972d77fa349fd55cd1ee98f47a7b0")


