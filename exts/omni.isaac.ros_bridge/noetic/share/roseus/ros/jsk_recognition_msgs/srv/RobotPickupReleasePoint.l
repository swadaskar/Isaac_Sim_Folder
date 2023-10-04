;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::RobotPickupReleasePoint)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'RobotPickupReleasePoint (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::ROBOTPICKUPRELEASEPOINT")
  (make-package "JSK_RECOGNITION_MSGS::ROBOTPICKUPRELEASEPOINT"))
(unless (find-package "JSK_RECOGNITION_MSGS::ROBOTPICKUPRELEASEPOINTREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::ROBOTPICKUPRELEASEPOINTREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::ROBOTPICKUPRELEASEPOINTRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::ROBOTPICKUPRELEASEPOINTRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass jsk_recognition_msgs::RobotPickupReleasePointRequest
  :super ros::object
  :slots (_header _target_point _pick_or_release ))

(defmethod jsk_recognition_msgs::RobotPickupReleasePointRequest
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:target_point __target_point) (instance geometry_msgs::Point :init))
    ((:pick_or_release __pick_or_release) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _target_point __target_point)
   (setq _pick_or_release (round __pick_or_release))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:target_point
   (&rest __target_point)
   (if (keywordp (car __target_point))
       (send* _target_point __target_point)
     (progn
       (if __target_point (setq _target_point (car __target_point)))
       _target_point)))
  (:pick_or_release
   (&optional __pick_or_release)
   (if __pick_or_release (setq _pick_or_release __pick_or_release)) _pick_or_release)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _target_point
    (send _target_point :serialization-length)
    ;; byte _pick_or_release
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _target_point
       (send _target_point :serialize s)
     ;; byte _pick_or_release
       (write-byte _pick_or_release s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _target_point
     (send _target_point :deserialize buf ptr-) (incf ptr- (send _target_point :serialization-length))
   ;; byte _pick_or_release
     (setq _pick_or_release (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _pick_or_release 127) (setq _pick_or_release (- _pick_or_release 256)))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::RobotPickupReleasePointResponse
  :super ros::object
  :slots (_success ))

(defmethod jsk_recognition_msgs::RobotPickupReleasePointResponse
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

(defclass jsk_recognition_msgs::RobotPickupReleasePoint
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::RobotPickupReleasePoint :md5sum-) "b6469871ddb5ec099685daff904ecc2f")
(setf (get jsk_recognition_msgs::RobotPickupReleasePoint :datatype-) "jsk_recognition_msgs/RobotPickupReleasePoint")
(setf (get jsk_recognition_msgs::RobotPickupReleasePoint :request) jsk_recognition_msgs::RobotPickupReleasePointRequest)
(setf (get jsk_recognition_msgs::RobotPickupReleasePoint :response) jsk_recognition_msgs::RobotPickupReleasePointResponse)

(defmethod jsk_recognition_msgs::RobotPickupReleasePointRequest
  (:response () (instance jsk_recognition_msgs::RobotPickupReleasePointResponse :init)))

(setf (get jsk_recognition_msgs::RobotPickupReleasePointRequest :md5sum-) "b6469871ddb5ec099685daff904ecc2f")
(setf (get jsk_recognition_msgs::RobotPickupReleasePointRequest :datatype-) "jsk_recognition_msgs/RobotPickupReleasePointRequest")
(setf (get jsk_recognition_msgs::RobotPickupReleasePointRequest :definition-)
      "Header header
geometry_msgs/Point target_point
byte pick_or_release  # 0 -> pick, 1 -> release

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool success

")

(setf (get jsk_recognition_msgs::RobotPickupReleasePointResponse :md5sum-) "b6469871ddb5ec099685daff904ecc2f")
(setf (get jsk_recognition_msgs::RobotPickupReleasePointResponse :datatype-) "jsk_recognition_msgs/RobotPickupReleasePointResponse")
(setf (get jsk_recognition_msgs::RobotPickupReleasePointResponse :definition-)
      "Header header
geometry_msgs/Point target_point
byte pick_or_release  # 0 -> pick, 1 -> release

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool success

")



(provide :jsk_recognition_msgs/RobotPickupReleasePoint "b6469871ddb5ec099685daff904ecc2f")


