;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::TransformScreenpoint)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'TransformScreenpoint (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::TRANSFORMSCREENPOINT")
  (make-package "JSK_RECOGNITION_MSGS::TRANSFORMSCREENPOINT"))
(unless (find-package "JSK_RECOGNITION_MSGS::TRANSFORMSCREENPOINTREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::TRANSFORMSCREENPOINTREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::TRANSFORMSCREENPOINTRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::TRANSFORMSCREENPOINTRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::TransformScreenpointRequest
  :super ros::object
  :slots (_x _y _no_update ))

(defmethod jsk_recognition_msgs::TransformScreenpointRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:no_update __no_update) nil)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _no_update __no_update)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:no_update
   (&optional (__no_update :null))
   (if (not (eq __no_update :null)) (setq _no_update __no_update)) _no_update)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; bool _no_update
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _no_update
       (if _no_update (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _no_update
     (setq _no_update (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass jsk_recognition_msgs::TransformScreenpointResponse
  :super ros::object
  :slots (_header _point _vector ))

(defmethod jsk_recognition_msgs::TransformScreenpointResponse
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:point __point) (instance geometry_msgs::Point :init))
    ((:vector __vector) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _point __point)
   (setq _vector __vector)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:vector
   (&rest __vector)
   (if (keywordp (car __vector))
       (send* _vector __vector)
     (progn
       (if __vector (setq _vector (car __vector)))
       _vector)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ;; geometry_msgs/Vector3 _vector
    (send _vector :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;; geometry_msgs/Vector3 _vector
       (send _vector :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; geometry_msgs/Vector3 _vector
     (send _vector :deserialize buf ptr-) (incf ptr- (send _vector :serialization-length))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::TransformScreenpoint
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::TransformScreenpoint :md5sum-) "ad80b7dbbce8c5ae0833e8fb24e35c06")
(setf (get jsk_recognition_msgs::TransformScreenpoint :datatype-) "jsk_recognition_msgs/TransformScreenpoint")
(setf (get jsk_recognition_msgs::TransformScreenpoint :request) jsk_recognition_msgs::TransformScreenpointRequest)
(setf (get jsk_recognition_msgs::TransformScreenpoint :response) jsk_recognition_msgs::TransformScreenpointResponse)

(defmethod jsk_recognition_msgs::TransformScreenpointRequest
  (:response () (instance jsk_recognition_msgs::TransformScreenpointResponse :init)))

(setf (get jsk_recognition_msgs::TransformScreenpointRequest :md5sum-) "ad80b7dbbce8c5ae0833e8fb24e35c06")
(setf (get jsk_recognition_msgs::TransformScreenpointRequest :datatype-) "jsk_recognition_msgs/TransformScreenpointRequest")
(setf (get jsk_recognition_msgs::TransformScreenpointRequest :definition-)
      "# screen point
float32 x
float32 y
bool no_update
---
# position in actual world
std_msgs/Header header
geometry_msgs/Point point
geometry_msgs/Vector3 vector


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

(setf (get jsk_recognition_msgs::TransformScreenpointResponse :md5sum-) "ad80b7dbbce8c5ae0833e8fb24e35c06")
(setf (get jsk_recognition_msgs::TransformScreenpointResponse :datatype-) "jsk_recognition_msgs/TransformScreenpointResponse")
(setf (get jsk_recognition_msgs::TransformScreenpointResponse :definition-)
      "# screen point
float32 x
float32 y
bool no_update
---
# position in actual world
std_msgs/Header header
geometry_msgs/Point point
geometry_msgs/Vector3 vector


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



(provide :jsk_recognition_msgs/TransformScreenpoint "ad80b7dbbce8c5ae0833e8fb24e35c06")


