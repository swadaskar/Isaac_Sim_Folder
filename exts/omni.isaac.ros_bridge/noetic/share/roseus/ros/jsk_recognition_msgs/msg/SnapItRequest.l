;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SnapItRequest)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SnapItRequest (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SNAPITREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::SNAPITREQUEST"))

(in-package "ROS")
;;//! \htmlinclude SnapItRequest.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*MODEL_PLANE*" (find-package "JSK_RECOGNITION_MSGS::SNAPITREQUEST"))
(shadow '*MODEL_PLANE* (find-package "JSK_RECOGNITION_MSGS::SNAPITREQUEST"))
(defconstant jsk_recognition_msgs::SnapItRequest::*MODEL_PLANE* 0)
(intern "*MODEL_CYLINDER*" (find-package "JSK_RECOGNITION_MSGS::SNAPITREQUEST"))
(shadow '*MODEL_CYLINDER* (find-package "JSK_RECOGNITION_MSGS::SNAPITREQUEST"))
(defconstant jsk_recognition_msgs::SnapItRequest::*MODEL_CYLINDER* 1)

(defun jsk_recognition_msgs::SnapItRequest-to-symbol (const)
  (cond
        ((= const 0) 'jsk_recognition_msgs::SnapItRequest::*MODEL_PLANE*)
        ((= const 1) 'jsk_recognition_msgs::SnapItRequest::*MODEL_CYLINDER*)
        (t nil)))

(defclass jsk_recognition_msgs::SnapItRequest
  :super ros::object
  :slots (_header _model_type _target_plane _center _direction _radius _height _max_distance _eps_angle ))

(defmethod jsk_recognition_msgs::SnapItRequest
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:model_type __model_type) 0)
    ((:target_plane __target_plane) (instance geometry_msgs::PolygonStamped :init))
    ((:center __center) (instance geometry_msgs::PointStamped :init))
    ((:direction __direction) (instance geometry_msgs::Vector3Stamped :init))
    ((:radius __radius) 0.0)
    ((:height __height) 0.0)
    ((:max_distance __max_distance) 0.0)
    ((:eps_angle __eps_angle) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _model_type (round __model_type))
   (setq _target_plane __target_plane)
   (setq _center __center)
   (setq _direction __direction)
   (setq _radius (float __radius))
   (setq _height (float __height))
   (setq _max_distance (float __max_distance))
   (setq _eps_angle (float __eps_angle))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:model_type
   (&optional __model_type)
   (if __model_type (setq _model_type __model_type)) _model_type)
  (:target_plane
   (&rest __target_plane)
   (if (keywordp (car __target_plane))
       (send* _target_plane __target_plane)
     (progn
       (if __target_plane (setq _target_plane (car __target_plane)))
       _target_plane)))
  (:center
   (&rest __center)
   (if (keywordp (car __center))
       (send* _center __center)
     (progn
       (if __center (setq _center (car __center)))
       _center)))
  (:direction
   (&rest __direction)
   (if (keywordp (car __direction))
       (send* _direction __direction)
     (progn
       (if __direction (setq _direction (car __direction)))
       _direction)))
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:max_distance
   (&optional __max_distance)
   (if __max_distance (setq _max_distance __max_distance)) _max_distance)
  (:eps_angle
   (&optional __eps_angle)
   (if __eps_angle (setq _eps_angle __eps_angle)) _eps_angle)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _model_type
    1
    ;; geometry_msgs/PolygonStamped _target_plane
    (send _target_plane :serialization-length)
    ;; geometry_msgs/PointStamped _center
    (send _center :serialization-length)
    ;; geometry_msgs/Vector3Stamped _direction
    (send _direction :serialization-length)
    ;; float64 _radius
    8
    ;; float64 _height
    8
    ;; float64 _max_distance
    8
    ;; float64 _eps_angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _model_type
       (write-byte _model_type s)
     ;; geometry_msgs/PolygonStamped _target_plane
       (send _target_plane :serialize s)
     ;; geometry_msgs/PointStamped _center
       (send _center :serialize s)
     ;; geometry_msgs/Vector3Stamped _direction
       (send _direction :serialize s)
     ;; float64 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _height
       (sys::poke _height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max_distance
       (sys::poke _max_distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _eps_angle
       (sys::poke _eps_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _model_type
     (setq _model_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/PolygonStamped _target_plane
     (send _target_plane :deserialize buf ptr-) (incf ptr- (send _target_plane :serialization-length))
   ;; geometry_msgs/PointStamped _center
     (send _center :deserialize buf ptr-) (incf ptr- (send _center :serialization-length))
   ;; geometry_msgs/Vector3Stamped _direction
     (send _direction :deserialize buf ptr-) (incf ptr- (send _direction :serialization-length))
   ;; float64 _radius
     (setq _radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _height
     (setq _height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_distance
     (setq _max_distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _eps_angle
     (setq _eps_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::SnapItRequest :md5sum-) "5733f480694296678d81cff0483b399b")
(setf (get jsk_recognition_msgs::SnapItRequest :datatype-) "jsk_recognition_msgs/SnapItRequest")
(setf (get jsk_recognition_msgs::SnapItRequest :definition-)
      "Header header
uint8 MODEL_PLANE=0
uint8 MODEL_CYLINDER=1
uint8 model_type

geometry_msgs/PolygonStamped target_plane

geometry_msgs/PointStamped center
geometry_msgs/Vector3Stamped direction
float64 radius
float64 height
# parameters, 0 means 
float64 max_distance
float64 eps_angle

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
MSG: geometry_msgs/PolygonStamped
# This represents a Polygon with reference coordinate frame and timestamp
Header header
Polygon polygon

================================================================================
MSG: geometry_msgs/Polygon
#A specification of a polygon where the first and last points are assumed to be connected
Point32[] points

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z
================================================================================
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3Stamped
# This represents a Vector3 with reference coordinate frame and timestamp
Header header
Vector3 vector

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



(provide :jsk_recognition_msgs/SnapItRequest "5733f480694296678d81cff0483b399b")


