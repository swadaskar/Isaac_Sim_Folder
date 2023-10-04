;; Auto-generated. Do not edit!


(when (boundp 'nav_msgs::LoadMap)
  (if (not (find-package "NAV_MSGS"))
    (make-package "NAV_MSGS"))
  (shadow 'LoadMap (find-package "NAV_MSGS")))
(unless (find-package "NAV_MSGS::LOADMAP")
  (make-package "NAV_MSGS::LOADMAP"))
(unless (find-package "NAV_MSGS::LOADMAPREQUEST")
  (make-package "NAV_MSGS::LOADMAPREQUEST"))
(unless (find-package "NAV_MSGS::LOADMAPRESPONSE")
  (make-package "NAV_MSGS::LOADMAPRESPONSE"))

(in-package "ROS")





(defclass nav_msgs::LoadMapRequest
  :super ros::object
  :slots (_map_url ))

(defmethod nav_msgs::LoadMapRequest
  (:init
   (&key
    ((:map_url __map_url) "")
    )
   (send-super :init)
   (setq _map_url (string __map_url))
   self)
  (:map_url
   (&optional __map_url)
   (if __map_url (setq _map_url __map_url)) _map_url)
  (:serialization-length
   ()
   (+
    ;; string _map_url
    4 (length _map_url)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _map_url
       (write-long (length _map_url) s) (princ _map_url s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _map_url
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _map_url (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(intern "*RESULT_SUCCESS*" (find-package "NAV_MSGS::LOADMAPRESPONSE"))
(shadow '*RESULT_SUCCESS* (find-package "NAV_MSGS::LOADMAPRESPONSE"))
(defconstant nav_msgs::LoadMapResponse::*RESULT_SUCCESS* 0)
(intern "*RESULT_MAP_DOES_NOT_EXIST*" (find-package "NAV_MSGS::LOADMAPRESPONSE"))
(shadow '*RESULT_MAP_DOES_NOT_EXIST* (find-package "NAV_MSGS::LOADMAPRESPONSE"))
(defconstant nav_msgs::LoadMapResponse::*RESULT_MAP_DOES_NOT_EXIST* 1)
(intern "*RESULT_INVALID_MAP_DATA*" (find-package "NAV_MSGS::LOADMAPRESPONSE"))
(shadow '*RESULT_INVALID_MAP_DATA* (find-package "NAV_MSGS::LOADMAPRESPONSE"))
(defconstant nav_msgs::LoadMapResponse::*RESULT_INVALID_MAP_DATA* 2)
(intern "*RESULT_INVALID_MAP_METADATA*" (find-package "NAV_MSGS::LOADMAPRESPONSE"))
(shadow '*RESULT_INVALID_MAP_METADATA* (find-package "NAV_MSGS::LOADMAPRESPONSE"))
(defconstant nav_msgs::LoadMapResponse::*RESULT_INVALID_MAP_METADATA* 3)
(intern "*RESULT_UNDEFINED_FAILURE*" (find-package "NAV_MSGS::LOADMAPRESPONSE"))
(shadow '*RESULT_UNDEFINED_FAILURE* (find-package "NAV_MSGS::LOADMAPRESPONSE"))
(defconstant nav_msgs::LoadMapResponse::*RESULT_UNDEFINED_FAILURE* 255)

(defun nav_msgs::LoadMapResponse-to-symbol (const)
  (cond
        ((= const 0) 'nav_msgs::LoadMapResponse::*RESULT_SUCCESS*)
        ((= const 1) 'nav_msgs::LoadMapResponse::*RESULT_MAP_DOES_NOT_EXIST*)
        ((= const 2) 'nav_msgs::LoadMapResponse::*RESULT_INVALID_MAP_DATA*)
        ((= const 3) 'nav_msgs::LoadMapResponse::*RESULT_INVALID_MAP_METADATA*)
        ((= const 255) 'nav_msgs::LoadMapResponse::*RESULT_UNDEFINED_FAILURE*)
        (t nil)))

(defclass nav_msgs::LoadMapResponse
  :super ros::object
  :slots (_map _result ))

(defmethod nav_msgs::LoadMapResponse
  (:init
   (&key
    ((:map __map) (instance nav_msgs::OccupancyGrid :init))
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _map __map)
   (setq _result (round __result))
   self)
  (:map
   (&rest __map)
   (if (keywordp (car __map))
       (send* _map __map)
     (progn
       (if __map (setq _map (car __map)))
       _map)))
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; nav_msgs/OccupancyGrid _map
    (send _map :serialization-length)
    ;; uint8 _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/OccupancyGrid _map
       (send _map :serialize s)
     ;; uint8 _result
       (write-byte _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/OccupancyGrid _map
     (send _map :deserialize buf ptr-) (incf ptr- (send _map :serialization-length))
   ;; uint8 _result
     (setq _result (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass nav_msgs::LoadMap
  :super ros::object
  :slots ())

(setf (get nav_msgs::LoadMap :md5sum-) "22e647fdfbe3b23c8c9f419908afaebd")
(setf (get nav_msgs::LoadMap :datatype-) "nav_msgs/LoadMap")
(setf (get nav_msgs::LoadMap :request) nav_msgs::LoadMapRequest)
(setf (get nav_msgs::LoadMap :response) nav_msgs::LoadMapResponse)

(defmethod nav_msgs::LoadMapRequest
  (:response () (instance nav_msgs::LoadMapResponse :init)))

(setf (get nav_msgs::LoadMapRequest :md5sum-) "22e647fdfbe3b23c8c9f419908afaebd")
(setf (get nav_msgs::LoadMapRequest :datatype-) "nav_msgs/LoadMapRequest")
(setf (get nav_msgs::LoadMapRequest :definition-)
      "# URL of map resource
# Can be an absolute path to a file: file:///path/to/maps/floor1.yaml
# Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml
string map_url
---
# Result code defintions
uint8 RESULT_SUCCESS=0
uint8 RESULT_MAP_DOES_NOT_EXIST=1
uint8 RESULT_INVALID_MAP_DATA=2
uint8 RESULT_INVALID_MAP_METADATA=3
uint8 RESULT_UNDEFINED_FAILURE=255

# Returned map is only valid if result equals RESULT_SUCCESS
nav_msgs/OccupancyGrid map
uint8 result


================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data

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
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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

(setf (get nav_msgs::LoadMapResponse :md5sum-) "22e647fdfbe3b23c8c9f419908afaebd")
(setf (get nav_msgs::LoadMapResponse :datatype-) "nav_msgs/LoadMapResponse")
(setf (get nav_msgs::LoadMapResponse :definition-)
      "# URL of map resource
# Can be an absolute path to a file: file:///path/to/maps/floor1.yaml
# Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml
string map_url
---
# Result code defintions
uint8 RESULT_SUCCESS=0
uint8 RESULT_MAP_DOES_NOT_EXIST=1
uint8 RESULT_INVALID_MAP_DATA=2
uint8 RESULT_INVALID_MAP_METADATA=3
uint8 RESULT_UNDEFINED_FAILURE=255

# Returned map is only valid if result equals RESULT_SUCCESS
nav_msgs/OccupancyGrid map
uint8 result


================================================================================
MSG: nav_msgs/OccupancyGrid
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data

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
MSG: nav_msgs/MapMetaData
# This hold basic information about the characterists of the OccupancyGrid

# The time at which the map was loaded
time map_load_time
# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map height [cells]
uint32 height
# The origin of the map [m, m, rad].  This is the real-world pose of the
# cell (0,0) in the map.
geometry_msgs/Pose origin
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



(provide :nav_msgs/LoadMap "22e647fdfbe3b23c8c9f419908afaebd")


