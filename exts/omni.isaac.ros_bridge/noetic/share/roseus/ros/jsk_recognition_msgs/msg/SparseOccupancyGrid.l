;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SparseOccupancyGrid)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SparseOccupancyGrid (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SPARSEOCCUPANCYGRID")
  (make-package "JSK_RECOGNITION_MSGS::SPARSEOCCUPANCYGRID"))

(in-package "ROS")
;;//! \htmlinclude SparseOccupancyGrid.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::SparseOccupancyGrid
  :super ros::object
  :slots (_header _origin_pose _resolution _columns ))

(defmethod jsk_recognition_msgs::SparseOccupancyGrid
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:origin_pose __origin_pose) (instance geometry_msgs::Pose :init))
    ((:resolution __resolution) 0.0)
    ((:columns __columns) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _origin_pose __origin_pose)
   (setq _resolution (float __resolution))
   (setq _columns __columns)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:origin_pose
   (&rest __origin_pose)
   (if (keywordp (car __origin_pose))
       (send* _origin_pose __origin_pose)
     (progn
       (if __origin_pose (setq _origin_pose (car __origin_pose)))
       _origin_pose)))
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:columns
   (&rest __columns)
   (if (keywordp (car __columns))
       (send* _columns __columns)
     (progn
       (if __columns (setq _columns (car __columns)))
       _columns)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose _origin_pose
    (send _origin_pose :serialization-length)
    ;; float32 _resolution
    4
    ;; jsk_recognition_msgs/SparseOccupancyGridColumn[] _columns
    (apply #'+ (send-all _columns :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose _origin_pose
       (send _origin_pose :serialize s)
     ;; float32 _resolution
       (sys::poke _resolution (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; jsk_recognition_msgs/SparseOccupancyGridColumn[] _columns
     (write-long (length _columns) s)
     (dolist (elem _columns)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose _origin_pose
     (send _origin_pose :deserialize buf ptr-) (incf ptr- (send _origin_pose :serialization-length))
   ;; float32 _resolution
     (setq _resolution (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; jsk_recognition_msgs/SparseOccupancyGridColumn[] _columns
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _columns (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::SparseOccupancyGridColumn :init) r)) r))
     (dolist (elem- _columns)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::SparseOccupancyGrid :md5sum-) "497269ddab6058d0d4860f25dc49448f")
(setf (get jsk_recognition_msgs::SparseOccupancyGrid :datatype-) "jsk_recognition_msgs/SparseOccupancyGrid")
(setf (get jsk_recognition_msgs::SparseOccupancyGrid :definition-)
      "Header header
geometry_msgs/Pose origin_pose
float32 resolution

SparseOccupancyGridColumn[] columns

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
MSG: jsk_recognition_msgs/SparseOccupancyGridColumn
int32 column_index
SparseOccupancyGridCell[] cells

================================================================================
MSG: jsk_recognition_msgs/SparseOccupancyGridCell
int32 row_index
float32 value

")



(provide :jsk_recognition_msgs/SparseOccupancyGrid "497269ddab6058d0d4860f25dc49448f")


