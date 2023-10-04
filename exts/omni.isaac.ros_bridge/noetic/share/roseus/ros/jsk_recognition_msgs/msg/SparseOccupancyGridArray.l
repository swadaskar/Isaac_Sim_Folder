;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SparseOccupancyGridArray)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SparseOccupancyGridArray (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SPARSEOCCUPANCYGRIDARRAY")
  (make-package "JSK_RECOGNITION_MSGS::SPARSEOCCUPANCYGRIDARRAY"))

(in-package "ROS")
;;//! \htmlinclude SparseOccupancyGridArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::SparseOccupancyGridArray
  :super ros::object
  :slots (_header _grids ))

(defmethod jsk_recognition_msgs::SparseOccupancyGridArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:grids __grids) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _grids __grids)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:grids
   (&rest __grids)
   (if (keywordp (car __grids))
       (send* _grids __grids)
     (progn
       (if __grids (setq _grids (car __grids)))
       _grids)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; jsk_recognition_msgs/SparseOccupancyGrid[] _grids
    (apply #'+ (send-all _grids :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jsk_recognition_msgs/SparseOccupancyGrid[] _grids
     (write-long (length _grids) s)
     (dolist (elem _grids)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; jsk_recognition_msgs/SparseOccupancyGrid[] _grids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _grids (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::SparseOccupancyGrid :init) r)) r))
     (dolist (elem- _grids)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::SparseOccupancyGridArray :md5sum-) "fa9a1f59b783128c759e159dd0c46731")
(setf (get jsk_recognition_msgs::SparseOccupancyGridArray :datatype-) "jsk_recognition_msgs/SparseOccupancyGridArray")
(setf (get jsk_recognition_msgs::SparseOccupancyGridArray :definition-)
      "Header header
SparseOccupancyGrid[] grids

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
MSG: jsk_recognition_msgs/SparseOccupancyGrid
Header header
geometry_msgs/Pose origin_pose
float32 resolution

SparseOccupancyGridColumn[] columns

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



(provide :jsk_recognition_msgs/SparseOccupancyGridArray "fa9a1f59b783128c759e159dd0c46731")


