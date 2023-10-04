;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SlicedPointCloud)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SlicedPointCloud (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SLICEDPOINTCLOUD")
  (make-package "JSK_RECOGNITION_MSGS::SLICEDPOINTCLOUD"))

(in-package "ROS")
;;//! \htmlinclude SlicedPointCloud.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass jsk_recognition_msgs::SlicedPointCloud
  :super ros::object
  :slots (_point_cloud _slice_index _sequence_id ))

(defmethod jsk_recognition_msgs::SlicedPointCloud
  (:init
   (&key
    ((:point_cloud __point_cloud) (instance sensor_msgs::PointCloud2 :init))
    ((:slice_index __slice_index) 0)
    ((:sequence_id __sequence_id) 0)
    )
   (send-super :init)
   (setq _point_cloud __point_cloud)
   (setq _slice_index (round __slice_index))
   (setq _sequence_id (round __sequence_id))
   self)
  (:point_cloud
   (&rest __point_cloud)
   (if (keywordp (car __point_cloud))
       (send* _point_cloud __point_cloud)
     (progn
       (if __point_cloud (setq _point_cloud (car __point_cloud)))
       _point_cloud)))
  (:slice_index
   (&optional __slice_index)
   (if __slice_index (setq _slice_index __slice_index)) _slice_index)
  (:sequence_id
   (&optional __sequence_id)
   (if __sequence_id (setq _sequence_id __sequence_id)) _sequence_id)
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _point_cloud
    (send _point_cloud :serialization-length)
    ;; uint8 _slice_index
    1
    ;; uint8 _sequence_id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _point_cloud
       (send _point_cloud :serialize s)
     ;; uint8 _slice_index
       (write-byte _slice_index s)
     ;; uint8 _sequence_id
       (write-byte _sequence_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _point_cloud
     (send _point_cloud :deserialize buf ptr-) (incf ptr- (send _point_cloud :serialization-length))
   ;; uint8 _slice_index
     (setq _slice_index (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sequence_id
     (setq _sequence_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::SlicedPointCloud :md5sum-) "37376a48b034bf543f8cfd86844487c7")
(setf (get jsk_recognition_msgs::SlicedPointCloud :datatype-) "jsk_recognition_msgs/SlicedPointCloud")
(setf (get jsk_recognition_msgs::SlicedPointCloud :definition-)
      "sensor_msgs/PointCloud2 point_cloud
uint8 slice_index
uint8 sequence_id

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

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
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

")



(provide :jsk_recognition_msgs/SlicedPointCloud "37376a48b034bf543f8cfd86844487c7")


