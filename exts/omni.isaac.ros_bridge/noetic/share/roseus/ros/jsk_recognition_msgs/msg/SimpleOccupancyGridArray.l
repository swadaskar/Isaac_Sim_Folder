;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SimpleOccupancyGridArray)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SimpleOccupancyGridArray (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SIMPLEOCCUPANCYGRIDARRAY")
  (make-package "JSK_RECOGNITION_MSGS::SIMPLEOCCUPANCYGRIDARRAY"))

(in-package "ROS")
;;//! \htmlinclude SimpleOccupancyGridArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::SimpleOccupancyGridArray
  :super ros::object
  :slots (_header _grids ))

(defmethod jsk_recognition_msgs::SimpleOccupancyGridArray
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
    ;; jsk_recognition_msgs/SimpleOccupancyGrid[] _grids
    (apply #'+ (send-all _grids :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jsk_recognition_msgs/SimpleOccupancyGrid[] _grids
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
   ;; jsk_recognition_msgs/SimpleOccupancyGrid[] _grids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _grids (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::SimpleOccupancyGrid :init) r)) r))
     (dolist (elem- _grids)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::SimpleOccupancyGridArray :md5sum-) "e448b458270a6ec465d66169c4180932")
(setf (get jsk_recognition_msgs::SimpleOccupancyGridArray :datatype-) "jsk_recognition_msgs/SimpleOccupancyGridArray")
(setf (get jsk_recognition_msgs::SimpleOccupancyGridArray :definition-)
      "Header header
SimpleOccupancyGrid[] grids

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
MSG: jsk_recognition_msgs/SimpleOccupancyGrid
Header header
# plane coefficients
float32[4] coefficients
# cells
float32 resolution
geometry_msgs/Point[] cells

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :jsk_recognition_msgs/SimpleOccupancyGridArray "e448b458270a6ec465d66169c4180932")


