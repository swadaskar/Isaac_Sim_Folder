;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SimpleOccupancyGrid)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SimpleOccupancyGrid (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SIMPLEOCCUPANCYGRID")
  (make-package "JSK_RECOGNITION_MSGS::SIMPLEOCCUPANCYGRID"))

(in-package "ROS")
;;//! \htmlinclude SimpleOccupancyGrid.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::SimpleOccupancyGrid
  :super ros::object
  :slots (_header _coefficients _resolution _cells ))

(defmethod jsk_recognition_msgs::SimpleOccupancyGrid
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:coefficients __coefficients) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:resolution __resolution) 0.0)
    ((:cells __cells) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _coefficients __coefficients)
   (setq _resolution (float __resolution))
   (setq _cells __cells)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:coefficients
   (&optional __coefficients)
   (if __coefficients (setq _coefficients __coefficients)) _coefficients)
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:cells
   (&rest __cells)
   (if (keywordp (car __cells))
       (send* _cells __cells)
     (progn
       (if __cells (setq _cells (car __cells)))
       _cells)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[4] _coefficients
    (* 4    4)
    ;; float32 _resolution
    4
    ;; geometry_msgs/Point[] _cells
    (apply #'+ (send-all _cells :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[4] _coefficients
     (dotimes (i 4)
       (sys::poke (elt _coefficients i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _resolution
       (sys::poke _resolution (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Point[] _cells
     (write-long (length _cells) s)
     (dolist (elem _cells)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[4] _coefficients
   (dotimes (i (length _coefficients))
     (setf (elt _coefficients i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32 _resolution
     (setq _resolution (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Point[] _cells
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cells (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _cells)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::SimpleOccupancyGrid :md5sum-) "25fb4ce5a31aab052ba1250fcdda9da7")
(setf (get jsk_recognition_msgs::SimpleOccupancyGrid :datatype-) "jsk_recognition_msgs/SimpleOccupancyGrid")
(setf (get jsk_recognition_msgs::SimpleOccupancyGrid :definition-)
      "Header header
# plane coefficients
float32[4] coefficients
# cells
float32 resolution
geometry_msgs/Point[] cells

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

")



(provide :jsk_recognition_msgs/SimpleOccupancyGrid "25fb4ce5a31aab052ba1250fcdda9da7")


