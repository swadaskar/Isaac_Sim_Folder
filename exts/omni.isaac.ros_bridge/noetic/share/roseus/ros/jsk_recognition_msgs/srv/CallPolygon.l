;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::CallPolygon)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'CallPolygon (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::CALLPOLYGON")
  (make-package "JSK_RECOGNITION_MSGS::CALLPOLYGON"))
(unless (find-package "JSK_RECOGNITION_MSGS::CALLPOLYGONREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::CALLPOLYGONREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::CALLPOLYGONRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::CALLPOLYGONRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass jsk_recognition_msgs::CallPolygonRequest
  :super ros::object
  :slots (_filename ))

(defmethod jsk_recognition_msgs::CallPolygonRequest
  (:init
   (&key
    ((:filename __filename) "")
    )
   (send-super :init)
   (setq _filename (string __filename))
   self)
  (:filename
   (&optional __filename)
   (if __filename (setq _filename __filename)) _filename)
  (:serialization-length
   ()
   (+
    ;; string _filename
    4 (length _filename)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _filename
       (write-long (length _filename) s) (princ _filename s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _filename
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _filename (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::CallPolygonResponse
  :super ros::object
  :slots (_points ))

(defmethod jsk_recognition_msgs::CallPolygonResponse
  (:init
   (&key
    ((:points __points) (instance geometry_msgs::PolygonStamped :init))
    )
   (send-super :init)
   (setq _points __points)
   self)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PolygonStamped _points
    (send _points :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PolygonStamped _points
       (send _points :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PolygonStamped _points
     (send _points :deserialize buf ptr-) (incf ptr- (send _points :serialization-length))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::CallPolygon
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::CallPolygon :md5sum-) "b28bb04ea4e0c4a8f9c4d715ba76b7ac")
(setf (get jsk_recognition_msgs::CallPolygon :datatype-) "jsk_recognition_msgs/CallPolygon")
(setf (get jsk_recognition_msgs::CallPolygon :request) jsk_recognition_msgs::CallPolygonRequest)
(setf (get jsk_recognition_msgs::CallPolygon :response) jsk_recognition_msgs::CallPolygonResponse)

(defmethod jsk_recognition_msgs::CallPolygonRequest
  (:response () (instance jsk_recognition_msgs::CallPolygonResponse :init)))

(setf (get jsk_recognition_msgs::CallPolygonRequest :md5sum-) "b28bb04ea4e0c4a8f9c4d715ba76b7ac")
(setf (get jsk_recognition_msgs::CallPolygonRequest :datatype-) "jsk_recognition_msgs/CallPolygonRequest")
(setf (get jsk_recognition_msgs::CallPolygonRequest :definition-)
      "string filename
---
geometry_msgs/PolygonStamped points


================================================================================
MSG: geometry_msgs/PolygonStamped
# This represents a Polygon with reference coordinate frame and timestamp
Header header
Polygon polygon

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
")

(setf (get jsk_recognition_msgs::CallPolygonResponse :md5sum-) "b28bb04ea4e0c4a8f9c4d715ba76b7ac")
(setf (get jsk_recognition_msgs::CallPolygonResponse :datatype-) "jsk_recognition_msgs/CallPolygonResponse")
(setf (get jsk_recognition_msgs::CallPolygonResponse :definition-)
      "string filename
---
geometry_msgs/PolygonStamped points


================================================================================
MSG: geometry_msgs/PolygonStamped
# This represents a Polygon with reference coordinate frame and timestamp
Header header
Polygon polygon

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
")



(provide :jsk_recognition_msgs/CallPolygon "b28bb04ea4e0c4a8f9c4d715ba76b7ac")


