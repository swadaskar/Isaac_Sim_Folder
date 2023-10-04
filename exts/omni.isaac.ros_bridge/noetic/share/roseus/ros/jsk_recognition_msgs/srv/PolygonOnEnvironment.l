;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::PolygonOnEnvironment)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'PolygonOnEnvironment (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::POLYGONONENVIRONMENT")
  (make-package "JSK_RECOGNITION_MSGS::POLYGONONENVIRONMENT"))
(unless (find-package "JSK_RECOGNITION_MSGS::POLYGONONENVIRONMENTREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::POLYGONONENVIRONMENTREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::POLYGONONENVIRONMENTRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::POLYGONONENVIRONMENTRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass jsk_recognition_msgs::PolygonOnEnvironmentRequest
  :super ros::object
  :slots (_environment_id _plane_index _polygon ))

(defmethod jsk_recognition_msgs::PolygonOnEnvironmentRequest
  (:init
   (&key
    ((:environment_id __environment_id) 0)
    ((:plane_index __plane_index) 0)
    ((:polygon __polygon) (instance geometry_msgs::PolygonStamped :init))
    )
   (send-super :init)
   (setq _environment_id (round __environment_id))
   (setq _plane_index (round __plane_index))
   (setq _polygon __polygon)
   self)
  (:environment_id
   (&optional __environment_id)
   (if __environment_id (setq _environment_id __environment_id)) _environment_id)
  (:plane_index
   (&optional __plane_index)
   (if __plane_index (setq _plane_index __plane_index)) _plane_index)
  (:polygon
   (&rest __polygon)
   (if (keywordp (car __polygon))
       (send* _polygon __polygon)
     (progn
       (if __polygon (setq _polygon (car __polygon)))
       _polygon)))
  (:serialization-length
   ()
   (+
    ;; uint32 _environment_id
    4
    ;; uint32 _plane_index
    4
    ;; geometry_msgs/PolygonStamped _polygon
    (send _polygon :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _environment_id
       (write-long _environment_id s)
     ;; uint32 _plane_index
       (write-long _plane_index s)
     ;; geometry_msgs/PolygonStamped _polygon
       (send _polygon :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _environment_id
     (setq _environment_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _plane_index
     (setq _plane_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/PolygonStamped _polygon
     (send _polygon :deserialize buf ptr-) (incf ptr- (send _polygon :serialization-length))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::PolygonOnEnvironmentResponse
  :super ros::object
  :slots (_result _reason ))

(defmethod jsk_recognition_msgs::PolygonOnEnvironmentResponse
  (:init
   (&key
    ((:result __result) nil)
    ((:reason __reason) "")
    )
   (send-super :init)
   (setq _result __result)
   (setq _reason (string __reason))
   self)
  (:result
   (&optional (__result :null))
   (if (not (eq __result :null)) (setq _result __result)) _result)
  (:reason
   (&optional __reason)
   (if __reason (setq _reason __reason)) _reason)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ;; string _reason
    4 (length _reason)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;; string _reason
       (write-long (length _reason) s) (princ _reason s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::PolygonOnEnvironment
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::PolygonOnEnvironment :md5sum-) "a8ff091210c071091863d880820e505c")
(setf (get jsk_recognition_msgs::PolygonOnEnvironment :datatype-) "jsk_recognition_msgs/PolygonOnEnvironment")
(setf (get jsk_recognition_msgs::PolygonOnEnvironment :request) jsk_recognition_msgs::PolygonOnEnvironmentRequest)
(setf (get jsk_recognition_msgs::PolygonOnEnvironment :response) jsk_recognition_msgs::PolygonOnEnvironmentResponse)

(defmethod jsk_recognition_msgs::PolygonOnEnvironmentRequest
  (:response () (instance jsk_recognition_msgs::PolygonOnEnvironmentResponse :init)))

(setf (get jsk_recognition_msgs::PolygonOnEnvironmentRequest :md5sum-) "a8ff091210c071091863d880820e505c")
(setf (get jsk_recognition_msgs::PolygonOnEnvironmentRequest :datatype-) "jsk_recognition_msgs/PolygonOnEnvironmentRequest")
(setf (get jsk_recognition_msgs::PolygonOnEnvironmentRequest :definition-)
      "uint32 environment_id
uint32 plane_index
geometry_msgs/PolygonStamped polygon

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
---
bool result
string reason

")

(setf (get jsk_recognition_msgs::PolygonOnEnvironmentResponse :md5sum-) "a8ff091210c071091863d880820e505c")
(setf (get jsk_recognition_msgs::PolygonOnEnvironmentResponse :datatype-) "jsk_recognition_msgs/PolygonOnEnvironmentResponse")
(setf (get jsk_recognition_msgs::PolygonOnEnvironmentResponse :definition-)
      "uint32 environment_id
uint32 plane_index
geometry_msgs/PolygonStamped polygon

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
---
bool result
string reason

")



(provide :jsk_recognition_msgs/PolygonOnEnvironment "a8ff091210c071091863d880820e505c")


