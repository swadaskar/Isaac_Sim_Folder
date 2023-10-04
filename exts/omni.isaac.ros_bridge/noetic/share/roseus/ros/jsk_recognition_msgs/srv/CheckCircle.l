;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::CheckCircle)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'CheckCircle (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::CHECKCIRCLE")
  (make-package "JSK_RECOGNITION_MSGS::CHECKCIRCLE"))
(unless (find-package "JSK_RECOGNITION_MSGS::CHECKCIRCLEREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::CHECKCIRCLEREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::CHECKCIRCLERESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::CHECKCIRCLERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass jsk_recognition_msgs::CheckCircleRequest
  :super ros::object
  :slots (_point ))

(defmethod jsk_recognition_msgs::CheckCircleRequest
  (:init
   (&key
    ((:point __point) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _point __point)
   self)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::CheckCircleResponse
  :super ros::object
  :slots (_clicked _index _msg ))

(defmethod jsk_recognition_msgs::CheckCircleResponse
  (:init
   (&key
    ((:clicked __clicked) nil)
    ((:index __index) 0)
    ((:msg __msg) "")
    )
   (send-super :init)
   (setq _clicked __clicked)
   (setq _index (round __index))
   (setq _msg (string __msg))
   self)
  (:clicked
   (&optional (__clicked :null))
   (if (not (eq __clicked :null)) (setq _clicked __clicked)) _clicked)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:msg
   (&optional __msg)
   (if __msg (setq _msg __msg)) _msg)
  (:serialization-length
   ()
   (+
    ;; bool _clicked
    1
    ;; int32 _index
    4
    ;; string _msg
    4 (length _msg)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _clicked
       (if _clicked (write-byte -1 s) (write-byte 0 s))
     ;; int32 _index
       (write-long _index s)
     ;; string _msg
       (write-long (length _msg) s) (princ _msg s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _clicked
     (setq _clicked (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _index
     (setq _index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::CheckCircle
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::CheckCircle :md5sum-) "36549e2f705ec32b8c873cf22d68b357")
(setf (get jsk_recognition_msgs::CheckCircle :datatype-) "jsk_recognition_msgs/CheckCircle")
(setf (get jsk_recognition_msgs::CheckCircle :request) jsk_recognition_msgs::CheckCircleRequest)
(setf (get jsk_recognition_msgs::CheckCircle :response) jsk_recognition_msgs::CheckCircleResponse)

(defmethod jsk_recognition_msgs::CheckCircleRequest
  (:response () (instance jsk_recognition_msgs::CheckCircleResponse :init)))

(setf (get jsk_recognition_msgs::CheckCircleRequest :md5sum-) "36549e2f705ec32b8c873cf22d68b357")
(setf (get jsk_recognition_msgs::CheckCircleRequest :datatype-) "jsk_recognition_msgs/CheckCircleRequest")
(setf (get jsk_recognition_msgs::CheckCircleRequest :definition-)
      "geometry_msgs/Point point

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool clicked
int32 index
string msg

")

(setf (get jsk_recognition_msgs::CheckCircleResponse :md5sum-) "36549e2f705ec32b8c873cf22d68b357")
(setf (get jsk_recognition_msgs::CheckCircleResponse :datatype-) "jsk_recognition_msgs/CheckCircleResponse")
(setf (get jsk_recognition_msgs::CheckCircleResponse :definition-)
      "geometry_msgs/Point point

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool clicked
int32 index
string msg

")



(provide :jsk_recognition_msgs/CheckCircle "36549e2f705ec32b8c873cf22d68b357")


