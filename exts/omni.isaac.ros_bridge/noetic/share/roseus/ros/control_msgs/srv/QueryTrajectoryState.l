;; Auto-generated. Do not edit!


(when (boundp 'control_msgs::QueryTrajectoryState)
  (if (not (find-package "CONTROL_MSGS"))
    (make-package "CONTROL_MSGS"))
  (shadow 'QueryTrajectoryState (find-package "CONTROL_MSGS")))
(unless (find-package "CONTROL_MSGS::QUERYTRAJECTORYSTATE")
  (make-package "CONTROL_MSGS::QUERYTRAJECTORYSTATE"))
(unless (find-package "CONTROL_MSGS::QUERYTRAJECTORYSTATEREQUEST")
  (make-package "CONTROL_MSGS::QUERYTRAJECTORYSTATEREQUEST"))
(unless (find-package "CONTROL_MSGS::QUERYTRAJECTORYSTATERESPONSE")
  (make-package "CONTROL_MSGS::QUERYTRAJECTORYSTATERESPONSE"))

(in-package "ROS")





(defclass control_msgs::QueryTrajectoryStateRequest
  :super ros::object
  :slots (_time ))

(defmethod control_msgs::QueryTrajectoryStateRequest
  (:init
   (&key
    ((:time __time) (instance ros::time :init))
    )
   (send-super :init)
   (setq _time __time)
   self)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; time _time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _time
       (write-long (send _time :sec) s) (write-long (send _time :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _time
     (send _time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass control_msgs::QueryTrajectoryStateResponse
  :super ros::object
  :slots (_name _position _velocity _acceleration ))

(defmethod control_msgs::QueryTrajectoryStateResponse
  (:init
   (&key
    ((:name __name) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:position __position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocity __velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:acceleration __acceleration) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _name __name)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _acceleration __acceleration)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:acceleration
   (&optional __acceleration)
   (if __acceleration (setq _acceleration __acceleration)) _acceleration)
  (:serialization-length
   ()
   (+
    ;; string[] _name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _name)) 4
    ;; float64[] _position
    (* 8    (length _position)) 4
    ;; float64[] _velocity
    (* 8    (length _velocity)) 4
    ;; float64[] _acceleration
    (* 8    (length _acceleration)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _name
     (write-long (length _name) s)
     (dolist (elem _name)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _position
     (write-long (length _position) s)
     (dotimes (i (length _position))
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _velocity
     (write-long (length _velocity) s)
     (dotimes (i (length _velocity))
       (sys::poke (elt _velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _acceleration
     (write-long (length _acceleration) s)
     (dotimes (i (length _acceleration))
       (sys::poke (elt _acceleration i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _name
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _name (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _name i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _acceleration
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _acceleration (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _acceleration i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass control_msgs::QueryTrajectoryState
  :super ros::object
  :slots ())

(setf (get control_msgs::QueryTrajectoryState :md5sum-) "ec93cdecbd8062d761aa52b7c90cd44b")
(setf (get control_msgs::QueryTrajectoryState :datatype-) "control_msgs/QueryTrajectoryState")
(setf (get control_msgs::QueryTrajectoryState :request) control_msgs::QueryTrajectoryStateRequest)
(setf (get control_msgs::QueryTrajectoryState :response) control_msgs::QueryTrajectoryStateResponse)

(defmethod control_msgs::QueryTrajectoryStateRequest
  (:response () (instance control_msgs::QueryTrajectoryStateResponse :init)))

(setf (get control_msgs::QueryTrajectoryStateRequest :md5sum-) "ec93cdecbd8062d761aa52b7c90cd44b")
(setf (get control_msgs::QueryTrajectoryStateRequest :datatype-) "control_msgs/QueryTrajectoryStateRequest")
(setf (get control_msgs::QueryTrajectoryStateRequest :definition-)
      "time time
---
string[] name
float64[] position
float64[] velocity
float64[] acceleration

")

(setf (get control_msgs::QueryTrajectoryStateResponse :md5sum-) "ec93cdecbd8062d761aa52b7c90cd44b")
(setf (get control_msgs::QueryTrajectoryStateResponse :datatype-) "control_msgs/QueryTrajectoryStateResponse")
(setf (get control_msgs::QueryTrajectoryStateResponse :definition-)
      "time time
---
string[] name
float64[] position
float64[] velocity
float64[] acceleration

")



(provide :control_msgs/QueryTrajectoryState "ec93cdecbd8062d761aa52b7c90cd44b")


