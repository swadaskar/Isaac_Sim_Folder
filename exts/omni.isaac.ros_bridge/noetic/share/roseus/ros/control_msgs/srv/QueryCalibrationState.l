;; Auto-generated. Do not edit!


(when (boundp 'control_msgs::QueryCalibrationState)
  (if (not (find-package "CONTROL_MSGS"))
    (make-package "CONTROL_MSGS"))
  (shadow 'QueryCalibrationState (find-package "CONTROL_MSGS")))
(unless (find-package "CONTROL_MSGS::QUERYCALIBRATIONSTATE")
  (make-package "CONTROL_MSGS::QUERYCALIBRATIONSTATE"))
(unless (find-package "CONTROL_MSGS::QUERYCALIBRATIONSTATEREQUEST")
  (make-package "CONTROL_MSGS::QUERYCALIBRATIONSTATEREQUEST"))
(unless (find-package "CONTROL_MSGS::QUERYCALIBRATIONSTATERESPONSE")
  (make-package "CONTROL_MSGS::QUERYCALIBRATIONSTATERESPONSE"))

(in-package "ROS")





(defclass control_msgs::QueryCalibrationStateRequest
  :super ros::object
  :slots ())

(defmethod control_msgs::QueryCalibrationStateRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass control_msgs::QueryCalibrationStateResponse
  :super ros::object
  :slots (_is_calibrated ))

(defmethod control_msgs::QueryCalibrationStateResponse
  (:init
   (&key
    ((:is_calibrated __is_calibrated) nil)
    )
   (send-super :init)
   (setq _is_calibrated __is_calibrated)
   self)
  (:is_calibrated
   (&optional (__is_calibrated :null))
   (if (not (eq __is_calibrated :null)) (setq _is_calibrated __is_calibrated)) _is_calibrated)
  (:serialization-length
   ()
   (+
    ;; bool _is_calibrated
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_calibrated
       (if _is_calibrated (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_calibrated
     (setq _is_calibrated (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass control_msgs::QueryCalibrationState
  :super ros::object
  :slots ())

(setf (get control_msgs::QueryCalibrationState :md5sum-) "28af3beedcb84986b8e470dc5470507d")
(setf (get control_msgs::QueryCalibrationState :datatype-) "control_msgs/QueryCalibrationState")
(setf (get control_msgs::QueryCalibrationState :request) control_msgs::QueryCalibrationStateRequest)
(setf (get control_msgs::QueryCalibrationState :response) control_msgs::QueryCalibrationStateResponse)

(defmethod control_msgs::QueryCalibrationStateRequest
  (:response () (instance control_msgs::QueryCalibrationStateResponse :init)))

(setf (get control_msgs::QueryCalibrationStateRequest :md5sum-) "28af3beedcb84986b8e470dc5470507d")
(setf (get control_msgs::QueryCalibrationStateRequest :datatype-) "control_msgs/QueryCalibrationStateRequest")
(setf (get control_msgs::QueryCalibrationStateRequest :definition-)
      "---
bool is_calibrated
")

(setf (get control_msgs::QueryCalibrationStateResponse :md5sum-) "28af3beedcb84986b8e470dc5470507d")
(setf (get control_msgs::QueryCalibrationStateResponse :datatype-) "control_msgs/QueryCalibrationStateResponse")
(setf (get control_msgs::QueryCalibrationStateResponse :definition-)
      "---
bool is_calibrated
")



(provide :control_msgs/QueryCalibrationState "28af3beedcb84986b8e470dc5470507d")


