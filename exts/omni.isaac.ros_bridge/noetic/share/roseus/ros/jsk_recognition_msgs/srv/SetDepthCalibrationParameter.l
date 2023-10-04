;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SetDepthCalibrationParameter)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SetDepthCalibrationParameter (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SETDEPTHCALIBRATIONPARAMETER")
  (make-package "JSK_RECOGNITION_MSGS::SETDEPTHCALIBRATIONPARAMETER"))
(unless (find-package "JSK_RECOGNITION_MSGS::SETDEPTHCALIBRATIONPARAMETERREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::SETDEPTHCALIBRATIONPARAMETERREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::SETDEPTHCALIBRATIONPARAMETERRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::SETDEPTHCALIBRATIONPARAMETERRESPONSE"))

(in-package "ROS")





(defclass jsk_recognition_msgs::SetDepthCalibrationParameterRequest
  :super ros::object
  :slots (_parameter ))

(defmethod jsk_recognition_msgs::SetDepthCalibrationParameterRequest
  (:init
   (&key
    ((:parameter __parameter) (instance jsk_recognition_msgs::DepthCalibrationParameter :init))
    )
   (send-super :init)
   (setq _parameter __parameter)
   self)
  (:parameter
   (&rest __parameter)
   (if (keywordp (car __parameter))
       (send* _parameter __parameter)
     (progn
       (if __parameter (setq _parameter (car __parameter)))
       _parameter)))
  (:serialization-length
   ()
   (+
    ;; jsk_recognition_msgs/DepthCalibrationParameter _parameter
    (send _parameter :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; jsk_recognition_msgs/DepthCalibrationParameter _parameter
       (send _parameter :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; jsk_recognition_msgs/DepthCalibrationParameter _parameter
     (send _parameter :deserialize buf ptr-) (incf ptr- (send _parameter :serialization-length))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::SetDepthCalibrationParameterResponse
  :super ros::object
  :slots ())

(defmethod jsk_recognition_msgs::SetDepthCalibrationParameterResponse
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

(defclass jsk_recognition_msgs::SetDepthCalibrationParameter
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::SetDepthCalibrationParameter :md5sum-) "46b1552cd9512adcf9bbec97800b0e0d")
(setf (get jsk_recognition_msgs::SetDepthCalibrationParameter :datatype-) "jsk_recognition_msgs/SetDepthCalibrationParameter")
(setf (get jsk_recognition_msgs::SetDepthCalibrationParameter :request) jsk_recognition_msgs::SetDepthCalibrationParameterRequest)
(setf (get jsk_recognition_msgs::SetDepthCalibrationParameter :response) jsk_recognition_msgs::SetDepthCalibrationParameterResponse)

(defmethod jsk_recognition_msgs::SetDepthCalibrationParameterRequest
  (:response () (instance jsk_recognition_msgs::SetDepthCalibrationParameterResponse :init)))

(setf (get jsk_recognition_msgs::SetDepthCalibrationParameterRequest :md5sum-) "46b1552cd9512adcf9bbec97800b0e0d")
(setf (get jsk_recognition_msgs::SetDepthCalibrationParameterRequest :datatype-) "jsk_recognition_msgs/SetDepthCalibrationParameterRequest")
(setf (get jsk_recognition_msgs::SetDepthCalibrationParameterRequest :definition-)
      "jsk_recognition_msgs/DepthCalibrationParameter parameter

================================================================================
MSG: jsk_recognition_msgs/DepthCalibrationParameter
# each vector stands for C(u, v)
# C(u, v) = a_0 * u^2 + a_1 * u + a_2 * v^2 + a_3 * v + a_4
float64[] coefficients2
float64[] coefficients1
float64[] coefficients0
bool use_abs
---

")

(setf (get jsk_recognition_msgs::SetDepthCalibrationParameterResponse :md5sum-) "46b1552cd9512adcf9bbec97800b0e0d")
(setf (get jsk_recognition_msgs::SetDepthCalibrationParameterResponse :datatype-) "jsk_recognition_msgs/SetDepthCalibrationParameterResponse")
(setf (get jsk_recognition_msgs::SetDepthCalibrationParameterResponse :definition-)
      "jsk_recognition_msgs/DepthCalibrationParameter parameter

================================================================================
MSG: jsk_recognition_msgs/DepthCalibrationParameter
# each vector stands for C(u, v)
# C(u, v) = a_0 * u^2 + a_1 * u + a_2 * v^2 + a_3 * v + a_4
float64[] coefficients2
float64[] coefficients1
float64[] coefficients0
bool use_abs
---

")



(provide :jsk_recognition_msgs/SetDepthCalibrationParameter "46b1552cd9512adcf9bbec97800b0e0d")


