;; Auto-generated. Do not edit!


(when (boundp 'dynamic_reconfigure::SensorLevels)
  (if (not (find-package "DYNAMIC_RECONFIGURE"))
    (make-package "DYNAMIC_RECONFIGURE"))
  (shadow 'SensorLevels (find-package "DYNAMIC_RECONFIGURE")))
(unless (find-package "DYNAMIC_RECONFIGURE::SENSORLEVELS")
  (make-package "DYNAMIC_RECONFIGURE::SENSORLEVELS"))

(in-package "ROS")
;;//! \htmlinclude SensorLevels.msg.html


(intern "*RECONFIGURE_CLOSE*" (find-package "DYNAMIC_RECONFIGURE::SENSORLEVELS"))
(shadow '*RECONFIGURE_CLOSE* (find-package "DYNAMIC_RECONFIGURE::SENSORLEVELS"))
(defconstant dynamic_reconfigure::SensorLevels::*RECONFIGURE_CLOSE* 3)
(intern "*RECONFIGURE_STOP*" (find-package "DYNAMIC_RECONFIGURE::SENSORLEVELS"))
(shadow '*RECONFIGURE_STOP* (find-package "DYNAMIC_RECONFIGURE::SENSORLEVELS"))
(defconstant dynamic_reconfigure::SensorLevels::*RECONFIGURE_STOP* 1)
(intern "*RECONFIGURE_RUNNING*" (find-package "DYNAMIC_RECONFIGURE::SENSORLEVELS"))
(shadow '*RECONFIGURE_RUNNING* (find-package "DYNAMIC_RECONFIGURE::SENSORLEVELS"))
(defconstant dynamic_reconfigure::SensorLevels::*RECONFIGURE_RUNNING* 0)

(defun dynamic_reconfigure::SensorLevels-to-symbol (const)
  (cond
        ((= const 3) 'dynamic_reconfigure::SensorLevels::*RECONFIGURE_CLOSE*)
        ((= const 1) 'dynamic_reconfigure::SensorLevels::*RECONFIGURE_STOP*)
        ((= const 0) 'dynamic_reconfigure::SensorLevels::*RECONFIGURE_RUNNING*)
        (t nil)))

(defclass dynamic_reconfigure::SensorLevels
  :super ros::object
  :slots ())

(defmethod dynamic_reconfigure::SensorLevels
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

(setf (get dynamic_reconfigure::SensorLevels :md5sum-) "6322637bee96d5489db6e2127c47602c")
(setf (get dynamic_reconfigure::SensorLevels :datatype-) "dynamic_reconfigure/SensorLevels")
(setf (get dynamic_reconfigure::SensorLevels :definition-)
      "# This message is deprecated, please use driver_base/SensorLevels instead.

byte RECONFIGURE_CLOSE = 3  # Parameters that need a sensor to be stopped completely when changed
byte RECONFIGURE_STOP = 1  # Parameters that need a sensor to stop streaming when changed
byte RECONFIGURE_RUNNING = 0 # Parameters that can be changed while a sensor is streaming

")



(provide :dynamic_reconfigure/SensorLevels "6322637bee96d5489db6e2127c47602c")


