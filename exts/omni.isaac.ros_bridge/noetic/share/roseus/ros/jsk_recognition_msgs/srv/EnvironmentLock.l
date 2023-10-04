;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::EnvironmentLock)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'EnvironmentLock (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::ENVIRONMENTLOCK")
  (make-package "JSK_RECOGNITION_MSGS::ENVIRONMENTLOCK"))
(unless (find-package "JSK_RECOGNITION_MSGS::ENVIRONMENTLOCKREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::ENVIRONMENTLOCKREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::ENVIRONMENTLOCKRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::ENVIRONMENTLOCKRESPONSE"))

(in-package "ROS")





(defclass jsk_recognition_msgs::EnvironmentLockRequest
  :super ros::object
  :slots ())

(defmethod jsk_recognition_msgs::EnvironmentLockRequest
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

(defclass jsk_recognition_msgs::EnvironmentLockResponse
  :super ros::object
  :slots (_environment_id ))

(defmethod jsk_recognition_msgs::EnvironmentLockResponse
  (:init
   (&key
    ((:environment_id __environment_id) 0)
    )
   (send-super :init)
   (setq _environment_id (round __environment_id))
   self)
  (:environment_id
   (&optional __environment_id)
   (if __environment_id (setq _environment_id __environment_id)) _environment_id)
  (:serialization-length
   ()
   (+
    ;; uint32 _environment_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _environment_id
       (write-long _environment_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _environment_id
     (setq _environment_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass jsk_recognition_msgs::EnvironmentLock
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::EnvironmentLock :md5sum-) "109afc0d3bd22aa461d45c8ef5ab6d75")
(setf (get jsk_recognition_msgs::EnvironmentLock :datatype-) "jsk_recognition_msgs/EnvironmentLock")
(setf (get jsk_recognition_msgs::EnvironmentLock :request) jsk_recognition_msgs::EnvironmentLockRequest)
(setf (get jsk_recognition_msgs::EnvironmentLock :response) jsk_recognition_msgs::EnvironmentLockResponse)

(defmethod jsk_recognition_msgs::EnvironmentLockRequest
  (:response () (instance jsk_recognition_msgs::EnvironmentLockResponse :init)))

(setf (get jsk_recognition_msgs::EnvironmentLockRequest :md5sum-) "109afc0d3bd22aa461d45c8ef5ab6d75")
(setf (get jsk_recognition_msgs::EnvironmentLockRequest :datatype-) "jsk_recognition_msgs/EnvironmentLockRequest")
(setf (get jsk_recognition_msgs::EnvironmentLockRequest :definition-)
      "---
uint32 environment_id

")

(setf (get jsk_recognition_msgs::EnvironmentLockResponse :md5sum-) "109afc0d3bd22aa461d45c8ef5ab6d75")
(setf (get jsk_recognition_msgs::EnvironmentLockResponse :datatype-) "jsk_recognition_msgs/EnvironmentLockResponse")
(setf (get jsk_recognition_msgs::EnvironmentLockResponse :definition-)
      "---
uint32 environment_id

")



(provide :jsk_recognition_msgs/EnvironmentLock "109afc0d3bd22aa461d45c8ef5ab6d75")


