;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::TowerPickUp)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'TowerPickUp (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::TOWERPICKUP")
  (make-package "JSK_RECOGNITION_MSGS::TOWERPICKUP"))
(unless (find-package "JSK_RECOGNITION_MSGS::TOWERPICKUPREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::TOWERPICKUPREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::TOWERPICKUPRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::TOWERPICKUPRESPONSE"))

(in-package "ROS")





(defclass jsk_recognition_msgs::TowerPickUpRequest
  :super ros::object
  :slots (_tower_index ))

(defmethod jsk_recognition_msgs::TowerPickUpRequest
  (:init
   (&key
    ((:tower_index __tower_index) 0)
    )
   (send-super :init)
   (setq _tower_index (round __tower_index))
   self)
  (:tower_index
   (&optional __tower_index)
   (if __tower_index (setq _tower_index __tower_index)) _tower_index)
  (:serialization-length
   ()
   (+
    ;; int32 _tower_index
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _tower_index
       (write-long _tower_index s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _tower_index
     (setq _tower_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass jsk_recognition_msgs::TowerPickUpResponse
  :super ros::object
  :slots ())

(defmethod jsk_recognition_msgs::TowerPickUpResponse
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

(defclass jsk_recognition_msgs::TowerPickUp
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::TowerPickUp :md5sum-) "e8bd24109f26b6d833bc4570d67d71c9")
(setf (get jsk_recognition_msgs::TowerPickUp :datatype-) "jsk_recognition_msgs/TowerPickUp")
(setf (get jsk_recognition_msgs::TowerPickUp :request) jsk_recognition_msgs::TowerPickUpRequest)
(setf (get jsk_recognition_msgs::TowerPickUp :response) jsk_recognition_msgs::TowerPickUpResponse)

(defmethod jsk_recognition_msgs::TowerPickUpRequest
  (:response () (instance jsk_recognition_msgs::TowerPickUpResponse :init)))

(setf (get jsk_recognition_msgs::TowerPickUpRequest :md5sum-) "e8bd24109f26b6d833bc4570d67d71c9")
(setf (get jsk_recognition_msgs::TowerPickUpRequest :datatype-) "jsk_recognition_msgs/TowerPickUpRequest")
(setf (get jsk_recognition_msgs::TowerPickUpRequest :definition-)
      "int32 tower_index
---

")

(setf (get jsk_recognition_msgs::TowerPickUpResponse :md5sum-) "e8bd24109f26b6d833bc4570d67d71c9")
(setf (get jsk_recognition_msgs::TowerPickUpResponse :datatype-) "jsk_recognition_msgs/TowerPickUpResponse")
(setf (get jsk_recognition_msgs::TowerPickUpResponse :definition-)
      "int32 tower_index
---

")



(provide :jsk_recognition_msgs/TowerPickUp "e8bd24109f26b6d833bc4570d67d71c9")


