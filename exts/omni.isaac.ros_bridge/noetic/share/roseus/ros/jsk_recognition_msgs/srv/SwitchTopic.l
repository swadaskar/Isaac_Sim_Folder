;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SwitchTopic)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SwitchTopic (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SWITCHTOPIC")
  (make-package "JSK_RECOGNITION_MSGS::SWITCHTOPIC"))
(unless (find-package "JSK_RECOGNITION_MSGS::SWITCHTOPICREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::SWITCHTOPICREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::SWITCHTOPICRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::SWITCHTOPICRESPONSE"))

(in-package "ROS")





(defclass jsk_recognition_msgs::SwitchTopicRequest
  :super ros::object
  :slots (_camera_info _points ))

(defmethod jsk_recognition_msgs::SwitchTopicRequest
  (:init
   (&key
    ((:camera_info __camera_info) "")
    ((:points __points) "")
    )
   (send-super :init)
   (setq _camera_info (string __camera_info))
   (setq _points (string __points))
   self)
  (:camera_info
   (&optional __camera_info)
   (if __camera_info (setq _camera_info __camera_info)) _camera_info)
  (:points
   (&optional __points)
   (if __points (setq _points __points)) _points)
  (:serialization-length
   ()
   (+
    ;; string _camera_info
    4 (length _camera_info)
    ;; string _points
    4 (length _points)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _camera_info
       (write-long (length _camera_info) s) (princ _camera_info s)
     ;; string _points
       (write-long (length _points) s) (princ _points s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _camera_info
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _camera_info (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _points
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _points (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::SwitchTopicResponse
  :super ros::object
  :slots ())

(defmethod jsk_recognition_msgs::SwitchTopicResponse
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

(defclass jsk_recognition_msgs::SwitchTopic
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::SwitchTopic :md5sum-) "e4a276b5a9b7b8fd97441d0fd991bdb9")
(setf (get jsk_recognition_msgs::SwitchTopic :datatype-) "jsk_recognition_msgs/SwitchTopic")
(setf (get jsk_recognition_msgs::SwitchTopic :request) jsk_recognition_msgs::SwitchTopicRequest)
(setf (get jsk_recognition_msgs::SwitchTopic :response) jsk_recognition_msgs::SwitchTopicResponse)

(defmethod jsk_recognition_msgs::SwitchTopicRequest
  (:response () (instance jsk_recognition_msgs::SwitchTopicResponse :init)))

(setf (get jsk_recognition_msgs::SwitchTopicRequest :md5sum-) "e4a276b5a9b7b8fd97441d0fd991bdb9")
(setf (get jsk_recognition_msgs::SwitchTopicRequest :datatype-) "jsk_recognition_msgs/SwitchTopicRequest")
(setf (get jsk_recognition_msgs::SwitchTopicRequest :definition-)
      "# new topics
string camera_info
string points
---

")

(setf (get jsk_recognition_msgs::SwitchTopicResponse :md5sum-) "e4a276b5a9b7b8fd97441d0fd991bdb9")
(setf (get jsk_recognition_msgs::SwitchTopicResponse :datatype-) "jsk_recognition_msgs/SwitchTopicResponse")
(setf (get jsk_recognition_msgs::SwitchTopicResponse :definition-)
      "# new topics
string camera_info
string points
---

")



(provide :jsk_recognition_msgs/SwitchTopic "e4a276b5a9b7b8fd97441d0fd991bdb9")


