;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::HeightmapConfig)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'HeightmapConfig (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::HEIGHTMAPCONFIG")
  (make-package "JSK_RECOGNITION_MSGS::HEIGHTMAPCONFIG"))

(in-package "ROS")
;;//! \htmlinclude HeightmapConfig.msg.html


(defclass jsk_recognition_msgs::HeightmapConfig
  :super ros::object
  :slots (_min_x _max_x _min_y _max_y ))

(defmethod jsk_recognition_msgs::HeightmapConfig
  (:init
   (&key
    ((:min_x __min_x) 0.0)
    ((:max_x __max_x) 0.0)
    ((:min_y __min_y) 0.0)
    ((:max_y __max_y) 0.0)
    )
   (send-super :init)
   (setq _min_x (float __min_x))
   (setq _max_x (float __max_x))
   (setq _min_y (float __min_y))
   (setq _max_y (float __max_y))
   self)
  (:min_x
   (&optional __min_x)
   (if __min_x (setq _min_x __min_x)) _min_x)
  (:max_x
   (&optional __max_x)
   (if __max_x (setq _max_x __max_x)) _max_x)
  (:min_y
   (&optional __min_y)
   (if __min_y (setq _min_y __min_y)) _min_y)
  (:max_y
   (&optional __max_y)
   (if __max_y (setq _max_y __max_y)) _max_y)
  (:serialization-length
   ()
   (+
    ;; float32 _min_x
    4
    ;; float32 _max_x
    4
    ;; float32 _min_y
    4
    ;; float32 _max_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _min_x
       (sys::poke _min_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_x
       (sys::poke _max_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _min_y
       (sys::poke _min_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_y
       (sys::poke _max_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _min_x
     (setq _min_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_x
     (setq _max_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _min_y
     (setq _min_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_y
     (setq _max_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::HeightmapConfig :md5sum-) "f2f90f6dd5aeedef48b062e1a4aabb89")
(setf (get jsk_recognition_msgs::HeightmapConfig :datatype-) "jsk_recognition_msgs/HeightmapConfig")
(setf (get jsk_recognition_msgs::HeightmapConfig :definition-)
      "float32 min_x
float32 max_x
float32 min_y
float32 max_y

")



(provide :jsk_recognition_msgs/HeightmapConfig "f2f90f6dd5aeedef48b062e1a4aabb89")


