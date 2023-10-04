;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::Segment)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'Segment (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SEGMENT")
  (make-package "JSK_RECOGNITION_MSGS::SEGMENT"))

(in-package "ROS")
;;//! \htmlinclude Segment.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass jsk_recognition_msgs::Segment
  :super ros::object
  :slots (_start_point _end_point ))

(defmethod jsk_recognition_msgs::Segment
  (:init
   (&key
    ((:start_point __start_point) (instance geometry_msgs::Point :init))
    ((:end_point __end_point) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _start_point __start_point)
   (setq _end_point __end_point)
   self)
  (:start_point
   (&rest __start_point)
   (if (keywordp (car __start_point))
       (send* _start_point __start_point)
     (progn
       (if __start_point (setq _start_point (car __start_point)))
       _start_point)))
  (:end_point
   (&rest __end_point)
   (if (keywordp (car __end_point))
       (send* _end_point __end_point)
     (progn
       (if __end_point (setq _end_point (car __end_point)))
       _end_point)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _start_point
    (send _start_point :serialization-length)
    ;; geometry_msgs/Point _end_point
    (send _end_point :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _start_point
       (send _start_point :serialize s)
     ;; geometry_msgs/Point _end_point
       (send _end_point :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _start_point
     (send _start_point :deserialize buf ptr-) (incf ptr- (send _start_point :serialization-length))
   ;; geometry_msgs/Point _end_point
     (send _end_point :deserialize buf ptr-) (incf ptr- (send _end_point :serialization-length))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::Segment :md5sum-) "0125c553546d7123dccaeab992a9e29e")
(setf (get jsk_recognition_msgs::Segment :datatype-) "jsk_recognition_msgs/Segment")
(setf (get jsk_recognition_msgs::Segment :definition-)
      "geometry_msgs/Point start_point
geometry_msgs/Point end_point

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :jsk_recognition_msgs/Segment "0125c553546d7123dccaeab992a9e29e")


