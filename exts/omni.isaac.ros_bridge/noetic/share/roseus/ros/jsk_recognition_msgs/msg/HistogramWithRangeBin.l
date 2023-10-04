;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::HistogramWithRangeBin)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'HistogramWithRangeBin (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::HISTOGRAMWITHRANGEBIN")
  (make-package "JSK_RECOGNITION_MSGS::HISTOGRAMWITHRANGEBIN"))

(in-package "ROS")
;;//! \htmlinclude HistogramWithRangeBin.msg.html


(defclass jsk_recognition_msgs::HistogramWithRangeBin
  :super ros::object
  :slots (_min_value _max_value _count ))

(defmethod jsk_recognition_msgs::HistogramWithRangeBin
  (:init
   (&key
    ((:min_value __min_value) 0.0)
    ((:max_value __max_value) 0.0)
    ((:count __count) 0)
    )
   (send-super :init)
   (setq _min_value (float __min_value))
   (setq _max_value (float __max_value))
   (setq _count (round __count))
   self)
  (:min_value
   (&optional __min_value)
   (if __min_value (setq _min_value __min_value)) _min_value)
  (:max_value
   (&optional __max_value)
   (if __max_value (setq _max_value __max_value)) _max_value)
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:serialization-length
   ()
   (+
    ;; float64 _min_value
    8
    ;; float64 _max_value
    8
    ;; uint32 _count
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _min_value
       (sys::poke _min_value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max_value
       (sys::poke _max_value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint32 _count
       (write-long _count s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _min_value
     (setq _min_value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_value
     (setq _max_value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint32 _count
     (setq _count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::HistogramWithRangeBin :md5sum-) "a7fe6c3021fcba2c6357f3db21601551")
(setf (get jsk_recognition_msgs::HistogramWithRangeBin :datatype-) "jsk_recognition_msgs/HistogramWithRangeBin")
(setf (get jsk_recognition_msgs::HistogramWithRangeBin :definition-)
      "float64 min_value
float64 max_value
uint32 count

")



(provide :jsk_recognition_msgs/HistogramWithRangeBin "a7fe6c3021fcba2c6357f3db21601551")


