;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SparseOccupancyGridCell)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SparseOccupancyGridCell (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SPARSEOCCUPANCYGRIDCELL")
  (make-package "JSK_RECOGNITION_MSGS::SPARSEOCCUPANCYGRIDCELL"))

(in-package "ROS")
;;//! \htmlinclude SparseOccupancyGridCell.msg.html


(defclass jsk_recognition_msgs::SparseOccupancyGridCell
  :super ros::object
  :slots (_row_index _value ))

(defmethod jsk_recognition_msgs::SparseOccupancyGridCell
  (:init
   (&key
    ((:row_index __row_index) 0)
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _row_index (round __row_index))
   (setq _value (float __value))
   self)
  (:row_index
   (&optional __row_index)
   (if __row_index (setq _row_index __row_index)) _row_index)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; int32 _row_index
    4
    ;; float32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _row_index
       (write-long _row_index s)
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _row_index
     (setq _row_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::SparseOccupancyGridCell :md5sum-) "a5179e922852f82ee6322db1f097ab64")
(setf (get jsk_recognition_msgs::SparseOccupancyGridCell :datatype-) "jsk_recognition_msgs/SparseOccupancyGridCell")
(setf (get jsk_recognition_msgs::SparseOccupancyGridCell :definition-)
      "int32 row_index
float32 value

")



(provide :jsk_recognition_msgs/SparseOccupancyGridCell "a5179e922852f82ee6322db1f097ab64")


