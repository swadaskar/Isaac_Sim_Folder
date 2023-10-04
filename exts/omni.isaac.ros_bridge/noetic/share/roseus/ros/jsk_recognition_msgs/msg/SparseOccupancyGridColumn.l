;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SparseOccupancyGridColumn)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SparseOccupancyGridColumn (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SPARSEOCCUPANCYGRIDCOLUMN")
  (make-package "JSK_RECOGNITION_MSGS::SPARSEOCCUPANCYGRIDCOLUMN"))

(in-package "ROS")
;;//! \htmlinclude SparseOccupancyGridColumn.msg.html


(defclass jsk_recognition_msgs::SparseOccupancyGridColumn
  :super ros::object
  :slots (_column_index _cells ))

(defmethod jsk_recognition_msgs::SparseOccupancyGridColumn
  (:init
   (&key
    ((:column_index __column_index) 0)
    ((:cells __cells) ())
    )
   (send-super :init)
   (setq _column_index (round __column_index))
   (setq _cells __cells)
   self)
  (:column_index
   (&optional __column_index)
   (if __column_index (setq _column_index __column_index)) _column_index)
  (:cells
   (&rest __cells)
   (if (keywordp (car __cells))
       (send* _cells __cells)
     (progn
       (if __cells (setq _cells (car __cells)))
       _cells)))
  (:serialization-length
   ()
   (+
    ;; int32 _column_index
    4
    ;; jsk_recognition_msgs/SparseOccupancyGridCell[] _cells
    (apply #'+ (send-all _cells :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _column_index
       (write-long _column_index s)
     ;; jsk_recognition_msgs/SparseOccupancyGridCell[] _cells
     (write-long (length _cells) s)
     (dolist (elem _cells)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _column_index
     (setq _column_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; jsk_recognition_msgs/SparseOccupancyGridCell[] _cells
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cells (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::SparseOccupancyGridCell :init) r)) r))
     (dolist (elem- _cells)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::SparseOccupancyGridColumn :md5sum-) "55074b193e722d5ead092ffe27f06522")
(setf (get jsk_recognition_msgs::SparseOccupancyGridColumn :datatype-) "jsk_recognition_msgs/SparseOccupancyGridColumn")
(setf (get jsk_recognition_msgs::SparseOccupancyGridColumn :definition-)
      "int32 column_index
SparseOccupancyGridCell[] cells

================================================================================
MSG: jsk_recognition_msgs/SparseOccupancyGridCell
int32 row_index
float32 value

")



(provide :jsk_recognition_msgs/SparseOccupancyGridColumn "55074b193e722d5ead092ffe27f06522")


