;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::ClusterPointIndices)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'ClusterPointIndices (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::CLUSTERPOINTINDICES")
  (make-package "JSK_RECOGNITION_MSGS::CLUSTERPOINTINDICES"))

(in-package "ROS")
;;//! \htmlinclude ClusterPointIndices.msg.html
(if (not (find-package "PCL_MSGS"))
  (ros::roseus-add-msgs "pcl_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::ClusterPointIndices
  :super ros::object
  :slots (_header _cluster_indices ))

(defmethod jsk_recognition_msgs::ClusterPointIndices
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:cluster_indices __cluster_indices) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _cluster_indices __cluster_indices)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:cluster_indices
   (&rest __cluster_indices)
   (if (keywordp (car __cluster_indices))
       (send* _cluster_indices __cluster_indices)
     (progn
       (if __cluster_indices (setq _cluster_indices (car __cluster_indices)))
       _cluster_indices)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; pcl_msgs/PointIndices[] _cluster_indices
    (apply #'+ (send-all _cluster_indices :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; pcl_msgs/PointIndices[] _cluster_indices
     (write-long (length _cluster_indices) s)
     (dolist (elem _cluster_indices)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; pcl_msgs/PointIndices[] _cluster_indices
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cluster_indices (let (r) (dotimes (i n) (push (instance pcl_msgs::PointIndices :init) r)) r))
     (dolist (elem- _cluster_indices)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::ClusterPointIndices :md5sum-) "d43e94ea5e491effac7685a42b7b9d14")
(setf (get jsk_recognition_msgs::ClusterPointIndices :datatype-) "jsk_recognition_msgs/ClusterPointIndices")
(setf (get jsk_recognition_msgs::ClusterPointIndices :definition-)
      "# ClusterPointIndices is used to represent segmentation result.
# Simply put, ClusterPointIndices is a list of PointIndices.
Header header
pcl_msgs/PointIndices[] cluster_indices

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: pcl_msgs/PointIndices
Header header
int32[] indices


")



(provide :jsk_recognition_msgs/ClusterPointIndices "d43e94ea5e491effac7685a42b7b9d14")


