;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::ParallelEdgeArray)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'ParallelEdgeArray (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::PARALLELEDGEARRAY")
  (make-package "JSK_RECOGNITION_MSGS::PARALLELEDGEARRAY"))

(in-package "ROS")
;;//! \htmlinclude ParallelEdgeArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::ParallelEdgeArray
  :super ros::object
  :slots (_header _edge_groups ))

(defmethod jsk_recognition_msgs::ParallelEdgeArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:edge_groups __edge_groups) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _edge_groups __edge_groups)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:edge_groups
   (&rest __edge_groups)
   (if (keywordp (car __edge_groups))
       (send* _edge_groups __edge_groups)
     (progn
       (if __edge_groups (setq _edge_groups (car __edge_groups)))
       _edge_groups)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; jsk_recognition_msgs/ParallelEdge[] _edge_groups
    (apply #'+ (send-all _edge_groups :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jsk_recognition_msgs/ParallelEdge[] _edge_groups
     (write-long (length _edge_groups) s)
     (dolist (elem _edge_groups)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; jsk_recognition_msgs/ParallelEdge[] _edge_groups
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _edge_groups (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::ParallelEdge :init) r)) r))
     (dolist (elem- _edge_groups)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::ParallelEdgeArray :md5sum-) "7c8ef4f5976c55fb32293ceaa19a1189")
(setf (get jsk_recognition_msgs::ParallelEdgeArray :datatype-) "jsk_recognition_msgs/ParallelEdgeArray")
(setf (get jsk_recognition_msgs::ParallelEdgeArray :definition-)
      "Header header
ParallelEdge[] edge_groups

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
MSG: jsk_recognition_msgs/ParallelEdge
Header header
pcl_msgs/PointIndices[] cluster_indices
pcl_msgs/ModelCoefficients[] coefficients

================================================================================
MSG: pcl_msgs/PointIndices
Header header
int32[] indices


================================================================================
MSG: pcl_msgs/ModelCoefficients
Header header
float32[] values


")



(provide :jsk_recognition_msgs/ParallelEdgeArray "7c8ef4f5976c55fb32293ceaa19a1189")


