;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::HumanSkeleton)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'HumanSkeleton (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::HUMANSKELETON")
  (make-package "JSK_RECOGNITION_MSGS::HUMANSKELETON"))

(in-package "ROS")
;;//! \htmlinclude HumanSkeleton.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::HumanSkeleton
  :super ros::object
  :slots (_header _bone_names _bones ))

(defmethod jsk_recognition_msgs::HumanSkeleton
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:bone_names __bone_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:bones __bones) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _bone_names __bone_names)
   (setq _bones __bones)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:bone_names
   (&optional __bone_names)
   (if __bone_names (setq _bone_names __bone_names)) _bone_names)
  (:bones
   (&rest __bones)
   (if (keywordp (car __bones))
       (send* _bones __bones)
     (progn
       (if __bones (setq _bones (car __bones)))
       _bones)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _bone_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _bone_names)) 4
    ;; jsk_recognition_msgs/Segment[] _bones
    (apply #'+ (send-all _bones :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _bone_names
     (write-long (length _bone_names) s)
     (dolist (elem _bone_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; jsk_recognition_msgs/Segment[] _bones
     (write-long (length _bones) s)
     (dolist (elem _bones)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string[] _bone_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bone_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _bone_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; jsk_recognition_msgs/Segment[] _bones
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bones (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::Segment :init) r)) r))
     (dolist (elem- _bones)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::HumanSkeleton :md5sum-) "b8098e195ee1734e91bc2e38c969a23f")
(setf (get jsk_recognition_msgs::HumanSkeleton :datatype-) "jsk_recognition_msgs/HumanSkeleton")
(setf (get jsk_recognition_msgs::HumanSkeleton :definition-)
      "std_msgs/Header header
string[] bone_names
jsk_recognition_msgs/Segment[] bones

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
MSG: jsk_recognition_msgs/Segment
geometry_msgs/Point start_point
geometry_msgs/Point end_point

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :jsk_recognition_msgs/HumanSkeleton "b8098e195ee1734e91bc2e38c969a23f")


