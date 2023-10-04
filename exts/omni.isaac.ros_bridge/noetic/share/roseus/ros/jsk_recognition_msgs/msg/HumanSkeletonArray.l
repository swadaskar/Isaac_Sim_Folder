;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::HumanSkeletonArray)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'HumanSkeletonArray (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::HUMANSKELETONARRAY")
  (make-package "JSK_RECOGNITION_MSGS::HUMANSKELETONARRAY"))

(in-package "ROS")
;;//! \htmlinclude HumanSkeletonArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::HumanSkeletonArray
  :super ros::object
  :slots (_header _human_ids _skeletons ))

(defmethod jsk_recognition_msgs::HumanSkeletonArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:human_ids __human_ids) ())
    ((:skeletons __skeletons) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _human_ids __human_ids)
   (setq _skeletons __skeletons)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:human_ids
   (&rest __human_ids)
   (if (keywordp (car __human_ids))
       (send* _human_ids __human_ids)
     (progn
       (if __human_ids (setq _human_ids (car __human_ids)))
       _human_ids)))
  (:skeletons
   (&rest __skeletons)
   (if (keywordp (car __skeletons))
       (send* _skeletons __skeletons)
     (progn
       (if __skeletons (setq _skeletons (car __skeletons)))
       _skeletons)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/Int32[] _human_ids
    (apply #'+ (send-all _human_ids :serialization-length)) 4
    ;; jsk_recognition_msgs/HumanSkeleton[] _skeletons
    (apply #'+ (send-all _skeletons :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/Int32[] _human_ids
     (write-long (length _human_ids) s)
     (dolist (elem _human_ids)
       (send elem :serialize s)
       )
     ;; jsk_recognition_msgs/HumanSkeleton[] _skeletons
     (write-long (length _skeletons) s)
     (dolist (elem _skeletons)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/Int32[] _human_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _human_ids (let (r) (dotimes (i n) (push (instance std_msgs::Int32 :init) r)) r))
     (dolist (elem- _human_ids)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; jsk_recognition_msgs/HumanSkeleton[] _skeletons
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _skeletons (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::HumanSkeleton :init) r)) r))
     (dolist (elem- _skeletons)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::HumanSkeletonArray :md5sum-) "4013ce44abe9aafa9055e095797a9d50")
(setf (get jsk_recognition_msgs::HumanSkeletonArray :datatype-) "jsk_recognition_msgs/HumanSkeletonArray")
(setf (get jsk_recognition_msgs::HumanSkeletonArray :definition-)
      "std_msgs/Header header
std_msgs/Int32[] human_ids
jsk_recognition_msgs/HumanSkeleton[] skeletons

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
MSG: std_msgs/Int32
int32 data
================================================================================
MSG: jsk_recognition_msgs/HumanSkeleton
std_msgs/Header header
string[] bone_names
jsk_recognition_msgs/Segment[] bones

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



(provide :jsk_recognition_msgs/HumanSkeletonArray "4013ce44abe9aafa9055e095797a9d50")


