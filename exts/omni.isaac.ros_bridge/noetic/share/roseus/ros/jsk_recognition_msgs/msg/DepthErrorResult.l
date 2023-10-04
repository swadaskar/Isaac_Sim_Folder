;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::DepthErrorResult)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'DepthErrorResult (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::DEPTHERRORRESULT")
  (make-package "JSK_RECOGNITION_MSGS::DEPTHERRORRESULT"))

(in-package "ROS")
;;//! \htmlinclude DepthErrorResult.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::DepthErrorResult
  :super ros::object
  :slots (_header _u _v _center_u _center_v _true_depth _observed_depth ))

(defmethod jsk_recognition_msgs::DepthErrorResult
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:u __u) 0)
    ((:v __v) 0)
    ((:center_u __center_u) 0.0)
    ((:center_v __center_v) 0.0)
    ((:true_depth __true_depth) 0.0)
    ((:observed_depth __observed_depth) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _u (round __u))
   (setq _v (round __v))
   (setq _center_u (float __center_u))
   (setq _center_v (float __center_v))
   (setq _true_depth (float __true_depth))
   (setq _observed_depth (float __observed_depth))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:u
   (&optional __u)
   (if __u (setq _u __u)) _u)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:center_u
   (&optional __center_u)
   (if __center_u (setq _center_u __center_u)) _center_u)
  (:center_v
   (&optional __center_v)
   (if __center_v (setq _center_v __center_v)) _center_v)
  (:true_depth
   (&optional __true_depth)
   (if __true_depth (setq _true_depth __true_depth)) _true_depth)
  (:observed_depth
   (&optional __observed_depth)
   (if __observed_depth (setq _observed_depth __observed_depth)) _observed_depth)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _u
    4
    ;; uint32 _v
    4
    ;; float32 _center_u
    4
    ;; float32 _center_v
    4
    ;; float32 _true_depth
    4
    ;; float32 _observed_depth
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _u
       (write-long _u s)
     ;; uint32 _v
       (write-long _v s)
     ;; float32 _center_u
       (sys::poke _center_u (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _center_v
       (sys::poke _center_v (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _true_depth
       (sys::poke _true_depth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _observed_depth
       (sys::poke _observed_depth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _u
     (setq _u (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _v
     (setq _v (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _center_u
     (setq _center_u (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _center_v
     (setq _center_v (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _true_depth
     (setq _true_depth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _observed_depth
     (setq _observed_depth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::DepthErrorResult :md5sum-) "cebbb2d000457fe5d971a29ee02ffb16")
(setf (get jsk_recognition_msgs::DepthErrorResult :datatype-) "jsk_recognition_msgs/DepthErrorResult")
(setf (get jsk_recognition_msgs::DepthErrorResult :definition-)
      "Header header
uint32 u
uint32 v
float32 center_u
float32 center_v
float32 true_depth
float32 observed_depth

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

")



(provide :jsk_recognition_msgs/DepthErrorResult "cebbb2d000457fe5d971a29ee02ffb16")


