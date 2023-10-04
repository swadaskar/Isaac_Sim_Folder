;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::ModelCoefficientsArray)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'ModelCoefficientsArray (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::MODELCOEFFICIENTSARRAY")
  (make-package "JSK_RECOGNITION_MSGS::MODELCOEFFICIENTSARRAY"))

(in-package "ROS")
;;//! \htmlinclude ModelCoefficientsArray.msg.html
(if (not (find-package "PCL_MSGS"))
  (ros::roseus-add-msgs "pcl_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::ModelCoefficientsArray
  :super ros::object
  :slots (_header _coefficients ))

(defmethod jsk_recognition_msgs::ModelCoefficientsArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:coefficients __coefficients) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _coefficients __coefficients)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:coefficients
   (&rest __coefficients)
   (if (keywordp (car __coefficients))
       (send* _coefficients __coefficients)
     (progn
       (if __coefficients (setq _coefficients (car __coefficients)))
       _coefficients)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; pcl_msgs/ModelCoefficients[] _coefficients
    (apply #'+ (send-all _coefficients :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; pcl_msgs/ModelCoefficients[] _coefficients
     (write-long (length _coefficients) s)
     (dolist (elem _coefficients)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; pcl_msgs/ModelCoefficients[] _coefficients
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _coefficients (let (r) (dotimes (i n) (push (instance pcl_msgs::ModelCoefficients :init) r)) r))
     (dolist (elem- _coefficients)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::ModelCoefficientsArray :md5sum-) "059efee897c3f4ae027a493e30c4c26b")
(setf (get jsk_recognition_msgs::ModelCoefficientsArray :datatype-) "jsk_recognition_msgs/ModelCoefficientsArray")
(setf (get jsk_recognition_msgs::ModelCoefficientsArray :definition-)
      "# ModelCoefficientsArray is used to represent coefficients of model
# for each segmented clusters.
# Simply put, ModelCoefficientsArray is a list of ModelCoefficients.
Header header
pcl_msgs/ModelCoefficients[] coefficients

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
MSG: pcl_msgs/ModelCoefficients
Header header
float32[] values


")



(provide :jsk_recognition_msgs/ModelCoefficientsArray "059efee897c3f4ae027a493e30c4c26b")


