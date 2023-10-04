;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::PeoplePose)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'PeoplePose (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::PEOPLEPOSE")
  (make-package "JSK_RECOGNITION_MSGS::PEOPLEPOSE"))

(in-package "ROS")
;;//! \htmlinclude PeoplePose.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass jsk_recognition_msgs::PeoplePose
  :super ros::object
  :slots (_limb_names _poses _scores ))

(defmethod jsk_recognition_msgs::PeoplePose
  (:init
   (&key
    ((:limb_names __limb_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:poses __poses) ())
    ((:scores __scores) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _limb_names __limb_names)
   (setq _poses __poses)
   (setq _scores __scores)
   self)
  (:limb_names
   (&optional __limb_names)
   (if __limb_names (setq _limb_names __limb_names)) _limb_names)
  (:poses
   (&rest __poses)
   (if (keywordp (car __poses))
       (send* _poses __poses)
     (progn
       (if __poses (setq _poses (car __poses)))
       _poses)))
  (:scores
   (&optional __scores)
   (if __scores (setq _scores __scores)) _scores)
  (:serialization-length
   ()
   (+
    ;; string[] _limb_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _limb_names)) 4
    ;; geometry_msgs/Pose[] _poses
    (apply #'+ (send-all _poses :serialization-length)) 4
    ;; float32[] _scores
    (* 4    (length _scores)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _limb_names
     (write-long (length _limb_names) s)
     (dolist (elem _limb_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; geometry_msgs/Pose[] _poses
     (write-long (length _poses) s)
     (dolist (elem _poses)
       (send elem :serialize s)
       )
     ;; float32[] _scores
     (write-long (length _scores) s)
     (dotimes (i (length _scores))
       (sys::poke (elt _scores i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _limb_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _limb_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _limb_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; geometry_msgs/Pose[] _poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poses (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32[] _scores
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _scores (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _scores i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::PeoplePose :md5sum-) "24f6e59dae1b7cbd9d480f0008a5a515")
(setf (get jsk_recognition_msgs::PeoplePose :datatype-) "jsk_recognition_msgs/PeoplePose")
(setf (get jsk_recognition_msgs::PeoplePose :definition-)
      "string[] limb_names
geometry_msgs/Pose[] poses
float32[] scores
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :jsk_recognition_msgs/PeoplePose "24f6e59dae1b7cbd9d480f0008a5a515")


