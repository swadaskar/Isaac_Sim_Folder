;; Auto-generated. Do not edit!


(when (boundp 'control_msgs::GripperCommand)
  (if (not (find-package "CONTROL_MSGS"))
    (make-package "CONTROL_MSGS"))
  (shadow 'GripperCommand (find-package "CONTROL_MSGS")))
(unless (find-package "CONTROL_MSGS::GRIPPERCOMMAND")
  (make-package "CONTROL_MSGS::GRIPPERCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude GripperCommand.msg.html


(defclass control_msgs::GripperCommand
  :super ros::object
  :slots (_position _max_effort ))

(defmethod control_msgs::GripperCommand
  (:init
   (&key
    ((:position __position) 0.0)
    ((:max_effort __max_effort) 0.0)
    )
   (send-super :init)
   (setq _position (float __position))
   (setq _max_effort (float __max_effort))
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:max_effort
   (&optional __max_effort)
   (if __max_effort (setq _max_effort __max_effort)) _max_effort)
  (:serialization-length
   ()
   (+
    ;; float64 _position
    8
    ;; float64 _max_effort
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _position
       (sys::poke _position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max_effort
       (sys::poke _max_effort (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _position
     (setq _position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_effort
     (setq _max_effort (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get control_msgs::GripperCommand :md5sum-) "680acaff79486f017132a7f198d40f08")
(setf (get control_msgs::GripperCommand :datatype-) "control_msgs/GripperCommand")
(setf (get control_msgs::GripperCommand :definition-)
      "float64 position
float64 max_effort

")



(provide :control_msgs/GripperCommand "680acaff79486f017132a7f198d40f08")


