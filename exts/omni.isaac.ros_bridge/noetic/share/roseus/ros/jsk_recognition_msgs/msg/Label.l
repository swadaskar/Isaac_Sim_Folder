;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::Label)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'Label (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::LABEL")
  (make-package "JSK_RECOGNITION_MSGS::LABEL"))

(in-package "ROS")
;;//! \htmlinclude Label.msg.html


(defclass jsk_recognition_msgs::Label
  :super ros::object
  :slots (_id _name ))

(defmethod jsk_recognition_msgs::Label
  (:init
   (&key
    ((:id __id) 0)
    ((:name __name) "")
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _name (string __name))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::Label :md5sum-) "8fe5a440459dcada9c353c016dfb49d2")
(setf (get jsk_recognition_msgs::Label :datatype-) "jsk_recognition_msgs/Label")
(setf (get jsk_recognition_msgs::Label :definition-)
      "int32 id
string name

")



(provide :jsk_recognition_msgs/Label "8fe5a440459dcada9c353c016dfb49d2")


