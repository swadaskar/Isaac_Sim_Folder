;; Auto-generated. Do not edit!


(when (boundp 'dynamic_reconfigure::IntParameter)
  (if (not (find-package "DYNAMIC_RECONFIGURE"))
    (make-package "DYNAMIC_RECONFIGURE"))
  (shadow 'IntParameter (find-package "DYNAMIC_RECONFIGURE")))
(unless (find-package "DYNAMIC_RECONFIGURE::INTPARAMETER")
  (make-package "DYNAMIC_RECONFIGURE::INTPARAMETER"))

(in-package "ROS")
;;//! \htmlinclude IntParameter.msg.html


(defclass dynamic_reconfigure::IntParameter
  :super ros::object
  :slots (_name _value ))

(defmethod dynamic_reconfigure::IntParameter
  (:init
   (&key
    ((:name __name) "")
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _value (round __value))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; int32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; int32 _value
       (write-long _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _value
     (setq _value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dynamic_reconfigure::IntParameter :md5sum-) "65fedc7a0cbfb8db035e46194a350bf1")
(setf (get dynamic_reconfigure::IntParameter :datatype-) "dynamic_reconfigure/IntParameter")
(setf (get dynamic_reconfigure::IntParameter :definition-)
      "string name
int32 value

")



(provide :dynamic_reconfigure/IntParameter "65fedc7a0cbfb8db035e46194a350bf1")


