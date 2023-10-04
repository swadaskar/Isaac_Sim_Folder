;; Auto-generated. Do not edit!


(when (boundp 'dynamic_reconfigure::DoubleParameter)
  (if (not (find-package "DYNAMIC_RECONFIGURE"))
    (make-package "DYNAMIC_RECONFIGURE"))
  (shadow 'DoubleParameter (find-package "DYNAMIC_RECONFIGURE")))
(unless (find-package "DYNAMIC_RECONFIGURE::DOUBLEPARAMETER")
  (make-package "DYNAMIC_RECONFIGURE::DOUBLEPARAMETER"))

(in-package "ROS")
;;//! \htmlinclude DoubleParameter.msg.html


(defclass dynamic_reconfigure::DoubleParameter
  :super ros::object
  :slots (_name _value ))

(defmethod dynamic_reconfigure::DoubleParameter
  (:init
   (&key
    ((:name __name) "")
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _value (float __value))
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
    ;; float64 _value
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float64 _value
       (sys::poke _value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _value
     (setq _value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get dynamic_reconfigure::DoubleParameter :md5sum-) "d8512f27253c0f65f928a67c329cd658")
(setf (get dynamic_reconfigure::DoubleParameter :datatype-) "dynamic_reconfigure/DoubleParameter")
(setf (get dynamic_reconfigure::DoubleParameter :definition-)
      "string name
float64 value

")



(provide :dynamic_reconfigure/DoubleParameter "d8512f27253c0f65f928a67c329cd658")


