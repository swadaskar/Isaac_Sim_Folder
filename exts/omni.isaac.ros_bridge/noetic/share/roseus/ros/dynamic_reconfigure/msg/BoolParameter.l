;; Auto-generated. Do not edit!


(when (boundp 'dynamic_reconfigure::BoolParameter)
  (if (not (find-package "DYNAMIC_RECONFIGURE"))
    (make-package "DYNAMIC_RECONFIGURE"))
  (shadow 'BoolParameter (find-package "DYNAMIC_RECONFIGURE")))
(unless (find-package "DYNAMIC_RECONFIGURE::BOOLPARAMETER")
  (make-package "DYNAMIC_RECONFIGURE::BOOLPARAMETER"))

(in-package "ROS")
;;//! \htmlinclude BoolParameter.msg.html


(defclass dynamic_reconfigure::BoolParameter
  :super ros::object
  :slots (_name _value ))

(defmethod dynamic_reconfigure::BoolParameter
  (:init
   (&key
    ((:name __name) "")
    ((:value __value) nil)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _value __value)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:value
   (&optional (__value :null))
   (if (not (eq __value :null)) (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; bool _value
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; bool _value
       (if _value (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _value
     (setq _value (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get dynamic_reconfigure::BoolParameter :md5sum-) "23f05028c1a699fb83e22401228c3a9e")
(setf (get dynamic_reconfigure::BoolParameter :datatype-) "dynamic_reconfigure/BoolParameter")
(setf (get dynamic_reconfigure::BoolParameter :definition-)
      "string name
bool value

")



(provide :dynamic_reconfigure/BoolParameter "23f05028c1a699fb83e22401228c3a9e")


