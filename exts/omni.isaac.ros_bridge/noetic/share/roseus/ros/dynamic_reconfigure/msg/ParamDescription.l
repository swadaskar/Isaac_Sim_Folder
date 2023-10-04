;; Auto-generated. Do not edit!


(when (boundp 'dynamic_reconfigure::ParamDescription)
  (if (not (find-package "DYNAMIC_RECONFIGURE"))
    (make-package "DYNAMIC_RECONFIGURE"))
  (shadow 'ParamDescription (find-package "DYNAMIC_RECONFIGURE")))
(unless (find-package "DYNAMIC_RECONFIGURE::PARAMDESCRIPTION")
  (make-package "DYNAMIC_RECONFIGURE::PARAMDESCRIPTION"))

(in-package "ROS")
;;//! \htmlinclude ParamDescription.msg.html


(defclass dynamic_reconfigure::ParamDescription
  :super ros::object
  :slots (_name _type _level _description _edit_method ))

(defmethod dynamic_reconfigure::ParamDescription
  (:init
   (&key
    ((:name __name) "")
    ((:type __type) "")
    ((:level __level) 0)
    ((:description __description) "")
    ((:edit_method __edit_method) "")
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _type (string __type))
   (setq _level (round __level))
   (setq _description (string __description))
   (setq _edit_method (string __edit_method))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:level
   (&optional __level)
   (if __level (setq _level __level)) _level)
  (:description
   (&optional __description)
   (if __description (setq _description __description)) _description)
  (:edit_method
   (&optional __edit_method)
   (if __edit_method (setq _edit_method __edit_method)) _edit_method)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _type
    4 (length _type)
    ;; uint32 _level
    4
    ;; string _description
    4 (length _description)
    ;; string _edit_method
    4 (length _edit_method)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; uint32 _level
       (write-long _level s)
     ;; string _description
       (write-long (length _description) s) (princ _description s)
     ;; string _edit_method
       (write-long (length _edit_method) s) (princ _edit_method s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _level
     (setq _level (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _edit_method
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _edit_method (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get dynamic_reconfigure::ParamDescription :md5sum-) "7434fcb9348c13054e0c3b267c8cb34d")
(setf (get dynamic_reconfigure::ParamDescription :datatype-) "dynamic_reconfigure/ParamDescription")
(setf (get dynamic_reconfigure::ParamDescription :definition-)
      "string name
string type
uint32 level
string description
string edit_method

")



(provide :dynamic_reconfigure/ParamDescription "7434fcb9348c13054e0c3b267c8cb34d")


