;; Auto-generated. Do not edit!


(when (boundp 'dynamic_reconfigure::Group)
  (if (not (find-package "DYNAMIC_RECONFIGURE"))
    (make-package "DYNAMIC_RECONFIGURE"))
  (shadow 'Group (find-package "DYNAMIC_RECONFIGURE")))
(unless (find-package "DYNAMIC_RECONFIGURE::GROUP")
  (make-package "DYNAMIC_RECONFIGURE::GROUP"))

(in-package "ROS")
;;//! \htmlinclude Group.msg.html


(defclass dynamic_reconfigure::Group
  :super ros::object
  :slots (_name _type _parameters _parent _id ))

(defmethod dynamic_reconfigure::Group
  (:init
   (&key
    ((:name __name) "")
    ((:type __type) "")
    ((:parameters __parameters) ())
    ((:parent __parent) 0)
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _type (string __type))
   (setq _parameters __parameters)
   (setq _parent (round __parent))
   (setq _id (round __id))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:parameters
   (&rest __parameters)
   (if (keywordp (car __parameters))
       (send* _parameters __parameters)
     (progn
       (if __parameters (setq _parameters (car __parameters)))
       _parameters)))
  (:parent
   (&optional __parent)
   (if __parent (setq _parent __parent)) _parent)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _type
    4 (length _type)
    ;; dynamic_reconfigure/ParamDescription[] _parameters
    (apply #'+ (send-all _parameters :serialization-length)) 4
    ;; int32 _parent
    4
    ;; int32 _id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; dynamic_reconfigure/ParamDescription[] _parameters
     (write-long (length _parameters) s)
     (dolist (elem _parameters)
       (send elem :serialize s)
       )
     ;; int32 _parent
       (write-long _parent s)
     ;; int32 _id
       (write-long _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; dynamic_reconfigure/ParamDescription[] _parameters
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _parameters (let (r) (dotimes (i n) (push (instance dynamic_reconfigure::ParamDescription :init) r)) r))
     (dolist (elem- _parameters)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32 _parent
     (setq _parent (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dynamic_reconfigure::Group :md5sum-) "9e8cd9e9423c94823db3614dd8b1cf7a")
(setf (get dynamic_reconfigure::Group :datatype-) "dynamic_reconfigure/Group")
(setf (get dynamic_reconfigure::Group :definition-)
      "string name
string type
ParamDescription[] parameters
int32 parent 
int32 id

================================================================================
MSG: dynamic_reconfigure/ParamDescription
string name
string type
uint32 level
string description
string edit_method

")



(provide :dynamic_reconfigure/Group "9e8cd9e9423c94823db3614dd8b1cf7a")


