;; Auto-generated. Do not edit!


(when (boundp 'dynamic_reconfigure::ConfigDescription)
  (if (not (find-package "DYNAMIC_RECONFIGURE"))
    (make-package "DYNAMIC_RECONFIGURE"))
  (shadow 'ConfigDescription (find-package "DYNAMIC_RECONFIGURE")))
(unless (find-package "DYNAMIC_RECONFIGURE::CONFIGDESCRIPTION")
  (make-package "DYNAMIC_RECONFIGURE::CONFIGDESCRIPTION"))

(in-package "ROS")
;;//! \htmlinclude ConfigDescription.msg.html


(defclass dynamic_reconfigure::ConfigDescription
  :super ros::object
  :slots (_groups _max _min _dflt ))

(defmethod dynamic_reconfigure::ConfigDescription
  (:init
   (&key
    ((:groups __groups) ())
    ((:max __max) (instance dynamic_reconfigure::Config :init))
    ((:min __min) (instance dynamic_reconfigure::Config :init))
    ((:dflt __dflt) (instance dynamic_reconfigure::Config :init))
    )
   (send-super :init)
   (setq _groups __groups)
   (setq _max __max)
   (setq _min __min)
   (setq _dflt __dflt)
   self)
  (:groups
   (&rest __groups)
   (if (keywordp (car __groups))
       (send* _groups __groups)
     (progn
       (if __groups (setq _groups (car __groups)))
       _groups)))
  (:max
   (&rest __max)
   (if (keywordp (car __max))
       (send* _max __max)
     (progn
       (if __max (setq _max (car __max)))
       _max)))
  (:min
   (&rest __min)
   (if (keywordp (car __min))
       (send* _min __min)
     (progn
       (if __min (setq _min (car __min)))
       _min)))
  (:dflt
   (&rest __dflt)
   (if (keywordp (car __dflt))
       (send* _dflt __dflt)
     (progn
       (if __dflt (setq _dflt (car __dflt)))
       _dflt)))
  (:serialization-length
   ()
   (+
    ;; dynamic_reconfigure/Group[] _groups
    (apply #'+ (send-all _groups :serialization-length)) 4
    ;; dynamic_reconfigure/Config _max
    (send _max :serialization-length)
    ;; dynamic_reconfigure/Config _min
    (send _min :serialization-length)
    ;; dynamic_reconfigure/Config _dflt
    (send _dflt :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; dynamic_reconfigure/Group[] _groups
     (write-long (length _groups) s)
     (dolist (elem _groups)
       (send elem :serialize s)
       )
     ;; dynamic_reconfigure/Config _max
       (send _max :serialize s)
     ;; dynamic_reconfigure/Config _min
       (send _min :serialize s)
     ;; dynamic_reconfigure/Config _dflt
       (send _dflt :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; dynamic_reconfigure/Group[] _groups
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _groups (let (r) (dotimes (i n) (push (instance dynamic_reconfigure::Group :init) r)) r))
     (dolist (elem- _groups)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; dynamic_reconfigure/Config _max
     (send _max :deserialize buf ptr-) (incf ptr- (send _max :serialization-length))
   ;; dynamic_reconfigure/Config _min
     (send _min :deserialize buf ptr-) (incf ptr- (send _min :serialization-length))
   ;; dynamic_reconfigure/Config _dflt
     (send _dflt :deserialize buf ptr-) (incf ptr- (send _dflt :serialization-length))
   ;;
   self)
  )

(setf (get dynamic_reconfigure::ConfigDescription :md5sum-) "757ce9d44ba8ddd801bb30bc456f946f")
(setf (get dynamic_reconfigure::ConfigDescription :datatype-) "dynamic_reconfigure/ConfigDescription")
(setf (get dynamic_reconfigure::ConfigDescription :definition-)
      "Group[] groups
Config max
Config min
Config dflt

================================================================================
MSG: dynamic_reconfigure/Group
string name
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

================================================================================
MSG: dynamic_reconfigure/Config
BoolParameter[] bools
IntParameter[] ints
StrParameter[] strs
DoubleParameter[] doubles
GroupState[] groups

================================================================================
MSG: dynamic_reconfigure/BoolParameter
string name
bool value

================================================================================
MSG: dynamic_reconfigure/IntParameter
string name
int32 value

================================================================================
MSG: dynamic_reconfigure/StrParameter
string name
string value

================================================================================
MSG: dynamic_reconfigure/DoubleParameter
string name
float64 value

================================================================================
MSG: dynamic_reconfigure/GroupState
string name
bool state
int32 id
int32 parent

")



(provide :dynamic_reconfigure/ConfigDescription "757ce9d44ba8ddd801bb30bc456f946f")


