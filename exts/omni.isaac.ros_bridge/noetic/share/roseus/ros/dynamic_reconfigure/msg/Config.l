;; Auto-generated. Do not edit!


(when (boundp 'dynamic_reconfigure::Config)
  (if (not (find-package "DYNAMIC_RECONFIGURE"))
    (make-package "DYNAMIC_RECONFIGURE"))
  (shadow 'Config (find-package "DYNAMIC_RECONFIGURE")))
(unless (find-package "DYNAMIC_RECONFIGURE::CONFIG")
  (make-package "DYNAMIC_RECONFIGURE::CONFIG"))

(in-package "ROS")
;;//! \htmlinclude Config.msg.html


(defclass dynamic_reconfigure::Config
  :super ros::object
  :slots (_bools _ints _strs _doubles _groups ))

(defmethod dynamic_reconfigure::Config
  (:init
   (&key
    ((:bools __bools) ())
    ((:ints __ints) ())
    ((:strs __strs) ())
    ((:doubles __doubles) ())
    ((:groups __groups) ())
    )
   (send-super :init)
   (setq _bools __bools)
   (setq _ints __ints)
   (setq _strs __strs)
   (setq _doubles __doubles)
   (setq _groups __groups)
   self)
  (:bools
   (&rest __bools)
   (if (keywordp (car __bools))
       (send* _bools __bools)
     (progn
       (if __bools (setq _bools (car __bools)))
       _bools)))
  (:ints
   (&rest __ints)
   (if (keywordp (car __ints))
       (send* _ints __ints)
     (progn
       (if __ints (setq _ints (car __ints)))
       _ints)))
  (:strs
   (&rest __strs)
   (if (keywordp (car __strs))
       (send* _strs __strs)
     (progn
       (if __strs (setq _strs (car __strs)))
       _strs)))
  (:doubles
   (&rest __doubles)
   (if (keywordp (car __doubles))
       (send* _doubles __doubles)
     (progn
       (if __doubles (setq _doubles (car __doubles)))
       _doubles)))
  (:groups
   (&rest __groups)
   (if (keywordp (car __groups))
       (send* _groups __groups)
     (progn
       (if __groups (setq _groups (car __groups)))
       _groups)))
  (:serialization-length
   ()
   (+
    ;; dynamic_reconfigure/BoolParameter[] _bools
    (apply #'+ (send-all _bools :serialization-length)) 4
    ;; dynamic_reconfigure/IntParameter[] _ints
    (apply #'+ (send-all _ints :serialization-length)) 4
    ;; dynamic_reconfigure/StrParameter[] _strs
    (apply #'+ (send-all _strs :serialization-length)) 4
    ;; dynamic_reconfigure/DoubleParameter[] _doubles
    (apply #'+ (send-all _doubles :serialization-length)) 4
    ;; dynamic_reconfigure/GroupState[] _groups
    (apply #'+ (send-all _groups :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; dynamic_reconfigure/BoolParameter[] _bools
     (write-long (length _bools) s)
     (dolist (elem _bools)
       (send elem :serialize s)
       )
     ;; dynamic_reconfigure/IntParameter[] _ints
     (write-long (length _ints) s)
     (dolist (elem _ints)
       (send elem :serialize s)
       )
     ;; dynamic_reconfigure/StrParameter[] _strs
     (write-long (length _strs) s)
     (dolist (elem _strs)
       (send elem :serialize s)
       )
     ;; dynamic_reconfigure/DoubleParameter[] _doubles
     (write-long (length _doubles) s)
     (dolist (elem _doubles)
       (send elem :serialize s)
       )
     ;; dynamic_reconfigure/GroupState[] _groups
     (write-long (length _groups) s)
     (dolist (elem _groups)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; dynamic_reconfigure/BoolParameter[] _bools
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bools (let (r) (dotimes (i n) (push (instance dynamic_reconfigure::BoolParameter :init) r)) r))
     (dolist (elem- _bools)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; dynamic_reconfigure/IntParameter[] _ints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ints (let (r) (dotimes (i n) (push (instance dynamic_reconfigure::IntParameter :init) r)) r))
     (dolist (elem- _ints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; dynamic_reconfigure/StrParameter[] _strs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _strs (let (r) (dotimes (i n) (push (instance dynamic_reconfigure::StrParameter :init) r)) r))
     (dolist (elem- _strs)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; dynamic_reconfigure/DoubleParameter[] _doubles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _doubles (let (r) (dotimes (i n) (push (instance dynamic_reconfigure::DoubleParameter :init) r)) r))
     (dolist (elem- _doubles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; dynamic_reconfigure/GroupState[] _groups
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _groups (let (r) (dotimes (i n) (push (instance dynamic_reconfigure::GroupState :init) r)) r))
     (dolist (elem- _groups)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get dynamic_reconfigure::Config :md5sum-) "958f16a05573709014982821e6822580")
(setf (get dynamic_reconfigure::Config :datatype-) "dynamic_reconfigure/Config")
(setf (get dynamic_reconfigure::Config :definition-)
      "BoolParameter[] bools
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



(provide :dynamic_reconfigure/Config "958f16a05573709014982821e6822580")


