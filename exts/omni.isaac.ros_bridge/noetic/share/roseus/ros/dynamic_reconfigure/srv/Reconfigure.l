;; Auto-generated. Do not edit!


(when (boundp 'dynamic_reconfigure::Reconfigure)
  (if (not (find-package "DYNAMIC_RECONFIGURE"))
    (make-package "DYNAMIC_RECONFIGURE"))
  (shadow 'Reconfigure (find-package "DYNAMIC_RECONFIGURE")))
(unless (find-package "DYNAMIC_RECONFIGURE::RECONFIGURE")
  (make-package "DYNAMIC_RECONFIGURE::RECONFIGURE"))
(unless (find-package "DYNAMIC_RECONFIGURE::RECONFIGUREREQUEST")
  (make-package "DYNAMIC_RECONFIGURE::RECONFIGUREREQUEST"))
(unless (find-package "DYNAMIC_RECONFIGURE::RECONFIGURERESPONSE")
  (make-package "DYNAMIC_RECONFIGURE::RECONFIGURERESPONSE"))

(in-package "ROS")





(defclass dynamic_reconfigure::ReconfigureRequest
  :super ros::object
  :slots (_config ))

(defmethod dynamic_reconfigure::ReconfigureRequest
  (:init
   (&key
    ((:config __config) (instance dynamic_reconfigure::Config :init))
    )
   (send-super :init)
   (setq _config __config)
   self)
  (:config
   (&rest __config)
   (if (keywordp (car __config))
       (send* _config __config)
     (progn
       (if __config (setq _config (car __config)))
       _config)))
  (:serialization-length
   ()
   (+
    ;; dynamic_reconfigure/Config _config
    (send _config :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; dynamic_reconfigure/Config _config
       (send _config :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; dynamic_reconfigure/Config _config
     (send _config :deserialize buf ptr-) (incf ptr- (send _config :serialization-length))
   ;;
   self)
  )

(defclass dynamic_reconfigure::ReconfigureResponse
  :super ros::object
  :slots (_config ))

(defmethod dynamic_reconfigure::ReconfigureResponse
  (:init
   (&key
    ((:config __config) (instance dynamic_reconfigure::Config :init))
    )
   (send-super :init)
   (setq _config __config)
   self)
  (:config
   (&rest __config)
   (if (keywordp (car __config))
       (send* _config __config)
     (progn
       (if __config (setq _config (car __config)))
       _config)))
  (:serialization-length
   ()
   (+
    ;; dynamic_reconfigure/Config _config
    (send _config :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; dynamic_reconfigure/Config _config
       (send _config :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; dynamic_reconfigure/Config _config
     (send _config :deserialize buf ptr-) (incf ptr- (send _config :serialization-length))
   ;;
   self)
  )

(defclass dynamic_reconfigure::Reconfigure
  :super ros::object
  :slots ())

(setf (get dynamic_reconfigure::Reconfigure :md5sum-) "bb125d226a21982a4a98760418dc2672")
(setf (get dynamic_reconfigure::Reconfigure :datatype-) "dynamic_reconfigure/Reconfigure")
(setf (get dynamic_reconfigure::Reconfigure :request) dynamic_reconfigure::ReconfigureRequest)
(setf (get dynamic_reconfigure::Reconfigure :response) dynamic_reconfigure::ReconfigureResponse)

(defmethod dynamic_reconfigure::ReconfigureRequest
  (:response () (instance dynamic_reconfigure::ReconfigureResponse :init)))

(setf (get dynamic_reconfigure::ReconfigureRequest :md5sum-) "bb125d226a21982a4a98760418dc2672")
(setf (get dynamic_reconfigure::ReconfigureRequest :datatype-) "dynamic_reconfigure/ReconfigureRequest")
(setf (get dynamic_reconfigure::ReconfigureRequest :definition-)
      "Config config

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
---
Config config


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

(setf (get dynamic_reconfigure::ReconfigureResponse :md5sum-) "bb125d226a21982a4a98760418dc2672")
(setf (get dynamic_reconfigure::ReconfigureResponse :datatype-) "dynamic_reconfigure/ReconfigureResponse")
(setf (get dynamic_reconfigure::ReconfigureResponse :definition-)
      "Config config

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
---
Config config


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



(provide :dynamic_reconfigure/Reconfigure "bb125d226a21982a4a98760418dc2672")


