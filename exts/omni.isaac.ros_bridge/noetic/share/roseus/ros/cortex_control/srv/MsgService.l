;; Auto-generated. Do not edit!


(when (boundp 'cortex_control::MsgService)
  (if (not (find-package "CORTEX_CONTROL"))
    (make-package "CORTEX_CONTROL"))
  (shadow 'MsgService (find-package "CORTEX_CONTROL")))
(unless (find-package "CORTEX_CONTROL::MSGSERVICE")
  (make-package "CORTEX_CONTROL::MSGSERVICE"))
(unless (find-package "CORTEX_CONTROL::MSGSERVICEREQUEST")
  (make-package "CORTEX_CONTROL::MSGSERVICEREQUEST"))
(unless (find-package "CORTEX_CONTROL::MSGSERVICERESPONSE")
  (make-package "CORTEX_CONTROL::MSGSERVICERESPONSE"))

(in-package "ROS")





(defclass cortex_control::MsgServiceRequest
  :super ros::object
  :slots (_msg ))

(defmethod cortex_control::MsgServiceRequest
  (:init
   (&key
    ((:msg __msg) "")
    )
   (send-super :init)
   (setq _msg (string __msg))
   self)
  (:msg
   (&optional __msg)
   (if __msg (setq _msg __msg)) _msg)
  (:serialization-length
   ()
   (+
    ;; string _msg
    4 (length _msg)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _msg
       (write-long (length _msg) s) (princ _msg s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(intern "*SUCCESS*" (find-package "CORTEX_CONTROL::MSGSERVICERESPONSE"))
(shadow '*SUCCESS* (find-package "CORTEX_CONTROL::MSGSERVICERESPONSE"))
(defconstant cortex_control::MsgServiceResponse::*SUCCESS* 0)
(intern "*FAILURE*" (find-package "CORTEX_CONTROL::MSGSERVICERESPONSE"))
(shadow '*FAILURE* (find-package "CORTEX_CONTROL::MSGSERVICERESPONSE"))
(defconstant cortex_control::MsgServiceResponse::*FAILURE* 1)

(defun cortex_control::MsgServiceResponse-to-symbol (const)
  (cond
        ((= const 0) 'cortex_control::MsgServiceResponse::*SUCCESS*)
        ((= const 1) 'cortex_control::MsgServiceResponse::*FAILURE*)
        (t nil)))

(defclass cortex_control::MsgServiceResponse
  :super ros::object
  :slots (_result _error_str ))

(defmethod cortex_control::MsgServiceResponse
  (:init
   (&key
    ((:result __result) 0)
    ((:error_str __error_str) "")
    )
   (send-super :init)
   (setq _result (round __result))
   (setq _error_str (string __error_str))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:error_str
   (&optional __error_str)
   (if __error_str (setq _error_str __error_str)) _error_str)
  (:serialization-length
   ()
   (+
    ;; uint8 _result
    1
    ;; string _error_str
    4 (length _error_str)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _result
       (write-byte _result s)
     ;; string _error_str
       (write-long (length _error_str) s) (princ _error_str s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _result
     (setq _result (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _error_str
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _error_str (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass cortex_control::MsgService
  :super ros::object
  :slots ())

(setf (get cortex_control::MsgService :md5sum-) "38a3da27a00f36b8fc53764490266dcb")
(setf (get cortex_control::MsgService :datatype-) "cortex_control/MsgService")
(setf (get cortex_control::MsgService :request) cortex_control::MsgServiceRequest)
(setf (get cortex_control::MsgService :response) cortex_control::MsgServiceResponse)

(defmethod cortex_control::MsgServiceRequest
  (:response () (instance cortex_control::MsgServiceResponse :init)))

(setf (get cortex_control::MsgServiceRequest :md5sum-) "38a3da27a00f36b8fc53764490266dcb")
(setf (get cortex_control::MsgServiceRequest :datatype-) "cortex_control/MsgServiceRequest")
(setf (get cortex_control::MsgServiceRequest :definition-)
      "# commands sent in json format
string msg
---
uint8 SUCCESS=0
uint8 FAILURE=1
uint8 result
string error_str

")

(setf (get cortex_control::MsgServiceResponse :md5sum-) "38a3da27a00f36b8fc53764490266dcb")
(setf (get cortex_control::MsgServiceResponse :datatype-) "cortex_control/MsgServiceResponse")
(setf (get cortex_control::MsgServiceResponse :definition-)
      "# commands sent in json format
string msg
---
uint8 SUCCESS=0
uint8 FAILURE=1
uint8 result
string error_str

")



(provide :cortex_control/MsgService "38a3da27a00f36b8fc53764490266dcb")


