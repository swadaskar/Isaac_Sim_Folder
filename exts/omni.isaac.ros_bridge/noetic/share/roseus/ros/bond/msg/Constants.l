;; Auto-generated. Do not edit!


(when (boundp 'bond::Constants)
  (if (not (find-package "BOND"))
    (make-package "BOND"))
  (shadow 'Constants (find-package "BOND")))
(unless (find-package "BOND::CONSTANTS")
  (make-package "BOND::CONSTANTS"))

(in-package "ROS")
;;//! \htmlinclude Constants.msg.html


(intern "*DEAD_PUBLISH_PERIOD*" (find-package "BOND::CONSTANTS"))
(shadow '*DEAD_PUBLISH_PERIOD* (find-package "BOND::CONSTANTS"))
(defconstant bond::Constants::*DEAD_PUBLISH_PERIOD* 0.05)
(intern "*DEFAULT_CONNECT_TIMEOUT*" (find-package "BOND::CONSTANTS"))
(shadow '*DEFAULT_CONNECT_TIMEOUT* (find-package "BOND::CONSTANTS"))
(defconstant bond::Constants::*DEFAULT_CONNECT_TIMEOUT* 10.0)
(intern "*DEFAULT_HEARTBEAT_TIMEOUT*" (find-package "BOND::CONSTANTS"))
(shadow '*DEFAULT_HEARTBEAT_TIMEOUT* (find-package "BOND::CONSTANTS"))
(defconstant bond::Constants::*DEFAULT_HEARTBEAT_TIMEOUT* 4.0)
(intern "*DEFAULT_DISCONNECT_TIMEOUT*" (find-package "BOND::CONSTANTS"))
(shadow '*DEFAULT_DISCONNECT_TIMEOUT* (find-package "BOND::CONSTANTS"))
(defconstant bond::Constants::*DEFAULT_DISCONNECT_TIMEOUT* 2.0)
(intern "*DEFAULT_HEARTBEAT_PERIOD*" (find-package "BOND::CONSTANTS"))
(shadow '*DEFAULT_HEARTBEAT_PERIOD* (find-package "BOND::CONSTANTS"))
(defconstant bond::Constants::*DEFAULT_HEARTBEAT_PERIOD* 1.0)
(intern "*DISABLE_HEARTBEAT_TIMEOUT_PARAM*" (find-package "BOND::CONSTANTS"))
(shadow '*DISABLE_HEARTBEAT_TIMEOUT_PARAM* (find-package "BOND::CONSTANTS"))
(defconstant bond::Constants::*DISABLE_HEARTBEAT_TIMEOUT_PARAM* "/bond_disable_heartbeat_timeout")

(defun bond::Constants-to-symbol (const)
  (cond
        (t nil)))

(defclass bond::Constants
  :super ros::object
  :slots ())

(defmethod bond::Constants
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get bond::Constants :md5sum-) "6fc594dc1d7bd7919077042712f8c8b0")
(setf (get bond::Constants :datatype-) "bond/Constants")
(setf (get bond::Constants :definition-)
      "float32 DEAD_PUBLISH_PERIOD = 0.05
float32 DEFAULT_CONNECT_TIMEOUT = 10.0
float32 DEFAULT_HEARTBEAT_TIMEOUT = 4.0
float32 DEFAULT_DISCONNECT_TIMEOUT = 2.0
float32 DEFAULT_HEARTBEAT_PERIOD = 1.0

string DISABLE_HEARTBEAT_TIMEOUT_PARAM=/bond_disable_heartbeat_timeout
")



(provide :bond/Constants "6fc594dc1d7bd7919077042712f8c8b0")


