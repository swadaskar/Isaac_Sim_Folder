
(cl:in-package :asdf)

(defsystem "cortex_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CortexCommandAck" :depends-on ("_package_CortexCommandAck"))
    (:file "_package_CortexCommandAck" :depends-on ("_package"))
    (:file "JointPosVelAccCommand" :depends-on ("_package_JointPosVelAccCommand"))
    (:file "_package_JointPosVelAccCommand" :depends-on ("_package"))
  ))