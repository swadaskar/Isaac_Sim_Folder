
(cl:in-package :asdf)

(defsystem "cortex_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MsgService" :depends-on ("_package_MsgService"))
    (:file "_package_MsgService" :depends-on ("_package"))
  ))