;; Auto-generated. Do not edit!


(when (boundp 'dynamic_reconfigure::GroupState)
  (if (not (find-package "DYNAMIC_RECONFIGURE"))
    (make-package "DYNAMIC_RECONFIGURE"))
  (shadow 'GroupState (find-package "DYNAMIC_RECONFIGURE")))
(unless (find-package "DYNAMIC_RECONFIGURE::GROUPSTATE")
  (make-package "DYNAMIC_RECONFIGURE::GROUPSTATE"))

(in-package "ROS")
;;//! \htmlinclude GroupState.msg.html


(defclass dynamic_reconfigure::GroupState
  :super ros::object
  :slots (_name _state _id _parent ))

(defmethod dynamic_reconfigure::GroupState
  (:init
   (&key
    ((:name __name) "")
    ((:state __state) nil)
    ((:id __id) 0)
    ((:parent __parent) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _state __state)
   (setq _id (round __id))
   (setq _parent (round __parent))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:state
   (&optional (__state :null))
   (if (not (eq __state :null)) (setq _state __state)) _state)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:parent
   (&optional __parent)
   (if __parent (setq _parent __parent)) _parent)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; bool _state
    1
    ;; int32 _id
    4
    ;; int32 _parent
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; bool _state
       (if _state (write-byte -1 s) (write-byte 0 s))
     ;; int32 _id
       (write-long _id s)
     ;; int32 _parent
       (write-long _parent s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _state
     (setq _state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _parent
     (setq _parent (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dynamic_reconfigure::GroupState :md5sum-) "a2d87f51dc22930325041a2f8b1571f8")
(setf (get dynamic_reconfigure::GroupState :datatype-) "dynamic_reconfigure/GroupState")
(setf (get dynamic_reconfigure::GroupState :definition-)
      "string name
bool state
int32 id
int32 parent

")



(provide :dynamic_reconfigure/GroupState "a2d87f51dc22930325041a2f8b1571f8")


