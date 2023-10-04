;; Auto-generated. Do not edit!


(when (boundp 'nodelet::NodeletUnload)
  (if (not (find-package "NODELET"))
    (make-package "NODELET"))
  (shadow 'NodeletUnload (find-package "NODELET")))
(unless (find-package "NODELET::NODELETUNLOAD")
  (make-package "NODELET::NODELETUNLOAD"))
(unless (find-package "NODELET::NODELETUNLOADREQUEST")
  (make-package "NODELET::NODELETUNLOADREQUEST"))
(unless (find-package "NODELET::NODELETUNLOADRESPONSE")
  (make-package "NODELET::NODELETUNLOADRESPONSE"))

(in-package "ROS")





(defclass nodelet::NodeletUnloadRequest
  :super ros::object
  :slots (_name ))

(defmethod nodelet::NodeletUnloadRequest
  (:init
   (&key
    ((:name __name) "")
    )
   (send-super :init)
   (setq _name (string __name))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass nodelet::NodeletUnloadResponse
  :super ros::object
  :slots (_success ))

(defmethod nodelet::NodeletUnloadResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass nodelet::NodeletUnload
  :super ros::object
  :slots ())

(setf (get nodelet::NodeletUnload :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get nodelet::NodeletUnload :datatype-) "nodelet/NodeletUnload")
(setf (get nodelet::NodeletUnload :request) nodelet::NodeletUnloadRequest)
(setf (get nodelet::NodeletUnload :response) nodelet::NodeletUnloadResponse)

(defmethod nodelet::NodeletUnloadRequest
  (:response () (instance nodelet::NodeletUnloadResponse :init)))

(setf (get nodelet::NodeletUnloadRequest :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get nodelet::NodeletUnloadRequest :datatype-) "nodelet/NodeletUnloadRequest")
(setf (get nodelet::NodeletUnloadRequest :definition-)
      "string name
---
bool success

")

(setf (get nodelet::NodeletUnloadResponse :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get nodelet::NodeletUnloadResponse :datatype-) "nodelet/NodeletUnloadResponse")
(setf (get nodelet::NodeletUnloadResponse :definition-)
      "string name
---
bool success

")



(provide :nodelet/NodeletUnload "d08a3b641c2f8680fbdfb1ea2e17a3e1")


