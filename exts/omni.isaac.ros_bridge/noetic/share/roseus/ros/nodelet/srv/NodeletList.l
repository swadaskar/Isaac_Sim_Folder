;; Auto-generated. Do not edit!


(when (boundp 'nodelet::NodeletList)
  (if (not (find-package "NODELET"))
    (make-package "NODELET"))
  (shadow 'NodeletList (find-package "NODELET")))
(unless (find-package "NODELET::NODELETLIST")
  (make-package "NODELET::NODELETLIST"))
(unless (find-package "NODELET::NODELETLISTREQUEST")
  (make-package "NODELET::NODELETLISTREQUEST"))
(unless (find-package "NODELET::NODELETLISTRESPONSE")
  (make-package "NODELET::NODELETLISTRESPONSE"))

(in-package "ROS")





(defclass nodelet::NodeletListRequest
  :super ros::object
  :slots ())

(defmethod nodelet::NodeletListRequest
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

(defclass nodelet::NodeletListResponse
  :super ros::object
  :slots (_nodelets ))

(defmethod nodelet::NodeletListResponse
  (:init
   (&key
    ((:nodelets __nodelets) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _nodelets __nodelets)
   self)
  (:nodelets
   (&optional __nodelets)
   (if __nodelets (setq _nodelets __nodelets)) _nodelets)
  (:serialization-length
   ()
   (+
    ;; string[] _nodelets
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _nodelets)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _nodelets
     (write-long (length _nodelets) s)
     (dolist (elem _nodelets)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _nodelets
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _nodelets (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _nodelets i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass nodelet::NodeletList
  :super ros::object
  :slots ())

(setf (get nodelet::NodeletList :md5sum-) "99c7b10e794f5600b8030e697e946ca7")
(setf (get nodelet::NodeletList :datatype-) "nodelet/NodeletList")
(setf (get nodelet::NodeletList :request) nodelet::NodeletListRequest)
(setf (get nodelet::NodeletList :response) nodelet::NodeletListResponse)

(defmethod nodelet::NodeletListRequest
  (:response () (instance nodelet::NodeletListResponse :init)))

(setf (get nodelet::NodeletListRequest :md5sum-) "99c7b10e794f5600b8030e697e946ca7")
(setf (get nodelet::NodeletListRequest :datatype-) "nodelet/NodeletListRequest")
(setf (get nodelet::NodeletListRequest :definition-)
      "---
string[] nodelets

")

(setf (get nodelet::NodeletListResponse :md5sum-) "99c7b10e794f5600b8030e697e946ca7")
(setf (get nodelet::NodeletListResponse :datatype-) "nodelet/NodeletListResponse")
(setf (get nodelet::NodeletListResponse :definition-)
      "---
string[] nodelets

")



(provide :nodelet/NodeletList "99c7b10e794f5600b8030e697e946ca7")


