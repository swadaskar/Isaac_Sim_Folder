;; Auto-generated. Do not edit!


(when (boundp 'nodelet::NodeletLoad)
  (if (not (find-package "NODELET"))
    (make-package "NODELET"))
  (shadow 'NodeletLoad (find-package "NODELET")))
(unless (find-package "NODELET::NODELETLOAD")
  (make-package "NODELET::NODELETLOAD"))
(unless (find-package "NODELET::NODELETLOADREQUEST")
  (make-package "NODELET::NODELETLOADREQUEST"))
(unless (find-package "NODELET::NODELETLOADRESPONSE")
  (make-package "NODELET::NODELETLOADRESPONSE"))

(in-package "ROS")





(defclass nodelet::NodeletLoadRequest
  :super ros::object
  :slots (_name _type _remap_source_args _remap_target_args _my_argv _bond_id ))

(defmethod nodelet::NodeletLoadRequest
  (:init
   (&key
    ((:name __name) "")
    ((:type __type) "")
    ((:remap_source_args __remap_source_args) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:remap_target_args __remap_target_args) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:my_argv __my_argv) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:bond_id __bond_id) "")
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _type (string __type))
   (setq _remap_source_args __remap_source_args)
   (setq _remap_target_args __remap_target_args)
   (setq _my_argv __my_argv)
   (setq _bond_id (string __bond_id))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:remap_source_args
   (&optional __remap_source_args)
   (if __remap_source_args (setq _remap_source_args __remap_source_args)) _remap_source_args)
  (:remap_target_args
   (&optional __remap_target_args)
   (if __remap_target_args (setq _remap_target_args __remap_target_args)) _remap_target_args)
  (:my_argv
   (&optional __my_argv)
   (if __my_argv (setq _my_argv __my_argv)) _my_argv)
  (:bond_id
   (&optional __bond_id)
   (if __bond_id (setq _bond_id __bond_id)) _bond_id)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _type
    4 (length _type)
    ;; string[] _remap_source_args
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _remap_source_args)) 4
    ;; string[] _remap_target_args
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _remap_target_args)) 4
    ;; string[] _my_argv
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _my_argv)) 4
    ;; string _bond_id
    4 (length _bond_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; string[] _remap_source_args
     (write-long (length _remap_source_args) s)
     (dolist (elem _remap_source_args)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _remap_target_args
     (write-long (length _remap_target_args) s)
     (dolist (elem _remap_target_args)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _my_argv
     (write-long (length _my_argv) s)
     (dolist (elem _my_argv)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string _bond_id
       (write-long (length _bond_id) s) (princ _bond_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _remap_source_args
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _remap_source_args (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _remap_source_args i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _remap_target_args
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _remap_target_args (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _remap_target_args i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _my_argv
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _my_argv (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _my_argv i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string _bond_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _bond_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass nodelet::NodeletLoadResponse
  :super ros::object
  :slots (_success ))

(defmethod nodelet::NodeletLoadResponse
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

(defclass nodelet::NodeletLoad
  :super ros::object
  :slots ())

(setf (get nodelet::NodeletLoad :md5sum-) "8cfbf33bf3fbd4c54a833f6e2b105331")
(setf (get nodelet::NodeletLoad :datatype-) "nodelet/NodeletLoad")
(setf (get nodelet::NodeletLoad :request) nodelet::NodeletLoadRequest)
(setf (get nodelet::NodeletLoad :response) nodelet::NodeletLoadResponse)

(defmethod nodelet::NodeletLoadRequest
  (:response () (instance nodelet::NodeletLoadResponse :init)))

(setf (get nodelet::NodeletLoadRequest :md5sum-) "8cfbf33bf3fbd4c54a833f6e2b105331")
(setf (get nodelet::NodeletLoadRequest :datatype-) "nodelet/NodeletLoadRequest")
(setf (get nodelet::NodeletLoadRequest :definition-)
      "string name
string type
string[] remap_source_args
string[] remap_target_args
string[] my_argv

string bond_id
---
bool success

")

(setf (get nodelet::NodeletLoadResponse :md5sum-) "8cfbf33bf3fbd4c54a833f6e2b105331")
(setf (get nodelet::NodeletLoadResponse :datatype-) "nodelet/NodeletLoadResponse")
(setf (get nodelet::NodeletLoadResponse :definition-)
      "string name
string type
string[] remap_source_args
string[] remap_target_args
string[] my_argv

string bond_id
---
bool success

")



(provide :nodelet/NodeletLoad "8cfbf33bf3fbd4c54a833f6e2b105331")


