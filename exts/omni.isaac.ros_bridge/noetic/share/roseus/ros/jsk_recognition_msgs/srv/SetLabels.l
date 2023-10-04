;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SetLabels)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SetLabels (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SETLABELS")
  (make-package "JSK_RECOGNITION_MSGS::SETLABELS"))
(unless (find-package "JSK_RECOGNITION_MSGS::SETLABELSREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::SETLABELSREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::SETLABELSRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::SETLABELSRESPONSE"))

(in-package "ROS")





(defclass jsk_recognition_msgs::SetLabelsRequest
  :super ros::object
  :slots (_labels ))

(defmethod jsk_recognition_msgs::SetLabelsRequest
  (:init
   (&key
    ((:labels __labels) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _labels __labels)
   self)
  (:labels
   (&optional __labels)
   (if __labels (setq _labels __labels)) _labels)
  (:serialization-length
   ()
   (+
    ;; int64[] _labels
    (* 8    (length _labels)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64[] _labels
     (write-long (length _labels) s)
     (dotimes (i (length _labels))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _labels i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _labels i)) (= (length ((elt _labels i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _labels i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _labels i) . bv) 1) -1) s))
             ((and (class (elt _labels i)) (= (length ((elt _labels i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _labels i) . bv) 0) s)
              (write-long (if (>= (elt _labels i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _labels i) s)(write-long (if (>= (elt _labels i) 0) 0 #xffffffff) s)))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64[] _labels
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _labels (instantiate integer-vector n))
     (dotimes (i n)
#+(or :alpha :irix6 :x86_64)
      (setf (elt _labels i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _labels i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::SetLabelsResponse
  :super ros::object
  :slots (_success ))

(defmethod jsk_recognition_msgs::SetLabelsResponse
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

(defclass jsk_recognition_msgs::SetLabels
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::SetLabels :md5sum-) "f23605aaff6e30abc41a406ed99d7e0a")
(setf (get jsk_recognition_msgs::SetLabels :datatype-) "jsk_recognition_msgs/SetLabels")
(setf (get jsk_recognition_msgs::SetLabels :request) jsk_recognition_msgs::SetLabelsRequest)
(setf (get jsk_recognition_msgs::SetLabels :response) jsk_recognition_msgs::SetLabelsResponse)

(defmethod jsk_recognition_msgs::SetLabelsRequest
  (:response () (instance jsk_recognition_msgs::SetLabelsResponse :init)))

(setf (get jsk_recognition_msgs::SetLabelsRequest :md5sum-) "f23605aaff6e30abc41a406ed99d7e0a")
(setf (get jsk_recognition_msgs::SetLabelsRequest :datatype-) "jsk_recognition_msgs/SetLabelsRequest")
(setf (get jsk_recognition_msgs::SetLabelsRequest :definition-)
      "int64[] labels
---
bool success

")

(setf (get jsk_recognition_msgs::SetLabelsResponse :md5sum-) "f23605aaff6e30abc41a406ed99d7e0a")
(setf (get jsk_recognition_msgs::SetLabelsResponse :datatype-) "jsk_recognition_msgs/SetLabelsResponse")
(setf (get jsk_recognition_msgs::SetLabelsResponse :definition-)
      "int64[] labels
---
bool success

")



(provide :jsk_recognition_msgs/SetLabels "f23605aaff6e30abc41a406ed99d7e0a")


