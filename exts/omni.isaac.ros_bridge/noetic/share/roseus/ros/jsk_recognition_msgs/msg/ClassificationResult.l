;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::ClassificationResult)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'ClassificationResult (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::CLASSIFICATIONRESULT")
  (make-package "JSK_RECOGNITION_MSGS::CLASSIFICATIONRESULT"))

(in-package "ROS")
;;//! \htmlinclude ClassificationResult.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::ClassificationResult
  :super ros::object
  :slots (_header _labels _label_names _label_proba _probabilities _classifier _target_names ))

(defmethod jsk_recognition_msgs::ClassificationResult
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:labels __labels) (make-array 0 :initial-element 0 :element-type :integer))
    ((:label_names __label_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:label_proba __label_proba) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:probabilities __probabilities) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:classifier __classifier) "")
    ((:target_names __target_names) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _labels __labels)
   (setq _label_names __label_names)
   (setq _label_proba __label_proba)
   (setq _probabilities __probabilities)
   (setq _classifier (string __classifier))
   (setq _target_names __target_names)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:labels
   (&optional __labels)
   (if __labels (setq _labels __labels)) _labels)
  (:label_names
   (&optional __label_names)
   (if __label_names (setq _label_names __label_names)) _label_names)
  (:label_proba
   (&optional __label_proba)
   (if __label_proba (setq _label_proba __label_proba)) _label_proba)
  (:probabilities
   (&optional __probabilities)
   (if __probabilities (setq _probabilities __probabilities)) _probabilities)
  (:classifier
   (&optional __classifier)
   (if __classifier (setq _classifier __classifier)) _classifier)
  (:target_names
   (&optional __target_names)
   (if __target_names (setq _target_names __target_names)) _target_names)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32[] _labels
    (* 4    (length _labels)) 4
    ;; string[] _label_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _label_names)) 4
    ;; float64[] _label_proba
    (* 8    (length _label_proba)) 4
    ;; float64[] _probabilities
    (* 8    (length _probabilities)) 4
    ;; string _classifier
    4 (length _classifier)
    ;; string[] _target_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _target_names)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32[] _labels
     (write-long (length _labels) s)
     (dotimes (i (length _labels))
       (write-long (elt _labels i) s)
       )
     ;; string[] _label_names
     (write-long (length _label_names) s)
     (dolist (elem _label_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _label_proba
     (write-long (length _label_proba) s)
     (dotimes (i (length _label_proba))
       (sys::poke (elt _label_proba i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _probabilities
     (write-long (length _probabilities) s)
     (dotimes (i (length _probabilities))
       (sys::poke (elt _probabilities i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; string _classifier
       (write-long (length _classifier) s) (princ _classifier s)
     ;; string[] _target_names
     (write-long (length _target_names) s)
     (dolist (elem _target_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32[] _labels
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _labels (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _labels i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; string[] _label_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _label_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _label_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _label_proba
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _label_proba (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _label_proba i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _probabilities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _probabilities (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _probabilities i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; string _classifier
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _classifier (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _target_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _target_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _target_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::ClassificationResult :md5sum-) "cce1f8edabff85a20e9cc013e319497c")
(setf (get jsk_recognition_msgs::ClassificationResult :datatype-) "jsk_recognition_msgs/ClassificationResult")
(setf (get jsk_recognition_msgs::ClassificationResult :definition-)
      "# information about frame and timestamp
Header header

# prediction results
uint32[] labels          # numerical labels
string[] label_names     # non-numerical labels
float64[] label_proba    # probabilities of labels
float64[] probabilities  # probabilities about each classification for all target_names

# information about classifier
string classifier        # name of classifier
string[] target_names    # set in which label_names should be

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :jsk_recognition_msgs/ClassificationResult "cce1f8edabff85a20e9cc013e319497c")


