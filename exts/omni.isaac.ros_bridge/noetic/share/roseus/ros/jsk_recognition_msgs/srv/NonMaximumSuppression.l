;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::NonMaximumSuppression)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'NonMaximumSuppression (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::NONMAXIMUMSUPPRESSION")
  (make-package "JSK_RECOGNITION_MSGS::NONMAXIMUMSUPPRESSION"))
(unless (find-package "JSK_RECOGNITION_MSGS::NONMAXIMUMSUPPRESSIONREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::NONMAXIMUMSUPPRESSIONREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::NONMAXIMUMSUPPRESSIONRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::NONMAXIMUMSUPPRESSIONRESPONSE"))

(in-package "ROS")





(defclass jsk_recognition_msgs::NonMaximumSuppressionRequest
  :super ros::object
  :slots (_rect _threshold ))

(defmethod jsk_recognition_msgs::NonMaximumSuppressionRequest
  (:init
   (&key
    ((:rect __rect) ())
    ((:threshold __threshold) 0.0)
    )
   (send-super :init)
   (setq _rect __rect)
   (setq _threshold (float __threshold))
   self)
  (:rect
   (&rest __rect)
   (if (keywordp (car __rect))
       (send* _rect __rect)
     (progn
       (if __rect (setq _rect (car __rect)))
       _rect)))
  (:threshold
   (&optional __threshold)
   (if __threshold (setq _threshold __threshold)) _threshold)
  (:serialization-length
   ()
   (+
    ;; jsk_recognition_msgs/Rect[] _rect
    (apply #'+ (send-all _rect :serialization-length)) 4
    ;; float32 _threshold
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; jsk_recognition_msgs/Rect[] _rect
     (write-long (length _rect) s)
     (dolist (elem _rect)
       (send elem :serialize s)
       )
     ;; float32 _threshold
       (sys::poke _threshold (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; jsk_recognition_msgs/Rect[] _rect
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rect (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::Rect :init) r)) r))
     (dolist (elem- _rect)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32 _threshold
     (setq _threshold (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass jsk_recognition_msgs::NonMaximumSuppressionResponse
  :super ros::object
  :slots (_bbox _bbox_count ))

(defmethod jsk_recognition_msgs::NonMaximumSuppressionResponse
  (:init
   (&key
    ((:bbox __bbox) ())
    ((:bbox_count __bbox_count) 0)
    )
   (send-super :init)
   (setq _bbox __bbox)
   (setq _bbox_count (round __bbox_count))
   self)
  (:bbox
   (&rest __bbox)
   (if (keywordp (car __bbox))
       (send* _bbox __bbox)
     (progn
       (if __bbox (setq _bbox (car __bbox)))
       _bbox)))
  (:bbox_count
   (&optional __bbox_count)
   (if __bbox_count (setq _bbox_count __bbox_count)) _bbox_count)
  (:serialization-length
   ()
   (+
    ;; jsk_recognition_msgs/Rect[] _bbox
    (apply #'+ (send-all _bbox :serialization-length)) 4
    ;; int64 _bbox_count
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; jsk_recognition_msgs/Rect[] _bbox
     (write-long (length _bbox) s)
     (dolist (elem _bbox)
       (send elem :serialize s)
       )
     ;; int64 _bbox_count
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _bbox_count (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _bbox_count) (= (length (_bbox_count . bv)) 2)) ;; bignum
              (write-long (ash (elt (_bbox_count . bv) 0) 0) s)
              (write-long (ash (elt (_bbox_count . bv) 1) -1) s))
             ((and (class _bbox_count) (= (length (_bbox_count . bv)) 1)) ;; big1
              (write-long (elt (_bbox_count . bv) 0) s)
              (write-long (if (>= _bbox_count 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _bbox_count s)(write-long (if (>= _bbox_count 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; jsk_recognition_msgs/Rect[] _bbox
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bbox (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::Rect :init) r)) r))
     (dolist (elem- _bbox)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int64 _bbox_count
#+(or :alpha :irix6 :x86_64)
      (setf _bbox_count (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _bbox_count (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::NonMaximumSuppression
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::NonMaximumSuppression :md5sum-) "b5f1e6a38f5b631194f786d010dab9fa")
(setf (get jsk_recognition_msgs::NonMaximumSuppression :datatype-) "jsk_recognition_msgs/NonMaximumSuppression")
(setf (get jsk_recognition_msgs::NonMaximumSuppression :request) jsk_recognition_msgs::NonMaximumSuppressionRequest)
(setf (get jsk_recognition_msgs::NonMaximumSuppression :response) jsk_recognition_msgs::NonMaximumSuppressionResponse)

(defmethod jsk_recognition_msgs::NonMaximumSuppressionRequest
  (:response () (instance jsk_recognition_msgs::NonMaximumSuppressionResponse :init)))

(setf (get jsk_recognition_msgs::NonMaximumSuppressionRequest :md5sum-) "b5f1e6a38f5b631194f786d010dab9fa")
(setf (get jsk_recognition_msgs::NonMaximumSuppressionRequest :datatype-) "jsk_recognition_msgs/NonMaximumSuppressionRequest")
(setf (get jsk_recognition_msgs::NonMaximumSuppressionRequest :definition-)
      "jsk_recognition_msgs/Rect[] rect
float32 threshold

================================================================================
MSG: jsk_recognition_msgs/Rect
int32 x
int32 y
int32 width
int32 height
---
jsk_recognition_msgs/Rect[] bbox
int64 bbox_count


================================================================================
MSG: jsk_recognition_msgs/Rect
int32 x
int32 y
int32 width
int32 height
")

(setf (get jsk_recognition_msgs::NonMaximumSuppressionResponse :md5sum-) "b5f1e6a38f5b631194f786d010dab9fa")
(setf (get jsk_recognition_msgs::NonMaximumSuppressionResponse :datatype-) "jsk_recognition_msgs/NonMaximumSuppressionResponse")
(setf (get jsk_recognition_msgs::NonMaximumSuppressionResponse :definition-)
      "jsk_recognition_msgs/Rect[] rect
float32 threshold

================================================================================
MSG: jsk_recognition_msgs/Rect
int32 x
int32 y
int32 width
int32 height
---
jsk_recognition_msgs/Rect[] bbox
int64 bbox_count


================================================================================
MSG: jsk_recognition_msgs/Rect
int32 x
int32 y
int32 width
int32 height
")



(provide :jsk_recognition_msgs/NonMaximumSuppression "b5f1e6a38f5b631194f786d010dab9fa")


