;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::DepthCalibrationParameter)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'DepthCalibrationParameter (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::DEPTHCALIBRATIONPARAMETER")
  (make-package "JSK_RECOGNITION_MSGS::DEPTHCALIBRATIONPARAMETER"))

(in-package "ROS")
;;//! \htmlinclude DepthCalibrationParameter.msg.html


(defclass jsk_recognition_msgs::DepthCalibrationParameter
  :super ros::object
  :slots (_coefficients2 _coefficients1 _coefficients0 _use_abs ))

(defmethod jsk_recognition_msgs::DepthCalibrationParameter
  (:init
   (&key
    ((:coefficients2 __coefficients2) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:coefficients1 __coefficients1) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:coefficients0 __coefficients0) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:use_abs __use_abs) nil)
    )
   (send-super :init)
   (setq _coefficients2 __coefficients2)
   (setq _coefficients1 __coefficients1)
   (setq _coefficients0 __coefficients0)
   (setq _use_abs __use_abs)
   self)
  (:coefficients2
   (&optional __coefficients2)
   (if __coefficients2 (setq _coefficients2 __coefficients2)) _coefficients2)
  (:coefficients1
   (&optional __coefficients1)
   (if __coefficients1 (setq _coefficients1 __coefficients1)) _coefficients1)
  (:coefficients0
   (&optional __coefficients0)
   (if __coefficients0 (setq _coefficients0 __coefficients0)) _coefficients0)
  (:use_abs
   (&optional (__use_abs :null))
   (if (not (eq __use_abs :null)) (setq _use_abs __use_abs)) _use_abs)
  (:serialization-length
   ()
   (+
    ;; float64[] _coefficients2
    (* 8    (length _coefficients2)) 4
    ;; float64[] _coefficients1
    (* 8    (length _coefficients1)) 4
    ;; float64[] _coefficients0
    (* 8    (length _coefficients0)) 4
    ;; bool _use_abs
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _coefficients2
     (write-long (length _coefficients2) s)
     (dotimes (i (length _coefficients2))
       (sys::poke (elt _coefficients2 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _coefficients1
     (write-long (length _coefficients1) s)
     (dotimes (i (length _coefficients1))
       (sys::poke (elt _coefficients1 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _coefficients0
     (write-long (length _coefficients0) s)
     (dotimes (i (length _coefficients0))
       (sys::poke (elt _coefficients0 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _use_abs
       (if _use_abs (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _coefficients2
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _coefficients2 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _coefficients2 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _coefficients1
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _coefficients1 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _coefficients1 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _coefficients0
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _coefficients0 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _coefficients0 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; bool _use_abs
     (setq _use_abs (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::DepthCalibrationParameter :md5sum-) "d8318983ee0a76ad66ecf4b504350888")
(setf (get jsk_recognition_msgs::DepthCalibrationParameter :datatype-) "jsk_recognition_msgs/DepthCalibrationParameter")
(setf (get jsk_recognition_msgs::DepthCalibrationParameter :definition-)
      "# each vector stands for C(u, v)
# C(u, v) = a_0 * u^2 + a_1 * u + a_2 * v^2 + a_3 * v + a_4
float64[] coefficients2
float64[] coefficients1
float64[] coefficients0
bool use_abs

")



(provide :jsk_recognition_msgs/DepthCalibrationParameter "d8318983ee0a76ad66ecf4b504350888")


