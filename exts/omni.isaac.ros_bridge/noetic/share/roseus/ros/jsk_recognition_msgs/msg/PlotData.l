;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::PlotData)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'PlotData (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::PLOTDATA")
  (make-package "JSK_RECOGNITION_MSGS::PLOTDATA"))

(in-package "ROS")
;;//! \htmlinclude PlotData.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*SCATTER*" (find-package "JSK_RECOGNITION_MSGS::PLOTDATA"))
(shadow '*SCATTER* (find-package "JSK_RECOGNITION_MSGS::PLOTDATA"))
(defconstant jsk_recognition_msgs::PlotData::*SCATTER* 1)
(intern "*LINE*" (find-package "JSK_RECOGNITION_MSGS::PLOTDATA"))
(shadow '*LINE* (find-package "JSK_RECOGNITION_MSGS::PLOTDATA"))
(defconstant jsk_recognition_msgs::PlotData::*LINE* 2)

(defun jsk_recognition_msgs::PlotData-to-symbol (const)
  (cond
        ((= const 1) 'jsk_recognition_msgs::PlotData::*SCATTER*)
        ((= const 2) 'jsk_recognition_msgs::PlotData::*LINE*)
        (t nil)))

(defclass jsk_recognition_msgs::PlotData
  :super ros::object
  :slots (_header _xs _ys _type _label _fit_line _fit_line_ransac ))

(defmethod jsk_recognition_msgs::PlotData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:xs __xs) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:ys __ys) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:type __type) 0)
    ((:label __label) "")
    ((:fit_line __fit_line) nil)
    ((:fit_line_ransac __fit_line_ransac) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _xs __xs)
   (setq _ys __ys)
   (setq _type (round __type))
   (setq _label (string __label))
   (setq _fit_line __fit_line)
   (setq _fit_line_ransac __fit_line_ransac)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:xs
   (&optional __xs)
   (if __xs (setq _xs __xs)) _xs)
  (:ys
   (&optional __ys)
   (if __ys (setq _ys __ys)) _ys)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:fit_line
   (&optional (__fit_line :null))
   (if (not (eq __fit_line :null)) (setq _fit_line __fit_line)) _fit_line)
  (:fit_line_ransac
   (&optional (__fit_line_ransac :null))
   (if (not (eq __fit_line_ransac :null)) (setq _fit_line_ransac __fit_line_ransac)) _fit_line_ransac)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[] _xs
    (* 4    (length _xs)) 4
    ;; float32[] _ys
    (* 4    (length _ys)) 4
    ;; uint32 _type
    4
    ;; string _label
    4 (length _label)
    ;; bool _fit_line
    1
    ;; bool _fit_line_ransac
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[] _xs
     (write-long (length _xs) s)
     (dotimes (i (length _xs))
       (sys::poke (elt _xs i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _ys
     (write-long (length _ys) s)
     (dotimes (i (length _ys))
       (sys::poke (elt _ys i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint32 _type
       (write-long _type s)
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; bool _fit_line
       (if _fit_line (write-byte -1 s) (write-byte 0 s))
     ;; bool _fit_line_ransac
       (if _fit_line_ransac (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[] _xs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _xs (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _xs i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _ys
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ys (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _ys i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; uint32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _fit_line
     (setq _fit_line (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _fit_line_ransac
     (setq _fit_line_ransac (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::PlotData :md5sum-) "abc388ba2207b305f8695ad025452af4")
(setf (get jsk_recognition_msgs::PlotData :datatype-) "jsk_recognition_msgs/PlotData")
(setf (get jsk_recognition_msgs::PlotData :definition-)
      "uint32 SCATTER=1
uint32 LINE=2

Header header
float32[] xs
float32[] ys
uint32 type                     #SCATTER or LINE
string label
bool fit_line
bool fit_line_ransac

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



(provide :jsk_recognition_msgs/PlotData "abc388ba2207b305f8695ad025452af4")


