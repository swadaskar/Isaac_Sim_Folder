;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SparseImage)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SparseImage (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SPARSEIMAGE")
  (make-package "JSK_RECOGNITION_MSGS::SPARSEIMAGE"))

(in-package "ROS")
;;//! \htmlinclude SparseImage.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::SparseImage
  :super ros::object
  :slots (_header _width _height _data16 _data32 ))

(defmethod jsk_recognition_msgs::SparseImage
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:width __width) 0)
    ((:height __height) 0)
    ((:data16 __data16) (make-array 0 :initial-element 0 :element-type :integer))
    ((:data32 __data32) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _width (round __width))
   (setq _height (round __height))
   (setq _data16 __data16)
   (setq _data32 __data32)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:data16
   (&optional __data16)
   (if __data16 (setq _data16 __data16)) _data16)
  (:data32
   (&optional __data32)
   (if __data32 (setq _data32 __data32)) _data32)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _width
    4
    ;; uint32 _height
    4
    ;; uint16[] _data16
    (* 2    (length _data16)) 4
    ;; uint32[] _data32
    (* 4    (length _data32)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _width
       (write-long _width s)
     ;; uint32 _height
       (write-long _height s)
     ;; uint16[] _data16
     (write-long (length _data16) s)
     (dotimes (i (length _data16))
       (write-word (elt _data16 i) s)
       )
     ;; uint32[] _data32
     (write-long (length _data32) s)
     (dotimes (i (length _data32))
       (write-long (elt _data32 i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16[] _data16
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data16 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data16 i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint32[] _data32
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data32 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data32 i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::SparseImage :md5sum-) "7c361d829424bc5984fc0a1831f84751")
(setf (get jsk_recognition_msgs::SparseImage :datatype-) "jsk_recognition_msgs/SparseImage")
(setf (get jsk_recognition_msgs::SparseImage :definition-)
      "Header header

uint32 width
uint32 height

# each uint8 or uint16 contains position of white pixel expressed like: \"(x << 8 or 16) ^ y\"
# if max(width, height) > 256(=2^8) use data32 array, else use data16 array.
uint16[] data16
uint32[] data32

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



(provide :jsk_recognition_msgs/SparseImage "7c361d829424bc5984fc0a1831f84751")


