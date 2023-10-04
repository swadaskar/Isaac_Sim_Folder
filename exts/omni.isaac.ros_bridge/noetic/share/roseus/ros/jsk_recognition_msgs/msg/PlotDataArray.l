;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::PlotDataArray)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'PlotDataArray (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::PLOTDATAARRAY")
  (make-package "JSK_RECOGNITION_MSGS::PLOTDATAARRAY"))

(in-package "ROS")
;;//! \htmlinclude PlotDataArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jsk_recognition_msgs::PlotDataArray
  :super ros::object
  :slots (_header _data _no_legend _legend_font_size _max_x _min_x _min_y _max_y ))

(defmethod jsk_recognition_msgs::PlotDataArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:data __data) ())
    ((:no_legend __no_legend) nil)
    ((:legend_font_size __legend_font_size) 0.0)
    ((:max_x __max_x) 0.0)
    ((:min_x __min_x) 0.0)
    ((:min_y __min_y) 0.0)
    ((:max_y __max_y) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _data __data)
   (setq _no_legend __no_legend)
   (setq _legend_font_size (float __legend_font_size))
   (setq _max_x (float __max_x))
   (setq _min_x (float __min_x))
   (setq _min_y (float __min_y))
   (setq _max_y (float __max_y))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:no_legend
   (&optional (__no_legend :null))
   (if (not (eq __no_legend :null)) (setq _no_legend __no_legend)) _no_legend)
  (:legend_font_size
   (&optional __legend_font_size)
   (if __legend_font_size (setq _legend_font_size __legend_font_size)) _legend_font_size)
  (:max_x
   (&optional __max_x)
   (if __max_x (setq _max_x __max_x)) _max_x)
  (:min_x
   (&optional __min_x)
   (if __min_x (setq _min_x __min_x)) _min_x)
  (:min_y
   (&optional __min_y)
   (if __min_y (setq _min_y __min_y)) _min_y)
  (:max_y
   (&optional __max_y)
   (if __max_y (setq _max_y __max_y)) _max_y)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; jsk_recognition_msgs/PlotData[] _data
    (apply #'+ (send-all _data :serialization-length)) 4
    ;; bool _no_legend
    1
    ;; float32 _legend_font_size
    4
    ;; float32 _max_x
    4
    ;; float32 _min_x
    4
    ;; float32 _min_y
    4
    ;; float32 _max_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jsk_recognition_msgs/PlotData[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (send elem :serialize s)
       )
     ;; bool _no_legend
       (if _no_legend (write-byte -1 s) (write-byte 0 s))
     ;; float32 _legend_font_size
       (sys::poke _legend_font_size (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_x
       (sys::poke _max_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _min_x
       (sys::poke _min_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _min_y
       (sys::poke _min_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_y
       (sys::poke _max_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; jsk_recognition_msgs/PlotData[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (let (r) (dotimes (i n) (push (instance jsk_recognition_msgs::PlotData :init) r)) r))
     (dolist (elem- _data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _no_legend
     (setq _no_legend (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _legend_font_size
     (setq _legend_font_size (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_x
     (setq _max_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _min_x
     (setq _min_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _min_y
     (setq _min_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_y
     (setq _max_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::PlotDataArray :md5sum-) "e83e9378b374b8436f955a6cd212770a")
(setf (get jsk_recognition_msgs::PlotDataArray :datatype-) "jsk_recognition_msgs/PlotDataArray")
(setf (get jsk_recognition_msgs::PlotDataArray :definition-)
      "Header header
jsk_recognition_msgs/PlotData[] data
bool no_legend
float32 legend_font_size
float32 max_x
float32 min_x
float32 min_y
float32 max_y

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

================================================================================
MSG: jsk_recognition_msgs/PlotData
uint32 SCATTER=1
uint32 LINE=2

Header header
float32[] xs
float32[] ys
uint32 type                     #SCATTER or LINE
string label
bool fit_line
bool fit_line_ransac

")



(provide :jsk_recognition_msgs/PlotDataArray "e83e9378b374b8436f955a6cd212770a")


