;; Auto-generated. Do not edit!


(when (boundp 'cortex_control::JointPosVelAccCommand)
  (if (not (find-package "CORTEX_CONTROL"))
    (make-package "CORTEX_CONTROL"))
  (shadow 'JointPosVelAccCommand (find-package "CORTEX_CONTROL")))
(unless (find-package "CORTEX_CONTROL::JOINTPOSVELACCCOMMAND")
  (make-package "CORTEX_CONTROL::JOINTPOSVELACCCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude JointPosVelAccCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cortex_control::JointPosVelAccCommand
  :super ros::object
  :slots (_header _id _period _t _names _q _qd _qdd ))

(defmethod cortex_control::JointPosVelAccCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:period __period) (instance ros::time :init))
    ((:t __t) (instance ros::time :init))
    ((:names __names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:q __q) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:qd __qd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:qdd __qdd) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _period __period)
   (setq _t __t)
   (setq _names __names)
   (setq _q __q)
   (setq _qd __qd)
   (setq _qdd __qdd)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:period
   (&optional __period)
   (if __period (setq _period __period)) _period)
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:names
   (&optional __names)
   (if __names (setq _names __names)) _names)
  (:q
   (&optional __q)
   (if __q (setq _q __q)) _q)
  (:qd
   (&optional __qd)
   (if __qd (setq _qd __qd)) _qd)
  (:qdd
   (&optional __qdd)
   (if __qdd (setq _qdd __qdd)) _qdd)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int64 _id
    8
    ;; duration _period
    8
    ;; time _t
    8
    ;; string[] _names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _names)) 4
    ;; float64[] _q
    (* 8    (length _q)) 4
    ;; float64[] _qd
    (* 8    (length _qd)) 4
    ;; float64[] _qdd
    (* 8    (length _qdd)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int64 _id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _id) (= (length (_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_id . bv) 0) 0) s)
              (write-long (ash (elt (_id . bv) 1) -1) s))
             ((and (class _id) (= (length (_id . bv)) 1)) ;; big1
              (write-long (elt (_id . bv) 0) s)
              (write-long (if (>= _id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _id s)(write-long (if (>= _id 0) 0 #xffffffff) s)))
     ;; duration _period
       (write-long (send _period :sec) s) (write-long (send _period :nsec) s)
     ;; time _t
       (write-long (send _t :sec) s) (write-long (send _t :nsec) s)
     ;; string[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _q
     (write-long (length _q) s)
     (dotimes (i (length _q))
       (sys::poke (elt _q i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _qd
     (write-long (length _qd) s)
     (dotimes (i (length _qd))
       (sys::poke (elt _qd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _qdd
     (write-long (length _qdd) s)
     (dotimes (i (length _qdd))
       (sys::poke (elt _qdd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int64 _id
#+(or :alpha :irix6 :x86_64)
      (setf _id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; duration _period
     (send _period :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _period :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _t
     (send _t :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _t :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _q
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _q (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _q i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _qd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _qd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _qd i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _qdd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _qdd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _qdd i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get cortex_control::JointPosVelAccCommand :md5sum-) "150bb2ee3b92d2156bc3b45a48477ca0")
(setf (get cortex_control::JointPosVelAccCommand :datatype-) "cortex_control/JointPosVelAccCommand")
(setf (get cortex_control::JointPosVelAccCommand :definition-)
      "# Basic joint position, velocity, and feed-forward acceleration command.
# Typically, q and qd are used for PID control, and qdd is used by inverse
# dynamics to compute a feed forward term.
#
# These message contain enough information to reconstruct the specific integral
# curve on the receiving side using quintic polynomial interpolation
# (rectifying the messages against jitter). See CommandStreamInterpolator.

# Contains the wall-clock time stamp (unless otherwise specified explicitly
# during construction of the Cortex commander).
std_msgs/Header header

# id's increment by one each, and period gives the amount of time between the
# the previous message (with message ID id-1) and this
int64 id
duration period

# This time stamp is the exact controller time in the sense that
#
#   msg[id+1].t - msg[id].t = msg[id+1].period
#
# The header gives the wall-clock time at publication (unless otherwise
# specified during initialization of the Cortex commander) so we can observe any
# jitter in plotters such as rqt_plot which read the header time stamps to
# rectify the incoming messages.
time t

string[] names
float64[] q
float64[] qd
float64[] qdd

#SymmetricMatrix32 metric

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



(provide :cortex_control/JointPosVelAccCommand "150bb2ee3b92d2156bc3b45a48477ca0")


