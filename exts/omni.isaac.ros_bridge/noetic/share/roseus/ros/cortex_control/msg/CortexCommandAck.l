;; Auto-generated. Do not edit!


(when (boundp 'cortex_control::CortexCommandAck)
  (if (not (find-package "CORTEX_CONTROL"))
    (make-package "CORTEX_CONTROL"))
  (shadow 'CortexCommandAck (find-package "CORTEX_CONTROL")))
(unless (find-package "CORTEX_CONTROL::CORTEXCOMMANDACK")
  (make-package "CORTEX_CONTROL::CORTEXCOMMANDACK"))

(in-package "ROS")
;;//! \htmlinclude CortexCommandAck.msg.html


(defclass cortex_control::CortexCommandAck
  :super ros::object
  :slots (_cortex_command_time _cortex_command_id _time_offset ))

(defmethod cortex_control::CortexCommandAck
  (:init
   (&key
    ((:cortex_command_time __cortex_command_time) (instance ros::time :init))
    ((:cortex_command_id __cortex_command_id) 0)
    ((:time_offset __time_offset) (instance ros::time :init))
    )
   (send-super :init)
   (setq _cortex_command_time __cortex_command_time)
   (setq _cortex_command_id (round __cortex_command_id))
   (setq _time_offset __time_offset)
   self)
  (:cortex_command_time
   (&optional __cortex_command_time)
   (if __cortex_command_time (setq _cortex_command_time __cortex_command_time)) _cortex_command_time)
  (:cortex_command_id
   (&optional __cortex_command_id)
   (if __cortex_command_id (setq _cortex_command_id __cortex_command_id)) _cortex_command_id)
  (:time_offset
   (&optional __time_offset)
   (if __time_offset (setq _time_offset __time_offset)) _time_offset)
  (:serialization-length
   ()
   (+
    ;; time _cortex_command_time
    8
    ;; int64 _cortex_command_id
    8
    ;; duration _time_offset
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _cortex_command_time
       (write-long (send _cortex_command_time :sec) s) (write-long (send _cortex_command_time :nsec) s)
     ;; int64 _cortex_command_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _cortex_command_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _cortex_command_id) (= (length (_cortex_command_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_cortex_command_id . bv) 0) 0) s)
              (write-long (ash (elt (_cortex_command_id . bv) 1) -1) s))
             ((and (class _cortex_command_id) (= (length (_cortex_command_id . bv)) 1)) ;; big1
              (write-long (elt (_cortex_command_id . bv) 0) s)
              (write-long (if (>= _cortex_command_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _cortex_command_id s)(write-long (if (>= _cortex_command_id 0) 0 #xffffffff) s)))
     ;; duration _time_offset
       (write-long (send _time_offset :sec) s) (write-long (send _time_offset :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _cortex_command_time
     (send _cortex_command_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _cortex_command_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int64 _cortex_command_id
#+(or :alpha :irix6 :x86_64)
      (setf _cortex_command_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _cortex_command_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; duration _time_offset
     (send _time_offset :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time_offset :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cortex_control::CortexCommandAck :md5sum-) "81e929ba8529002a2be583a8c6600952")
(setf (get cortex_control::CortexCommandAck :datatype-) "cortex_control/CortexCommandAck")
(setf (get cortex_control::CortexCommandAck :definition-)
      "# Command time stamp of the latest Cortex command received.
time cortex_command_time

# ID of the latest command received.
int64 cortex_command_id

# If there is a slight accumulative clock rate difference between the Cortex
# commander and the low-level controller, the time offset gives how much
# further ahead the controller's clock is from the Cortex commander's clock (note
# it can be negative). So synchronizing the clocks would entail
#
#    <cortex_commander_time_synced> = <cortex_commander_time> + time_offset
duration time_offset

")



(provide :cortex_control/CortexCommandAck "81e929ba8529002a2be583a8c6600952")


