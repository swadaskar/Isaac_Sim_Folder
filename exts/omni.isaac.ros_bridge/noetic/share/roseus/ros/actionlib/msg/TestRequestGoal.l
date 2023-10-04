;; Auto-generated. Do not edit!


(when (boundp 'actionlib::TestRequestGoal)
  (if (not (find-package "ACTIONLIB"))
    (make-package "ACTIONLIB"))
  (shadow 'TestRequestGoal (find-package "ACTIONLIB")))
(unless (find-package "ACTIONLIB::TESTREQUESTGOAL")
  (make-package "ACTIONLIB::TESTREQUESTGOAL"))

(in-package "ROS")
;;//! \htmlinclude TestRequestGoal.msg.html


(intern "*TERMINATE_SUCCESS*" (find-package "ACTIONLIB::TESTREQUESTGOAL"))
(shadow '*TERMINATE_SUCCESS* (find-package "ACTIONLIB::TESTREQUESTGOAL"))
(defconstant actionlib::TestRequestGoal::*TERMINATE_SUCCESS* 0)
(intern "*TERMINATE_ABORTED*" (find-package "ACTIONLIB::TESTREQUESTGOAL"))
(shadow '*TERMINATE_ABORTED* (find-package "ACTIONLIB::TESTREQUESTGOAL"))
(defconstant actionlib::TestRequestGoal::*TERMINATE_ABORTED* 1)
(intern "*TERMINATE_REJECTED*" (find-package "ACTIONLIB::TESTREQUESTGOAL"))
(shadow '*TERMINATE_REJECTED* (find-package "ACTIONLIB::TESTREQUESTGOAL"))
(defconstant actionlib::TestRequestGoal::*TERMINATE_REJECTED* 2)
(intern "*TERMINATE_LOSE*" (find-package "ACTIONLIB::TESTREQUESTGOAL"))
(shadow '*TERMINATE_LOSE* (find-package "ACTIONLIB::TESTREQUESTGOAL"))
(defconstant actionlib::TestRequestGoal::*TERMINATE_LOSE* 3)
(intern "*TERMINATE_DROP*" (find-package "ACTIONLIB::TESTREQUESTGOAL"))
(shadow '*TERMINATE_DROP* (find-package "ACTIONLIB::TESTREQUESTGOAL"))
(defconstant actionlib::TestRequestGoal::*TERMINATE_DROP* 4)
(intern "*TERMINATE_EXCEPTION*" (find-package "ACTIONLIB::TESTREQUESTGOAL"))
(shadow '*TERMINATE_EXCEPTION* (find-package "ACTIONLIB::TESTREQUESTGOAL"))
(defconstant actionlib::TestRequestGoal::*TERMINATE_EXCEPTION* 5)

(defun actionlib::TestRequestGoal-to-symbol (const)
  (cond
        ((= const 0) 'actionlib::TestRequestGoal::*TERMINATE_SUCCESS*)
        ((= const 1) 'actionlib::TestRequestGoal::*TERMINATE_ABORTED*)
        ((= const 2) 'actionlib::TestRequestGoal::*TERMINATE_REJECTED*)
        ((= const 3) 'actionlib::TestRequestGoal::*TERMINATE_LOSE*)
        ((= const 4) 'actionlib::TestRequestGoal::*TERMINATE_DROP*)
        ((= const 5) 'actionlib::TestRequestGoal::*TERMINATE_EXCEPTION*)
        (t nil)))

(defclass actionlib::TestRequestGoal
  :super ros::object
  :slots (_terminate_status _ignore_cancel _result_text _the_result _is_simple_client _delay_accept _delay_terminate _pause_status ))

(defmethod actionlib::TestRequestGoal
  (:init
   (&key
    ((:terminate_status __terminate_status) 0)
    ((:ignore_cancel __ignore_cancel) nil)
    ((:result_text __result_text) "")
    ((:the_result __the_result) 0)
    ((:is_simple_client __is_simple_client) nil)
    ((:delay_accept __delay_accept) (instance ros::time :init))
    ((:delay_terminate __delay_terminate) (instance ros::time :init))
    ((:pause_status __pause_status) (instance ros::time :init))
    )
   (send-super :init)
   (setq _terminate_status (round __terminate_status))
   (setq _ignore_cancel __ignore_cancel)
   (setq _result_text (string __result_text))
   (setq _the_result (round __the_result))
   (setq _is_simple_client __is_simple_client)
   (setq _delay_accept __delay_accept)
   (setq _delay_terminate __delay_terminate)
   (setq _pause_status __pause_status)
   self)
  (:terminate_status
   (&optional __terminate_status)
   (if __terminate_status (setq _terminate_status __terminate_status)) _terminate_status)
  (:ignore_cancel
   (&optional (__ignore_cancel :null))
   (if (not (eq __ignore_cancel :null)) (setq _ignore_cancel __ignore_cancel)) _ignore_cancel)
  (:result_text
   (&optional __result_text)
   (if __result_text (setq _result_text __result_text)) _result_text)
  (:the_result
   (&optional __the_result)
   (if __the_result (setq _the_result __the_result)) _the_result)
  (:is_simple_client
   (&optional (__is_simple_client :null))
   (if (not (eq __is_simple_client :null)) (setq _is_simple_client __is_simple_client)) _is_simple_client)
  (:delay_accept
   (&optional __delay_accept)
   (if __delay_accept (setq _delay_accept __delay_accept)) _delay_accept)
  (:delay_terminate
   (&optional __delay_terminate)
   (if __delay_terminate (setq _delay_terminate __delay_terminate)) _delay_terminate)
  (:pause_status
   (&optional __pause_status)
   (if __pause_status (setq _pause_status __pause_status)) _pause_status)
  (:serialization-length
   ()
   (+
    ;; int32 _terminate_status
    4
    ;; bool _ignore_cancel
    1
    ;; string _result_text
    4 (length _result_text)
    ;; int32 _the_result
    4
    ;; bool _is_simple_client
    1
    ;; duration _delay_accept
    8
    ;; duration _delay_terminate
    8
    ;; duration _pause_status
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _terminate_status
       (write-long _terminate_status s)
     ;; bool _ignore_cancel
       (if _ignore_cancel (write-byte -1 s) (write-byte 0 s))
     ;; string _result_text
       (write-long (length _result_text) s) (princ _result_text s)
     ;; int32 _the_result
       (write-long _the_result s)
     ;; bool _is_simple_client
       (if _is_simple_client (write-byte -1 s) (write-byte 0 s))
     ;; duration _delay_accept
       (write-long (send _delay_accept :sec) s) (write-long (send _delay_accept :nsec) s)
     ;; duration _delay_terminate
       (write-long (send _delay_terminate :sec) s) (write-long (send _delay_terminate :nsec) s)
     ;; duration _pause_status
       (write-long (send _pause_status :sec) s) (write-long (send _pause_status :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _terminate_status
     (setq _terminate_status (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _ignore_cancel
     (setq _ignore_cancel (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _result_text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _result_text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _the_result
     (setq _the_result (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _is_simple_client
     (setq _is_simple_client (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; duration _delay_accept
     (send _delay_accept :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _delay_accept :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; duration _delay_terminate
     (send _delay_terminate :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _delay_terminate :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; duration _pause_status
     (send _pause_status :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _pause_status :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get actionlib::TestRequestGoal :md5sum-) "db5d00ba98302d6c6dd3737e9a03ceea")
(setf (get actionlib::TestRequestGoal :datatype-) "actionlib/TestRequestGoal")
(setf (get actionlib::TestRequestGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
int32 TERMINATE_SUCCESS = 0
int32 TERMINATE_ABORTED = 1
int32 TERMINATE_REJECTED = 2
int32 TERMINATE_LOSE = 3
int32 TERMINATE_DROP = 4
int32 TERMINATE_EXCEPTION = 5
int32 terminate_status
bool ignore_cancel  # If true, ignores requests to cancel
string result_text
int32 the_result    # Desired value for the_result in the Result
bool is_simple_client
duration delay_accept  # Delays accepting the goal by this amount of time
duration delay_terminate  # Delays terminating for this amount of time
duration pause_status  # Pauses the status messages for this amount of time

")



(provide :actionlib/TestRequestGoal "db5d00ba98302d6c6dd3737e9a03ceea")


