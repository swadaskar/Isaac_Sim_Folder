;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::TowerRobotMoveCommand)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'TowerRobotMoveCommand (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMAND")
  (make-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMAND"))
(unless (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDRESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDRESPONSE"))

(in-package "ROS")





(intern "*ROBOT1*" (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(shadow '*ROBOT1* (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(defconstant jsk_recognition_msgs::TowerRobotMoveCommandRequest::*ROBOT1* 1)
(intern "*ROBOT2*" (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(shadow '*ROBOT2* (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(defconstant jsk_recognition_msgs::TowerRobotMoveCommandRequest::*ROBOT2* 2)
(intern "*ROBOT3*" (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(shadow '*ROBOT3* (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(defconstant jsk_recognition_msgs::TowerRobotMoveCommandRequest::*ROBOT3* 3)
(intern "*PLATE_SMALL*" (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(shadow '*PLATE_SMALL* (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(defconstant jsk_recognition_msgs::TowerRobotMoveCommandRequest::*PLATE_SMALL* 1)
(intern "*PLATE_MIDDLE*" (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(shadow '*PLATE_MIDDLE* (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(defconstant jsk_recognition_msgs::TowerRobotMoveCommandRequest::*PLATE_MIDDLE* 2)
(intern "*PLATE_LARGE*" (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(shadow '*PLATE_LARGE* (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(defconstant jsk_recognition_msgs::TowerRobotMoveCommandRequest::*PLATE_LARGE* 3)
(intern "*TOWER_LOWEST*" (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(shadow '*TOWER_LOWEST* (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(defconstant jsk_recognition_msgs::TowerRobotMoveCommandRequest::*TOWER_LOWEST* 1)
(intern "*TOWER_MIDDLE*" (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(shadow '*TOWER_MIDDLE* (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(defconstant jsk_recognition_msgs::TowerRobotMoveCommandRequest::*TOWER_MIDDLE* 2)
(intern "*TOWER_HIGHEST*" (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(shadow '*TOWER_HIGHEST* (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(defconstant jsk_recognition_msgs::TowerRobotMoveCommandRequest::*TOWER_HIGHEST* 3)
(intern "*TOWER_LOWEST2*" (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(shadow '*TOWER_LOWEST2* (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(defconstant jsk_recognition_msgs::TowerRobotMoveCommandRequest::*TOWER_LOWEST2* 1)
(intern "*OPTION_NONE*" (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(shadow '*OPTION_NONE* (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(defconstant jsk_recognition_msgs::TowerRobotMoveCommandRequest::*OPTION_NONE* 0)
(intern "*OPTION_MOVE_INITIAL*" (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(shadow '*OPTION_MOVE_INITIAL* (find-package "JSK_RECOGNITION_MSGS::TOWERROBOTMOVECOMMANDREQUEST"))
(defconstant jsk_recognition_msgs::TowerRobotMoveCommandRequest::*OPTION_MOVE_INITIAL* 1)

(defun jsk_recognition_msgs::TowerRobotMoveCommandRequest-to-symbol (const)
  (cond
        ((= const 1) 'jsk_recognition_msgs::TowerRobotMoveCommandRequest::*ROBOT1*)
        ((= const 2) 'jsk_recognition_msgs::TowerRobotMoveCommandRequest::*ROBOT2*)
        ((= const 3) 'jsk_recognition_msgs::TowerRobotMoveCommandRequest::*ROBOT3*)
        ((= const 1) 'jsk_recognition_msgs::TowerRobotMoveCommandRequest::*PLATE_SMALL*)
        ((= const 2) 'jsk_recognition_msgs::TowerRobotMoveCommandRequest::*PLATE_MIDDLE*)
        ((= const 3) 'jsk_recognition_msgs::TowerRobotMoveCommandRequest::*PLATE_LARGE*)
        ((= const 1) 'jsk_recognition_msgs::TowerRobotMoveCommandRequest::*TOWER_LOWEST*)
        ((= const 2) 'jsk_recognition_msgs::TowerRobotMoveCommandRequest::*TOWER_MIDDLE*)
        ((= const 3) 'jsk_recognition_msgs::TowerRobotMoveCommandRequest::*TOWER_HIGHEST*)
        ((= const 1) 'jsk_recognition_msgs::TowerRobotMoveCommandRequest::*TOWER_LOWEST2*)
        ((= const 0) 'jsk_recognition_msgs::TowerRobotMoveCommandRequest::*OPTION_NONE*)
        ((= const 1) 'jsk_recognition_msgs::TowerRobotMoveCommandRequest::*OPTION_MOVE_INITIAL*)
        (t nil)))

(defclass jsk_recognition_msgs::TowerRobotMoveCommandRequest
  :super ros::object
  :slots (_robot_index _plate_index _from_tower _to_tower _option_command ))

(defmethod jsk_recognition_msgs::TowerRobotMoveCommandRequest
  (:init
   (&key
    ((:robot_index __robot_index) 0)
    ((:plate_index __plate_index) 0)
    ((:from_tower __from_tower) 0)
    ((:to_tower __to_tower) 0)
    ((:option_command __option_command) 0)
    )
   (send-super :init)
   (setq _robot_index (round __robot_index))
   (setq _plate_index (round __plate_index))
   (setq _from_tower (round __from_tower))
   (setq _to_tower (round __to_tower))
   (setq _option_command (round __option_command))
   self)
  (:robot_index
   (&optional __robot_index)
   (if __robot_index (setq _robot_index __robot_index)) _robot_index)
  (:plate_index
   (&optional __plate_index)
   (if __plate_index (setq _plate_index __plate_index)) _plate_index)
  (:from_tower
   (&optional __from_tower)
   (if __from_tower (setq _from_tower __from_tower)) _from_tower)
  (:to_tower
   (&optional __to_tower)
   (if __to_tower (setq _to_tower __to_tower)) _to_tower)
  (:option_command
   (&optional __option_command)
   (if __option_command (setq _option_command __option_command)) _option_command)
  (:serialization-length
   ()
   (+
    ;; int32 _robot_index
    4
    ;; int32 _plate_index
    4
    ;; int32 _from_tower
    4
    ;; int32 _to_tower
    4
    ;; int32 _option_command
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _robot_index
       (write-long _robot_index s)
     ;; int32 _plate_index
       (write-long _plate_index s)
     ;; int32 _from_tower
       (write-long _from_tower s)
     ;; int32 _to_tower
       (write-long _to_tower s)
     ;; int32 _option_command
       (write-long _option_command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _robot_index
     (setq _robot_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _plate_index
     (setq _plate_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _from_tower
     (setq _from_tower (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _to_tower
     (setq _to_tower (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _option_command
     (setq _option_command (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass jsk_recognition_msgs::TowerRobotMoveCommandResponse
  :super ros::object
  :slots ())

(defmethod jsk_recognition_msgs::TowerRobotMoveCommandResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::TowerRobotMoveCommand
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::TowerRobotMoveCommand :md5sum-) "aadba056bdce0494569ab50ecd2ec90c")
(setf (get jsk_recognition_msgs::TowerRobotMoveCommand :datatype-) "jsk_recognition_msgs/TowerRobotMoveCommand")
(setf (get jsk_recognition_msgs::TowerRobotMoveCommand :request) jsk_recognition_msgs::TowerRobotMoveCommandRequest)
(setf (get jsk_recognition_msgs::TowerRobotMoveCommand :response) jsk_recognition_msgs::TowerRobotMoveCommandResponse)

(defmethod jsk_recognition_msgs::TowerRobotMoveCommandRequest
  (:response () (instance jsk_recognition_msgs::TowerRobotMoveCommandResponse :init)))

(setf (get jsk_recognition_msgs::TowerRobotMoveCommandRequest :md5sum-) "aadba056bdce0494569ab50ecd2ec90c")
(setf (get jsk_recognition_msgs::TowerRobotMoveCommandRequest :datatype-) "jsk_recognition_msgs/TowerRobotMoveCommandRequest")
(setf (get jsk_recognition_msgs::TowerRobotMoveCommandRequest :definition-)
      "# a service to move robot with tower index
byte ROBOT1=1
byte ROBOT2=2
byte ROBOT3=3

byte PLATE_SMALL=1
byte PLATE_MIDDLE=2
byte PLATE_LARGE=3

byte TOWER_LOWEST=1
byte TOWER_MIDDLE=2
byte TOWER_HIGHEST=3
byte TOWER_LOWEST2=1

byte OPTION_NONE=0
byte OPTION_MOVE_INITIAL=1

int32 robot_index
int32 plate_index
int32 from_tower
int32 to_tower
int32 option_command
---

")

(setf (get jsk_recognition_msgs::TowerRobotMoveCommandResponse :md5sum-) "aadba056bdce0494569ab50ecd2ec90c")
(setf (get jsk_recognition_msgs::TowerRobotMoveCommandResponse :datatype-) "jsk_recognition_msgs/TowerRobotMoveCommandResponse")
(setf (get jsk_recognition_msgs::TowerRobotMoveCommandResponse :definition-)
      "# a service to move robot with tower index
byte ROBOT1=1
byte ROBOT2=2
byte ROBOT3=3

byte PLATE_SMALL=1
byte PLATE_MIDDLE=2
byte PLATE_LARGE=3

byte TOWER_LOWEST=1
byte TOWER_MIDDLE=2
byte TOWER_HIGHEST=3
byte TOWER_LOWEST2=1

byte OPTION_NONE=0
byte OPTION_MOVE_INITIAL=1

int32 robot_index
int32 plate_index
int32 from_tower
int32 to_tower
int32 option_command
---

")



(provide :jsk_recognition_msgs/TowerRobotMoveCommand "aadba056bdce0494569ab50ecd2ec90c")


