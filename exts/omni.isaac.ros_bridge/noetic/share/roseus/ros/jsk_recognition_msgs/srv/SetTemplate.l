;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::SetTemplate)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'SetTemplate (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::SETTEMPLATE")
  (make-package "JSK_RECOGNITION_MSGS::SETTEMPLATE"))
(unless (find-package "JSK_RECOGNITION_MSGS::SETTEMPLATEREQUEST")
  (make-package "JSK_RECOGNITION_MSGS::SETTEMPLATEREQUEST"))
(unless (find-package "JSK_RECOGNITION_MSGS::SETTEMPLATERESPONSE")
  (make-package "JSK_RECOGNITION_MSGS::SETTEMPLATERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))




(defclass jsk_recognition_msgs::SetTemplateRequest
  :super ros::object
  :slots (_type _image _dimx _dimy _relativepose _savefilename ))

(defmethod jsk_recognition_msgs::SetTemplateRequest
  (:init
   (&key
    ((:type __type) "")
    ((:image __image) (instance sensor_msgs::Image :init))
    ((:dimx __dimx) 0.0)
    ((:dimy __dimy) 0.0)
    ((:relativepose __relativepose) (instance geometry_msgs::Pose :init))
    ((:savefilename __savefilename) "")
    )
   (send-super :init)
   (setq _type (string __type))
   (setq _image __image)
   (setq _dimx (float __dimx))
   (setq _dimy (float __dimy))
   (setq _relativepose __relativepose)
   (setq _savefilename (string __savefilename))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:dimx
   (&optional __dimx)
   (if __dimx (setq _dimx __dimx)) _dimx)
  (:dimy
   (&optional __dimy)
   (if __dimy (setq _dimy __dimy)) _dimy)
  (:relativepose
   (&rest __relativepose)
   (if (keywordp (car __relativepose))
       (send* _relativepose __relativepose)
     (progn
       (if __relativepose (setq _relativepose (car __relativepose)))
       _relativepose)))
  (:savefilename
   (&optional __savefilename)
   (if __savefilename (setq _savefilename __savefilename)) _savefilename)
  (:serialization-length
   ()
   (+
    ;; string _type
    4 (length _type)
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
    ;; float32 _dimx
    4
    ;; float32 _dimy
    4
    ;; geometry_msgs/Pose _relativepose
    (send _relativepose :serialization-length)
    ;; string _savefilename
    4 (length _savefilename)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
     ;; float32 _dimx
       (sys::poke _dimx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dimy
       (sys::poke _dimy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Pose _relativepose
       (send _relativepose :serialize s)
     ;; string _savefilename
       (write-long (length _savefilename) s) (princ _savefilename s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;; float32 _dimx
     (setq _dimx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dimy
     (setq _dimy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Pose _relativepose
     (send _relativepose :deserialize buf ptr-) (incf ptr- (send _relativepose :serialization-length))
   ;; string _savefilename
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _savefilename (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass jsk_recognition_msgs::SetTemplateResponse
  :super ros::object
  :slots ())

(defmethod jsk_recognition_msgs::SetTemplateResponse
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

(defclass jsk_recognition_msgs::SetTemplate
  :super ros::object
  :slots ())

(setf (get jsk_recognition_msgs::SetTemplate :md5sum-) "116fa80f27cbdfcd76d0b57a30ef79ec")
(setf (get jsk_recognition_msgs::SetTemplate :datatype-) "jsk_recognition_msgs/SetTemplate")
(setf (get jsk_recognition_msgs::SetTemplate :request) jsk_recognition_msgs::SetTemplateRequest)
(setf (get jsk_recognition_msgs::SetTemplate :response) jsk_recognition_msgs::SetTemplateResponse)

(defmethod jsk_recognition_msgs::SetTemplateRequest
  (:response () (instance jsk_recognition_msgs::SetTemplateResponse :init)))

(setf (get jsk_recognition_msgs::SetTemplateRequest :md5sum-) "116fa80f27cbdfcd76d0b57a30ef79ec")
(setf (get jsk_recognition_msgs::SetTemplateRequest :datatype-) "jsk_recognition_msgs/SetTemplateRequest")
(setf (get jsk_recognition_msgs::SetTemplateRequest :definition-)
      "string type
sensor_msgs/Image image
# dimensions of the texture in the real world (in meters)
float32 dimx
float32 dimy
geometry_msgs/Pose relativepose # used to set the coordinate system of the object relative to the texture
string savefilename # if not empty, will save the resulting pp file to here

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---

")

(setf (get jsk_recognition_msgs::SetTemplateResponse :md5sum-) "116fa80f27cbdfcd76d0b57a30ef79ec")
(setf (get jsk_recognition_msgs::SetTemplateResponse :datatype-) "jsk_recognition_msgs/SetTemplateResponse")
(setf (get jsk_recognition_msgs::SetTemplateResponse :definition-)
      "string type
sensor_msgs/Image image
# dimensions of the texture in the real world (in meters)
float32 dimx
float32 dimy
geometry_msgs/Pose relativepose # used to set the coordinate system of the object relative to the texture
string savefilename # if not empty, will save the resulting pp file to here

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---

")



(provide :jsk_recognition_msgs/SetTemplate "116fa80f27cbdfcd76d0b57a30ef79ec")


