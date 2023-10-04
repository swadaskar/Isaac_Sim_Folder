;; Auto-generated. Do not edit!


(when (boundp 'jsk_recognition_msgs::Object)
  (if (not (find-package "JSK_RECOGNITION_MSGS"))
    (make-package "JSK_RECOGNITION_MSGS"))
  (shadow 'Object (find-package "JSK_RECOGNITION_MSGS")))
(unless (find-package "JSK_RECOGNITION_MSGS::OBJECT")
  (make-package "JSK_RECOGNITION_MSGS::OBJECT"))

(in-package "ROS")
;;//! \htmlinclude Object.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass jsk_recognition_msgs::Object
  :super ros::object
  :slots (_id _name _class_id _class_name _image_resources _mesh_resource _weight _dimensions ))

(defmethod jsk_recognition_msgs::Object
  (:init
   (&key
    ((:id __id) 0)
    ((:name __name) "")
    ((:class_id __class_id) 0)
    ((:class_name __class_name) "")
    ((:image_resources __image_resources) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:mesh_resource __mesh_resource) "")
    ((:weight __weight) 0.0)
    ((:dimensions __dimensions) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _name (string __name))
   (setq _class_id (round __class_id))
   (setq _class_name (string __class_name))
   (setq _image_resources __image_resources)
   (setq _mesh_resource (string __mesh_resource))
   (setq _weight (float __weight))
   (setq _dimensions __dimensions)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:class_id
   (&optional __class_id)
   (if __class_id (setq _class_id __class_id)) _class_id)
  (:class_name
   (&optional __class_name)
   (if __class_name (setq _class_name __class_name)) _class_name)
  (:image_resources
   (&optional __image_resources)
   (if __image_resources (setq _image_resources __image_resources)) _image_resources)
  (:mesh_resource
   (&optional __mesh_resource)
   (if __mesh_resource (setq _mesh_resource __mesh_resource)) _mesh_resource)
  (:weight
   (&optional __weight)
   (if __weight (setq _weight __weight)) _weight)
  (:dimensions
   (&rest __dimensions)
   (if (keywordp (car __dimensions))
       (send* _dimensions __dimensions)
     (progn
       (if __dimensions (setq _dimensions (car __dimensions)))
       _dimensions)))
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; string _name
    4 (length _name)
    ;; int32 _class_id
    4
    ;; string _class_name
    4 (length _class_name)
    ;; string[] _image_resources
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _image_resources)) 4
    ;; string _mesh_resource
    4 (length _mesh_resource)
    ;; float32 _weight
    4
    ;; geometry_msgs/Vector3 _dimensions
    (send _dimensions :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; int32 _class_id
       (write-long _class_id s)
     ;; string _class_name
       (write-long (length _class_name) s) (princ _class_name s)
     ;; string[] _image_resources
     (write-long (length _image_resources) s)
     (dolist (elem _image_resources)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string _mesh_resource
       (write-long (length _mesh_resource) s) (princ _mesh_resource s)
     ;; float32 _weight
       (sys::poke _weight (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Vector3 _dimensions
       (send _dimensions :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _class_id
     (setq _class_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _class_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _class_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _image_resources
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _image_resources (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _image_resources i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string _mesh_resource
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mesh_resource (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _weight
     (setq _weight (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Vector3 _dimensions
     (send _dimensions :deserialize buf ptr-) (incf ptr- (send _dimensions :serialization-length))
   ;;
   self)
  )

(setf (get jsk_recognition_msgs::Object :md5sum-) "57c53e712043df0244b0482d0447adee")
(setf (get jsk_recognition_msgs::Object :datatype-) "jsk_recognition_msgs/Object")
(setf (get jsk_recognition_msgs::Object :definition-)
      "# object instant info (ex. oreo_cookie)
int32 id  # object id
string name  # object name

# object class info (ex. snack)
int32 class_id
string class_name

string[] image_resources  # image urls
string mesh_resource  # mesh file url

float32 weight  # weight [kg]
geometry_msgs/Vector3 dimensions  # bounding box [m]

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :jsk_recognition_msgs/Object "57c53e712043df0244b0482d0447adee")


