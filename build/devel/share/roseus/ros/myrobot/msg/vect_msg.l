;; Auto-generated. Do not edit!


(when (boundp 'myrobot::vect_msg)
  (if (not (find-package "MYROBOT"))
    (make-package "MYROBOT"))
  (shadow 'vect_msg (find-package "MYROBOT")))
(unless (find-package "MYROBOT::VECT_MSG")
  (make-package "MYROBOT::VECT_MSG"))

(in-package "ROS")
;;//! \htmlinclude vect_msg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass myrobot::vect_msg
  :super ros::object
  :slots (_header _angle _value ))

(defmethod myrobot::vect_msg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:angle __angle) 0.0)
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _angle (float __angle))
   (setq _value (float __value))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _angle
    8
    ;; float64 _value
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _value
       (sys::poke _value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _value
     (setq _value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get myrobot::vect_msg :md5sum-) "26dcad4ad00fe38c5a5613b640615033")
(setf (get myrobot::vect_msg :datatype-) "myrobot/vect_msg")
(setf (get myrobot::vect_msg :definition-)
      "Header header
float64 angle
float64 value
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



(provide :myrobot/vect_msg "26dcad4ad00fe38c5a5613b640615033")


