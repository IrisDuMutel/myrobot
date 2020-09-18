
(cl:in-package :asdf)

(defsystem "myrobot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "vect_msg" :depends-on ("_package_vect_msg"))
    (:file "_package_vect_msg" :depends-on ("_package"))
  ))