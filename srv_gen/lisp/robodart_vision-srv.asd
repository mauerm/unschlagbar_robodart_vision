
(cl:in-package :asdf)

(defsystem "robodart_vision-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Point" :depends-on ("_package_Point"))
    (:file "_package_Point" :depends-on ("_package"))
  ))