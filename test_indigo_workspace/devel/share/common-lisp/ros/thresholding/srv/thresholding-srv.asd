
(cl:in-package :asdf)

(defsystem "thresholding-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ProcessImage" :depends-on ("_package_ProcessImage"))
    (:file "_package_ProcessImage" :depends-on ("_package"))
  ))