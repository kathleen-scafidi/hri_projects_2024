
(cl:in-package :asdf)

(defsystem "week0-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "keep_track" :depends-on ("_package_keep_track"))
    (:file "_package_keep_track" :depends-on ("_package"))
  ))