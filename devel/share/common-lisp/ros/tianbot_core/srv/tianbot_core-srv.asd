
(cl:in-package :asdf)

(defsystem "tianbot_core-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DebugCmd" :depends-on ("_package_DebugCmd"))
    (:file "_package_DebugCmd" :depends-on ("_package"))
  ))