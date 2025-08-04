;; Auto-generated. Do not edit!


(when (boundp 'tianbot_core::DebugCmd)
  (if (not (find-package "TIANBOT_CORE"))
    (make-package "TIANBOT_CORE"))
  (shadow 'DebugCmd (find-package "TIANBOT_CORE")))
(unless (find-package "TIANBOT_CORE::DEBUGCMD")
  (make-package "TIANBOT_CORE::DEBUGCMD"))
(unless (find-package "TIANBOT_CORE::DEBUGCMDREQUEST")
  (make-package "TIANBOT_CORE::DEBUGCMDREQUEST"))
(unless (find-package "TIANBOT_CORE::DEBUGCMDRESPONSE")
  (make-package "TIANBOT_CORE::DEBUGCMDRESPONSE"))

(in-package "ROS")





(defclass tianbot_core::DebugCmdRequest
  :super ros::object
  :slots (_cmd ))

(defmethod tianbot_core::DebugCmdRequest
  (:init
   (&key
    ((:cmd __cmd) "")
    )
   (send-super :init)
   (setq _cmd (string __cmd))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:serialization-length
   ()
   (+
    ;; string _cmd
    4 (length _cmd)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _cmd
       (write-long (length _cmd) s) (princ _cmd s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _cmd
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cmd (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass tianbot_core::DebugCmdResponse
  :super ros::object
  :slots (_result ))

(defmethod tianbot_core::DebugCmdResponse
  (:init
   (&key
    ((:result __result) "")
    )
   (send-super :init)
   (setq _result (string __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; string _result
    4 (length _result)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _result
       (write-long (length _result) s) (princ _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _result
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _result (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass tianbot_core::DebugCmd
  :super ros::object
  :slots ())

(setf (get tianbot_core::DebugCmd :md5sum-) "a34869486bf541f5f26e7eea27f4ece8")
(setf (get tianbot_core::DebugCmd :datatype-) "tianbot_core/DebugCmd")
(setf (get tianbot_core::DebugCmd :request) tianbot_core::DebugCmdRequest)
(setf (get tianbot_core::DebugCmd :response) tianbot_core::DebugCmdResponse)

(defmethod tianbot_core::DebugCmdRequest
  (:response () (instance tianbot_core::DebugCmdResponse :init)))

(setf (get tianbot_core::DebugCmdRequest :md5sum-) "a34869486bf541f5f26e7eea27f4ece8")
(setf (get tianbot_core::DebugCmdRequest :datatype-) "tianbot_core/DebugCmdRequest")
(setf (get tianbot_core::DebugCmdRequest :definition-)
      "string cmd
---
string result

")

(setf (get tianbot_core::DebugCmdResponse :md5sum-) "a34869486bf541f5f26e7eea27f4ece8")
(setf (get tianbot_core::DebugCmdResponse :datatype-) "tianbot_core/DebugCmdResponse")
(setf (get tianbot_core::DebugCmdResponse :definition-)
      "string cmd
---
string result

")



(provide :tianbot_core/DebugCmd "a34869486bf541f5f26e7eea27f4ece8")


