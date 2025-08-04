; Auto-generated. Do not edit!


(cl:in-package tianbot_core-srv)


;//! \htmlinclude DebugCmd-request.msg.html

(cl:defclass <DebugCmd-request> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:string
    :initform ""))
)

(cl:defclass DebugCmd-request (<DebugCmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DebugCmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DebugCmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tianbot_core-srv:<DebugCmd-request> is deprecated: use tianbot_core-srv:DebugCmd-request instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <DebugCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tianbot_core-srv:cmd-val is deprecated.  Use tianbot_core-srv:cmd instead.")
  (cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DebugCmd-request>) ostream)
  "Serializes a message object of type '<DebugCmd-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cmd))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DebugCmd-request>) istream)
  "Deserializes a message object of type '<DebugCmd-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cmd) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DebugCmd-request>)))
  "Returns string type for a service object of type '<DebugCmd-request>"
  "tianbot_core/DebugCmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DebugCmd-request)))
  "Returns string type for a service object of type 'DebugCmd-request"
  "tianbot_core/DebugCmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DebugCmd-request>)))
  "Returns md5sum for a message object of type '<DebugCmd-request>"
  "a34869486bf541f5f26e7eea27f4ece8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DebugCmd-request)))
  "Returns md5sum for a message object of type 'DebugCmd-request"
  "a34869486bf541f5f26e7eea27f4ece8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DebugCmd-request>)))
  "Returns full string definition for message of type '<DebugCmd-request>"
  (cl:format cl:nil "string cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DebugCmd-request)))
  "Returns full string definition for message of type 'DebugCmd-request"
  (cl:format cl:nil "string cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DebugCmd-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'cmd))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DebugCmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DebugCmd-request
    (cl:cons ':cmd (cmd msg))
))
;//! \htmlinclude DebugCmd-response.msg.html

(cl:defclass <DebugCmd-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass DebugCmd-response (<DebugCmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DebugCmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DebugCmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tianbot_core-srv:<DebugCmd-response> is deprecated: use tianbot_core-srv:DebugCmd-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <DebugCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tianbot_core-srv:result-val is deprecated.  Use tianbot_core-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DebugCmd-response>) ostream)
  "Serializes a message object of type '<DebugCmd-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DebugCmd-response>) istream)
  "Deserializes a message object of type '<DebugCmd-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DebugCmd-response>)))
  "Returns string type for a service object of type '<DebugCmd-response>"
  "tianbot_core/DebugCmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DebugCmd-response)))
  "Returns string type for a service object of type 'DebugCmd-response"
  "tianbot_core/DebugCmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DebugCmd-response>)))
  "Returns md5sum for a message object of type '<DebugCmd-response>"
  "a34869486bf541f5f26e7eea27f4ece8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DebugCmd-response)))
  "Returns md5sum for a message object of type 'DebugCmd-response"
  "a34869486bf541f5f26e7eea27f4ece8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DebugCmd-response>)))
  "Returns full string definition for message of type '<DebugCmd-response>"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DebugCmd-response)))
  "Returns full string definition for message of type 'DebugCmd-response"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DebugCmd-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DebugCmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DebugCmd-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DebugCmd)))
  'DebugCmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DebugCmd)))
  'DebugCmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DebugCmd)))
  "Returns string type for a service object of type '<DebugCmd>"
  "tianbot_core/DebugCmd")