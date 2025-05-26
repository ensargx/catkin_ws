; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-srv)


;//! \htmlinclude TurtleWrite-request.msg.html

(cl:defclass <TurtleWrite-request> (roslisp-msg-protocol:ros-message)
  ((message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (scale
    :reader scale
    :initarg :scale
    :type cl:float
    :initform 0.0))
)

(cl:defclass TurtleWrite-request (<TurtleWrite-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TurtleWrite-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TurtleWrite-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-srv:<TurtleWrite-request> is deprecated: use beginner_tutorials-srv:TurtleWrite-request instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <TurtleWrite-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-srv:message-val is deprecated.  Use beginner_tutorials-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'scale-val :lambda-list '(m))
(cl:defmethod scale-val ((m <TurtleWrite-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-srv:scale-val is deprecated.  Use beginner_tutorials-srv:scale instead.")
  (scale m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TurtleWrite-request>) ostream)
  "Serializes a message object of type '<TurtleWrite-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'scale))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TurtleWrite-request>) istream)
  "Deserializes a message object of type '<TurtleWrite-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'scale) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TurtleWrite-request>)))
  "Returns string type for a service object of type '<TurtleWrite-request>"
  "beginner_tutorials/TurtleWriteRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TurtleWrite-request)))
  "Returns string type for a service object of type 'TurtleWrite-request"
  "beginner_tutorials/TurtleWriteRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TurtleWrite-request>)))
  "Returns md5sum for a message object of type '<TurtleWrite-request>"
  "d965374db2978a5042a2f36a27b7fe3f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TurtleWrite-request)))
  "Returns md5sum for a message object of type 'TurtleWrite-request"
  "d965374db2978a5042a2f36a27b7fe3f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TurtleWrite-request>)))
  "Returns full string definition for message of type '<TurtleWrite-request>"
  (cl:format cl:nil "string message~%float32 scale~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TurtleWrite-request)))
  "Returns full string definition for message of type 'TurtleWrite-request"
  (cl:format cl:nil "string message~%float32 scale~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TurtleWrite-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TurtleWrite-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TurtleWrite-request
    (cl:cons ':message (message msg))
    (cl:cons ':scale (scale msg))
))
;//! \htmlinclude TurtleWrite-response.msg.html

(cl:defclass <TurtleWrite-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TurtleWrite-response (<TurtleWrite-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TurtleWrite-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TurtleWrite-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-srv:<TurtleWrite-response> is deprecated: use beginner_tutorials-srv:TurtleWrite-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TurtleWrite-response>) ostream)
  "Serializes a message object of type '<TurtleWrite-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TurtleWrite-response>) istream)
  "Deserializes a message object of type '<TurtleWrite-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TurtleWrite-response>)))
  "Returns string type for a service object of type '<TurtleWrite-response>"
  "beginner_tutorials/TurtleWriteResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TurtleWrite-response)))
  "Returns string type for a service object of type 'TurtleWrite-response"
  "beginner_tutorials/TurtleWriteResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TurtleWrite-response>)))
  "Returns md5sum for a message object of type '<TurtleWrite-response>"
  "d965374db2978a5042a2f36a27b7fe3f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TurtleWrite-response)))
  "Returns md5sum for a message object of type 'TurtleWrite-response"
  "d965374db2978a5042a2f36a27b7fe3f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TurtleWrite-response>)))
  "Returns full string definition for message of type '<TurtleWrite-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TurtleWrite-response)))
  "Returns full string definition for message of type 'TurtleWrite-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TurtleWrite-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TurtleWrite-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TurtleWrite-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TurtleWrite)))
  'TurtleWrite-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TurtleWrite)))
  'TurtleWrite-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TurtleWrite)))
  "Returns string type for a service object of type '<TurtleWrite>"
  "beginner_tutorials/TurtleWrite")