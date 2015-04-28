; Auto-generated. Do not edit!


(cl:in-package thresholding-srv)


;//! \htmlinclude ProcessImage-request.msg.html

(cl:defclass <ProcessImage-request> (roslisp-msg-protocol:ros-message)
  ((imagename
    :reader imagename
    :initarg :imagename
    :type cl:string
    :initform ""))
)

(cl:defclass ProcessImage-request (<ProcessImage-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProcessImage-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProcessImage-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name thresholding-srv:<ProcessImage-request> is deprecated: use thresholding-srv:ProcessImage-request instead.")))

(cl:ensure-generic-function 'imagename-val :lambda-list '(m))
(cl:defmethod imagename-val ((m <ProcessImage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thresholding-srv:imagename-val is deprecated.  Use thresholding-srv:imagename instead.")
  (imagename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProcessImage-request>) ostream)
  "Serializes a message object of type '<ProcessImage-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'imagename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'imagename))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProcessImage-request>) istream)
  "Deserializes a message object of type '<ProcessImage-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'imagename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'imagename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProcessImage-request>)))
  "Returns string type for a service object of type '<ProcessImage-request>"
  "thresholding/ProcessImageRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProcessImage-request)))
  "Returns string type for a service object of type 'ProcessImage-request"
  "thresholding/ProcessImageRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProcessImage-request>)))
  "Returns md5sum for a message object of type '<ProcessImage-request>"
  "c3e760ca42cb04034d3ea4bedb8b4008")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProcessImage-request)))
  "Returns md5sum for a message object of type 'ProcessImage-request"
  "c3e760ca42cb04034d3ea4bedb8b4008")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProcessImage-request>)))
  "Returns full string definition for message of type '<ProcessImage-request>"
  (cl:format cl:nil "string imagename~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProcessImage-request)))
  "Returns full string definition for message of type 'ProcessImage-request"
  (cl:format cl:nil "string imagename~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProcessImage-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'imagename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProcessImage-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ProcessImage-request
    (cl:cons ':imagename (imagename msg))
))
;//! \htmlinclude ProcessImage-response.msg.html

(cl:defclass <ProcessImage-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ProcessImage-response (<ProcessImage-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProcessImage-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProcessImage-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name thresholding-srv:<ProcessImage-response> is deprecated: use thresholding-srv:ProcessImage-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProcessImage-response>) ostream)
  "Serializes a message object of type '<ProcessImage-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProcessImage-response>) istream)
  "Deserializes a message object of type '<ProcessImage-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProcessImage-response>)))
  "Returns string type for a service object of type '<ProcessImage-response>"
  "thresholding/ProcessImageResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProcessImage-response)))
  "Returns string type for a service object of type 'ProcessImage-response"
  "thresholding/ProcessImageResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProcessImage-response>)))
  "Returns md5sum for a message object of type '<ProcessImage-response>"
  "c3e760ca42cb04034d3ea4bedb8b4008")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProcessImage-response)))
  "Returns md5sum for a message object of type 'ProcessImage-response"
  "c3e760ca42cb04034d3ea4bedb8b4008")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProcessImage-response>)))
  "Returns full string definition for message of type '<ProcessImage-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProcessImage-response)))
  "Returns full string definition for message of type 'ProcessImage-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProcessImage-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProcessImage-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ProcessImage-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ProcessImage)))
  'ProcessImage-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ProcessImage)))
  'ProcessImage-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProcessImage)))
  "Returns string type for a service object of type '<ProcessImage>"
  "thresholding/ProcessImage")