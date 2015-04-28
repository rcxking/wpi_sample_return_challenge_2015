; Auto-generated. Do not edit!


(cl:in-package object_recognition-srv)


;//! \htmlinclude Image-request.msg.html

(cl:defclass <Image-request> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type cl:string
    :initform ""))
)

(cl:defclass Image-request (<Image-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Image-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Image-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_recognition-srv:<Image-request> is deprecated: use object_recognition-srv:Image-request instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <Image-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition-srv:path-val is deprecated.  Use object_recognition-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Image-request>) ostream)
  "Serializes a message object of type '<Image-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Image-request>) istream)
  "Deserializes a message object of type '<Image-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Image-request>)))
  "Returns string type for a service object of type '<Image-request>"
  "object_recognition/ImageRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Image-request)))
  "Returns string type for a service object of type 'Image-request"
  "object_recognition/ImageRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Image-request>)))
  "Returns md5sum for a message object of type '<Image-request>"
  "1d00cd540af97efeb6b1589112fab63e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Image-request)))
  "Returns md5sum for a message object of type 'Image-request"
  "1d00cd540af97efeb6b1589112fab63e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Image-request>)))
  "Returns full string definition for message of type '<Image-request>"
  (cl:format cl:nil "string path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Image-request)))
  "Returns full string definition for message of type 'Image-request"
  (cl:format cl:nil "string path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Image-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Image-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Image-request
    (cl:cons ':path (path msg))
))
;//! \htmlinclude Image-response.msg.html

(cl:defclass <Image-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Image-response (<Image-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Image-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Image-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_recognition-srv:<Image-response> is deprecated: use object_recognition-srv:Image-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Image-response>) ostream)
  "Serializes a message object of type '<Image-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Image-response>) istream)
  "Deserializes a message object of type '<Image-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Image-response>)))
  "Returns string type for a service object of type '<Image-response>"
  "object_recognition/ImageResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Image-response)))
  "Returns string type for a service object of type 'Image-response"
  "object_recognition/ImageResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Image-response>)))
  "Returns md5sum for a message object of type '<Image-response>"
  "1d00cd540af97efeb6b1589112fab63e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Image-response)))
  "Returns md5sum for a message object of type 'Image-response"
  "1d00cd540af97efeb6b1589112fab63e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Image-response>)))
  "Returns full string definition for message of type '<Image-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Image-response)))
  "Returns full string definition for message of type 'Image-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Image-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Image-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Image-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Image)))
  'Image-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Image)))
  'Image-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Image)))
  "Returns string type for a service object of type '<Image>"
  "object_recognition/Image")