; Auto-generated. Do not edit!


(cl:in-package robodart_vision-srv)


;//! \htmlinclude Point-request.msg.html

(cl:defclass <Point-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Point-request (<Point-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Point-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Point-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robodart_vision-srv:<Point-request> is deprecated: use robodart_vision-srv:Point-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Point-request>) ostream)
  "Serializes a message object of type '<Point-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Point-request>) istream)
  "Deserializes a message object of type '<Point-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Point-request>)))
  "Returns string type for a service object of type '<Point-request>"
  "robodart_vision/PointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Point-request)))
  "Returns string type for a service object of type 'Point-request"
  "robodart_vision/PointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Point-request>)))
  "Returns md5sum for a message object of type '<Point-request>"
  "209f516d3eb691f0663e25cb750d67c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Point-request)))
  "Returns md5sum for a message object of type 'Point-request"
  "209f516d3eb691f0663e25cb750d67c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Point-request>)))
  "Returns full string definition for message of type '<Point-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Point-request)))
  "Returns full string definition for message of type 'Point-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Point-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Point-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Point-request
))
;//! \htmlinclude Point-response.msg.html

(cl:defclass <Point-response> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0))
)

(cl:defclass Point-response (<Point-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Point-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Point-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robodart_vision-srv:<Point-response> is deprecated: use robodart_vision-srv:Point-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Point-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robodart_vision-srv:x-val is deprecated.  Use robodart_vision-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Point-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robodart_vision-srv:y-val is deprecated.  Use robodart_vision-srv:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Point-response>) ostream)
  "Serializes a message object of type '<Point-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Point-response>) istream)
  "Deserializes a message object of type '<Point-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Point-response>)))
  "Returns string type for a service object of type '<Point-response>"
  "robodart_vision/PointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Point-response)))
  "Returns string type for a service object of type 'Point-response"
  "robodart_vision/PointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Point-response>)))
  "Returns md5sum for a message object of type '<Point-response>"
  "209f516d3eb691f0663e25cb750d67c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Point-response)))
  "Returns md5sum for a message object of type 'Point-response"
  "209f516d3eb691f0663e25cb750d67c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Point-response>)))
  "Returns full string definition for message of type '<Point-response>"
  (cl:format cl:nil "~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Point-response)))
  "Returns full string definition for message of type 'Point-response"
  (cl:format cl:nil "~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Point-response>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Point-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Point-response
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Point)))
  'Point-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Point)))
  'Point-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Point)))
  "Returns string type for a service object of type '<Point>"
  "robodart_vision/Point")