; Auto-generated. Do not edit!


(cl:in-package unitree_legged_msgs-msg)


;//! \htmlinclude MoveCmd.msg.html

(cl:defclass <MoveCmd> (roslisp-msg-protocol:ros-message)
  ((vx
    :reader vx
    :initarg :vx
    :type cl:fixnum
    :initform 0)
   (vy
    :reader vy
    :initarg :vy
    :type cl:fixnum
    :initform 0)
   (vyaw
    :reader vyaw
    :initarg :vyaw
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MoveCmd (<MoveCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name unitree_legged_msgs-msg:<MoveCmd> is deprecated: use unitree_legged_msgs-msg:MoveCmd instead.")))

(cl:ensure-generic-function 'vx-val :lambda-list '(m))
(cl:defmethod vx-val ((m <MoveCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:vx-val is deprecated.  Use unitree_legged_msgs-msg:vx instead.")
  (vx m))

(cl:ensure-generic-function 'vy-val :lambda-list '(m))
(cl:defmethod vy-val ((m <MoveCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:vy-val is deprecated.  Use unitree_legged_msgs-msg:vy instead.")
  (vy m))

(cl:ensure-generic-function 'vyaw-val :lambda-list '(m))
(cl:defmethod vyaw-val ((m <MoveCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:vyaw-val is deprecated.  Use unitree_legged_msgs-msg:vyaw instead.")
  (vyaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveCmd>) ostream)
  "Serializes a message object of type '<MoveCmd>"
  (cl:let* ((signed (cl:slot-value msg 'vx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'vy)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'vyaw)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveCmd>) istream)
  "Deserializes a message object of type '<MoveCmd>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vx) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vy) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vyaw) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveCmd>)))
  "Returns string type for a message object of type '<MoveCmd>"
  "unitree_legged_msgs/MoveCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveCmd)))
  "Returns string type for a message object of type 'MoveCmd"
  "unitree_legged_msgs/MoveCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveCmd>)))
  "Returns md5sum for a message object of type '<MoveCmd>"
  "18fcabcb0c44aa42a27c53f36efde302")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveCmd)))
  "Returns md5sum for a message object of type 'MoveCmd"
  "18fcabcb0c44aa42a27c53f36efde302")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveCmd>)))
  "Returns full string definition for message of type '<MoveCmd>"
  (cl:format cl:nil "int8 vx~%int8 vy~%int8 vyaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveCmd)))
  "Returns full string definition for message of type 'MoveCmd"
  (cl:format cl:nil "int8 vx~%int8 vy~%int8 vyaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveCmd>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveCmd
    (cl:cons ':vx (vx msg))
    (cl:cons ':vy (vy msg))
    (cl:cons ':vyaw (vyaw msg))
))
