;; Auto-generated. Do not edit!


(when (boundp 'unitree_legged_msgs::MoveCmd)
  (if (not (find-package "UNITREE_LEGGED_MSGS"))
    (make-package "UNITREE_LEGGED_MSGS"))
  (shadow 'MoveCmd (find-package "UNITREE_LEGGED_MSGS")))
(unless (find-package "UNITREE_LEGGED_MSGS::MOVECMD")
  (make-package "UNITREE_LEGGED_MSGS::MOVECMD"))

(in-package "ROS")
;;//! \htmlinclude MoveCmd.msg.html


(defclass unitree_legged_msgs::MoveCmd
  :super ros::object
  :slots (_vx _vy _vyaw ))

(defmethod unitree_legged_msgs::MoveCmd
  (:init
   (&key
    ((:vx __vx) 0)
    ((:vy __vy) 0)
    ((:vyaw __vyaw) 0)
    )
   (send-super :init)
   (setq _vx (round __vx))
   (setq _vy (round __vy))
   (setq _vyaw (round __vyaw))
   self)
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:vy
   (&optional __vy)
   (if __vy (setq _vy __vy)) _vy)
  (:vyaw
   (&optional __vyaw)
   (if __vyaw (setq _vyaw __vyaw)) _vyaw)
  (:serialization-length
   ()
   (+
    ;; int8 _vx
    1
    ;; int8 _vy
    1
    ;; int8 _vyaw
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _vx
       (write-byte _vx s)
     ;; int8 _vy
       (write-byte _vy s)
     ;; int8 _vyaw
       (write-byte _vyaw s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _vx
     (setq _vx (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _vx 127) (setq _vx (- _vx 256)))
   ;; int8 _vy
     (setq _vy (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _vy 127) (setq _vy (- _vy 256)))
   ;; int8 _vyaw
     (setq _vyaw (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _vyaw 127) (setq _vyaw (- _vyaw 256)))
   ;;
   self)
  )

(setf (get unitree_legged_msgs::MoveCmd :md5sum-) "18fcabcb0c44aa42a27c53f36efde302")
(setf (get unitree_legged_msgs::MoveCmd :datatype-) "unitree_legged_msgs/MoveCmd")
(setf (get unitree_legged_msgs::MoveCmd :definition-)
      "int8 vx
int8 vy
int8 vyaw

")



(provide :unitree_legged_msgs/MoveCmd "18fcabcb0c44aa42a27c53f36efde302")


