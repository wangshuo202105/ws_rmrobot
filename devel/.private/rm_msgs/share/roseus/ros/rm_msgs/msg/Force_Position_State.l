;; Auto-generated. Do not edit!


(when (boundp 'rm_msgs::Force_Position_State)
  (if (not (find-package "RM_MSGS"))
    (make-package "RM_MSGS"))
  (shadow 'Force_Position_State (find-package "RM_MSGS")))
(unless (find-package "RM_MSGS::FORCE_POSITION_STATE")
  (make-package "RM_MSGS::FORCE_POSITION_STATE"))

(in-package "ROS")
;;//! \htmlinclude Force_Position_State.msg.html


(defclass rm_msgs::Force_Position_State
  :super ros::object
  :slots (_joint _force ))

(defmethod rm_msgs::Force_Position_State
  (:init
   (&key
    ((:joint __joint) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:force __force) 0.0)
    )
   (send-super :init)
   (setq _joint __joint)
   (setq _force (float __force))
   self)
  (:joint
   (&optional __joint)
   (if __joint (setq _joint __joint)) _joint)
  (:force
   (&optional __force)
   (if __force (setq _force __force)) _force)
  (:serialization-length
   ()
   (+
    ;; float32[6] _joint
    (* 4    6)
    ;; float32 _force
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[6] _joint
     (dotimes (i 6)
       (sys::poke (elt _joint i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _force
       (sys::poke _force (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[6] _joint
   (dotimes (i (length _joint))
     (setf (elt _joint i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32 _force
     (setq _force (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rm_msgs::Force_Position_State :md5sum-) "baccacd8e94a89960b7322bda551af6d")
(setf (get rm_msgs::Force_Position_State :datatype-) "rm_msgs/Force_Position_State")
(setf (get rm_msgs::Force_Position_State :definition-)
      "float32[6] joint
float32 force

")



(provide :rm_msgs/Force_Position_State "baccacd8e94a89960b7322bda551af6d")


