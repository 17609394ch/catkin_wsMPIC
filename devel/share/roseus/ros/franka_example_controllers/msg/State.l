;; Auto-generated. Do not edit!


(when (boundp 'franka_example_controllers::State)
  (if (not (find-package "FRANKA_EXAMPLE_CONTROLLERS"))
    (make-package "FRANKA_EXAMPLE_CONTROLLERS"))
  (shadow 'State (find-package "FRANKA_EXAMPLE_CONTROLLERS")))
(unless (find-package "FRANKA_EXAMPLE_CONTROLLERS::STATE")
  (make-package "FRANKA_EXAMPLE_CONTROLLERS::STATE"))

(in-package "ROS")
;;//! \htmlinclude State.msg.html


(defclass franka_example_controllers::State
  :super ros::object
  :slots (_position _position_d _orientation _orientation_d _tau_commanded _error_orientation _error_position _tau__ext ))

(defmethod franka_example_controllers::State
  (:init
   (&key
    ((:position __position) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:position_d __position_d) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:orientation __orientation) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:orientation_d __orientation_d) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:tau_commanded __tau_commanded) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:error_orientation __error_orientation) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:error_position __error_position) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:tau__ext __tau__ext) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _position __position)
   (setq _position_d __position_d)
   (setq _orientation __orientation)
   (setq _orientation_d __orientation_d)
   (setq _tau_commanded __tau_commanded)
   (setq _error_orientation __error_orientation)
   (setq _error_position __error_position)
   (setq _tau__ext __tau__ext)
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:position_d
   (&optional __position_d)
   (if __position_d (setq _position_d __position_d)) _position_d)
  (:orientation
   (&optional __orientation)
   (if __orientation (setq _orientation __orientation)) _orientation)
  (:orientation_d
   (&optional __orientation_d)
   (if __orientation_d (setq _orientation_d __orientation_d)) _orientation_d)
  (:tau_commanded
   (&optional __tau_commanded)
   (if __tau_commanded (setq _tau_commanded __tau_commanded)) _tau_commanded)
  (:error_orientation
   (&optional __error_orientation)
   (if __error_orientation (setq _error_orientation __error_orientation)) _error_orientation)
  (:error_position
   (&optional __error_position)
   (if __error_position (setq _error_position __error_position)) _error_position)
  (:tau__ext
   (&optional __tau__ext)
   (if __tau__ext (setq _tau__ext __tau__ext)) _tau__ext)
  (:serialization-length
   ()
   (+
    ;; float64[3] _position
    (* 8    3)
    ;; float64[3] _position_d
    (* 8    3)
    ;; float64[3] _orientation
    (* 8    3)
    ;; float64[3] _orientation_d
    (* 8    3)
    ;; float64[7] _tau_commanded
    (* 8    7)
    ;; float64[3] _error_orientation
    (* 8    3)
    ;; float64[3] _error_position
    (* 8    3)
    ;; float64[6] _tau__ext
    (* 8    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[3] _position
     (dotimes (i 3)
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _position_d
     (dotimes (i 3)
       (sys::poke (elt _position_d i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _orientation
     (dotimes (i 3)
       (sys::poke (elt _orientation i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _orientation_d
     (dotimes (i 3)
       (sys::poke (elt _orientation_d i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _tau_commanded
     (dotimes (i 7)
       (sys::poke (elt _tau_commanded i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _error_orientation
     (dotimes (i 3)
       (sys::poke (elt _error_orientation i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _error_position
     (dotimes (i 3)
       (sys::poke (elt _error_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _tau__ext
     (dotimes (i 6)
       (sys::poke (elt _tau__ext i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[3] _position
   (dotimes (i (length _position))
     (setf (elt _position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _position_d
   (dotimes (i (length _position_d))
     (setf (elt _position_d i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _orientation
   (dotimes (i (length _orientation))
     (setf (elt _orientation i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _orientation_d
   (dotimes (i (length _orientation_d))
     (setf (elt _orientation_d i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _tau_commanded
   (dotimes (i (length _tau_commanded))
     (setf (elt _tau_commanded i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _error_orientation
   (dotimes (i (length _error_orientation))
     (setf (elt _error_orientation i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _error_position
   (dotimes (i (length _error_position))
     (setf (elt _error_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _tau__ext
   (dotimes (i (length _tau__ext))
     (setf (elt _tau__ext i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get franka_example_controllers::State :md5sum-) "2557e31f2a34e246ce8c881d4f7d2cb2")
(setf (get franka_example_controllers::State :datatype-) "franka_example_controllers/State")
(setf (get franka_example_controllers::State :definition-)
      "float64[3] position
float64[3] position_d
float64[3] orientation
float64[3] orientation_d
float64[7] tau_commanded
float64[3] error_orientation
float64[3] error_position
float64[6] tau__ext
")



(provide :franka_example_controllers/State "2557e31f2a34e246ce8c881d4f7d2cb2")


