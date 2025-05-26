;; Auto-generated. Do not edit!


(when (boundp 'beginner_tutorials::TurtleWrite)
  (if (not (find-package "BEGINNER_TUTORIALS"))
    (make-package "BEGINNER_TUTORIALS"))
  (shadow 'TurtleWrite (find-package "BEGINNER_TUTORIALS")))
(unless (find-package "BEGINNER_TUTORIALS::TURTLEWRITE")
  (make-package "BEGINNER_TUTORIALS::TURTLEWRITE"))
(unless (find-package "BEGINNER_TUTORIALS::TURTLEWRITEREQUEST")
  (make-package "BEGINNER_TUTORIALS::TURTLEWRITEREQUEST"))
(unless (find-package "BEGINNER_TUTORIALS::TURTLEWRITERESPONSE")
  (make-package "BEGINNER_TUTORIALS::TURTLEWRITERESPONSE"))

(in-package "ROS")





(defclass beginner_tutorials::TurtleWriteRequest
  :super ros::object
  :slots (_message _scale ))

(defmethod beginner_tutorials::TurtleWriteRequest
  (:init
   (&key
    ((:message __message) "")
    ((:scale __scale) 0.0)
    )
   (send-super :init)
   (setq _message (string __message))
   (setq _scale (float __scale))
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:scale
   (&optional __scale)
   (if __scale (setq _scale __scale)) _scale)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ;; float32 _scale
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; float32 _scale
       (sys::poke _scale (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _scale
     (setq _scale (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass beginner_tutorials::TurtleWriteResponse
  :super ros::object
  :slots ())

(defmethod beginner_tutorials::TurtleWriteResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass beginner_tutorials::TurtleWrite
  :super ros::object
  :slots ())

(setf (get beginner_tutorials::TurtleWrite :md5sum-) "d965374db2978a5042a2f36a27b7fe3f")
(setf (get beginner_tutorials::TurtleWrite :datatype-) "beginner_tutorials/TurtleWrite")
(setf (get beginner_tutorials::TurtleWrite :request) beginner_tutorials::TurtleWriteRequest)
(setf (get beginner_tutorials::TurtleWrite :response) beginner_tutorials::TurtleWriteResponse)

(defmethod beginner_tutorials::TurtleWriteRequest
  (:response () (instance beginner_tutorials::TurtleWriteResponse :init)))

(setf (get beginner_tutorials::TurtleWriteRequest :md5sum-) "d965374db2978a5042a2f36a27b7fe3f")
(setf (get beginner_tutorials::TurtleWriteRequest :datatype-) "beginner_tutorials/TurtleWriteRequest")
(setf (get beginner_tutorials::TurtleWriteRequest :definition-)
      "string message
float32 scale
---

")

(setf (get beginner_tutorials::TurtleWriteResponse :md5sum-) "d965374db2978a5042a2f36a27b7fe3f")
(setf (get beginner_tutorials::TurtleWriteResponse :datatype-) "beginner_tutorials/TurtleWriteResponse")
(setf (get beginner_tutorials::TurtleWriteResponse :definition-)
      "string message
float32 scale
---

")



(provide :beginner_tutorials/TurtleWrite "d965374db2978a5042a2f36a27b7fe3f")


