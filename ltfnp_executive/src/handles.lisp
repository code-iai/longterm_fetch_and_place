;;; Copyright (c) 2016, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;; * Redistributions of source code must retain the above copyright
;;;   notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;;   notice, this list of conditions and the following disclaimer in the
;;;   documentation and/or other materials provided with the distribution.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :ltfnp-executive)


(defun open-dishwasher ()
  (let ((model "IAI_kitchen")
        (link "sink_area_dish_washer_door_handle")
        (joint "sink_area_dish_washer_door_joint"))
    (move-to-relative-position
     (tf:make-pose-stamped
      "map" 0.0
      (tf:make-3d-vector 0.5 0.0 0.0)
      (tf:euler->quaternion))
     (tf:make-identity-pose))
    (pr2-manip-pm::execute-move-arm-poses
     :left `(,(tf:make-pose-stamped
               "torso_lift_link" 0.0
               (tf:make-3d-vector 0.45 0.0 -0.05)
               (tf:euler->quaternion :ax (/ pi 2))))
     (mot-man:make-goal-specification :moveit-goal-specification))
    (pr2-manip-pm::open-gripper :left)
    (pr2-manip-pm::execute-move-arm-poses
     :left `(,(tf:make-pose-stamped
               "torso_lift_link" 0.0
               (tf:make-3d-vector 0.58 0.0 -0.05)
               (tf:euler->quaternion :ax (/ pi 2))))
     (mot-man:make-goal-specification
      :moveit-goal-specification))
    (pr2-manip-pm::close-gripper :left)
    (sleep 3)
    (set-joint-limits model joint 0 1.57)
    ;;;(attach-to-joint-object "PR2" "l_wrist_roll_link" model link
    ;;;joint 0.0 1.57)
    
    ;; The gripper should now be wrapped around the handle enough in
    ;; order to pull it open. No need to fixate a joint between them,
    ;; as that will make non-radial movement of the gripper
    ;; impossible.
    
    ;;;---
    ;; TODO: Here, a sequence of opening-motions should be executed,
    ;; similar to how the fridge was opened. 3-4 segments should be
    ;; enough.
    ;;;---
    
    (fixate-joint model joint)
    (pr2-manip-pm::open-gripper :left)
    ;; NOTE/TODO: Actually, its probably better to stand besides the
    ;; dish washer and just pull the door open and let it smash to the
    ;; ground. Everything else is too much of a hassle, takes too
    ;; long, and will probably break a fraction of the simulation
    ;; attempts.
    ))

(defun get-semantic-object (name)
  (first (cram-semantic-map-designators:designator->semantic-map-objects
          (make-designator :object `((:name ,name))))))

(defun get-handle-strategy (handle)
  (cond ((or (string= handle "iai_kitchen_sink_area_left_upper_drawer_handle")
             (string= handle "iai_kitchen_sink_area_left_middle_drawer_handle")
             (string= handle "iai_kitchen_kitchen_island_left_upper_drawer_handle"))
         :linear-pull)
        ((or (string= handle "iai_kitchen_sink_area_dish_washer_door_handle")
             (string= handle "iai_kitchen_fridge_door_handle"))
         :revolute-pull)))

(defun get-handle-base-pose (handle &key (frame "map"))
  (let ((handle-object (get-semantic-object handle)))
    (ensure-pose-stamped
     (cram-semantic-map-utils:pose handle-object)
     :frame frame)))

(defun get-handle-axis (handle)
  (cond ((or (string= handle "iai_kitchen_sink_area_left_upper_drawer_handle")
             (string= handle "iai_kitchen_sink_area_left_middle_drawer_handle"))
         (tf:make-3d-vector -1 0 0))
        ((or (string= handle "iai_kitchen_kitchen_island_left_upper_drawer_handle"))
         (tf:make-3d-vector 1 0 0))
        ((or (string= handle "iai_kitchen_sink_area_dish_washer_door_handle"))
         (tf:make-3d-vector 0 -1 0))
        ((or (string= handle "iai_kitchen_fridge_door_handle"))
         (tf:make-3d-vector 0 0 1))))

(defun get-global-handle-axis (handle &key (frame "map"))
  (let* ((axis (get-handle-axis handle))
         (global-pose (get-handle-base-pose handle :frame frame))
         (transformed-axis-pose
           (cl-transforms:transform-pose
            (tf:make-transform (tf:make-identity-vector)
                               (tf:orientation global-pose))
            (tf:make-pose axis (tf:make-identity-rotation)))))
    (tf:origin transformed-axis-pose)))

(defmethod handle-motion-function (handle (strategy (eql :linear-pull)) &key (limits `(0 0.4)) (offset (tf:make-identity-pose)))
  (let* ((base-handle-pose (get-handle-base-pose handle))
         (transformed-handle-pose
           (tf:pose->pose-stamped
            (tf:frame-id base-handle-pose)
            (tf:stamp base-handle-pose)
            (cl-transforms:transform-pose
             (tf:pose->transform base-handle-pose)
             offset)))
         (axis (get-global-handle-axis handle)))
    (lambda (degree)
      (destructuring-bind (lower upper) limits
        (let ((normalized-degree
                (+ lower (* degree (- upper lower)))))
          (tf:pose->pose-stamped
           (tf:frame-id transformed-handle-pose)
           (tf:stamp transformed-handle-pose)
           (cl-transforms:transform-pose
            (tf:make-transform
             (tf:v* axis normalized-degree)
             (tf:make-identity-rotation))
            transformed-handle-pose)))))))

(defmethod handle-motion-function (handle (strategy (eql :revolute-pull)) &key (limits `(0 ,(/ pi 2))) (offset (tf:make-identity-pose)))
  ;; This, again, is pretty hacky
  (cond ((string= handle "iai_kitchen_sink_area_dish_washer_door_handle")
         ;; TODO: Write this!
         )
        ((string= handle "iai_kitchen_fridge_door_handle")
         (let ((base-position (tf:make-3d-vector 1.03 -0.79 0.985))
               (offset-angle (/ pi 2)) ;; Fix me
               (radius 1.0)) ;; Fix me
           (lambda (degree)
             (tf:make-pose base-position (tf:euler->quaternion)))))))
             ;; (destructuring-bind (lower upper) limits
             ;;   (let ((normalized-degree
             ;;           (+ lower (* degree (- upper lower)))))
             ;;     (tf:make-pose-stamped
             ;;      "map" 0.0
             ;;      (tf:v+ base-position
             ;;             (tf:make-3d-vector
             ;;              (* radius (cos (+ offset-angle normalized-degree)))
             ;;              (* radius (sin (+ offset-angle normalized-degree)))
             ;;              0.0))
             ;;      (tf:euler->quaternion :az normalized-degree)) ;; Fix me; rotate?
             ;;     )))))))

(defmethod handle-motion-function (handle (strategy (eql nil)) &key (offset (tf:make-identity-pose)) limits)
  (let ((strategy (get-handle-strategy handle))
        (limits (handle-limits handle)))
    (handle-motion-function handle strategy :offset offset :limits limits)))

(defun trace-handle-trajectory (handle trace-func &key (limits `(0 1) limitsp) (from-degree 0.0) (to-degree 1.0) (step 0.1) (offset (tf:make-identity-pose)))
  (let* ((motion-function
           (cond ((and limits limitsp) (handle-motion-function
                                        handle nil
                                        :limits limits :offset offset))
                 (t (handle-motion-function
                     handle nil
                     :offset offset))))
         (steps (/ (- to-degree from-degree) step)))
    (loop for i from 0 to steps
          ;;as ttt = (format t "!L!K!K!K!L!KE!LJEO!J    ~a / ~a" i steps)
          as current-degree = (+ from-degree (* i step))
          as current-pose = (funcall motion-function current-degree)
          collect (funcall trace-func i steps current-degree current-pose))))

(defun show-handle-trajectory (handle &key (offset (tf:make-identity-pose)))
  (let* ((trace-func
           (lambda (step steps degree pose-stamped)
             (declare (ignore step steps degree))
             pose-stamped))
         (trace (trace-handle-trajectory handle trace-func :offset offset))
         (markers
           (roslisp:make-msg
            "visualization_msgs/MarkerArray"
            markers
            (map 'vector #'identity
                 (loop for i from 0 below (length trace)
                       as dot-msg = (roslisp:make-msg
                                     "visualization_msgs/Marker"
                                     (frame_id header) "map"
                                     pose (tf:to-msg (tf:pose-stamped->pose
                                                      (nth i trace)))
                                     ns "trace"
                                     id i
                                     type 2
                                     action 0
                                     (x scale) 0.03
                                     (y scale) 0.03
                                     (z scale) 0.03
                                     (r color) (- 1.0 (/ i (length trace)))
                                     (g color) (/ i (length trace))
                                     (b color) 0
                                     (a color) 1)
                       collect dot-msg)))))
    (roslisp:publish
     (roslisp:advertise "/handletrace" "visualization_msgs/MarkerArray")
     markers)))

(defun get-pose-difference (p-base p-ext &key (frame "map"))
  (let ((p-base (ensure-pose-stamped p-base :frame frame))
        (p-ext (ensure-pose-stamped p-ext :frame frame)))
    (tf:make-pose
     (tf:v- (tf:origin p-ext) (tf:origin p-base))
     (tf:orientation p-ext))))

(defun get-current-handle-pose (handle)
  (let ((motion-func (handle-motion-function handle nil)))
    (funcall motion-func (handle-degree handle))))

(defun get-hand-handle-difference (arm handle)
  (let* ((handle-pose (get-current-handle-pose handle));;(get-handle-base-pose handle))
         (wrist-link (ecase arm
                       (:left "l_wrist_roll_link")
                       (:right "r_wrist_roll_link")))
         (pose-diff (get-pose-difference
                     handle-pose
                     (tf:pose->pose-stamped
                      wrist-link 0.0
                      (tf:make-identity-pose)))))
    pose-diff))

(defun move-base-relative (pose)
  (let* ((current-pose (ensure-pose-stamped (get-robot-pose)))
         (transformed-pose
           (tf:pose->pose-stamped
            (tf:frame-id current-pose)
            (tf:stamp current-pose)
            (cl-transforms:transform-pose
             (tf:pose->transform current-pose)
             pose))))
    (at-definite-location
     (make-designator
      :location `((:pose ,transformed-pose))))))

(defun handle-hand-offset-pose (handle &key (offset (tf:make-identity-pose)) (limits `(0 1) limitsp) (degree (handle-degree handle)))
  (let* ((hand-offset (tf:make-pose
                       (tf:make-3d-vector -0.0 0.0 0.0)
                       (tf:euler->quaternion :ax (/ pi 2))))
         ;(offset (cl-transforms:transform-pose (tf:pose->transform offset)
         ;                                      hand-offset))
         (motion-func (cond ((and limits limitsp)
                             (handle-motion-function
                              handle nil
                              :offset offset
                              :limits limits))
                            (t (handle-motion-function
                                handle nil :offset offset))))
         (pose (funcall motion-func degree)))
    pose))

(defun handle-dir-sign (handle)
  -1);(if (< (tf:x (get-handle-axis handle)) 0)
     ; -1
     ; 1))

(defun execute-handle-trace (arm handle &key (from-degree 0.0) (to-degree 1.0) (step 0.1))
  (let* ((close-x 0.4)
         (far-x 0.55);0.8)
         (trace-func
           (lambda (inner-step steps degree pose-stamped)
             (declare (ignore inner-step steps))
             (set-handle-degree handle degree :hold t)
             (look-at-handle-container handle)
             (let ((in-tll (ensure-pose-stamped
                            pose-stamped
                            :frame "torso_lift_link")))
               (cond ((or (and (> step 0.0) (>= (tf:x (tf:origin in-tll)) close-x))
                          (and (< step 0.0) (<= (tf:x (tf:origin in-tll)) far-x)))
                      (move-arm-pose arm in-tll :ignore-collisions t))
                     (t (let ((current-hand-in-tll
                                (ensure-pose-stamped
                                 (tf:pose->pose-stamped
                                  (ecase arm
                                    (:left "l_wrist_roll_link")
                                    (:right "r_wrist_roll_link"))
                                  0.0
                                  (tf:make-identity-pose))
                                 :frame "torso_lift_link")))
                          (cond ((> step 0.0)
                                 (when (> (* (handle-dir-sign handle)
                                             (tf:x (tf:origin current-hand-in-tll)))
                                          (* (handle-dir-sign handle) close-x))
                                   (move-arm-pose
                                    arm
                                    (tf:make-pose-stamped
                                     (tf:frame-id in-tll)
                                     (tf:stamp in-tll)
                                     (tf:make-3d-vector
                                      (+ (tf:x (tf:origin in-tll))
                                         (- (tf:x (tf:origin in-tll)) close-x))
                                      (tf:y (tf:origin in-tll))
                                      (tf:z (tf:origin in-tll)))
                                     (tf:orientation in-tll))
                                    :ignore-collisions t))
                                 (move-base-relative
                                  (tf:make-pose
                                   (tf:make-3d-vector
                                    (* (handle-dir-sign handle)
                                       (- close-x (tf:x (tf:origin in-tll))))
                                    0 0)
                                   (tf:make-identity-rotation))))
                                ((< step 0.0)
                                 (when (< (* (handle-dir-sign handle)
                                             (tf:x (tf:origin current-hand-in-tll)))
                                          (* (handle-dir-sign handle) far-x))
                                   (move-arm-pose
                                    arm
                                    (tf:make-pose-stamped
                                     (tf:frame-id in-tll)
                                     (tf:stamp in-tll)
                                     (tf:make-3d-vector
                                      (+ (tf:x (tf:origin in-tll))
                                         (- (tf:x (tf:origin in-tll)) far-x))
                                      (tf:y (tf:origin in-tll))
                                      (tf:z (tf:origin in-tll)))
                                     (tf:orientation in-tll))
                                    :ignore-collisions t))
                                 (move-base-relative
                                  (tf:make-pose
                                   (tf:make-3d-vector
                                    (* (handle-dir-sign handle)
                                       (- far-x (tf:x (tf:origin in-tll))))
                                    0 0)
                                   (tf:make-identity-rotation)))))))))))
         (hand-diff (get-hand-handle-difference arm handle)))
    (trace-handle-trajectory
     handle trace-func
     :from-degree from-degree
     :to-degree to-degree
     :offset hand-diff
     :step step
     :limits (handle-limits handle))))

(defun handle-open-amount (handle)
  (let ((degree (handle-degree handle))
        (limits (handle-limits handle)))
    (destructuring-bind (lower upper) limits
      (+ lower (* degree (- upper lower))))))

(defun in-front-of-handle-pose (handle)
  (let* ((axis (get-global-handle-axis handle))
         (amount-open (handle-open-amount handle))
         (v-offset (tf:v* axis amount-open)))
    (ensure-pose-stamped
     (cond ((string= handle "iai_kitchen_sink_area_left_upper_drawer_handle")
            (tf:make-pose (tf:v+ (tf:make-3d-vector 0.5 0.6 0.0) v-offset)
                          (tf:euler->quaternion :az 0)))
           ((string= handle "iai_kitchen_sink_area_left_middle_drawer_handle")
            (tf:make-pose (tf:v+ (tf:make-3d-vector 0.5 0.6 0.0) v-offset)
                          (tf:euler->quaternion :az 0)))
           ((string= handle "iai_kitchen_kitchen_island_left_upper_drawer_handle")
            (tf:make-pose (tf:v+ (tf:make-3d-vector 0.0 0.6 0.0) v-offset)
                          (tf:euler->quaternion :az pi)))
           ((string= handle "iai_kitchen_sink_area_dish_washer_door_handle")
            (tf:make-pose (tf:v+ (tf:make-3d-vector 0.5 0.0 0.0) v-offset)
                          (tf:euler->quaternion :az 0)))
           ((string= handle "iai_kitchen_fridge_door_handle")
            (tf:make-pose (tf:v+ (tf:make-3d-vector 0.5 -0.8 0.0) v-offset)
                          (tf:euler->quaternion :az 0)))))))

(defun handle-limits (handle)
  (cond ((string= handle "iai_kitchen_sink_area_left_upper_drawer_handle")
         `(0.0 0.4))
        ((string= handle "iai_kitchen_sink_area_left_middle_drawer_handle")
         `(0.0 0.4))
        ((string= handle "iai_kitchen_kitchen_island_left_upper_drawer_handle")
         `(0.0 0.4))
        ((string= handle "iai_kitchen_sink_area_dish_washer_door_handle")
         `(0.0 ,(/ pi 2)))
        ((string= handle "iai_kitchen_fridge_door_handle")
         `(0.0 ,(/ pi 2)))))

(defun handle-torso-height (handle)
  (cond ((string= handle "iai_kitchen_sink_area_left_upper_drawer_handle")
         0.3)
        ((string= handle "iai_kitchen_sink_area_left_middle_drawer_handle")
         0.2)
        ((string= handle "iai_kitchen_kitchen_island_left_upper_drawer_handle")
         0.2)
        ((string= handle "iai_kitchen_sink_area_dish_washer_door_handle")
         0.3)
        ((string= handle "iai_kitchen_fridge_door_handle")
         0.3)))

(defun handle-orientation-transformation (handle)
  (cond ((string= handle "iai_kitchen_sink_area_left_upper_drawer_handle")
         (tf:euler->quaternion :az 0 :ax (/ pi 2)))
        ((string= handle "iai_kitchen_sink_area_left_middle_drawer_handle")
         (tf:euler->quaternion :az 0 :ax (/ pi 2)))
        ((string= handle "iai_kitchen_kitchen_island_left_upper_drawer_handle")
         (tf:euler->quaternion :az pi :ax (/ pi -2)))
        ((string= handle "iai_kitchen_sink_area_dish_washer_door_handle")
         (tf:euler->quaternion :az 0 :ax (/ pi 2)))
        ((string= handle "iai_kitchen_fridge_door_handle")
         (tf:euler->quaternion))))

(defun go-in-front-of-handle (handle)
  (let ((loc (make-designator
              :location
              `((:pose ,(in-front-of-handle-pose handle))))))
    (at-definite-location loc)))

(defun grasp-handle (arm handle &key double)
  (let* ((handle-base-pose
           (handle-hand-offset-pose
            handle
            :limits (handle-limits handle)))
           ;;(get-handle-base-pose handle))
         (handle-axis (get-handle-axis handle))
         (applied-rotation (cl-transforms:transform-pose
                            (tf:make-transform
                             (tf:make-identity-vector)
                             (handle-orientation-transformation handle))
                            (tf:make-pose (tf:make-identity-vector)
                                          (tf:orientation handle-base-pose)))))
    (labels ((grasp-pose (distance-factor)
               (ensure-pose-stamped
                (tf:make-pose-stamped
                 "map"
                 0.0
                 (tf:v+ (tf:origin handle-base-pose)
                        (ecase (get-handle-strategy handle)
                          (:linear-pull (tf:v* handle-axis distance-factor))
                          ;; This is pretty hacky
                          (:revolute-pull
                           (tf:make-3d-vector 0 0 0))))
                           ;;(tf:v* (tf:make-3d-vector -1 0 0) distance-factor))))
                 (tf:orientation applied-rotation))
                :frame "torso_lift_link")))
      (when double
        (roslisp:publish (roslisp:advertise "/blablabla" "geometry_msgs/PoseStamped")
                         (tf:to-msg (grasp-pose 0.4)))
        (format t "Whack ~a~%" handle-base-pose)
        (move-arm-pose arm (grasp-pose 0.4) :ignore-collisions t))
      (roslisp:publish (roslisp:advertise "/blablabla" "geometry_msgs/PoseStamped")
                       (tf:to-msg (grasp-pose 0.2)))
      (format t "Darn~%")
      (move-arm-pose arm (grasp-pose 0.2) :ignore-collisions t))))

(defun move-arm-relative (arm offset &key ignore-collisions)
  (move-arm-pose
   arm
   (ensure-pose-stamped
    (tf:pose->pose-stamped
     (ecase arm
       (:left "l_wrist_roll_link")
       (:right "r_wrist_roll_link"))
     0.0
     offset)
    :frame "torso_lift_link")
   :ignore-collisions ignore-collisions))

(defun handle-joint (handle)
  (cond ((string= handle "iai_kitchen_sink_area_left_upper_drawer_handle")
         "sink_area_left_upper_drawer_main_joint")
        ((string= handle "iai_kitchen_sink_area_left_middle_drawer_handle")
         "sink_area_left_middle_drawer_main_joint")
        ((string= handle "iai_kitchen_kitchen_island_left_upper_drawer_handle")
         "kitchen_island_left_upper_drawer_main_joint")
        ((string= handle "iai_kitchen_sink_area_dish_washer_door_handle")
         "sink_area_dish_washer_door_joint")
        ((string= handle "iai_kitchen_fridge_door_handle")
         "iai_fridge_door_joint")))

(defun degree->joint-position (handle degree)
  (destructuring-bind (lower upper) (handle-limits handle)
    (+ lower (* degree (- upper lower)))))

(defun handle-model (handle)
  (declare (ignore handle))
  ;; Its always the kitchen for this scenario.
  "IAI_kitchen")

(defun set-handle-degree (handle degree &key hold)
  (set-joint-position (handle-model handle)
                      (handle-joint handle)
                      (degree->joint-position handle degree)
                      :hold hold))

(defun initialize-handle-joint-controller ()
  (let ((handles `("iai_kitchen_sink_area_left_upper_drawer_handle"
                   "iai_kitchen_sink_area_left_middle_drawer_handle"
                   "iai_kitchen_kitchen_island_left_upper_drawer_handle"
                   "iai_kitchen_sink_area_dish_washer_door_handle"
                   "iai_kitchen_fridge_door_handle")))
    (loop for handle in handles
          as joint = (handle-joint handle)
          as model = (handle-model handle)
          as joint-limits = (handle-limits handle)
          do (destructuring-bind (lower upper) joint-limits
               (set-joint-limits model joint lower upper))
             (set-joint-position model joint 0 :hold t)))
  ;; This should secure all loose joints we're not using
  (let ((inactive-joints `("fridge_area_lower_drawer_main_joint"
                           "kitchen_island_left_lower_drawer_main_joint"
                           "kitchen_island_middle_lower_drawer_main_joint"
                           "kitchen_island_middle_upper_drawer_main_joint"
                           "kitchen_island_right_lower_drawer_main_joint"
                           "kitchen_island_right_upper_drawer_main_joint"
                           "oven_area_area_left_drawer_main_joint"
                           "oven_area_area_middle_lower_drawer_main_joint"
                           "oven_area_area_middle_upper_drawer_main_joint"
                           "oven_area_area_right_drawer_main_joint"
                           "oven_area_oven_door_joint"
                           "oven_area_oven_knob_oven_joint"
                           "oven_area_oven_knob_stove_1_joint"
                           "oven_area_oven_knob_stove_2_joint"
                           "oven_area_oven_knob_stove_3_joint"
                           "oven_area_oven_knob_stove_4_joint"
                           "sink_area_left_bottom_drawer_main_joint"
                           "sink_area_trash_drawer_main_joint")))
    (loop for joint in inactive-joints
          do (set-joint-limits "IAI_kitchen" joint 0.0 0.0))))

(defun container-frame-for-handle (handle)
  (cond ((string= handle "iai_kitchen_sink_area_left_upper_drawer_handle")
         "iai_kitchen/sink_area_left_upper_drawer_main")
        ((string= handle "iai_kitchen_sink_area_left_middle_drawer_handle")
         "iai_kitchen/sink_area_left_middle_drawer_main")
        ((string= handle "iai_kitchen_kitchen_island_left_upper_drawer_handle")
         "iai_kitchen/kitchen_island_left_upper_drawer_main")
        ((string= handle "iai_kitchen_sink_area_dish_washer_door_handle")
         "iai_kitchen/sink_area_dish_washer_main")
        ((string= handle "iai_kitchen_fridge_door_handle")
         "iai_kitchen/iai_fridge_main")))

(defun container-center-pose (handle)
  (let* ((frame (container-frame-for-handle handle))
         (axis (get-handle-axis handle))
         (degree (handle-degree handle))
         (amount-open
           (destructuring-bind (lower upper) (handle-limits handle)
             (+ lower (* degree (- upper lower)))))
         (base-pose (ensure-pose-stamped
                     (tf:transform->pose
                      (tf:lookup-transform
                       *transformer*
                       "map" frame)))))
    (ensure-pose-stamped
     (tf:make-pose (tf:v+ (tf:v* axis amount-open)
                          (tf:origin base-pose))
                   (tf:orientation base-pose)))))

(defun look-at-handle-container (handle)
  (let* ((container-frame (container-frame-for-handle handle))
         (container-pose (container-center-pose handle)))
    (look-at (make-designator :location `((:pose ,container-pose))))))

(defun make-inside-container-radial-cost-function (center-pose radius)
  (let* ((center-origin (tf:origin center-pose))
         (center-flat-origin (tf:make-3d-vector (tf:x center-origin)
                                                (tf:y center-origin)
                                                0)))
    (lambda (x y)
      (if (<= (tf:v-dist center-flat-origin
                         (tf:make-3d-vector x y 0))
              radius)
          1.0d0
          0.0d0))))

(defun make-inside-container-radial-height-function (center-pose)
  (let ((z (tf:z (tf:origin center-pose))))
    (lambda (x y)
      (declare (ignore x y))
      (list z))))

(defun container-type (handle)
  (cond ((string= handle "iai_kitchen_kitchen_island_counter_top")
         :countertop)
        ((string= handle "iai_kitchen_sink_area_counter_top")
         :countertop)
        ((string= handle "iai_kitchen_meal_table_counter_top")
         :countertop)
        ((string= handle "iai_kitchen_sink_area_left_upper_drawer_handle")
         :drawer)
        ((string= handle "iai_kitchen_sink_area_left_middle_drawer_handle")
         :drawer)
        ((string= handle "iai_kitchen_kitchen_island_left_upper_drawer_handle")
         :drawer)
        ((string= handle "iai_kitchen_sink_area_dish_washer_door_handle")
         :dishwasher)
        ((string= handle "iai_kitchen_fridge_door_handle")
         :fridge)))

(defun open-handle (arm handle)
  (roslisp:ros-info (open handle) "Go in front")
  (go-in-front-of-handle handle)
  (roslisp:ros-info (open handle) "Move torso")
  (move-torso (handle-torso-height handle))
  (roslisp:ros-info (open handle) "Look at handled container")
  (look-at-handle-container handle)
  (roslisp:ros-info (open handle) "Open gripper")
  (pr2-manip-pm::open-gripper arm)
  (roslisp:ros-info (open handle) "Grasp handle")
  (grasp-handle arm handle :double t)
  (roslisp:ros-info (open handle) "Close gripper")
  (pr2-manip-pm::close-gripper arm)
  (ecase (container-type handle)
    (:drawer ;; Drawers
     (roslisp:ros-info (open handle) "Execute handle trace")
     (execute-handle-trace
      arm handle
      :from-degree (handle-degree handle))
     (roslisp:ros-info (open handle) "Open gripper")
     (pr2-manip-pm::open-gripper arm)
     (roslisp:ros-info (open handle) "Move arm relative")
     (move-arm-relative
      arm (tf:make-pose (tf:make-3d-vector -0.1 0.0 0.0)
                        (tf:make-identity-rotation))
      :ignore-collisions t))
    (:fridge ;; Fridge
     (roslisp:ros-info (open handle) "Open fridge (apply magic sauce)")
     (set-handle-degree handle 1.0 :hold t)
     (roslisp:ros-info (open handle) "Open gripper")
     (pr2-manip-pm::open-gripper arm)
     (roslisp:ros-info (open handle) "Move arm relative")
     (move-arm-relative
      arm (tf:make-pose (tf:make-3d-vector -0.1 0.0 0.0)
                        (tf:make-identity-rotation))
      :ignore-collisions t))
    (:dishwasher ;; Dishwasher
     (roslisp:ros-info (open handle) "Open dishwasher (apply magic sauce)")
     (set-handle-degree handle 1.0 :hold t)
     (roslisp:ros-info (open handle) "Open gripper")
     (pr2-manip-pm::open-gripper arm)
     (roslisp:ros-info (open handle) "Move arm relative")
     (move-arm-relative
      arm (tf:make-pose (tf:make-3d-vector -0.1 0.0 0.0)
                        (tf:make-identity-rotation))
      :ignore-collisions t)
     ))
  (roslisp:ros-info (open handle) "And up")
  (move-arms-up))

(defun close-handle (arm handle)
  (go-in-front-of-handle handle)
  (move-torso (handle-torso-height handle))
  (roslisp:ros-info (open handle) "Look at handled container")
  (look-at-handle-container handle)
  (pr2-manip-pm::open-gripper arm)
  (grasp-handle arm handle)
  (pr2-manip-pm::close-gripper arm)
  (ecase (container-type handle)
    (:drawer ;; Drawers
     (execute-handle-trace
      arm handle
      :from-degree (handle-degree handle)
      :to-degree 0.15
      :step -0.1)
     (pr2-manip-pm::open-gripper arm)
     (move-arm-relative
      arm (tf:make-pose (tf:make-3d-vector -0.2 0.0 0.0)
                        (tf:make-identity-rotation))
      :ignore-collisions t))
    (:fridge
     (roslisp:ros-info (open handle) "Close fridge (apply magic sauce)")
     (set-handle-degree handle 0.0 :hold t)
     (roslisp:ros-info (open handle) "Open gripper")
     (pr2-manip-pm::open-gripper arm)
     (roslisp:ros-info (open handle) "Move arm relative")
     (move-arm-relative
      arm (tf:make-pose (tf:make-3d-vector -0.1 0.0 0.0)
                        (tf:make-identity-rotation))
      :ignore-collisions t)
     ))
  (move-arms-up))

(defun ideal-arm-for-handle (handle)
  (cond ((string= handle "iai_kitchen_sink_area_left_upper_drawer_handle")
         :left)
        ((string= handle "iai_kitchen_sink_area_left_middle_drawer_handle")
         :left)
        ((string= handle "iai_kitchen_kitchen_island_left_upper_drawer_handle")
         :right)
        ((string= handle "iai_kitchen_sink_area_dish_washer_door_handle")
         :left) ;; Needs fixing
        ((string= handle "iai_kitchen_fridge_door_handle")
         :right)))

(defun open-auto-handle (handle)
  (let ((arm (or (context-constraint :open-handle-with-arm)
                 (ideal-arm-for-handle handle))))
    (open-handle arm handle)))

(defun close-auto-handle (handle)
  (let ((arm (ideal-arm-for-handle handle)))
    (close-handle arm handle)))

(defun handle-degree (handle)
  (let* ((model (handle-model handle))
         (joint (handle-joint handle))
         (info (get-joint-information model joint))
         (limits (handle-limits handle)))
    (destructuring-bind (lower upper) limits
      (destructuring-bind (position min max) info
        (declare (ignore min max))
        (/ (- position lower) (- upper lower))))))

(defun open-close (handle)
  (let ((arm (ideal-arm-for-handle handle)))
    (open-handle arm handle)
    (close-handle arm handle))
  (move-arms-up :ignore-collisions t))

(defvar *container-stored-objects* (make-hash-table :test 'equal))

(defun store-object-in-handled-container (object handle)
  ;; Object: (id objclass relative-pose)
  (push object (gethash handle *container-stored-objects*)))

(defun remove-object-from-handled-container (id handle)
  (format t "REMOVING OBJECT '~a' FROM CONTAINER '~a'~%" id handle)
  (setf (gethash handle *container-stored-objects*)
        (remove id (gethash handle *container-stored-objects*)
                :test (lambda (x pair)
                        (string= x (car pair))))))

(defun objects-stored-in-handled-container (handle)
  (gethash handle *container-stored-objects*))

(defun open-handled-storage-container (handle)
  (open-auto-handle handle)
  (dolist (object (objects-stored-in-handled-container handle))
    (destructuring-bind (id objclass relative-pose) object
      (let* ((container-pose (container-center-pose handle))
             (object-pose (ensure-pose-stamped
                           (cl-transforms:transform-pose
                            (tf:pose->transform container-pose)
                            relative-pose))))
        (roslisp:ros-info (ltfnp) "Add gazebo object model '~a'" id)
        (cram-gazebo-utilities::with-physics-paused
          (spawn-class id objclass object-pose)
          (sleep 0.1)
          (attach-object id "link" "ground_plane" "link"))))))

(defun close-handled-storage-container (handle)
  (cram-gazebo-utilities::with-physics-paused
    (dolist (object (objects-stored-in-handled-container handle))
      (destructuring-bind (id objclass relative-pose) object
        (declare (ignore objclass relative-pose))
        (detach-object "ground_plane" "link" id "link")
        (sleep 0.1)
        (roslisp:ros-info (ltfnp) "Remove gazebo object model '~a'" id)
        (roslisp:ros-info (ltfnp) "Result: ~a" (cram-gazebo-utilities::delete-gazebo-model id))
        (cram-moveit::remove-collision-object id))))
  (close-auto-handle handle))

(def-top-level-cram-function open-close-test ()
  (with-process-modules-simulated
    ;; Lights
    (semantic-map-collision-environment::publish-semantic-map-collision-objects)
    (initialize-handle-joint-controller)
    ;; Camera
    (move-arms-up :ignore-collisions t)
    (move-torso 0.3)
    ;; Action!
    ;;(open-close "iai_kitchen_sink_area_left_upper_drawer_handle")
    ;;(open-close "iai_kitchen_sink_area_left_middle_drawer_handle")
    (open-handled-storage-container "iai_kitchen_sink_area_left_upper_drawer_handle")))
    ;;(open-auto-handle "iai_kitchen_sink_area_left_upper_drawer_handle")))
;;(open-close "iai_kitchen_kitchen_island_left_upper_drawer_handle")))

(defun other-hand (hand)
  (if (eql hand :left) :right :left))

(defun allowed-hands-for-location (locname)
  (let ((loctype (container-type locname)))
    (cond ((eql loctype :countertop)
           `(:left :right))
          ((eql loctype :fridge)
           `(:right))
          (t `(,(other-hand (ideal-arm-for-handle locname)))))))

(defun location-closeable (handle)
  (if (eql (container-type handle) :countertop) nil t))

(defun go-to-container-grasping-pose (loc)
  (cond ((string= loc "iai_kitchen_fridge_door_handle")
         (let ((loc-desig
                 (make-designator
                  :location
                  `((:pose ,(tf:make-pose-stamped
                             "map" 0.0
                             (tf:make-3d-vector 0.65 -0.8 0.05)
                             (tf:euler->quaternion :az 0)))))))
                             ;; (tf:make-3d-vector 0.7 -1.1 0.0509)
                             ;; (tf:euler->quaternion :az (/ pi -4))))))))
           (at-definite-location loc-desig)))
        (t nil)))

(defun location-order-for-object (object)
  (let ((objcls (cond (object (desig:desig-prop-value object :type))
                      (t "Object")))
        (proto-locs `("iai_kitchen_kitchen_island_counter_top"
                      "iai_kitchen_sink_area_counter_top"
                      "iai_kitchen_sink_area_left_upper_drawer_handle"
                      "iai_kitchen_fridge_door_handle"
                      "iai_kitchen_sink_area_left_middle_drawer_handle"
                      "iai_kitchen_kitchen_island_left_upper_drawer_handle"
                      "iai_kitchen_sink_area_dish_washer_door_handle"
                      "iai_kitchen_meal_table_counter_top")))
    (labels ((leading (lead-locations)
               (append lead-locations
                       (loop for pl in proto-locs
                             when (not (find pl lead-locations :test #'string=))
                               collect pl))))
      (cond ((string= objcls "Milk")
             (leading `("iai_kitchen_fridge_door_handle")))
            ((or (string= objcls "Fork") (string= objcls "Spoon") (string= objcls "Knife"))
             (leading `("iai_kitchen_sink_area_left_upper_drawer_handle")))
            ((or (string= objcls "RedMetalBowl"))
             (leading `("iai_kitchen_kitchen_island_left_upper_drawer_handle")))
            (t (leading nil))))))

(defun store (objclass relpose place)
  (let ((new-name (concatenate 'string (string-downcase objclass)
                               (write-to-string (find-first-free-index objclass)))))
    (store-object-in-handled-container `(,new-name ,objclass ,relpose) place)))

(defun maybe-store (objclass relpose place)
  (let ((should-store (< (random 100) 100)))
    ;; Fixed right now: 75% probability that objects are *actually*
    ;; stored in the environment
    (when should-store
      (store objclass relpose place))))

(defun countertop-center-pose (name)
  (let ((semobj (first (cram-semantic-map-designators::designator->semantic-map-objects (make-designator :object `((:name ,name)))))))
    (slot-value semobj 'cram-semantic-map-utils::pose)))

(defun find-first-free-index (objclass)
  (let ((highest-idx nil))
    (loop for h being the hash-keys of *container-stored-objects*
          as obj-list = (gethash h *container-stored-objects*)
          do (loop for obj in obj-list
                   do (destructuring-bind (nam cls pos) obj
                        (declare (ignore pos))
                        (when (string= cls objclass)
                          (let* ((num (parse-integer (subseq nam (length objclass)))))
                            (when (or (not highest-idx)
                                      (> num highest-idx))
                              (setf highest-idx num)))))))
    (or (when highest-idx (1+ (or highest-idx 0))) 0)))
