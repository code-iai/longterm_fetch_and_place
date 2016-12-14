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


(defvar *action-client-torso* nil)
(defvar *simulated* nil)


;;;
;;; Add utility functions here
;;;

(defmacro with-process-modules-simulated (&body body)
  `(cpm:with-process-modules-running
       (pr2-manipulation-process-module:pr2-manipulation-process-module
        pr2-navigation-process-module:pr2-navigation-process-module
        point-head-process-module:point-head-process-module
        gazebo-perception-process-module:gazebo-perception-process-module)
     ,@body))

(defmacro with-process-modules (&body body)
  `(cpm:with-process-modules-running
       (pr2-manipulation-process-module:pr2-manipulation-process-module
        pr2-navigation-process-module:pr2-navigation-process-module
        point-head-process-module:point-head-process-module
        robosherlock-process-module:robosherlock-process-module)
     ,@body))

(defmacro with-variance (variables variance &body body)
  `(let ((var-values
           (mapcar (lambda (variable)
                     (destructuring-bind (name var &rest rest) variable
                       (let ((entry (gethash var ,variance))
                             (default-value
                               (when (> (length rest) 0) (first rest))))
                         (cond (entry
                                `(,var ,entry))
                               (default-value
                                `(,var ,default-value))))))
                   ,variables)))
     (when (= (length var-values) (length ,variables))
       (let ,var-values
         ,@body))))

(defun look-at (loc-desig)
  (let ((reference (cram-designators:reference loc-desig)))
    (when reference
      (achieve `(cram-plan-library:looking-at ,reference)))))

(defun move-arm-pose (arm pose &key ignore-collisions)
  (let ((goal-spec (mot-man:make-goal-specification
                    :moveit-goal-specification)))
    (pr2-manip-pm::execute-move-arm-poses arm `(,pose) goal-spec :ignore-collisions ignore-collisions)))

(defun test-move-arm-pose ()
  (move-arm-pose :left (tf:make-pose-stamped
                        "torso_lift_link" 0.0
                        (tf:make-3d-vector 0.1 0.45 0.4)
                        (tf:make-identity-rotation)))
  (move-arm-pose :right (tf:make-pose-stamped
                         "torso_lift_link" 0.0
                         (tf:make-3d-vector 0.1 -0.45 0.4)
                         (tf:make-identity-rotation))))


(defun get-supporting-surfaces ()
  ;; TODO: Add reasoning mechanisms to CRAM that read the new
  ;; properties from the semantic map
  )

(defun get-robot-pose (&optional (frame-id "base_link"))
  (cl-tf:transform-pose
   *transformer*
   :pose (tf:make-pose-stamped
          frame-id
          0.0
          (tf:make-identity-vector)
          (tf:make-identity-rotation))
   :target-frame "map"))

(defun ensure-pose-stamped (pose &key (frame "map") (stamp 0.0) (transform t))
  (let ((pose-stamped
          (let ((type (type-of pose)))
            (ecase type
              (tf:pose-stamped pose)
              (cl-transforms:pose (tf:pose->pose-stamped frame stamp pose))))))
    (tf:copy-pose-stamped
     (cond ((and transform (not (string= (tf:frame-id pose-stamped) frame)))
            (tf:transform-pose-stamped
             *transformer*
             :pose pose-stamped
             :target-frame frame))
           (t pose-stamped))
     :stamp stamp)))

(defun atan2 (y x)
  (cond ((> x 0)
         (atan (/ y x)))
        ((> y 0)
         (- (/ pi 2) (atan (/ x y))))
        ((< y 0)
         (- (/ pi -2) (atan (/ x y))))
        ((< x 0)
         (+ (atan (/ y x)) pi))
        ((and (= x 0) (= y 0))
         (format t "Error: (atan2 x y) undefined for x=0, y=0!"))))

(defun quaternion->euler (q)
  ;; Euler angles: RPY
  (let* ((q0 (tf:w q))
         (q1 (tf:x q))
         (q2 (tf:y q))
         (q3 (tf:z q))
         (q0q1 (* q0 q1))
         (q2q3 (* q2 q3))
         (q1q1 (* q1 q1))
         (q2q2 (* q2 q2))
         (q0q3 (* q0 q3))
         (q1q2 (* q1 q2))
         (q3q3 (* q3 q3))
         (q1q3 (* q1 q3))
         (q0q2 (* q0 q2))
         (tst (+ q1q2 q0q3)))
    (cond ((> tst 0.499)
           `(,(* 2 (atan2 q1 q0)) (/ pi 2) 0.0))
          ((< tst -0.499)
           `(,(* -2 (atan2 q1 q0)) (/ pi -2) 0.0))
          (t `(,(atan2 (* 2 (+ q0q1 q2q3)) (- 1 (* 2 (+ q1q1 q2q2))))
               ,(asin (* 2 (- q0q2 q1q3)))
               ,(atan2 (* 2 (+ q0q3 q1q2)) (- 1 (* 2 (+ q2q2 q3q3)))))))))

(defmacro at-definite-location (location &key (threshold-cartesian 0.01) (threshold-angular 0.05) (retries 25) body)
  `(labels ((distance-2d (p-1 p-2)
              (tf:v-dist (tf:make-3d-vector (tf:x (tf:origin p-1))
                                            (tf:y (tf:origin p-1)) 0.0)
                         (tf:make-3d-vector (tf:x (tf:origin p-2))
                                            (tf:y (tf:origin p-2)) 0.0)))
            (distance-angular-z (p-1 p-2)
              (let ((qd (tf:q* (tf:q-inv (tf:orientation p-1))
                               (tf:orientation p-2))))
                (1- (tf:squared-norm qd)))))
     (let* ((target-pose (desig:reference ,location))
            (current-robot-pose (get-robot-pose))
            (distance-cartesian (distance-2d current-robot-pose
                                             target-pose))
            (distance-angular
              (let* ((euler-robot (quaternion->euler (tf:orientation current-robot-pose)))
                     (euler-target (quaternion->euler (tf:orientation target-pose)))
                     (z-robot (nth 2 euler-robot))
                     (z-target (nth 2 euler-target)))
                (abs (- z-robot z-target)))))
              ;; (distance-angular-z current-robot-pose
              ;;target-pose)))
       (loop while (or (> distance-cartesian ,threshold-cartesian)
                       (> distance-angular ,threshold-angular))
             do (with-failure-handling
                    ((cram-plan-failures:location-not-reached-failure (f)
                       (declare (ignore f))
                       (cpl:retry)))
                  (format t "Distance Cartesian: ~a~%" distance-cartesian)
                  (format t "Distance Angular: ~a~%" distance-angular)
                  (when (or (> distance-cartesian ,threshold-cartesian)
                            (> distance-angular ,threshold-angular))
                    (at-location (,location) ,@body)))
                (setf current-robot-pose (get-robot-pose))
                (setf distance-cartesian (distance-2d current-robot-pose
                                                      target-pose))
                (setf distance-angular (distance-angular-z current-robot-pose
                                                           target-pose))))))

(defun move-to-relative-position (pose offset)
  (let* ((pose-stamped (ensure-pose-stamped pose))
         (transformed-pose-stamped
           (tf:pose->pose-stamped
            (tf:frame-id pose-stamped)
            (tf:stamp pose-stamped)
            (cl-transforms:transform-pose
             (tf:pose->transform pose-stamped)
             offset)))
         (loc (make-designator :location `((:pose ,transformed-pose-stamped)))))
    (at-definite-location loc)))

(defun go-to-pose (position orientation &key (frame "base_link"))
  (let* ((pose (tf:make-pose-stamped frame 0.0 position orientation))
         (pose-map (tf:transform-pose-stamped *transformer* :pose pose :target-frame "map")))
    (with-designators ((loc :location `((:pose ,pose-map))))
      (at-definite-location loc))))

(defun init-3d-world (&key (robot t) (debug-window t))
  (let* ((urdf-robot
           (and robot
                (cl-urdf:parse-urdf
                 (roslisp:get-param "robot_description_lowres"))))
         (urdf-kitchen
           (cl-urdf:parse-urdf
            (roslisp:get-param "kitchen_description")))
         (kitchen-rot-quaternion (tf:euler->quaternion)); :az -3.141))
         (kitchen-rot `(,(tf:x kitchen-rot-quaternion)
                     ,(tf:y kitchen-rot-quaternion)
                     ,(tf:z kitchen-rot-quaternion)
                     ,(tf:w kitchen-rot-quaternion)))
         (kitchen-trans `(0 0 0)));-3.45 -4.35 0)))
    (force-ll
     (cram-prolog:prolog
      `(and (btr:clear-bullet-world)
            (btr:bullet-world ?w)
            (btr:assert (btr:object
                         ?w :static-plane floor
                         ((0 0 0) (0 0 0 1))
                         :normal (0 0 1) :constant 0))
            (btr:assert (btr:object
                         ?w :semantic-map kitchen-area
                         (,kitchen-trans ,kitchen-rot)
                         :urdf ,urdf-kitchen)))))
    (when debug-window
      (cram-prolog:prolog `(and (btr:bullet-world ?w)
                                (btr:debug-window ?w))))
    (when robot
      (force-ll
       (cram-prolog:prolog
        `(and (btr:bullet-world ?w)
              (btr::robot ?robot)
              (btr:assert (btr:object
                           ?w :urdf ?robot ,(get-robot-pose)
                           :urdf ,urdf-robot))))))))

;; (cram-language:def-top-level-cram-function test-perception ()
;;   (with-process-modules
;;     (with-designators ((obj :object `((:name "IAI_kitchen"))))
;;       (cram-plan-library:perceive-object :currently-visible obj))))

(defun make-handles (distance-from-center
                     &key
                       (segments 1)
                       (ax 0.0) (ay 0.0) (az 0.0)
                       (offset-angle 0.0)
                       grasp-type
                       (center-offset
                        (tf:make-identity-vector)))
  (loop for i from 0 below segments
        as current-angle = (+ (* 2 pi (float (/ i segments)))
                              offset-angle)
        as handle-pose = (tf:make-pose
                          (tf:make-3d-vector
                           (+ (* distance-from-center (cos current-angle))
                              (tf:x center-offset))
                           (+ (* distance-from-center (sin current-angle))
                              (tf:y center-offset))
                           (+ 0.0
                              (tf:z center-offset)))
                          (tf:euler->quaternion
                           :ax ax :ay ay :az (+ az current-angle)))
        as handle-object = (make-designator
                            :object
                            (append
                             `((:type :handle)
                               (:at ,(make-designator :location `((:pose ,handle-pose)))))
                             (when grasp-type
                               `((:grasp-type ,grasp-type)))))
        collect handle-object))

(defmacro do-fail (failure-keyword)
  `(cpl:fail ',(intern (subseq (write-to-string failure-keyword) 1)
                       'cram-plan-failures)))

(defmacro when-failure (clauses &body body)
  `(with-failure-handling
       ,(mapcar (lambda (clause)
                  (destructuring-bind (failure-keyword &rest code) clause
                    (let ((failure (intern (subseq (write-to-string failure-keyword) 1)
                                           'cram-plan-failures)))
                      `(,failure (f) (declare (ignore f)) ,@code))))
                clauses)
     ,@body))

(defmacro catch-all (context &body body)
  `(labels ((notif (message)
              (roslisp:ros-warn
               (ltfnp catch-all) "Catch-All (~a): ~a"
               ,context message)))
     (when-failure ((:object-not-found
                     (notif "object-not-found")
                     (cram-language:retry))
                    (:manipulation-pose-unreachable
                     (notif "manipulation-pose-unreachable")
                     (cram-language:retry))
                    (:location-not-reached-failure
                     (notif "location-not-reached-failure")
                     (cram-language:retry))
                    (:manipulation-failed
                     (notif "manipulation-failed")
                     (cram-language:retry)))
       ,@body)))

(defun go-to-origin (&key keep-orientation)
  (let* ((orientation (or (when keep-orientation
                            (let ((robot-pose (get-robot-pose)))
                              (tf:orientation robot-pose)))
                          (tf:euler->quaternion :az (* PI 1.5))))
         (origin-pose (cl-tf:make-pose-stamped
                       "map" 0.0
                       (tf:make-identity-vector)
                       orientation))
         (origin-loc (make-designator :location `((:pose ,origin-pose)))))
    (at-definite-location origin-loc)))

(defun prepare-settings (&key (simulated t) headless variance)
  (setf cram-tf::*tf-default-timeout* 100)
  (setf actionlib::*action-server-timeout* 20)
  (cram-designators:disable-location-validation-function
   'btr-desig::check-ik-solution)
  (cram-designators:disable-location-validation-function
   'btr-desig::validate-designator-solution)
  (when simulated
    (gazebo-perception-pm:set-tf-camera "head_mount_kinect_rgb_link" (* pi 1.5) 2.5)
    (gazebo-perception-pm::ignore-object "ground_plane")
    (gazebo-perception-pm::ignore-object "pr2")
    (gazebo-perception-pm::ignore-object "IAI_kitchen"))
  (init-3d-world :debug-window (not headless))
  (semantic-map-collision-environment:publish-semantic-map-collision-objects)
  (let ((arms (mapcar (lambda (x) (intern (string-upcase x) :keyword))
                      (gethash "allowed_arms" variance))))
    (when arms (setf pr2-manip-pm::*allowed-arms* arms))))

(defun lift-up (pose distance)
  (let* ((origin (cl-transforms:origin pose))
         (lifted-origin (cl-transforms:make-3d-vector
                         (cl-transforms:x origin)
                         (cl-transforms:y origin)
                         (+ (cl-transforms:z origin)
                            distance))))
    (cl-transforms:copy-pose pose :origin lifted-origin)))

(defun spawn-scene ()
  (delete-scene)
  (let ((base-pose (tf:make-pose
                    (tf:make-3d-vector 1.40 0.50 0.90)
                    (tf:euler->quaternion))))
    (flet* ((spawn (class &optional
                    (translation (tf:make-identity-vector))
                    (orientation (tf:make-identity-rotation)))
              (spawn-object-relative class (tf:make-pose translation orientation) base-pose))
            (spawn-series-simplified (series)
              (dolist (item series)
                (destructuring-bind (class (x y) theta) item
                  (spawn class (tf:make-3d-vector x y 0.0) (tf:euler->quaternion :az theta))))))
      (spawn-series-simplified `(("RedMetalPlate" (0.0 0.0) 0.0)
                                 ("RedMetalBowl" (0.05 0.2) 0.0)
                                 ("RedMetalCup" (0.05 -0.2) 0.0)))
      (spawn-series-simplified `(("Milk" (-0.05 0.5) 0.0))))))

(defun relative-destination-pose (object-id)
  (let* ((poses `(("RedMetalPlate0" (0.0 0.0) 0.0)
                  ("RedMetalBowl0" (0.05 0.2) 0.0)
                  ("RedMetalCup0" (0.05 -0.2) 0.0)
                  ("Milk0" (-0.05 0.5) 0.0)))
         (pose-data (assoc object-id poses :test #'equal)))
    (destructuring-bind (object-id (x y) theta) pose-data
      (declare (ignore object-id))
      (tf:make-pose (tf:make-3d-vector x y 0.0)
                    (tf:euler->quaternion :az theta)))))

(defun destination-pose (object-id base-pose)
  (let ((rel-pose (relative-destination-pose object-id)))
    (tf:pose->pose-stamped
     (tf:frame-id base-pose)
     0.0
     (cl-transforms:transform-pose
      (tf:pose->transform base-pose)
      rel-pose))))

(defun delete-scene ()
  (cram-gazebo-utilities:delete-spawned-objects))

(defun object-instance (class index)
  (concatenate 'string class (write-to-string index)))

(defun object-instance-spawned (class index)
  (let ((instance-name (object-instance class index)))
    (cram-gazebo-utilities::model-present instance-name)))

(defun spawn-object-relative (class relative-pose relative-to)
  (let ((index 0))
    (loop while (object-instance-spawned class index) do
      (incf index))
    (let* ((dimensions (get-class-dimensions class))
           (pose (tf:pose->pose-stamped
                 "map" 0.0
                 (lift-up
                  (cl-transforms:transform-pose
                   (cl-transforms:pose->transform relative-to)
                   relative-pose)
                  (/ (third dimensions) 2))))
           (instance-name (object-instance class index)))
      (spawn-class instance-name class pose)
      instance-name)))

(defun move-arms-up (&key side ignore-collisions)
  (when (or (eql side :left) (not side))
    (move-arm-pose :left (tf:make-pose-stamped
                          "torso_lift_link" 0.0
                          (tf:make-3d-vector 0.1 0.45 0.3)
                          (tf:euler->quaternion :ay (/ pi -2)))
                   :ignore-collisions ignore-collisions))
  (when (or (eql side :right) (not side))
    (move-arm-pose :right (tf:make-pose-stamped
                          "torso_lift_link" 0.0
                          (tf:make-3d-vector 0.1 -0.45 0.3)
                          (tf:euler->quaternion :ay (/ pi -2)))
                   :ignore-collisions ignore-collisions)))

(defun move-torso (&optional (position 0.3))
  ;; Hack
  (setf *action-client-torso* nil)
  (let* ((action-client (actionlib:make-action-client
                         "/torso_controller/position_joint_action"
                         "pr2_controllers_msgs/SingleJointPositionAction"))
         (goal (actionlib:make-action-goal
                   action-client
                 position position)))
    (actionlib:wait-for-server action-client)
    (actionlib:send-goal-and-wait
     action-client goal
     :result-timeout 30.0
     :exec-timeout 30.0)))

(defun attach-object (model-1 link-1 model-2 link-2)
  (roslisp:call-service "/gazebo/attach"
                        'attache_msgs-srv:Attachment
                        :model1 model-1
                        :link1 link-1
                        :model2 model-2
                        :link2 link-2))

(defun detach-object (model-1 link-1 model-2 link-2)
  (roslisp:call-service "/gazebo/detach"
                        'attache_msgs-srv:Attachment
                        :model1 model-1
                        :link1 link-1
                        :model2 model-2
                        :link2 link-2))

(defmethod cram-language::on-grasp-object (object-name side)
  (roslisp:ros-info (shopping utils) "Grasp object ~a with side ~a." object-name side)
  (when *simulated*
    (cram-gazebo-utilities::with-physics-paused
      (detach-object "ground_plane" "link" object-name "link")
      (sleep 0.1)
      (attach-object "pr2"
                     (case side
                       (:left "l_wrist_roll_link")
                       (:right "r_wrist_roll_link"))
                     object-name
                     "link"))))

(defmethod cram-language::on-putdown-object (object-name side)
  (roslisp:ros-info (shopping utils) "Put down object ~a with side ~a." object-name side)
  (when *simulated*
    (cram-gazebo-utilities::with-physics-paused
      (attach-object object-name "link" "ground_plane" "link")
      (sleep 0.1)
      (detach-object "pr2"
                     (case side
                       (:left "l_wrist_roll_link")
                       (:right "r_wrist_roll_link"))
                     object-name
                     "link"))))

(defun set-joint-limits (model joint lower upper)
  (roslisp:call-service "/gazebo/joint_set_limits"
                        'attache_msgs-srv:JointSetLimits
                        :model model
                        :joint joint
                        :lower lower
                        :upper upper))

(defun set-joint-position (model joint position &key hold)
  (roslisp:call-service "/gazebo/joint_control"
                        'attache_msgs-srv:JointControl
                        :model model
                        :joint joint
                        :position position
                        :hold_position (not (not hold))))

(defun attach-to-joint-object (source-model source-link joint-model joint-link joint lower-limit upper-limit)
  (when *simulated*
    (roslisp:ros-info (ltfnp utils) "Attaching '~a.~a' to joint model '~a.~a' (joint '~a'), using limits [~a, ~a]" source-model source-link joint-model joint-link joint lower-limit upper-limit)
    (roslisp:call-service "/gazebo/attach"
                          'attache_msgs-srv:Attachment
                          :model1 source-model
                          :link1 source-link
                          :model2 joint-model
                          :link2 joint-link)
    (set-joint-limits joint-model joint lower-limit upper-limit)))

(defun get-joint-information (model joint)
  (let ((result (roslisp:call-service "/gazebo/joint_information"
                                      'attache_msgs-srv:JointInformation
                                      :model model
                                      :joint joint)))
    (with-fields (success position min max) result
      (when success
        `(,position ,min ,max)))))

(defun fixate-joint (model joint)
  (let ((info (get-joint-information model joint)))
    (when info
      (destructuring-bind (position lower upper) info
        (declare (ignore lower upper))
        (set-joint-limits model joint position position)))))

(defun detach-from-joint-object (source-model source-link joint-model joint-link joint)
  (when *simulated*
    (roslisp:ros-info (ltfnp utils) "Detaching '~a.~a' from joint model '~a.~a' (joint '~a'), fixing limits to current state" source-model source-link joint-model joint-link joint)
    (let ((info (get-joint-information joint-model joint)))
      (cond (info
             (destructuring-bind (position lower upper) info
               (declare (ignore lower upper))
               (set-joint-limits joint-model joint position position)
               (roslisp:call-service "/gazebo/detach"
                                     'attache_msgs-srv:Attachment
                                     :model1 source-model
                                     :link1 source-link
                                     :model2 joint-model
                                     :link2 joint-link)))
            (t (roslisp:ros-warn (ltfnp info) "Failed: No such joint"))))))

(defun enrich-description (description)
  (let ((object-class (cadr (assoc :type description))))
    (desig:update-designator-properties
     description
     (when object-class
       (make-class-description object-class)))))

(defun make-fixed-tabletop-goal (objects+poses source)
  (let ((countertop (make-designator
                     :location
                     `((:on "CounterTop")
                       (:name ,source)))))
    (labels ((obj-desc (type)
               (enrich-description
                `((:type ,type) (:at ,countertop)))))
      (make-tabletop-goal
       "fixed-tabletop-goal"
       (mapcar (lambda (object+pose)
                 (destructuring-bind (object pose) object+pose
                   `(,(make-designator :object (obj-desc object))
                     ,(make-designator :location `((:pose ,pose))))))
               objects+poses)))))

(defun make-random-tabletop-goal (target-table &key objects source)
  (let ((location (make-designator :location
                                   `((:on "CounterTop")
                                     (:name ,target-table)
                                     (:theme :meal-table-setting))))
        (countertop (make-designator :location
                                     (append
                                      `((:on "CounterTop"))
                                      (when source
                                        `((:name ,source)))))))
    (let ((arrangements
            `((("RedMetalPlate" ,(tf:make-pose-stamped
                                  "map" 0.0
                                  (tf:make-3d-vector -1.0 -0.9 0.76)
                                  (tf:euler->quaternion :az (/ pi -2))))
               ("RedMetalCup" ,(tf:make-pose-stamped
                                "map" 0.0
                                (tf:make-3d-vector -0.85 -1.0 0.76)
                                (tf:euler->quaternion :az (/ pi -2))))
               ("RedMetalBowl" ,(tf:make-pose-stamped
                                 "map" 0.0
                                 (tf:make-3d-vector -1.65 -0.9 0.76)
                                 (tf:euler->quaternion :az (/ pi -2))))
               ("Milk" ,(tf:make-pose-stamped
                         "map" 0.0
                         (tf:make-3d-vector -1.4 -0.9 0.76)
                         (tf:euler->quaternion :az (/ pi -2)))))
              (("RedMetalPlate" ,(tf:make-pose-stamped
                                  "map" 0.0
                                  (tf:make-3d-vector -1.1 -0.9 0.76)
                                  (tf:euler->quaternion :az (/ pi -2))))
               ("RedMetalCup" ,(tf:make-pose-stamped
                                "map" 0.0
                                (tf:make-3d-vector -0.95 -1.0 0.76)
                                (tf:euler->quaternion :az (/ pi -2))))
               ("Milk" ,(tf:make-pose-stamped
                         "map" 0.0
                         (tf:make-3d-vector -1.3 -0.9 0.76)
                         (tf:euler->quaternion :az (/ pi -2)))))
              (("RedMetalPlate" ,(tf:make-pose-stamped
                                  "map" 0.0
                                  (tf:make-3d-vector -1.1 -0.9 0.76)
                                  (tf:euler->quaternion :az (/ pi -2))))
               ("RedMetalCup" ,(tf:make-pose-stamped
                                "map" 0.0
                                (tf:make-3d-vector -0.95 -1.0 0.76)
                                (tf:euler->quaternion :az (/ pi -2))))
               ("RedMetalPlate" ,(tf:make-pose-stamped
                                  "map" 0.0
                                  (tf:make-3d-vector -1.4 -0.9 0.76)
                                  (tf:euler->quaternion :az (/ pi -2))))
               ("RedMetalCup" ,(tf:make-pose-stamped
                                "map" 0.0
                                (tf:make-3d-vector -1.25 -1.0 0.76)
                                (tf:euler->quaternion :az (/ pi -2))))))))
      (labels ((obj-desc (type)
                 (enrich-description
                  `((:type ,type) (:at ,countertop))))
               (make-random-arrangement-goal (name-prefix)
                 (let ((random-index (random (length arrangements))))
                   (make-tabletop-goal
                    (concatenate 'string name-prefix (write-to-string random-index))
                    (mapcar (lambda (object+pose)
                              (destructuring-bind (object pose) object+pose
                                `(,(make-designator :object (obj-desc object))
                                  ,(make-designator :location `((:pose ,pose))))))
                            (elt arrangements random-index)))))
               (make-fixed-arrangement-goal (objects)
                 (make-tabletop-goal
                  "fixed-tabletop-goal"
                  (mapcar (lambda (object)
                            `(,(make-designator :object (obj-desc object))
                              ,(make-designator :location
                                                `((:on "CounterTop")
                                                  (:name ,target-table)
                                                  (:theme :meal-table-setting)))))
                          objects))))
        (make-random-arrangement-goal "tabletop-goal-")
        (when (and objects)
          (make-fixed-arrangement-goal objects))))))

(defun spawn-goal-objects (goal table)
  (when *simulated* ;; This is only necessary when simulating
    (let ((objects
            (loop for precondition in (slot-value goal 'preconditions)
             as object = (destructuring-bind (type &rest rest)
                             precondition
                           (when (eql type :object-location)
                             (first rest)))
             when object
               collect object))
          (tables (remove-if
                   (lambda (x) (string= x table))
                   `("iai_kitchen_meal_table_counter_top"
                     "iai_kitchen_sink_area_counter_top"))))
                     ;;"iai_kitchen_kitchen_island_counter_top"))))
      (labels ((spawn (class &optional
                       (translation (tf:make-identity-vector))
                       (orientation (tf:make-identity-rotation)))
                 (spawn-object-relative
                  class (tf:make-pose translation orientation)
                  (tf:pose->pose-stamped
                   "map" 0.0 (tf:make-identity-pose)))))
        (let ((spawn-poses nil))
          (labels ((is-pose-close-to-spawned-object (pose)
                     (let ((threshold-distance 0.2))
                       (block check
                         (dolist (spawn-pose spawn-poses)
                           (when (<= (tf:v-dist (tf:origin spawn-pose)
                                                (tf:origin pose))
                                     threshold-distance)
                             (return-from check t))))))
                   (get-free-spawn-pose (location)
                     (let ((pose (desig:reference location)))
                       (loop while (and (is-pose-close-to-spawned-object pose)
                                        location)
                             do (setf location (desig:next-solution location))
                                (when location
                                  (setf pose (desig:reference location))))
                       ;; If all else fails, use the last pose we
                       ;; got. This is not optimal, but works for most
                       ;; situations for now.
                       (push pose spawn-poses)
                       pose)))
            (dolist (object objects)
              (let* ((class (desig:desig-prop-value object :type))
                     (random-table (elt tables (random (length tables))))
                     (location (make-designator
                                :location
                                `((:on "CounterTop")
                                  (:name ,random-table))))
                     (pose (get-free-spawn-pose location)))
                (spawn class (tf:origin pose) (tf:orientation pose))))))))))

(defun extract-semantic-map-ground (filepath)
  (let* ((map (sem-map-utils:get-semantic-map))
         (parts (slot-value map 'cram-semantic-map-utils::parts))
         (datasets
           (loop for part-name being the hash-key of parts
                 as part = (gethash part-name parts)
                 when (eql (type-of part) 'CRAM-SEMANTIC-MAP-UTILS:SEMANTIC-MAP-GEOM)
                   collect
                   (let* ((pose (slot-value part 'cram-semantic-map-utils::pose))
                          (dimensions (slot-value part 'cram-semantic-map-utils::dimensions))
                          (width (coerce (tf:x dimensions) 'float))
                          (height (coerce (tf:y dimensions) 'float))
                          (x (coerce (tf:x (tf:origin pose)) 'float))
                          (y (coerce (tf:y (tf:origin pose)) 'float))
                          (theta (coerce 0 'float))) ;; Fix this
                     (format nil "{\"name\": \"~a\", \"width\": ~a, \"height\": ~a, \"x\": ~a, \"y\": ~a, \"theta\": ~a}" part-name width height x y theta)))))
    (with-open-file (stream filepath :direction :output)
      (dolist (line datasets)
        (format stream "~a~%" line)))))

(defun current-arm-pose (side &key (relative-to-frame "torso_lift_link"))
  (let ((frame (ecase side
                 (:left "l_wrist_roll_link")
                 (:right "r_wrist_roll_link"))))
    (when (tf:wait-for-transform
           *transformer*
           :timeout 0 :time 0
           :source-frame frame
           :target-frame relative-to-frame)
      (tf:transform-pose
       *transformer*
       :pose (tf:pose->pose-stamped
              frame 0.0
              (tf:make-identity-pose))
       :target-frame relative-to-frame))))

(defvar *stored-poses* (make-hash-table :test 'equal))

(defun store-current-arm-pose (side name)
  (setf (gethash name *stored-poses*)
        `(,side ,(current-arm-pose side))))

(defun go-to-stored-pose (side name)
  (let ((pose (gethash name *stored-poses*)))
    (assert pose)
    (move-arm-pose side (elt pose 1))))

(defun the-awesome-mockup-plan ()
  (move-arms-up :side :left)
  (sleep 3)
  (marker-sequence)
  (go-to-stored-pose :left "pregrasp")
  (go-to-stored-pose :left "grasp")
  (pr2-manip-pm::close-gripper :left)
  (go-to-stored-pose :left "lift")
  (go-to-stored-pose :left "carry")
  (sleep 2)
  (go-to-stored-pose :left "lift")
  (go-to-stored-pose :left "grasp")
  (pr2-manip-pm::open-gripper :left)
  (go-to-stored-pose :left "pregrasp"))

(defun make-arrow-marker (pose index color)
  (let* ((dist 0.3)
         (angle-rad (+ (* (/ (* (/ index 8) 360.0) 180.0) 3.1415) 0.5))
         (origin (tf:origin pose))
         (pose (tf:make-pose
                (tf:v+ origin
                       (tf:make-3d-vector
                        (* dist (cos angle-rad))
                        (* dist (sin angle-rad))
                        0.0))
                (tf:euler->quaternion
                 :az
                 (+ pi angle-rad)))))
    (roslisp:make-msg
     "visualization_msgs/Marker"
     (frame_id header) "map"
     pose (tf:to-msg pose)
     ns "arrows"
     id index
     type 0
     action 0
     (x scale) 0.2
     (y scale) 0.04
     (z scale) 0.04
     (r color) (first color)
     (g color) (second color)
     (b color) (third color)
     (a color) (fourth color))))

(defun make-object-marker (pose &key remove)
  (roslisp:make-msg
   "visualization_msgs/Marker"
   (frame_id header) "map"
   pose (tf:to-msg pose)
   ns "object"
   id 0
   type 3
   action (if remove 2 0)
   (x scale) 0.1
   (y scale) 0.1
   (z scale) 0.1
   (r color) 1.0
   (g color) 1.0
   (b color) 0.0
   (a color) 1.0))

(defun make-object-text (pose text &key remove)
  (roslisp:make-msg
   "visualization_msgs/Marker"
   (frame_id header) "map"
   pose (tf:to-msg pose)
   ns "text"
   id 1
   type 9
   action (if remove 2 0)
   (x scale) 0.1
   (y scale) 0.1
   (z scale) 0.1
   (r color) 1.0
   (g color) 1.0
   (b color) 0.0
   (a color) (if remove 0.0 1.0)
   text text))

(defun place-search-area-marker (&key remove)
  (let* ((obj-pose
           (tf:make-pose
            (tf:make-3d-vector -1.15 -1.25 0.72)
            (tf:euler->quaternion :az 0.0)))
         (msgs
           (roslisp:make-msg
            "visualization_msgs/MarkerArray"
            :markers
            (vector
             (roslisp:make-msg
              "visualization_msgs/Marker"
              (frame_id header) "map"
              pose (tf:to-msg obj-pose)
              ns "searcharea"
              id 0
              type 1
              action (if remove 2 0)
              (x scale) 1.0
              (y scale) 1.0
              (z scale) 0.1
              (r color) 0.0
              (g color) 1.0
              (b color) 0.0
              (a color) 0.5))))
         (pub (roslisp:advertise
               "/mockmarkers"
               "visualization_msgs/MarkerArray")))
    (roslisp:publish pub msgs)))

(defun place-arrow-markers (light-up)
  (let* ((obj-pose
           (tf:make-pose
            (tf:make-3d-vector -1.15 -1.19 0.80)
            (tf:euler->quaternion :az 0.0)))
         (marker-msgs
          (roslisp:make-msg
           "visualization_msgs/MarkerArray"
           :markers
           (map
            'vector #'identity
            (loop for i from 0 below 8
                  as color = (case (elt light-up i)
                               (0 `(0.8 0.8 0.8 1.0))
                               (1 `(0.0 1.0 0.0 1.0))
                               (2 `(1.0 0.0 0.0 0.25))
                               (3 `(0.0 0.0 0.0 0.0)))
                  collect
                  (make-arrow-marker obj-pose i color)))))
         (pub (roslisp:advertise
               "/mockmarkers"
               "visualization_msgs/MarkerArray")))
    (roslisp:publish pub marker-msgs)))

(defun place-cylinder-marker (&key text remove)
  (let* ((obj-pose
           (tf:make-pose
            (tf:make-3d-vector -1.15 -1.19 0.80)
            (tf:euler->quaternion :az 0.0)))
         (obj-text-pose
           (tf:make-pose
            (tf:make-3d-vector -1.15 -1.25 1.0)
            (tf:euler->quaternion :az 0.0)))
         (arrows-poses)
         (marker-msgs
          (roslisp:make-msg
           "visualization_msgs/MarkerArray"
           :markers
           (map 'vector #'identity
                (append
                 (when text
                   `(,(make-object-text obj-text-pose text :remove remove)))
                 `(,(make-object-marker obj-pose :remove remove))))))
         (pub (roslisp:advertise
               "/mockmarkers"
               "visualization_msgs/MarkerArray")))
    (roslisp:publish pub marker-msgs)))

(defun marker-sequence ()
  (place-search-area-marker)
  (sleep 3)
  (place-search-area-marker :remove t)
  (place-cylinder-marker)
  (sleep 1)
  (place-cylinder-marker :text "obj0 (cylinder, cup)")
  (sleep 2)
  ;;(place-cylinder-marker :text " "))
  ;; Show them
  (place-arrow-markers `(3 3 3 3 3 3 3 3))
  (sleep 0.1)
  (place-arrow-markers `(0 3 3 3 3 3 3 3))
  (sleep 0.1)
  (place-arrow-markers `(0 0 3 3 3 3 3 3))
  (sleep 0.1)
  (place-arrow-markers `(0 0 0 3 3 3 3 3))
  (sleep 0.1)
  (place-arrow-markers `(0 0 0 0 3 3 3 3))
  (sleep 0.1)
  (place-arrow-markers `(0 0 0 0 0 3 3 3))
  (sleep 0.1)
  (place-arrow-markers `(0 0 0 0 0 0 3 3))
  (sleep 0.1)
  (place-arrow-markers `(0 0 0 0 0 0 0 3))
  (sleep 0.1)
  (place-arrow-markers `(0 0 0 0 0 0 0 0))
  (sleep 1)
  ;; Mark them
  (place-arrow-markers `(0 0 0 0 2 0 0 0))
  (sleep 0.1)
  (place-arrow-markers `(0 0 0 0 2 2 0 0))
  (sleep 0.1)
  (place-arrow-markers `(0 0 0 0 2 2 2 0))
  (sleep 0.1)
  (place-arrow-markers `(0 0 0 0 2 2 2 2))
  (sleep 0.1)
  (place-arrow-markers `(1 0 0 0 2 2 2 2))
  (sleep 0.1)
  (place-arrow-markers `(1 2 0 0 2 2 2 2))
  (sleep 0.1)
  (place-arrow-markers `(1 2 2 0 2 2 2 2))
  (sleep 0.1)
  (place-arrow-markers `(1 2 2 2 2 2 2 2))
  (sleep 1)
  (place-arrow-markers `(1 3 3 3 3 3 3 3))
  (sleep 3)
  (place-arrow-markers `(3 3 3 3 3 3 3 3))
  (place-cylinder-marker :remove t))

(defvar *contextual-constraints* (make-hash-table))

(defun get-contextual-constraints ()
  *contextual-constraints*)

(defun set-contextual-constraints (constraints)
  (setf *contextual-constraints* constraints))

(defun context-constraint (constraint)
  (gethash constraint *contextual-constraints*))

(defun context-constraints (constraints)
  (mapcar (lambda (constraint)
            `(,constraint ,(context-constraint constraint)))
          constraints))

(defmacro with-context (constraints &body code)
  `(let ((old-constraints (get-contextual-constraints)))
     (dolist (constraint ,constraints)
      (destructuring-bind (name value) constraint
        (setf (gethash name ,*contextual-constraints*) value)))
     (unwind-protect (progn ,@code)
       (set-contextual-constraints old-constraints))))
