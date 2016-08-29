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

(defun go-to-pose (position orientation)
  (let* ((pose (tf:make-pose-stamped "base_link" 0.0 position orientation))
         (pose-map (tf:transform-pose-stamped *transformer* :pose pose :target-frame "map")))
    (with-designators ((loc :location `((:pose ,pose-map))))
      (at-location (loc)))))

(defun look-at (loc-desig)
  (let ((reference (cram-designators:reference loc-desig)))
    (when reference
      (achieve `(cram-plan-library:looking-at ,reference)))))

(defun move-arm-pose (arm pose)
  (let ((goal-spec (mot-man:make-goal-specification
                    :moveit-goal-specification)))
    (pr2-manip-pm::execute-move-arm-poses arm `(,pose) goal-spec)))

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
   :target-frame "/map"))

(defun init-3d-world (&key (robot t))
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
            (btr:debug-window ?w)
            (btr:assert (btr:object
                         ?w :static-plane floor
                         ((0 0 0) (0 0 0 1))
                         :normal (0 0 1) :constant 0))
            (btr:assert (btr:object
                         ?w :semantic-map kitchen-area
                         (,kitchen-trans ,kitchen-rot)
                         :urdf ,urdf-kitchen)))))
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

(defun go-to-origin ()
  (let* ((origin-pose (cl-tf:make-pose-stamped
                       "map" 0.0
                       (tf:make-identity-vector)
                       (tf:euler->quaternion :az (* PI 1.5))))
         (origin-loc (make-designator :location `((:pose ,origin-pose)))))
  (at-location (origin-loc)
    )))

(defun prepare-settings (&key (simulated t))
  (beliefstate:enable-logging t)
  (when simulated
    (setf beliefstate::*kinect-topic-rgb* "/head_mount_kinect/rgb/image_raw"))
  (setf cram-tf::*tf-default-timeout* 100)
  (setf actionlib::*action-server-timeout* 20)
  (cram-designators:disable-location-validation-function
   'btr-desig::check-ik-solution)
  (cram-designators:disable-location-validation-function
   'btr-desig::validate-designator-solution)
  (gazebo-perception-pm::ignore-object "ground_plane")
  (gazebo-perception-pm::ignore-object "pr2")
  (gazebo-perception-pm::ignore-object "IAI_kitchen")
  (init-3d-world)
  (semantic-map-collision-environment:publish-semantic-map-collision-objects))

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

(defun move-arms-up (&key side)
  (when (or (eql side :left) (not side))
    (move-arm-pose :left (tf:make-pose-stamped
                          "torso_lift_link" 0.0
                          (tf:make-3d-vector 0.1 0.45 0.3)
                          (tf:euler->quaternion :ay (/ pi -2)))))
  (when (or (eql side :right) (not side))
    (move-arm-pose :right (tf:make-pose-stamped
                          "torso_lift_link" 0.0
                          (tf:make-3d-vector 0.1 -0.45 0.3)
                          (tf:euler->quaternion :ay (/ pi -2))))))

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

(defmethod cram-language::on-grasp-object (object-name side)
  (roslisp:ros-info (shopping utils) "Grasp object ~a with side ~a." object-name side)
  (when *simulated*
    (roslisp:call-service "/gazebo/attach"
                          'attache_msgs-srv:Attachment
                          :model1 "pr2"
                          :link1 (case side
                                   (:left "l_wrist_roll_link")
                                   (:right "r_wrist_roll_link"))
                          :model2 object-name
                          :link2 "link")))

(defmethod cram-language::on-putdown-object (object-name side)
  (roslisp:ros-info (shopping utils) "Put down object ~a with side ~a." object-name side)
  (when *simulated*
    (roslisp:call-service "/gazebo/detach"
                          'attache_msgs-srv:Attachment
                          :model1 "pr2"
                          :link1 (case side
                                   (:left "l_wrist_roll_link")
                                   (:right "r_wrist_roll_link"))
                          :model2 object-name
                          :link2 "link")))

(defun enrich-description (description)
  (let ((object-class (cadr (assoc :type description))))
    (desig:update-designator-properties
     description
     (when object-class
       (make-class-description object-class)))))

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
                            (elt arrangements random-index))))))
        (make-random-arrangement-goal "tabletop-goal-")))))
        ;; (make-tabletop-goal
        ;;  "tabletop-goal-0"
        ;;  (mapcar (lambda (object-type)
        ;;            (let ((object (make-designator
        ;;                           :object (obj-desc object-type))))
        ;;              `(,object ,location)))
        ;;          `("RedMetalCup"
        ;;            "RedMetalPlate"
        ;;            "RedMetalBowl"
        ;;            "Milk")))))))

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
                     "iai_kitchen_sink_area_counter_top"
                     "iai_kitchen_kitchen_island_counter_top"))))
      (labels ((spawn (class &optional
                       (translation (tf:make-identity-vector))
                       (orientation (tf:make-identity-rotation)))
                 (spawn-object-relative
                  class (tf:make-pose translation orientation)
                  (tf:pose->pose-stamped
                   "map" 0.0 (tf:make-identity-pose)))))
        (dolist (object objects)
          (let* ((class (desig:desig-prop-value object :type))
                 (random-table (elt tables (random (length tables))))
                 (location (make-designator
                            :location
                            `((:on "CounterTop")
                              (:name ,random-table))))
                 (pose (desig:reference location)))
            (spawn class (tf:origin pose) (tf:orientation pose))))))))
