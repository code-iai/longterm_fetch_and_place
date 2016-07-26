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

;;;
;;; Add utility functions here
;;;

(defmacro with-process-modules (&body body)
  `(cpm:with-process-modules-running
       (pr2-manipulation-process-module:pr2-manipulation-process-module
        pr2-navigation-process-module:pr2-navigation-process-module
        point-head-process-module:point-head-process-module
        ;robosherlock-process-module:robosherlock-process-module
        gazebo-perception-process-module:gazebo-perception-process-module)
     ,@body))

(defun go-to-pose (position orientation)
  (let ((pose (tf:make-pose-stamped "/base_link" 0.0 position orientation)))
    (with-designators ((loc :location `((:pose ,pose))))
      (at-location (loc)))))

(defun look-at (loc-desig)
  (let ((reference (cram-designators:reference loc-desig)))
    (when reference
      (achieve `(cram-plan-library:looking-at ,reference)))))

(defun move-arm-pose (arm pose)
  (pr2-manip-pm::execute-move-arm-pose arm pose))

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

(defun init-3d-world ()
  (let* ((urdf-robot
           (cl-urdf:parse-urdf
            (roslisp:get-param "robot_description_lowres")))
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
            (btr::robot ?robot)
            (btr:assert (btr:object
                         ?w :urdf ?robot ,(get-robot-pose)
                         :urdf ,urdf-robot))
            (btr:assert (btr:object
                         ?w :semantic-map kitchen-area
                         (,kitchen-trans ,kitchen-rot)
                         :urdf ,urdf-kitchen))
            (btr:debug-window ?w))))))

(cram-language:def-top-level-cram-function test-perception ()
  (with-process-modules
    (with-designators ((obj :object `((:name "IAI_kitchen"))))
      (cram-plan-library:perceive-object :currently-visible obj))))

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

(defun go-to-origin ()
  (let* ((origin-pose (cl-tf:make-pose-stamped
                       "map" 0.0
                       (tf:make-identity-vector)
                       (tf:euler->quaternion :az (* PI 1.5))))
         (origin-loc (make-designator :location `((:pose ,origin-pose)))))
  (at-location (origin-loc)
    )))

(defun prepare-settings ()
  (setf cram-tf::*tf-default-timeout* 100)
  (setf actionlib::*action-server-timeout* 20)
  (cram-designators:disable-location-validation-function
   'btr-desig::check-ik-solution)
  (cram-designators:disable-location-validation-function
   'btr-desig::validate-designator-solution)
  (gazebo-perception-pm::ignore-object "ground_plane")
  (gazebo-perception-pm::ignore-object "pr2")
  (gazebo-perception-pm::ignore-object "IAI_kitchen")
  (init-3d-world))

(defun spawn-scene () ;; (instantiate-object "Milk")
  (let ((base-pose (pose-on-countertop (first (get-countertops)))))
    (spawn-object-relative "Milk" (tf:make-identity-pose) base-pose)))

(defun object-instance (class index)
  (concatenate 'string class (write-to-string index)))

(defun object-instance-spawned (class index)
  (let ((instance-name (object-instance class index)))
    (cram-gazebo-utilities::model-present instance-name)))

(defun spawn-object-relative (class relative-pose relative-to)
  (let ((index 0))
    (loop while (object-instance-spawned class index) do
      (incf index))
    (let ((pose (tf:pose->pose-stamped
                 "map" 0.0
                 (cl-transforms:transform-pose
                  (cl-transforms:pose->transform relative-to)
                  relative-pose)))
          (instance-name (object-instance class index)))
      (spawn-class instance-name class pose)
      instance-name)))

(defun move-arms-up (&key allowed-collision-objects side ignore-collisions)
  (when (or (eql side :left) (not side))
    (pr2-manip-pm::execute-move-arm-pose
     :left
     (tf:make-pose-stamped
      "torso_lift_link" (roslisp:ros-time)
      (tf:make-3d-vector 0.1 0.45 0.3)
      (tf:euler->quaternion :ay (/ pi -2)))
     :ignore-collisions ignore-collisions
     :allowed-collision-objects allowed-collision-objects))
  (when (or (eql side :right) (not side))
    (pr2-manip-pm::execute-move-arm-pose
     :right
     (tf:make-pose-stamped
      "torso_lift_link" (roslisp:ros-time)
      (tf:make-3d-vector 0.1 -0.45 0.3)
      (tf:euler->quaternion :ay (/ pi -2)))
     :ignore-collisions ignore-collisions
     :allowed-collision-objects allowed-collision-objects)))

(defun move-torso (&optional (position 0.3))
  (let* ((action-client (or *action-client-torso*
                            (setf *action-client-torso*
                                  (actionlib:make-action-client
                                   "/torso_controller/position_joint_action"
                                   "pr2_controllers_msgs/SingleJointPositionAction"))))
         (goal (actionlib:make-action-goal
                   action-client
                 position position)))
    (setf *action-client-torso* action-client)
    (actionlib:send-goal-and-wait
     action-client goal
     :result-timeout 30.0
     :exec-timeout 30.0)))
