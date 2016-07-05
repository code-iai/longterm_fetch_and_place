;;; Copyright (c) 2015, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
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

;;;
;;; Hints Documentation
;;;

;; The hints system allows customization of the scenario before
;; running it, without actually touching any plan code or changing the
;; knowledge base. This is the second-level, quasi static task
;; parameter knowledge. All hint keys are precedet with a colon (`:')
;; and are therefore Lisp keyword symbols.
;; 
;; Available hints (and their expected values) are:
;; 
;;  * `:world':
;;    - `:simulation': Runs the scenario in a Gazebo
;;      simulation. Prepares the simulated scene, and spawns
;;      appropriate object scenes.
;;    - `:reality': Runs the scenatio on a real PR2 robot.
;;
;;  * `:simulation-type':
;;    - `:simple': Only valid in `:simulation' worlds. Triggers
;;      spawning only one `Kelloggs' object in the third rack level,
;;      easily reachable for the PR2.
;;    - `:default': Spawns random object arrangements all over the
;;      whole rack. This is the default setting.
;;
;;  * `:items-scene-amount':
;;    - Number of objects to spawn in the simulated world (and to
;;      assert into the knowledge base). Defaults to 8.
;;
;;  * `:items-scene-classes':
;;    - Which classes to use for spawning random objects in the
;;      simulated world (and to assert into the knowledge
;;      base). Defaults to all known shopping item classes.


;;;
;;; Shortcuts
;;;

(defun start-scenario-external ()
  (roslisp:ros-info (shopping) "Connecting to ROS")
  (roslisp-utilities:startup-ros)
  (roslisp:ros-info (shopping) "Running Shopping Scenario (simulated, simplified)")
  (run-simulated-simple))

(defun run-simulated (&key hints)
  "Shortcut for running the rack arrangement scenario in simulation."
  (run-rack-arrangement-protected
   :hints (update-hints hints `((:world :simulation)
                                (:perceive-scene-rack-level 2)))))

(defun run-simulated-simple (&key hints)
  "Shortcut for running the rack arrangement scenario in simulation in a simplified version."
  (run-simulated :hints (update-hints
                         hints
                         `((:items-scene-classes ("Lion" "Kelloggs"))
                           (:items-scene-amount 4)
                           (:allowed-rack-levels (1 2))))))

(defun run-simulated-problem (&key hints)
  (run-simulated :hints (update-hints hints `((:version :problem)))))

(defun run-reality (&key hints)
  "Shortcut for running the rack arrangement scenario in reality."
  (run-rack-arrangement-protected
   :hints (update-hints hints `((:world :reality)))))

(defun run-rack-arrangement-protected (&key hints)
  "Runs the rack arrangement scenario in an environment protected by `check-system-settings'."
  (when (check-system-settings :hints hints)
    (run-rack-arrangement :hints hints)))


;;;
;;; Top-Level Plans
;;;

(def-top-level-cram-function run-rack-arrangement (&key hints)
  "Main scenario entry point to start arranging objects. The `hints' (if defined) are forwarded to the target arrangement sampler."
  (prepare-settings)
  (let ((world (get-hint hints :world :reality)))
    (ecase world
      (:simulation
       (with-simulation-process-modules
         (prepare-simulated-scene :hints hints)
         (case (get-hint hints :version :normal)
           (:normal (rack-arrangement :hints hints))
           (:handover (rack-arrangement-handover :hints hints))
           (:problem (solve-arrangement-problem :hints hints))
           (:problem-2 (solve-arrangement-problem-2 :hints hints)))))
      (:reality
       (with-process-modules
         (rack-arrangement :hints hints))))))

(def-cram-function rack-arrangement (&key hints)
  "Performs a rack-tidying up scenario by controlling a PR2 robot that rearranges objects, based on a given target arrangement."
  (let ((rack (first (get-racks))))
    (move-torso)
    (move-arms-away)
    (achieve `(rack-scene-perceived ,rack ,hints))
    (loop for i from 0 to 2 do
      (let* ((objects (get-shopping-objects)))
        (dolist (object objects)
          (achieve `(objects-detected-in-rack ,rack ,object)))
        (dolist (object objects)
          (let ((detected-objects
                  (achieve `(objects-detected-in-rack ,rack ,object))))
            (unless detected-objects
              (cpl:fail 'cram-plan-failures:object-not-found))
            (try-all-objects (detected-object detected-objects)
              (when (desig-prop-value detected-object 'handle)
                (achieve `(object-picked-from-rack ,rack ,detected-object))
                (unless (desig:desig-equal object detected-object)
                  (equate object detected-object))
                (try-forever
                  (multiple-value-bind (rack-level x y)
                      (get-free-position-on-rack rack :hints hints)
                    (let ((elevation (get-rack-level-elevation
                                      (get-rack-on-level rack rack-level))))
                      (move-torso (/ elevation 5.0))
                      (achieve `(object-placed-on-rack
                                 ,object ,(get-rack-on-level rack rack-level)
                                 ,x ,y)))))))))))))

(def-cram-function rack-arrangement-handover (&key hints)
  "Performs a rack-tidying up scenario by controlling a PR2 robot that rearranges objects, based on a given target arrangement."
  (let ((rack (first (get-racks))))
    (move-torso)
    (move-arms-away)
    (achieve `(rack-scene-perceived ,rack ,hints))
      (let* ((objects (get-shopping-objects)))
        (dolist (object objects)
          (achieve `(objects-detected-in-rack ,rack ,object)))
        (let ((object (first objects)))
          (let ((detected-objects
                  (achieve `(objects-detected-in-rack ,rack ,object))))
            (unless detected-objects
              (cpl:fail 'cram-plan-failures:object-not-found))
            (try-all-objects (detected-object detected-objects)
              (when (desig-prop-value detected-object 'handle)
                (achieve `(object-picked-from-rack ,rack ,detected-object))
                (go-in-front-of-rack rack)
                (achieve `(switched-holding-hand ,detected-object))
                )))))))
                ;; (unless (desig:desig-equal object detected-object)
                ;;   (equate object detected-object))
                ;; (try-forever
                ;;   (multiple-value-bind (rack-level x y)
                ;;       (get-free-position-on-rack rack :hints hints)
                ;;     (let ((elevation (get-rack-level-elevation
                ;;                       (get-rack-on-level rack rack-level))))
                ;;       (move-torso (/ elevation 5.0))
                ;;       (achieve `(object-placed-on-rack
                ;;                  ,object ,(get-rack-on-level rack rack-level)
                ;;                  ,x ,y)))))))))))))

(def-cram-function solve-arrangement-problem (&key hints)
  (let* ((problem (assert-planning-problem))
         (target (first problem))
         (sequence (second problem)))
    (let ((rack (first (get-racks))))
      (move-torso)
      (move-arms-away)
      (achieve `(rack-scene-perceived ,rack ,hints))
      (let* ((objects (get-shopping-objects)))
        (dolist (object objects)
          (equate object
                  (first (achieve `(objects-detected-in-rack ,rack ,object)))))
        (labels ((relative-xy (x-index y-index)
                   (let* ((rack-level (get-rack-on-level rack y-index))
                          (rack-width (elt (get-item-dimensions rack-level) 1))
                          (item-space (/ rack-width 4))
                          (offset-h (- (+ (* (- 3 x-index) item-space)
                                          (* item-space 0.5))
                                       (/ rack-width 2))))
                     `(-0.2 ,offset-h)))
                 (pose-on-rack (x-index y-index)
                   (let* ((rack-level (get-rack-on-level rack y-index))
                          (rack-width (elt (get-item-dimensions rack-level) 1))
                          (item-space (/ rack-width 4))
                          (offset-h (- (+ (* (- 3 x-index) item-space)
                                          (* item-space 0.5))
                                       (/ rack-width 2)))
                          (rel-pose (get-rack-level-relative-pose
                                     rack-level
                                     -0.2 offset-h 0.02)))
                     rel-pose))
                 (object-at-rack-position (x-index y-index)
                   (let ((closest-object nil)
                         (smallest-distance 1000.0))
                     (let ((adv (roslisp:advertise "/hhhhhh" "geometry_msgs/PoseStamped")))
                       (loop for object in objects
                             for global-pose = (pose-on-rack x-index y-index)
                             for testing = (roslisp:publish
                                            adv (tf:pose-stamped->msg global-pose))
                             for distance = (tf:v-dist
                                             (tf:origin global-pose)
                                             (tf:origin (reference
                                                         (desig-prop-value
                                                          (current-desig object) 'at))))
                             when (< distance smallest-distance)
                               do (setf smallest-distance distance)
                                  (setf closest-object object)))
                     closest-object)))
          (loop for step in sequence do
            (let ((command (first step))
                  (detail-1 (second step))
                  (detail-2 (third step)))
              (case command
                (:move
                 (let ((x-from (second detail-1))
                       (y-from (first detail-1))
                       (x-to (second detail-2))
                       (y-to (first detail-2)))
                   (let ((obj (object-at-rack-position x-from y-from)))
                     (equate obj (first (achieve `(objects-detected-in-rack
                                                   ,rack ,obj))))
                     (roslisp:ros-info (shopping plans) "Moving from ~a/~a to ~a/~a"
                                       x-from y-from x-to y-to)
                     (achieve `(object-picked-from-rack ,rack ,(current-desig obj)))
                     (let ((elevation (get-rack-level-elevation
                                       (get-rack-on-level rack y-to)))
                           (xy (relative-xy x-to y-to)))
                       (move-torso (/ elevation 5.0))
                       (achieve `(object-placed-on-rack
                                  ,obj ,(get-rack-on-level rack y-to)
                                  ,(first xy) ,(second xy)))))))))))))))

(defun spawn-sim-objs ()
  (remove-all-shopping-items)
  (labels ((set-arrangement-slot (arrangement i j)
             ))
    (let ((arrangement (make-empty-object-arrangement)))
      (setf (aref arrangement 1 1 0) (add-shopping-item "Kelloggs"))
      arrangement)))

(defun perceive-sim-objs ()
  (let* ((ignorable-objects `("pr2" "ground_plane" "shopping_area" "shopping_rack"))
         (perceived-objects
           (with-designators ((obj (object nil))
                              (perceive (action `((desig-props:to desig-props:perceive)
                                                  (desig-props:obj ,obj)))))
             (perform perceive)))
         (filtered-objects
           (remove-if (lambda (subject)
                        (find subject ignorable-objects
                              :test (lambda (subject list-item)
                                      (string= (desig-prop-value
                                                subject 'desig-props:name)
                                               list-item))))
                      perceived-objects)))
    (setf *object-poses* (make-hash-table :test 'equal))
    (setf *perceived-objects* filtered-objects)
    (setf *item-designators* (make-hash-table :test 'equal))
    (dolist (object filtered-objects)
      (let* ((at (desig-prop-value object 'desig-props:at))
             (pose (desig-prop-value at 'desig-props:pose))
             (name (desig-prop-value object 'desig-props:name)))
        (set-item-pose-cached name pose)
        (setf (gethash name *item-designators*) object)))
    filtered-objects))

(def-cram-function solve-arrangement-problem-2 (&key hints)
  (get-shopping-items))

(defun plan-sim-test ()
  (prepare-settings)
  (delete-shopping-items-from-gazebo)
  (remove-all-shopping-items)
  (setf *min-level* 1)
  (setf *max-level* 2)
  (setf *min-zone* 0)
  (setf *max-zone* 3)
  (setf *perceived-objects* nil)
  (setf *item-designators* (make-hash-table :test 'equal))
  (setf *object-poses* (make-hash-table :test 'equal))
  (setf *handover-forbidden* t)
  (setf *gazebo* t)
  (labels ((set-arrangement-slot (arrangement level zone x y theta content)
             (append (remove-if
                      (lambda (subject)
                        (equal (first (first subject)) `(,level ,zone)))
                      arrangement)
                     `((((,level ,zone) (,x ,y ,theta)) ,content))))
           (set-arrangement-slot (arrangement level zone content)
             (set-arrangement-slot arrangement level zone -0.15 0 (/ pi 2) content))
           (instantiate-arrangement (arrangement)
             (mapcar (lambda (entry)
                       (let* ((content (second entry))
                              (instance (add-shopping-item content)))
                         `(,(first entry) ,instance)))
                     arrangement))
           (spawn-arrangement (arrangement)
             (dolist (entry arrangement)
               (let* ((level (first (first (first entry))))
                      (zone (second (first (first entry))))
                      (x (first (second (first entry))))
                      (y (second (second (first entry))))
                      (theta (third (second (first entry))))
                      (content (second entry))
                      (zone-pose
                        (cl-tf2:ensure-pose-stamped-transformed
                         *tf*
                         (make-zone-pose
                          level zone x y
                          :orientation (tf:euler->quaternion :az theta))
                         "map")))
                 (spawn-shopping-item-at-pose content zone-pose)))))
    (macrolet ((setf-arrangement-slot-ex (arrangement level zone x y theta content)
                 `(setf ,arrangement
                        (set-arrangement-slot ,arrangement ,level ,zone ,x ,y ,theta ,content)))
               (setf-arrangement-slot (arrangement level zone content)
                 `(setf ,arrangement
                        (set-arrangement-slot ,arrangement ,level ,zone ,content))))
      (let ((arrangement nil))
        (setf-arrangement-slot arrangement 1 2 "Kelloggs")
        (setf-arrangement-slot arrangement 1 3 "Kelloggs")
        (let ((instantiated-arrangement (instantiate-arrangement arrangement)))
          (spawn-arrangement instantiated-arrangement)
          (perceive-rack-full :back-off nil)
          (dolist (object *perceived-objects*)
            (let* ((object (desig:newest-effective-designator
                            (enrich-object-description object)))
                   (pose (desig-prop-value
                          (desig-prop-value
                           object 'desig-props:at)
                          'desig-props:pose))
                   (item (desig-prop-value
                          object 'desig-props:name)))
              (moveit:set-collision-object-pose
               (desig-prop-value object 'desig-props:name)
               (desig-prop-value
                (desig-prop-value object 'desig-props:at)
                'desig-props:pose))
              (set-item-pose-cached item pose)
              (set-item-designator item object)))
          (let* ((current-state (make-planning-state 0 instantiated-arrangement))
                 (target-state (make-target-state current-state :mode :generic))
                 (solutions (modified-a-star current-state target-state))
                 (action-sequences
                   (mapcar (lambda (solution)
                             (action-sequence (rest solution)))
                           solutions)))
            (move-arms-away)
            (execute-action-step `(:move-base 0))
            (execute-action-step `(:move-torso 2))
            (pr2-manip-pm::open-gripper :left)
            (pr2-manip-pm::open-gripper :right)
            (loop while (or (< (pr2-manip-pm::get-gripper-state :left) 0.07)
                            (< (pr2-manip-pm::get-gripper-state :right) 0.07))
                  do (sleep 0.25))
            (execute-action-sequence (first action-sequences))))))))
