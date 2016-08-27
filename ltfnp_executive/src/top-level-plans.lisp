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


;;;
;;; Entry Point
;;;

(defun start-scenario (&key (simulated t) (logged nil))
  ;; This function is mainly meant as an entry point for external
  ;; runner scripts (for starting the scenario using launch files,
  ;; etc.)
  (setf *simulated* simulated)
  (unless simulated
    (setf cram-moveit::*needs-ft-fix* t))
  (roslisp-utilities:startup-ros)
  (prepare-settings :simulated simulated)
  (roslisp:ros-info (ltfnp) "Connecting to ROS")
  (when simulated
    (spawn-scene))
  (roslisp:ros-info (ltfnp) "Running Longterm Fetch and Place")
  (move-arms-up)
  (move-torso)
  (prog1
      (longterm-fetch-and-place)
    (when logged
      (beliefstate:extract-files))
    (roslisp:ros-info (ltfnp) "Done with LTFnP")))


;;;
;;; Top-Level Plans
;;;

(def-top-level-cram-function longterm-fetch-and-place ()
  ;; Agenda:
  ;;   1 Determine object to fetch and where to put it (both vaguely)
  ;;   2 Resolve next location at which it could reside (lazily)
  ;;   3 Approach location, and potentially articulate it (opening drawers/doors)
  ;;   4 Detect objects at that location, verifying whether the one in
  ;;     question is present; if not, articulate (close) and go to 2 until exhausted
  ;;   5 Pick up object and articulate (close) container if applicable
  ;;   6 Resolve next location that satisfies the target destination and approach it
  ;;   7 If location could not be reached, go back to 6
  ;;   8 Sample target location for places to put down object and try putting it down;
  ;;     if either fails, go to 6
  (roslisp:ros-info (ltfnp) "Preparation complete, beginning actual scenario")
  (cond (*simulated*
         (with-process-modules-simulated
           (fetch-and-place-instance)))
        (t
         (with-process-modules
           (fetch-and-place-instance)))))

(defun enrich-description (description)
  (let ((object-class (cadr (assoc :type description))))
    (desig:update-designator-properties
     description
     (when object-class
       (make-class-description object-class)))))

(defun make-random-tabletop-goal (target-table)
  (let ((location (make-designator :location
                                   `((:on "CounterTop")
                                     (:name ,target-table)
                                     (:theme :meal-table-setting))))
        (countertop (make-designator :location
                                     `((:on "CounterTop")
                                       ;;(:name "iai_kitchen_sink_area_counter_top")
                                       ))))
    ;; NOTE(winkler): Its just this one goal for now; in time, thi
    ;; swill get increased to more goals. The mechanism after this
    ;; will use this function's return value to determine which
    ;; objects to spawn, and on which table not to put them (using the
    ;; `target-table' parameter).
    (labels ((obj-desc (type)
               (enrich-description
                `((:type ,type) (:at ,countertop)))))
      (make-tabletop-goal
       "tabletop-goal-0"
       (mapcar (lambda (object-type)
                 (let ((object (make-designator
                                :object (obj-desc object-type))))
                   `(,object ,location)))
               `("RedMetalCup"
                 "RedMetalPlate"
                 "RedMetalBowl"
                 "Milk"))))))

(def-cram-function fetch-and-place-instance ()
  (let* ((goal (make-random-tabletop-goal "iai_kitchen_meal_table_counter_top"))
         (the-plan
           (plan
            (make-empty-state)
            goal)))
    (dolist (action the-plan)
      (destructuring-bind (type &rest rest) action
        (case type
          (:fetch (destructuring-bind (object) rest
                    (with-designators ((fetch-action :action `((:to :fetch)
                                                               (:obj ,object))))
                      (perform fetch-action))))
          (:place (destructuring-bind (object location) rest
                    (with-designators ((place-action :action
                                                     `((:to :place)
                                                       (:obj ,object)
                                                       (:at ,location))))
                      (perform place-action)))))))))
