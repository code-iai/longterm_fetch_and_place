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
  (when simulated
    (setf *gazebo* t))
  (roslisp-utilities:startup-ros)
  (prepare-settings)
  (roslisp:ros-info (ltfnp) "Connecting to ROS")
  (spawn-scene)
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
  (with-process-modules
    (with-designators ((loc-on-sink
                        :location `((:on "CounterTop")
                                    ;;(:name "iai_kitchen_sink_area_counter_top")
                                    )))
      (let ((locations `(,loc-on-sink)))
        (labels ((random-source-location ()
                   (elt locations (random (length locations)))))
          (with-designators ((cup :object `((:type "RedMetalCup")
                                            (:at ,(random-source-location))))
                             (bowl :object `((:type "RedMetalBowl")
                                             (:at ,(random-source-location))))
                             (plate :object `((:type "RedMetalPlate")
                                             (:at ,(random-source-location))))
                             (milk :object `((:type "Milk")
                                             (:at ,(random-source-location)))))
            (let ((objects `(,cup ,bowl ,plate ,milk)))
              (labels ((random-object ()
                         (elt objects (random (length objects))))
                       (random-object-subset (size)
                         (loop while (< (length set) size)
                               as object = (random-object)
                               when (not (find object set))
                                 collect object into set
                               finally (return set))))
                (let* ((random-set (loop while (not set)
                                         as set = (random-object-subset (+ (random (length objects)) 1))
                                         finally (return set)))
                       (random-set objects))
                  (dolist (object random-set)
                    (with-designators ((fetch-action :action `((:to :fetch)
                                                               (:obj ,object))))
                      (perform fetch-action)
                      (format t "Got object: ~a~%" (desig:current-desig object))
                      (with-designators ((loc-on-meal-table
                                          :location
                                          `((:on "CounterTop")
                                            (:name "iai_kitchen_meal_table_counter_top")
                                            (:theme :meal-table-setting)))
                                         (loc-destination
                                          :location
                                          `((:pose ,(destination-pose
                                                     (desig:desig-prop-value
                                                      (desig:current-desig object) :name)
                                                     (desig:reference loc-on-meal-table)))))
                                         (place-action :action
                                                       `((:to :place)
                                                         (:obj ,object)
                                                         (:at ,loc-on-meal-table))))
                        (perform place-action)))))))))))))
