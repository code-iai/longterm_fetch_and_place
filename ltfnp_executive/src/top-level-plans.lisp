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

(defun start-scenario (&key (simulated t))
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
  (longterm-fetch-and-place))


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
  (with-process-modules
    ;(go-to-origin)
    (with-designators ((loc-on-sink :location `((:on "CounterTop")
                                                (:name "iai_kitchen_sink_area_counter_top")))
                       (cup :object `((:type "RedMetalCup")
                                      (:at ,loc-on-sink)))
                       (fetch-action :action `((:to :fetch)
                                               (:obj ,cup))))
      (perform fetch-action)
      (with-designators ((loc-on-meal-table :location
                                            `((:on "CounterTop")
                                              (:name "iai_kitchen_meal_table_counter_top")))
                         (place-action :action
                                       `((:to :place)
                                         (:obj ,cup)
                                         (:at ,loc-on-meal-table))))
        (perform place-action)))))
