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

(declare-goal rack-scene-perceived (rack hints)
  "Triggers a scene perception action. This means that the robot backs off from the rack, goes into a suitable pose for looking at it, and looks at each rack level individually. All objects being perceived are automatically asserted into a) the belief state (bullet world), and b) the collision environment."
  (declare (ignore rack hints))
  (roslisp:ros-info (shopping plans) "SCENE-PERCEIVED"))

(def-goal (achieve (rack-scene-perceived ?rack ?hints))
  ;; Iterate through all rack levels and add their contents to the
  ;; collision environment.
  (go-in-front-of-rack ?rack)
  (let ((perceive-scene-rack-level (get-hint ?hints :perceive-scene-rack-level nil)))
    (loop for level in (or (when perceive-scene-rack-level
                             `(,(get-rack-on-level ?rack perceive-scene-rack-level)))
                           (get-rack-levels ?rack))
          as pose = (get-rack-level-relative-pose
                     level 0 0 0
                     (cl-transforms:euler->quaternion))
          do (achieve `(cram-plan-library:looking-at ,pose))
             (with-designators ((generic-object (object `())))
               (perceive-all
                generic-object :stationary t :move-head nil)))))

(declare-goal object-picked-from-rack (rack object)
  "Picks an object `object' from a given rack instance `rack'."
  (declare (ignore rack object))
  (roslisp:ros-info (shopping plans) "OBJECT-PICKED-FROM-RACK"))

(def-goal (achieve (object-picked-from-rack ?rack ?object))
  "Repositions the robot's torso in order to be able to properly reach the rack level the object is residing on, and start picking up the object, repositioning and reperceiving as necessary."
  (let* ((rack-level (get-object-rack-level ?rack ?object))
         (elevation (get-rack-level-elevation rack-level)))
    ;; NOTE(winkler): Reposition robot torso according to rack level
    ;; height. This is a heuristic transformation which maps the
    ;; absolute height of thelowest and highest rack level to the
    ;; lower and upper boundaries of the torso position.
    (move-torso (/ elevation 5.0))
    ;; Actually pick the object. This will trigger re-perception and
    ;; navigation in order to properly grasp the object, lift it, and
    ;; move into a safe carrying pose.
    (pick-object ?object)))

(declare-goal objects-detected-in-rack (rack object-template)
  "Tries to detect objects in the given rack `rack', according to the description of the object template given as `object-template'."
  (declare (ignore rack object-template))
  (roslisp:ros-info (shopping plans) "OBJECTS-DETECTED-IN-RACK"))

(def-goal (achieve (objects-detected-in-rack ?rack ?object-template))
  (declare (ignore ?rack))
  (let* ((type (desig-prop-value ?object-template 'type))
         (name (desig-prop-value ?object-template 'name))
         (at (desig-prop-value ?object-template 'at))
         (pose (when at (desig-prop-value at 'pose))))
    (declare (ignore type name pose))
    ;; TODO(winkler): Extend this such that it makes use of the above
    ;; properties, and reflects the respectiv behavior.

    ;; NOTE(winkler): Somehow, the `at' property is missing here,
    ;; making manipulation impossible. Need to figure out why it gets
    ;; cut out; this needs fixing next!
    (let ((enriched-objects
            (mapcar (lambda (perceived-object)
                      (enrich-object-description perceived-object))
                    (perceive-all ?object-template))))
      (dolist (object enriched-objects)
        (plan-knowledge:on-event
         (make-instance 'plan-knowledge:object-updated-event
                        :perception-source :generic
                        :object-designator object)))
      enriched-objects)))

(declare-goal object-handover (object target-hand)
  "Hands over the held object `object' such that it is held by the hand `target-hand', if not already true."
  (declare (ignore object target-hand))
  (roslisp:ros-info (shopping plans) "OBJECT-HANDOVER"))

(def-goal (achieve (object-handover ?object ?target-hand))
  (let ((current-hand
          (assoc '?hand
                 (lazy-car
                  (crs:prolog `(pr2-manip-pm::object-in-hand
                                ,?object ?hand))))))
    (unless (eql current-hand ?target-hand)
      ;; TODO(winkler): Implement the handover here. Ideally, the
      ;; `programmatic' motion strategy is done in the PR2
      ;; manipulation process module, while this plan only supplies it
      ;; with the necessary information and parameterizes the task.
      (roslisp:ros-warn
       (shopping plans) "Handover not yet implemented!"))))

(declare-goal object-placed-on-rack (object level x y)
  "Attempts to place the object `object' on the racklevel `level' of rack `rack'. `x' and `y' are center-relative coordinates on that racklevel."
  (declare (ignore object level x y))
  (roslisp:ros-info (shopping plans) "OBJECT-PLACED-ON-RACK"))

(def-goal (achieve (object-placed-on-rack ?object ?level ?x ?y))
  (let* ((elevation 0.0)
         (absolute-pose (get-rack-level-relative-pose
                         ?level ?x ?y elevation)))
    (with-designators ((loc (location `((desig-props:pose
                                         ,absolute-pose)))))
      (place-object ?object loc))))

(declare-goal switched-holding-hand (object)
  "Switches hands for the held object `object'."
  (declare (ignore object))
  (roslisp:ros-info (shopping plans) "SWITCHED-HOLDING-HANDS"))

(def-goal (achieve (switched-holding-hand ?object))
  (handover-object ?object)
  (roslisp:ros-info (shopping plans) "Handover complete~%"))
