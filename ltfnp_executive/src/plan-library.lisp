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
;;; Helper Plans
;;;

(def-cram-function perceive-scene (location)
  "Perceives the scene at any given `location'. Suitable for situations in which named table tops, drawers, or other containing pieces of furniture need to be examined for object presence."
  (block perceive
    (when-failure ((:object-not-found (return-from perceive)))
      (with-designators ((generic-object :object `((:at ,location))))
        (cram-plan-library:perceive-object
         :all generic-object)))))

(def-cram-function examine-object (object)
  "Further examines an already detected object by approaching it and directing cameras directly onto it."
  (with-retry-counters ((retry-location 2))
    (with-designators ((location-of-object
                        :location
                        `((:of ,object))))
      (cram-language:with-failure-handling
          (((or cram-plan-failures:location-not-reached-failure
              cram-plan-failures:navigation-failure
              cram-plan-failures:location-reached-but-not-terminated) (f)
             (declare (ignore f))
             (when (setf location-of-object
                         (cram-designators:next-solution location-of-object))
               (do-retry retry-location
                 (retry)))))
        (at-location (location-of-object)
          (cram-plan-library:perceive-object
           ;; This could potentially lead to an `ambiguous-perception'
           ;; failure; if this happens to frequently due to quirks in
           ;; RS, switch to `perceive-object a' instead and choose the
           ;; first result. Using `the' is cleaner, though.
           :the object))))))

(def-cram-function access-location (location)
  (let ((in-drawer (string= (desig:desig-prop-value location :in)
                            "Drawer"))
        (on-countertop (string= (desig:desig-prop-value location :on)
                                "CounterTop")))
    (cond (on-countertop) ;; Do nothing.
          (in-drawer ;; Open the drawer
           (let* ((name (desig:desig-prop-value location :name))
                  (sem-map-objs
                    (cram-semantic-map-designators:designator->semantic-map-objects
                     (make-designator
                      :object `((:name ,name)))))
                  (drawer-pose (slot-value (first sem-map-objs) 'cram-semantic-map-utils:pose))
                  (in-front-of-pose
                    (tf:make-pose-stamped
                     "map" 0.0
                     ;; This lacks generality (orientation of the
                     ;; "back-up" distance) and would need
                     ;; transformation via the given orientation in
                     ;; `drawer-pose'. Works for now as all drawers
                     ;; face x+.
                     (tf:v+ (tf:origin drawer-pose)
                            (tf:make-3d-vector -1.0 0.0 0.0))
                     (tf:orientation drawer-pose)))
                  (look-at-pose
                    (tf:make-pose-stamped
                     "map" 0.0
                     ;; This lacks generality (orientation of the
                     ;; "back-up" distance) and would need
                     ;; transformation via the given orientation in
                     ;; `drawer-pose'. Works for now as all drawers
                     ;; face x+.
                     (tf:v+ (tf:origin drawer-pose)
                            (tf:make-3d-vector 0.0 0.0 -0.3))
                     (tf:orientation drawer-pose))))
             (go-to-pose (tf:origin in-front-of-pose)
                         (tf:orientation in-front-of-pose)
                         :frame "map")
             (let* ((semantic-object (get-semantic-drawer name))
                    (robosherlock-handle (get-robosherlock-drawer-handle
                                          semantic-object))
                    (drawer-pose-map
                      (tf:pose->pose-stamped
                       "map" 0.0
                       drawer-pose)))
               ;; look at semantic handle
               (look-at (make-designator
                         :location `((:pose ,look-at-pose))))
               (sleep 3)
               ;; perceive real handle
               (let* ((perceived-handle
                        (first
                         (cram-uima::get-uima-result
                          (make-designator
                           :action
                           `((:handle ,robosherlock-handle))))))
                      (handle-pose-map
                        (tf:transform-pose-stamped
                         *transformer*
                         :timeout 10.0
                         :pose (desig:desig-prop-value
                                perceived-handle :pose)
                         :target-frame "map"))
                      (handle-pose
                        (tf:make-pose-stamped
                         "map" 0.0
                         (tf:origin handle-pose-map)
                         (tf:euler->quaternion)))
                      (grasp-handle-pose
                        (tf:copy-pose-stamped
                         handle-pose
                         :origin (tf:v+ (tf:origin handle-pose)
                                        (tf:make-3d-vector
                                         -0.3 0.0 0.0))
                         :orientation (tf:euler->quaternion
                                       :ax (/ pi 2))))
                      (grasp-handle-pose-2
                        (tf:copy-pose-stamped
                         handle-pose
                         :origin (tf:v+ (tf:origin handle-pose)
                                        (tf:make-3d-vector
                                         -0.2 0.0 0.0))
                         :orientation (tf:euler->quaternion
                                       :ax (/ pi 2))))
                      (grasp-handle-pose-open
                        (tf:copy-pose-stamped
                         handle-pose
                         :origin (tf:v+ (tf:origin handle-pose)
                                        (tf:make-3d-vector
                                         -0.4 0.0 0.0))
                         :orientation (tf:euler->quaternion
                                       :ax (/ pi 2)))))
                 (pr2-manip-pm::publish-pose grasp-handle-pose "/ppp")
                 ;; grasp handle: Use the left arm for now
                 (move-arm-pose :left grasp-handle-pose)
                 (pr2-manip-pm::open-gripper :left)
                 (move-arm-pose :left grasp-handle-pose-2)
                 (pr2-manip-pm::close-gripper :left)
                 ;; pull open
                 (move-arm-pose :left grasp-handle-pose-open)
                 ;; take away hand
                 (pr2-manip-pm::open-gripper :left)
                 (move-arm-pose :left pr2-manip-pm::*park-pose-left-default*)
                 )))))))

(def-cram-function close-location (location)
  ;; If required, close this location after having opened it through
  ;; `access-location'.

  ;; 1. Get semantic information (need to close, how to close it, type
  ;;    of location)
  )

(def-cram-function find-object (object &key (num-retries 100))
  ;; This should also cover articulating the environment while
  ;; searching for an object, ultimately leaving the container open
  ;; that contained the object looked for. Of course for table tops
  ;; this doesn't matter.
  (let* ((place (or (cram-designators:desig-prop-value object :at)
                    (make-designator :location `((:on "CounterTop")))))
         (object (make-designator
                  :object
                  (append (cram-designators:properties object)
                          `((:at ,place)))
                  object)))
    (when-failure ((:object-not-found
                    (when (> num-retries 0)
                      (setf place (cram-designators::next-solution place))
                      (decf num-retries)
                      (cram-language:retry))))
        (access-location place)
        ;; Perceive objects "inside" (if applicable)
        (perceive-object :a object)
        ;; If object(s) found, return them; otherwise, close-location.
        )))

(def-cram-function pick-object (object)
  ;; Assumptions: Object accessible
  (format t "Trying to pick object up~%")
  (achieve `(cram-plan-library:object-in-hand ,object)))

(def-cram-function fetch-object (object)
  (with-designators ((find-action :action `((:to :find)
                                            (:obj ,object)))
                     (pick-action :action `((:to :pick)
                                            (:obj ,object))))
    (when-failure ((:manipulation-pose-unreachable
                    (format t "Manip Fail, retry~%")
                    (cpl:retry)))
      (when-failure ((:LOCATION-NOT-REACHED-FAILURE
                      (cpl:retry))
                     (:object-not-found
                      (ros-warn (ltfnp) "fetch-object: object not found")))
        (perform find-action))
      (when-failure ((:LOCATION-NOT-REACHED-FAILURE
                      (format t "Loc Fail, retry~%")
                      (cpl:retry)))
        (perform pick-action)))))

(def-cram-function put-object (object location)
  ;; Assumptions: Location accessible, approached
  (format t "Trying to place object~%")
  (when-failure ((:LOCATION-NOT-REACHED-FAILURE
                  (cpl:retry))
                 (:manipulation-failure
                  (cpl:retry)))
    ;; NOTE(winkler): If this fails, we probably already put the
    ;; object where it belongs. This is done to the reason that
    ;; `park`ing after placing the object could fail (e.g. no IK
    ;; solution found) and would trigger a `manipulation-failure`
    ;; failure. In this case, we just go on.
    (when (cram-prolog:prolog `(pr2-manip-pm::object-in-hand ?o))
      (achieve `(cram-plan-library:object-placed-at ,object ,location)))))

(def-cram-function place-object (object location)
  ;; Assumptions: Object in hand
  (with-retry-counters ((pose-resampling 4)
                        (manipulation-retry 4))
    (when-failure ((:manipulation-pose-unreachable
                    (do-retry pose-resampling
                      (setf location (desig:next-solution location))
                      (cpl:retry)))
                   (:manipulation-failed
                    (do-retry manipulation-retry
                      (setf location (desig:next-solution location))
                      (cpl:retry))))
      (access-location location)))
  (with-retry-counters ((pose-resampling 2)
                        (manipulation-retry 2))
    (when-failure ((:manipulation-pose-unreachable
                    (do-retry pose-resampling
                      (cpl:retry)))
                   (:manipulation-failed
                    (do-retry manipulation-retry
                      (cpl:retry)))
                   (:manipulation-pose-occupied
                    (do-retry pose-resampling
                      (cpl:retry))))
      (put-object object location))))
