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
  ;; Makes a location accessible by either just approaching it, or by
  ;; approaching it and opening a container (drawer, cabinet, dish
  ;; washer, ...).
  
  ;; 1. Get semantic information (location type, possible articulation
  ;;    requirements, maybe ideal approach direction)
  ;; 2. Approach location
  ;; 3. Possibly articulate it to open it
  )

(def-cram-function close-location (location)
  ;; If required, close this location after having opened it through
  ;; `access-location'.

  ;; 1. Get semantic information (need to close, how to close it, type
  ;;    of location)
  )

(def-cram-function find-object (object)
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
                    (setf place (cram-designators::next-solution place))
                    (cram-language:retry)))
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
    (achieve `(cram-plan-library:object-placed-at ,object ,location))))

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
