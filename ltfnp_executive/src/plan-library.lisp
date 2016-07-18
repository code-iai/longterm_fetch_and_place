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
         'cram-plan-library:all generic-object)))))

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
           'cram-plan-library:the object))))))

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
  (let ((place (or (cram-designators:desig-prop-value object :at)
                   (make-designator :location `((:on "CounterTop"))))))
    (when-failure ((:object-not-found (setf place (cram-designators::next-solution place))
                                      (cram-language:retry)))
      (access-location)
      ;; Perceive objects "inside" (if applicable)
      ;; If object(s) found, return them; otherwise, close-location.
      )))

(def-cram-function pick-object (object)
  ;; Assumptions: Object accessible, approached
  )

(def-cram-function fetch-object (object)
  (when-failure ((:object-not-found (format t "Fail~%")))
    (find-object object))
  (when-failure ((:location-not-reached (format t "Fail~%"))
                 (:manipulation-pose-unreachable (format t "Fail~%"))
                 (:manipulation-failed (format t "Fail~%")))
    (pick-object object)))

(def-cram-function put-object (object location)
  ;; Assumptions: Location accessible, approached
  )

(def-cram-function place-object (object location)
  ;; Assumptions: Object in hand
  (with-retry-counters ((location-resampling 2)
                        (pose-resampling 2)
                        (manipulation-retry 2))
    (when-failure ((:location-not-reached
                    (do-retry location-resampling
                      (cpl:retry)))
                   (:manipulation-pose-unreachable
                    (do-retry pose-resampling
                      (cpl:retry)))
                   (:manipulation-failed
                    (do-retry manipulation-retry
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
                   (:pose-occupied
                    (do-retry pose-resampling
                      (cpl:retry))))
      (put-object object location))))

(def-cram-function fetch-and-place-object (object location)
  ;; Most naive implementation, develop further.
  (with-retry-counters ((location-resampling 2)
                        (pose-resampling 2)
                        (not-found-retry 2))
    (when-failure ((:location-not-reached
                    (do-retry location-resampling
                      (cpl:retry)))
                   (:object-not-found
                    (do-retry not-found-retry
                      (cpl:retry)))
                   (:manipulation-pose-unreachable
                    (do-retry pose-resampling
                      (cpl:retry))))
      (fetch-object object)))
  (with-retry-counters ((location-resampling 2)
                        (pose-resampling 2))
    (when-failure ((:location-not-reached
                    (do-retry location-resampling
                      (cpl:retry)))
                   (:manipulation-pose-unreachable
                    (do-retry pose-resampling
                      (cpl:retry)))
                   (:pose-occupied
                    (do-retry pose-resampling
                      (cpl:retry))))
      (place-object object location))))
