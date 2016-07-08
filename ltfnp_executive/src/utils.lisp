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

(defun init-3d-world ()
  (force-ll
   (cram-prolog:prolog
    `(and (btr:clear-bullet-world)
          (btr:bullet-world ?w)
          (btr:assert (btr:object
                       ?w :static-plane floor
                       ((0 0 0) (0 0 0 1))
                       :normal (0 0 1) :constant 0))
          (btr:debug-window ?w)))))

(cram-language:def-top-level-cram-function test-perception ()
  (with-process-modules
    (with-designators ((obj :object `((:name "IAI_kitchen"))))
      (cram-plan-library:perceive-object 'cram-plan-library:currently-visible obj))))
