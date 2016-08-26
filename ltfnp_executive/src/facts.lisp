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


(defun make-ltfnp-area-restriction-cost-function ()
  (let ((area-boxes ;; x y w h
          `((-0.45 -0.2 1.5 1.0) ;; Main center area
            (-0.45 0.2 0.75 1.5) ;; In front of kitchen island, kitchen side
            (-0.45 -1.2 0.75 1.5) ;; Side of meal table
            (-1.5 -0.75 1.2 0.75) ;; In front of meal table
            (1.3 0.1 0.3 0.75) ;; On sink area
            (-0.9 0.55 0.3 1.5) ;; On island area
            (-1.5 -1.15 0.9 0.35) ;; On meal table
            )))
    (lambda (x y)
      (block validity-check
        (loop for box in area-boxes do
          (destructuring-bind (bx by w h) box
            (when (and (>= x bx)
                       (>= y by)
                       (< x (+ bx w))
                       (< y (+ by h)))
              (return-from validity-check 1.0d0))))
        0.0d0))))

(defun make-ltfnp-meal-table-restriction-cost-function ()
  (let ((x-margin 0.6)
        (y-margin 0.1)
        (base-x -1.40)
        (base-y -0.95))
    (lambda (x y)
      (if (and (>= x (- base-x x-margin))
               (<= x (+ base-x x-margin))
               (>= y (- base-y y-margin))
               (<= y (+ base-y y-margin)))
          1.0d0
          0.0d0))))

(defun make-ltfnp-meal-table-orientation-generator ()
  (lambda (x y previous-orientation)
    (declare (ignore x y previous-orientation))
    (lazy-mapcar #'identity `(,(tf:make-quaternion 0.0 0.0 -0.70711 0.70711)))))

(defmethod costmap-generator-name->score
    ((name (common-lisp:eql 'ltfnp-costmap-area-restriction)))
  100)

(defmethod costmap-generator-name->score
    ((name (common-lisp:eql 'ltfnp-costmap-meal-table-restriction)))
  101)

(defun test-restriction-costmap ()
  "Asserts the debug costmap according to the area restriction as defined in this file into bullet. This function is intended for testing purposes in case the restriction changed."
  (cram-prolog:prolog `(and (costmap ?cm)
                            (costmap-padding ?pad)
                            (costmap-add-function
                             ltfnp-costmap-area-restriction
                             (make-ltfnp-area-restriction-cost-function)
                             ?cm)
                            (btr:debug-costmap ?cm))))


(def-fact-group ltfnp-costmap-area-restriction (desig-costmap)
  
  (<- (desig-costmap ?desig ?cm)
    (or (desig-prop ?desig (:to :see))
        (desig-prop ?desig (:to :reach))
        (desig-prop ?desig (:on ?_)))
    (costmap ?cm)
    (costmap-add-function
     ltfnp-costmap-area-restriction
     (make-ltfnp-area-restriction-cost-function)
     ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (:theme :meal-table-setting))
    (costmap ?cm)
    (costmap-add-function
     ltfnp-costmap-meal-table-restriction
     (make-ltfnp-meal-table-restriction-cost-function)
     ?cm)
    (costmap-add-orientation-generator
     (make-ltfnp-meal-table-orientation-generator)
     ?cm)))

(def-fact-group ltfnp-intrusive-hooks (close-radius)

  ;; NOTE(winkler): We could add extra rules below this one that
  ;; define `close-radius` properties when we're not currently
  ;; simulating. This is handled by the PR2 manipulation process
  ;; module though, so we just silently fail here.
  (<- (close-radius ?object ?radius)
    (symbol-value *simulated* ?simulated)
    (cram-prolog:bound ?simulated)
    (equal ?simulated t)
    (equal ?radius 0.08)))
