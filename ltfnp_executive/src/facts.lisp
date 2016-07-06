;;; Copyright (c) 2015, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(defun make-scenario-area-restriction-cost-function ()
  (let ((min-x -0.25)
        (max-x 0.50)
        (min-y -0.5)
        (max-y 0.5))
    (lambda (x y)
      (if (and (>= x min-x)
               (<= x max-x)
               (>= y min-y)
               (<= y max-y))
          1.0d0
          0.0d0))))

(defun make-scenario-rackposition-restriction-distribution (pose)
  (let* ((origin (tf:origin pose))
         (o-x (tf:x origin))
         (o-y (tf:y origin)))
    (lambda (x y)
      (if (and (> x (- o-x 0.15))
               (< x (+ o-x 0.15))
               (> y (- o-y 0.60))
               (< y (+ o-y 0.60)))
          1.0d0
          0.0d0))))

(defun make-rack-facing-orientation-generator (object-acted-on)
  (declare (ignore object-acted-on))
  (format t "~%~%~%!!!!!!!!!!!!!!!!!!!!!!!!!!!!~%~%~%")
  (let ((position (tf:make-identity-vector)))
    (location-costmap:make-orientation-generator
     (alexandria:rcurry (lambda (x y position)
                          (declare (ignore x y position))
                          ;; This always faces forward for now and
                          ;; impicitly only takes one rack into
                          ;; account. `object-acted-on', `x', and `y'
                          ;; don't have any effect right now.
                          0)
                        position))))

(defmethod costmap-generator-name->score
    ((name (common-lisp:eql 'scenario-area-restriction-distribution)))
  100)

(defmethod costmap-generator-name->score
    ((name (common-lisp:eql 'scenario-rackposition-restriction-distribution)))
  101)

(defun is-detection-source-object (detection)
  (string= (second (assoc 'desig-props::source detection))
           "JIRAnnotatorObject"))

(defun get-type-from-detection (detection)
  (second (assoc 'desig-props::type detection)))

(def-fact-group scenario-costmap-area-restriction (desig-costmap)
  
  (<- (desig-costmap ?desig ?cm)
    (or (desig-prop ?desig (desig-props:to desig-props:see))
        (desig-prop ?desig (desig-props:to desig-props:reach)))
    (costmap ?cm)
    (costmap-add-function scenario-area-restriction-distribution
                          (make-scenario-area-restriction-cost-function)
                          ?cm)
    (costmap-add-orientation-generator
     (make-rack-facing-orientation-generator ?desig)
     ?cm))

  (<- (desig-costmap ?desig ?cm)
    (cram-prolog:fail)
    (desig-prop ?desig (desig-props:to desig-props:reach))
    (desig-prop ?desig (desig-props:obj ?obj))
    (current-designator ?obj ?current-obj)
    (desig-prop ?current-obj (desig-props:at ?at))
    (desig-prop ?at (desig-props:pose ?pose))
    (costmap ?cm)
    (costmap-add-function scenario-rackposition-restriction-distribution
                          (make-scenario-rackposition-restriction-distribution ?pose)
                          ?cm)))

(def-fact-group inference-facts (infer-object-property object-handle)
  
  (<- (infer-object-property ?object desig-props:type ?value)
    (desig-prop ?object (desig-props::detection ?detection))
    (cram-prolog:lisp-pred is-detection-source-object ?detection)
    (cram-prolog:lisp-fun get-type-from-detection ?detection ?type)
    (cram-prolog:lisp-fun convert-object-name ?type ?value))
  
  (<- (infer-object-property ?object desig-props:dimensions ?value)
    (desig-prop ?object (desig-props:type "Lion"))
    (cram-prolog:lisp-fun vector 0.045 0.195 0.26 ?value))
  
  ;; (<- (infer-object-property ?object desig-props:shape ?value)
  ;;   (desig-prop ?object (desig-props:type ?type))
  ;;   (crs:lisp-fun get-item-type-primitive-shape-symbol ?type ?value)))
  )

(def-fact-group occassions (holds)

  (<- (object-picked-from-rack ?rack ?object)
    (cram-prolog:fail))

  (<- (objects-detected-in-rack ?rack ?object-template)
    (cram-prolog:fail))

  (<- (rack-scene-perceived)
    (cram-prolog:fail))
  
  (<- (object-handover ?object ?target-hand)
    (not (pr2-manip-pm::object-in-hand ?object ?target-hand)))
  
  (<- (object-placed-on-rack ?object ?level ?x ?y)
    (cram-prolog:fail)))
