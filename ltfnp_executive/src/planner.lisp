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


(defclass state ()
  ((preconditions :initarg :preconditions :reader preconditions)))

(defclass goal (state)
  ((name :initarg :name :reader name)))

(defclass action (goal)
  ((effects :initarg :effects :reader effects)))


(defun make-tabletop-goal (name object-locations)
  (let ((preconditions
          (mapcar (lambda (object-location)
                    (destructuring-bind (object location)
                        object-location
                      `(:object-location ,object ,location)))
                  object-locations)))
    (make-instance 'goal
                   :name name
                   :preconditions preconditions)))

(defun make-empty-state ()
  (make-instance 'state :preconditions nil))

(defun plan (initial-state goal-state)
  (declare (ignore initial-state))
  ;; NOTE(winkler): This is not a general planner yet, but a returner
  ;; of sequential actions that advise a robot to get one object after
  ;; the other. Also, it takes into account picking up multiple
  ;; objects with both hands before going to the target pose. This
  ;; function has domain knowledge about tablesetting and cannot be
  ;; used for other problems at the moment.
  (let ((goal-objects
          (cpl:mapcar-clean
           (lambda (precondition)
             (destructuring-bind (type &rest rest) precondition
               (when (eql type :object-location)
                 rest)))
           (preconditions goal-state))))
    (labels ((fetch-and-place-next-object-actions ()
               (when (> (length goal-objects) 0)
                 (let ((object+loc (nth 0 goal-objects)))
                   (setf goal-objects (subseq goal-objects 1))
                   (destructuring-bind (object location) object+loc
                     `((:fetch ,object)
                       (:place ,object ,location)))))))
      (let ((run t))
        (loop while run
              as next-action = (or (fetch-and-place-next-object-actions)
                                   (setf run nil))
              when next-action
                append next-action)))))
