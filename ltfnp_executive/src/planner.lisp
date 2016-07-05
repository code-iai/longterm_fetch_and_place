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

;; Possible target arrangements:
;;  - Four boxes of the same type on one of the rack levels

(defun make-target-arrangement () ;; PancakeMix
  (let ((object-types `("Kelloggs" "Lion"));;(shopping-item-classes))
        (arrangement-types `(:lineup)))
    (let* ((this-arrangement (nth (random (length arrangement-types))
                                  arrangement-types))
           (object-matrix
             (case this-arrangement
               (:lineup
                ;; Select two types
                (let ((selected-types
                        (labels ((random-object-type-index ()
                                   (random (length object-types))))
                          (let* ((first (random-object-type-index))
                                 (second
                                   (block generate-second
                                     (loop do
                                       (let ((temp-index (random-object-type-index)))
                                         (unless (= temp-index first)
                                           (return-from generate-second temp-index)))))))
                            `(,(nth first object-types) ,(nth second object-types)))))
                      (target-rows
                        (labels ((random-row ()
                                   (+ 1 (random 2))))
                          (let* ((first (random-row))
                                 (second
                                   (block generate-second
                                     (loop do
                                       (let ((temp-index (random-row)))
                                         (unless (= temp-index first)
                                           (return-from generate-second temp-index)))))))
                            `(,first ,second)))))
                  (loop for i from 0 below 4
                        collect (loop for j from 0 below 4
                                      collect
                                      (cond ((= j 0)
                                             nil)
                                            ((= j 3)
                                             nil)
                                            ((= i (first target-rows))
                                             (first selected-types))
                                            ((= i (second target-rows))
                                             (second selected-types))))))))))
      object-matrix)))

(defun randomize-arrangement (arrangement)
  (let* ((collected-objects
           (loop for line in arrangement
                 for contents = (loop for object in line
                                      when object
                                        collect object)
                 when contents
                   append contents))
         (reassigned-indices))
    (labels ((free-place ()
               (block find
                 (loop do
                   (let ((x (+ 1 (random 2)))
                         (y (+ 1 (random 2))))
                     (unless (find `(,y ,x) reassigned-indices
                                   :test (lambda (xy assigned)
                                           (equal xy (cadr assigned))))
                       (return-from find `(,y ,x))))))))
      (loop for object in collected-objects
            for place = (free-place)
            do (push `(,object ,place) reassigned-indices)))
    (loop for i from 0 below 4
          collect
          (loop for j from 0 below 4
                for entry = (find `(,i ,j) reassigned-indices
                                  :test (lambda (xy assigned)
                                          (equal xy (cadr assigned))))
                if entry
                  collect (car entry)
                else
                  collect nil))))

(defun print-arrangement (arrangement)
  (let ((longest-entry
          (+ (loop for line in arrangement
                   maximize (loop for object in line
                                  maximize (length object)))
             3)))
    (loop for line in arrangement do
      (format t "  ")
      (loop for object in line do
        (let ((format-string (concatenate
                              'string
                              "~" (write-to-string
                                    longest-entry)
                              "s")))
          (format t format-string object)))
      (format t "~%"))))

(defun discrepancies (current-arrangement target-arrangement)
  (mapcar (lambda (current-line target-line)
            (mapcar (lambda (current-object target-object)
                      (if (string= current-object target-object)
                        "OK"
                        "!"))
                    current-line target-line))
          current-arrangement target-arrangement))

(defun entropy (discrepancies)
  (loop for line in discrepancies
        sum
        (loop for object in line
              if (string= object "!")
                sum 1
              else
                sum 0)))

(defun find-next-misplaced (discrepancies state)
  (block misplaced
    (loop for i from 0 below 4 do
      (loop for j from 0 below 4 do
        (when (and (string=
                    (nth j (nth i discrepancies))
                    "!")
                   (nth j (nth i state)))
          (return-from misplaced `(,i ,j)))))))

(defun find-free-target (object-type state target)
  (block found
    (loop for i from 0 below 4 do
      (loop for j from 0 below 4 do
        (when (and (string= object-type (nth j (nth i target)))
                   (eql (nth j (nth i state)) nil))
          (return-from found `(,i ,j)))))))

(defun find-free-place (state target &optional obstruction-ok)
  (block find
    (loop for i from 1 below 3 do
      (loop for j from 1 below 4 do
        (when (and (not (nth j (nth i state)))
                   (or obstruction-ok
                       (not (nth j (nth i target)))))
          (return-from find `(,i ,j)))))))

(defun find-obstacle (object-type state target)
  ;;(print-arrangement state)
  ;;(format t "~%")
  ;;(print-arrangement target)
  (block find
    (loop for i from 1 below 3 do
      (loop for j from 0 below 4 do
        (when (and (nth j (nth i target))
                   (string= object-type
                            (nth j (nth i target)))
                   (nth j (nth i state))
                   (not (string= (nth j (nth i state)) object-type)))
          (return-from find `(,i ,j)))))))

(defun update-state (state x y new)
  (loop for i from 0 below 4
        collect
        (loop for j from 0 below 4
              if (and (= i x)
                      (= j y))
                collect new
              else
                collect (nth j (nth i state)))))

(defun find-action-sequence (current target)
  (let* ((sequence)
         (state current)
         (entropy (entropy (discrepancies state target))))
    (loop while (> entropy 0) do
      (let* ((current-discrepancies (discrepancies state target))
             (next-misplaced (find-next-misplaced
                              current-discrepancies state)))
        (format t "Next misplaced: ~a~%" next-misplaced)
        (when next-misplaced
          ;; find place where it should go, that is free
          (let ((free-target (find-free-target
                              (nth (second next-misplaced)
                                   (nth (first next-misplaced)
                                        state))
                              state
                              target)))
            (format t "Free target = ~a~%" free-target)
            (if free-target
                (progn
                  (setf sequence (append sequence
                                         `((:move ,next-misplaced ,free-target))))
                  (setf state
                        (update-state
                         state
                         (first free-target)
                         (second free-target)
                         (nth (second next-misplaced)
                              (nth (first next-misplaced) state))))
                  (setf state
                        (update-state
                         state
                         (first next-misplaced)
                         (second next-misplaced)
                         nil)))
                (progn
                  (let ((obstacle
                          (find-obstacle
                           (nth (second next-misplaced)
                                (nth (first next-misplaced) state))
                           state target))
                        (free-place (or (find-free-place state target)
                                        (find-free-place state target t))))
                    (format t "Free place = ~a~%" free-place)
                    (format t "Obstacle = ~a~%" obstacle)
                    (setf sequence (append sequence
                                           `((:move ,obstacle ,free-place))))
                    (setf state
                        (update-state
                         state
                         (first free-place)
                         (second free-place)
                         (nth (second obstacle)
                              (nth (first obstacle) state))))
                    (setf state
                          (update-state
                           state
                           (first obstacle)
                           (second obstacle)
                           nil)))))))
        (format t "~%")
        (print-arrangement state)
        (format t "~%Entropy = ~a%~%"
               (* (/ (entropy (discrepancies state target)) 16.0)
                  100.0))
        (setf entropy (entropy (discrepancies state target)))))
    sequence))

(defun plan-arrangement-actions (current-arrangement target-arrangement)
  ;;(format t "Current arrangement:~%")
  ;;(print-arrangement current-arrangement)
  ;;(format t "~%Target arrangement:~%")
  ;;(print-arrangement target-arrangement)
  ;;(format t "~%Discrepancies:~%")
   (let ((discrepancies (discrepancies current-arrangement
                                      target-arrangement))
    ;;(print-arrangement discrepancies)
    ;;(format t "~%Entropy = ~a%~%"
    ;;        (* (/ (entropy discrepancies) 16.0) 100.0))
        (as (find-action-sequence current-arrangement target-arrangement)))
    (upside-down as)))

(defun upside-down (action-sequence)
  (loop for (type . parameters) in action-sequence collect
        (case type
          (:move `(:move (,(- 3 (caar parameters)) ,(cadar parameters))
                         (,(- 3 (caadr parameters)) ,(cadadr parameters)))))))
        
(defun test-arrangement-planner ()
  (let* ((target (make-target-arrangement))
         (current (randomize-arrangement target)))
    (plan-arrangement-actions current target)))

(defun make-array-arrangement (arrangement)
  (let ((array-arrangement (make-empty-object-arrangement)))
    (loop for i from 0 below 4 do
      (loop for j from 0 below 4 do
        (if (nth j (nth i arrangement))
            (setf (aref array-arrangement (- 3 i) j 0) `(,(nth j (nth i arrangement))))
            (setf (aref array-arrangement (- 3 i) j 0) nil))))
    array-arrangement))

(defun repopulate-shopping-items (arrangement)
  (remove-all-shopping-items)
  (mapcar (lambda (line)
            (mapcar (lambda (class)
                      (when class
                        (add-shopping-item class)))
                    line))
          arrangement))

(defun state-arrangement-problem ()
  (let* ((target (make-target-arrangement))
         (resolved-items-target (repopulate-shopping-items target))
         (current (randomize-arrangement resolved-items-target)))
    (format t "Initial configuration:~%")
    (print-arrangement current)
    (format t "Target configuration:~%")
    (print-arrangement resolved-items-target)
    (list (make-array-arrangement resolved-items-target)
          (make-array-arrangement current)
          (plan-arrangement-actions current resolved-items-target))))

(defun assert-planning-problem ()
  (delete-shopping-items-from-gazebo)
  (let* ((problem (state-arrangement-problem))
         (target (first problem))
         (current (second problem))
         (sequence (third problem)))
    (roslisp:ros-info (planning problem) "Target: ~a" target)
    (roslisp:ros-info (planning problem) "Start: ~a" current)
    (roslisp:ros-info (planning problem) "Sequence: ~a" sequence)
    (resolve-and-spawn-object-arrangement current)
    (list target sequence)))
