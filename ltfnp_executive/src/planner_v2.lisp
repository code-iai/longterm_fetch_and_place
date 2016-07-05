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

(defvar *markers* (make-hash-table :test 'equal))
(defvar *rack-level-dimensions* (make-hash-table :test 'equal))
(defvar *object-dimensions* (make-hash-table :test 'equal))
(defvar *object-poses* (make-hash-table :test 'equal))
(defvar *object-classes* (make-hash-table :test 'equal))
(defvar *perceived-objects* nil)
(defvar *tic* 0.0)
(defvar *handover-forbidden* nil)
(defvar *min-level* 0)
(defvar *max-level* 3)
(defvar *min-zone* 0)
(defvar *max-zone* 3)
(defvar *item-designators* (make-hash-table :test 'equal))
(defvar *gazebo* nil)

(defun set-item-designator (item designator)
  (setf (gethash item *item-designators*) designator))

(defun get-item-designator (item)
  (gethash item *item-designators*))

(defmacro setstate (key-state state-map value)
  `(let ((new-state-map
           (append
            (remove-if (lambda (list-item)
                         (states-equal? ,key-state (first list-item)))
                       ,state-map)
            `((,,key-state ,,value)))))
     (setf ,state-map new-state-map)))

(defun get-item-class-cached (item)
  (or (gethash item *object-classes*)
      (setf (gethash item *object-classes*) (get-item-class item))))

(defun get-item-pose-cached (item)
  (or (gethash item *object-poses*)
      (setf (gethash item *object-poses*) (get-item-pose item))))

(defun set-item-pose-cached (item pose)
  (set-item-pose item pose)
  (setf (gethash item *object-poses*) pose))

(defun get-rack-level-dimensions (level)
  (or (gethash level *rack-level-dimensions*)
      (let* ((rack (first (get-racks)))
             (rack-level (get-rack-on-level rack 1))
             (rack-level-dimensions (get-item-dimensions rack-level)))
        (setf (gethash level *rack-level-dimensions*)
              rack-level-dimensions))))

(defun get-item-dimensions-cached (object)
  (or (gethash object *object-dimensions*)
      (setf (gethash object *object-dimensions*)
            (get-item-dimensions object))))

(defun pose->relative-pose (pose-stamped frame-id)
  (cl-tf2:ensure-pose-stamped-transformed
   *tf* pose-stamped frame-id))

(defun pose->rack-relative-pose (pose-stamped)
  (pose->relative-pose pose-stamped "/shopping_rack"))

(defun pose->map-relative-pose (pose-stamped)
  (pose->relative-pose pose-stamped "/map"))

(defun make-level-relative-pose (level x y z
                                 &key (roll 0.0)
                                   (pitch 0.0)
                                   (yaw 0.0))
  (let ((frame-id (concatenate 'string "/rack_level_"
                               (write-to-string level)))
        (yaw (+ (/ pi 2) yaw)))
    (tf:make-pose-stamped
     frame-id 0.0
     (tf:make-3d-vector x y z)
     (tf:euler->quaternion :ax roll :ay pitch :az yaw))))

(defun make-zone-pose (level zone x y
                       &key (z 0 z-p) (orientation (tf:euler->quaternion)) (elevation 0.0))
  (let* ((rack-level-dimensions (get-rack-level-dimensions level))
         (zones-per-level 4)
         (zone-width (/ (elt rack-level-dimensions 1) zones-per-level))
         (zone-height 0.4)
         (c-x (+ (- (* (+ zone 0.5) zone-width)
                    (/ (elt rack-level-dimensions 1) 2)
                    )
                 y))
         (c-y x)
         (c-z (+ (cond (z-p z)
                       (t (/ zone-height 2)))
                 (/ zone-height 2)
                 (/ (elt rack-level-dimensions 2) 2)
                 elevation)))
    (tf:make-pose-stamped
     (concatenate
      'string
      "/rack_level_" (write-to-string level))
     0.0
     (cl-transforms:transform-pose
     (tf:make-transform (tf:make-3d-vector c-x c-y c-z)
                        (tf:euler->quaternion))
     (tf:make-pose (tf:make-identity-vector)
                   orientation)))))

(defun display-zones (&key highlight-zones)
  (let* ((rack-level-dimensions (get-rack-level-dimensions 1))
         (levels 4)
         (zones-per-level 4)
         (zone-width (/ (elt rack-level-dimensions 1) zones-per-level))
         (zone-depth (elt rack-level-dimensions 0))
         (zone-height 0.4))
    (labels ((highlight-zone (level zone)
               (find `(,level ,zone) highlight-zones
                     :test
                     (lambda (subject list-item)
                       (and (eql (car subject) (caar list-item))
                            (eql (cadr subject) (cadar list-item))))))
             (zone->marker (level zone id)
               (let* ((dimensions `(,zone-width
                                    ,zone-depth
                                    ,zone-height))
                      (zone-alpha (cond ((highlight-zone level zone)
                                         0.6)
                                        (t 0.25)))
                      (color (cond ((evenp (+ level id))
                                    `(0.0 1.0 0.0 ,zone-alpha))
                                   (t `(0.0 1.0 1.0 ,zone-alpha))))
                      (pose (make-zone-pose level zone 0 0)))
                 (roslisp:make-message
                  "visualization_msgs/Marker"
                  (stamp header) (roslisp:ros-time)
                  (frame_id header) "/map"
                  ns "zones"
                  id id
                  type 1
                  action 0
                  (pose) (tf:pose->msg (pose->map-relative-pose pose))
                  (x scale) (elt dimensions 0)
                  (y scale) (elt dimensions 1)
                  (z scale) (elt dimensions 2)
                  (r color) (first color)
                  (g color) (second color)
                  (b color) (third color)
                  (a color) (fourth color)))))
      (let* ((markers
               (map 'vector #'identity
                    (loop for level from *min-level* below levels
                          append
                          (loop for zone from *min-zone* below zones-per-level
                                collect
                                (zone->marker
                                 level zone
                                 (+ (* level zones-per-level)
                                    zone))))))
             (markers-message
               (roslisp:make-message "visualization_msgs/MarkerArray"
                                     markers markers))
             (pub (roslisp:advertise
                   "/planner_objects"
                   "visualization_msgs/MarkerArray")))
        (roslisp:publish pub markers-message)))))

(defun display-objects (objects &key highlighted-objects object-poses)
  (labels ((object->marker-message (object id)
             (let ((dimensions (get-item-dimensions-cached object))
                   (color (cond ((find object highlighted-objects
                                       :test #'string=)
                                 `(1.0 1.0 1.0 1.0))
                                (t `(0.5 0.5 0.0 1.0)))))
               (roslisp:make-message
                "visualization_msgs/Marker"
                (stamp header) (roslisp:ros-time)
                (frame_id header) "/map"
                ns "objects"
                id id
                type 1
                action 0
                (pose) (tf:pose->msg
                        (pose->map-relative-pose
                         (or (when object-poses
                               (gethash object object-poses))
                             (get-item-pose-cached object))))
                (x scale) (elt dimensions 0)
                (y scale) (elt dimensions 1)
                (z scale) (elt dimensions 2)
                (r color) (first color)
                (g color) (second color)
                (b color) (third color)
                (a color) (fourth color))))
           (object->marker-message-text (object id)
             (let ((dimensions (get-item-dimensions-cached object))
                   (color `(1.0 1.0 1.0 1.0)))
               (roslisp:make-message
                "visualization_msgs/Marker"
                (stamp header) (roslisp:ros-time)
                (frame_id header) "/map"
                ns "labels"
                id id
                type 9
                action 0
                (pose) (tf:pose->msg (let* ((pose (pose->map-relative-pose
                                                   (or (when object-poses
                                                         (gethash object object-poses))
                                                       (get-item-pose-cached object))))
                                            (origin (tf:origin pose)))
                                       (tf:make-pose (tf:make-3d-vector
                                                      (tf:x origin) (tf:y origin)
                                                      (+ (tf:z origin) (/ (elt dimensions 2) 2)
                                                         0.02))
                                                     (tf:orientation pose))))
                (x scale) 0.02
                (y scale) 0.02
                (z scale) 0.02
                (r color) (first color)
                (g color) (second color)
                (b color) (third color)
                (a color) (fourth color)
                (text) object))))
    (let* ((markers
             (map 'vector #'identity
                  (loop for i from 0 below (length objects)
                        as object = (elt objects i)
                        collect
                        (progn
                          (setf (gethash object *markers*) i)
                          (object->marker-message object i))
                        collect (object->marker-message-text
                                 object i))))
           (markers-message
             (roslisp:make-message "visualization_msgs/MarkerArray"
                                   markers markers))
           (pub (roslisp:advertise
                 "/planner_objects" "visualization_msgs/MarkerArray")))
      (roslisp:publish pub markers-message))))

(defun place-object-on-rack (object level x y)
  (let ((dimensions (get-item-dimensions object)))
    (set-item-pose-cached
     object
     (make-level-relative-pose level x y (/ (elt dimensions 2) 2)))))

(defun place-object-in-zone (object level zone x y &optional (theta 0.0))
  (let* ((object-dimensions (get-item-dimensions object))
         (object-height (elt object-dimensions 2))
         (zone-pose (make-zone-pose level zone x (- y)
                                    :z (+ (- (/ object-height 2)) 0.02) :orientation (tf:euler->quaternion :az theta)))
         (origin (tf:origin zone-pose)))
    (place-object-on-rack object level (tf:x origin) (tf:y origin))))

(defun assess-object-zones (objects &key object-poses)
  (let* ((levels 4)
         (zones-per-level 4)
         (rack-level-dimensions (get-rack-level-dimensions 1))
         (zone-width
           (/ (elt rack-level-dimensions 1)
              zones-per-level))
         (zone-depth (elt rack-level-dimensions 0)))
    (mapcar
     (lambda (object)
       (let* ((pose (pose->rack-relative-pose
                     (or (when object-poses
                           (gethash object object-poses))
                         (get-item-pose-cached object))))
              (pose-elevation (tf:z (tf:origin pose)))
              (level
                (loop for level from 0 to (1- levels)
                      as level-pose = (pose->rack-relative-pose
                                       (make-level-relative-pose
                                        level 0 0 0))
                      as level-elevation = (tf:z (tf:origin
                                                  level-pose))
                      when (> pose-elevation level-elevation)
                        maximize level))
              (zone (find
                     pose (loop for zone from *min-zone* to *max-zone*
                                collect zone)
                     :test
                     (lambda (pose zone)
                       (let* ((zone-pose (pose->rack-relative-pose
                                          (make-zone-pose
                                           0 zone 0 0)))
                              (zone-x (tf:x (tf:origin zone-pose)))
                              (zone-y (tf:y (tf:origin zone-pose)))
                              (pose-x (tf:x (tf:origin pose)))
                              (pose-y (tf:y (tf:origin pose))))
                         (and (> pose-x
                                 (- zone-x (/ zone-width 2)))
                              (< pose-x
                                 (+ zone-x (/ zone-width 2)))
                              (> pose-y
                                 (- zone-y (/ zone-depth 2)))
                              (< pose-y
                                 (+ zone-y (/ zone-depth 2))))))))
              (zone-offset
                (let* ((zone-pose (pose->rack-relative-pose
                                   (make-zone-pose
                                    0 zone 0 0)))
                       (zone-x (tf:x (tf:origin zone-pose)))
                       (zone-y (tf:y (tf:origin zone-pose)))
                       (pose-x (tf:x (tf:origin pose)))
                       (pose-y (tf:y (tf:origin pose))))
                  `(,(- pose-y zone-y) ,(- pose-x zone-x)
                    ,(quaternion->yaw (tf:orientation pose))))))
         `((,level ,zone) ,zone-offset)))
     objects)))

(defun quaternion->yaw (q)
  (let ((q0 (tf:x q))
        (q1 (tf:y q))
        (q2 (tf:z q))
        (q3 (tf:w q)))
    (atan2 (* 2 (+ (* q0 q1) (* q2 q3)))
           (- 1 (* 2 (+ (* q1 q1) (* q2 q2)))))))

(defun atan2 (y x)
  (cond ((> x 0)
         (atan (/ y x)))
        ((and (< x 0) (>= y 0))
         (+ (atan (/ y x)) pi))
        ((and (< x 0) (< y 0))
         (- (atan (/ y x)) pi))
        ((and (= x 0) (> y 0))
         (/ pi 2))
        ((and (= x 0) (< y 0))
         (/ pi -2))))

(defun toy-problem-state ()
  (display-zones)
  (let ((item-1 (add-shopping-item "Kelloggs"))
        (item-2 (add-shopping-item "Kelloggs")))
    (place-object-in-zone item-1 0 3 0.0 0.0)
    (place-object-in-zone item-2 2 2 0.0 0.0)
    (display-objects `(,item-1 ,item-2))
    (let ((object-zones (assess-object-zones `(,item-1 ,item-2))))
      (display-zones :highlight-zones object-zones))))

(defun solve (&key allow-handover)
  (populate-knowledge-base)
  (let* ((handover-forbidden *handover-forbidden*))
    (setf *handover-forbidden* (not allow-handover))
    (let* ((current-arrangement (get-current-arrangement))
           (current-state (make-planning-state 0 current-arrangement))
           (goal-state (make-target-state current-state :mode :generic))
           (solutions (modified-a-star current-state goal-state))
           (action-sequence (action-sequence (rest (first solutions)))))
      (setf *handover-forbidden* handover-forbidden)
      action-sequence)))

(defun detected-type (object)
  (let ((found (find "JIRAnnotatorObject"
                     (desig-prop-values
                      object 'desig-props::detection)
                     :test (lambda (subject detail)
                             (string= (cadr (assoc 'desig-props::source
                                                   detail))
                                      subject)))))
    (when found
      (convert-object-name
       (cadr (assoc 'desig-props::type found))))))

(defun convert-object-name (name)
  (let ((new-name
          (cond
            ((string= name "pancake-mix") "PancakeMix")
            ((string= name "can") "Corn")
            ((string= name "tomato-sauce") "TomatoSauce")
            ((string= name "jodsalz-salt-container") "Salt")
            ((string= name "lion-cereals") "Lion")
            ((string= name "cornflakes") "Kelloggs"))))
    (cond (new-name new-name)
          (t "Salt")))) ;; Default model

(defun get-current-arrangement ()
  (let ((objects (get-shopping-items)))
    (mapcar (lambda (object)
              `(,(first (assess-object-zones `(,object)))
                ,object))
            objects)))

(defun populate-knowledge-base (&key (remove-all t))
  (when remove-all (remove-all-shopping-items))
  (let ((perceived-objects
          (or *perceived-objects*
              (setf *perceived-objects*
                    (top-level
                      (with-process-modules
                        (robosherlock-pm::perceive-object-designator
                         (make-designator 'object nil))))))))
    (format t "Found ~a object(s)~%" (length perceived-objects))
    (assert-perceived-objects perceived-objects)))

(defun assert-perceived-objects (perceived-objects)
  (let ((all-objects
          (mapcar (lambda (object)
                    (let ((pose
                            (let ((pose-stamped
                                    (desig-prop-value
                                     (desig-prop-value
                                      object 'desig-props:at)
                                     'desig-props:pose)))
                              (tf:pose->pose-stamped
                               (tf:frame-id pose-stamped)
                               (tf:stamp pose-stamped)
                               (cl-transforms:transform-pose
                                (tf:pose->transform pose-stamped)
                                (tf:make-pose
                                 (tf:make-3d-vector 0 0 0)
                                 (tf:euler->quaternion
                                  :az (/ pi 2)))))))
                          (item (add-shopping-item
                                 (or (detected-type object)
                                     "Kelloggs"))))
                      (set-item-pose-cached item pose)
                      (let ((new-designator
                              (make-effective-designator
                               object
                               :new-properties
                               (append
                                (remove-if
                                 (lambda (property)
                                   (eql (first property)
                                        'desig-props:handle))
                                 (description object))
                                (mapcar
                                 (lambda (handle)
                                   `(desig-props:handle ,handle))
                                 (get-item-semantic-handles item)))
                               :data-object (slot-value
                                             object
                                             'desig:data))))
                        (set-item-designator item (equate
                                                   object
                                                   new-designator)))
                      item))
                  perceived-objects)))
    (display-objects all-objects)
    (let ((object-zones (assess-object-zones all-objects)))
      (display-zones :highlight-zones object-zones))))

(defun toy-problem-solve (&key (number-of-solutions 1) (mode :generic))
  (populate-knowledge-base)
  (let* ((current-state (make-planning-state 0 (get-current-arrangement)))
         (target-state (make-target-state current-state :mode mode)))
    (modified-a-star current-state target-state
                     :number-of-solutions number-of-solutions)))

(defun action-sequence (solution)
  (loop for state-transition in solution
        as transition = (second state-transition)
        when transition
          collect transition))

(defun make-target-state (start-state &key (mode :explicit) (arrange t))
  (let* ((arrangement (cdr (assoc :arrangement start-state)))
         (torso-height (second (assoc :torso-height start-state)))
         (robot-pose (second (assoc :robot-pose start-state))))
    (make-planning-state
     robot-pose
     (ecase mode
       (:explicit
        (let ((index 0))
          (loop for level from *min-level* to *max-level*
                append
                (loop for zone from *min-zone* to *max-zone*
                      as it = (prog1
                                  (when (< index (length arrangement))
                                    `(((,level ,zone)
                                       (-0.15 0.0 0.0)) ;; x, y, theta (az)
                                      ,(second (elt arrangement index))))
                                (incf index))
                      when it
                        collect it))))
       (:generic
        (cond (arrange
               (let* ((classes-mult
                        (loop for level-zone-object in arrangement
                              collect (get-item-class-cached
                                       (second level-zone-object))))
                      (classes (remove-duplicates classes-mult
                                                  :test #'string=))
                      (classes-inst
                        (loop for class in classes
                              collect
                              `(,class ,(loop for instance in classes-mult
                                              when (string= instance class)
                                                sum 1)))))
                 (let ((level *min-level*)
                       (zone *min-zone*))
                   (loop for class-inst in classes-inst
                         append
                         (loop for i from 0 below (second class-inst)
                               collect
                               (prog1
                                   `(((,level ,zone) (0.0 0.0 0.0))
                                     ,(first class-inst))
                                 (incf zone)
                                 (when (> zone *max-zone*)
                                   (setf zone *min-zone*)
                                   (incf level))))))))
              (t (loop for level-zone-object in arrangement
                       collect
                       `(,(first level-zone-object)
                         ,(get-item-class-cached
                           (second level-zone-object))))))))
     :torso-height torso-height
     :mode mode)))

(defun make-planning-state (robot-pose arrangement &key (torso-height 2) in-hand-left in-hand-right (mode :explicit))
  `((:robot-pose ,robot-pose)
    (:torso-height ,torso-height)
    (:in-hand ((:left ,in-hand-left) (:right ,in-hand-right)))
    (:arrangement ,@arrangement)
    (:mode ,mode)))

(defun level-zone-in-state (level zone state)
  (let ((arrangement (cdr (assoc :arrangement state))))
    (not
     (not
      (find `(,level ,zone) arrangement
            :test (lambda (subject list-item)
                    (equal subject (first (first list-item)))))))))

(defun free-level-zones (state)
  (loop for level from *min-level* to *max-level*
        append
        (loop for zone from *min-zone* to *max-zone*
              when (not (level-zone-in-state level zone state))
                collect `(,level ,zone))))

(defun make-transitions (state goal-state)
  (let* ((in-hand (cadr (assoc :in-hand state)))
         (arrangement (cdr (assoc :arrangement state)))
         (transitions
           (append
            ;; Picking
            (loop for set in arrangement
                  as level-zone-source = (first set)
                  as object = (second set)
                  append `((:pick ,object :left)
                           (:pick ,object :right)))
            ;; Placing
            (loop for set in in-hand
                  when (second set)
                    append
                    (loop for free-level-zone in (free-level-zones state)
                          collect `(:place ,(second set)
                                           ,free-level-zone)))
            ;; Handover
            `((:handover))
            ;; Move base
            `((:move-base -1)
              (:move-base 0)
              (:move-base 1))
            ;; Move torso
            `((:move-torso 0)
              (:move-torso 1)
              (:move-torso 2)
              (:move-torso 3)))))
    (remove-if-not (lambda (transition)
                     (transition-valid? state transition goal-state))
                   transitions)))

(defun transition-valid? (state transition goal-state)
  (let* ((robot-pose (cadr (assoc :robot-pose state)))
         (torso-height (cadr (assoc :torso-height state)))
         (mode-goal (cadr (assoc :mode goal-state)))
         (in-hand-left
           (second (assoc :left (second
                                 (assoc :in-hand state)))))
         (in-hand-right
           (second (assoc :right (second
                                  (assoc :in-hand state)))))
         (arrangement-goal (cdr (assoc :arrangement goal-state)))
         (operation (first transition))
         (free-level-zones (free-level-zones state)))
    (labels ((level-zone-valid (level-zone)
               (let ((level (first level-zone))
                     (zone (second level-zone)))
                 (and (>= level *min-level*)
                      (<= level *max-level*)
                      (>= zone *min-zone*)
                      (<= zone *max-zone*))))
             (level-zone-for-object (object)
               (first (first (assess-object-zones `(,object)))))
             (level-zone-in-reach (robot-pose torso level-zone hand)
               (let* ((base-distance (abs (- (second level-zone)
                                             (+ robot-pose
                                                1
                                                (ecase hand
                                                  (:left -1)
                                                  (:right 1))))))
                      (torso-distance (abs (- (first level-zone)
                                              torso))))
                 (and (<= base-distance 1)
                      (<= torso-distance 1))))
             (object-in-reach (robot-pose torso-height object hand)
               (let* ((object-level-zone (level-zone-for-object
                                          object)))
                 (level-zone-in-reach robot-pose torso-height
                                      object-level-zone
                                      hand)))
             (entry-for-object-in-state (object state)
               (let ((arrangement (cdr (assoc :arrangement state))))
                 (find object arrangement
                       :test (lambda (subject list-item)
                               (string= subject
                                        (second list-item))))))
             (entries-in-level-zone-in-state (level zone state)
               (let ((arrangement (cdr (assoc :arrangement state))))
                 (loop for entry in arrangement
                       as level-zone = (first (first entry))
                       when (equal level-zone `(,level ,zone))
                         collect entry)))
             (object-occluded (object state)
               (let* ((entry (entry-for-object-in-state
                              object state))
                      (level-zone (first (first entry)))
                      (coordinates (second (first entry)))
                      (entries-in-level-zone
                        (remove-if
                         (lambda (entry)
                           (string= object (second entry)))
                         (entries-in-level-zone-in-state
                          (first level-zone) (second level-zone)
                          state)))
                      (occlusion
                        (find coordinates entries-in-level-zone
                              :test
                              (lambda (subject list-item)
                                (let ((s-x (first subject))
                                      (l-x (first (second
                                                   (first list-item)))))
                                  (< l-x s-x))))))
                 (not (not occlusion)))))
      (let ((is-valid
              (or (and (eql operation :pick)
                       (or (and (not in-hand-left)
                                (eql (third transition) :left))
                           (and (not in-hand-right)
                                (eql (third transition) :right)))
                       (object-in-reach robot-pose torso-height
                                        (second transition)
                                        (third transition))
                       (not (object-occluded (second transition)
                                             state)))
                  (and (eql operation :place)
                       (let* ((place-at (third transition))
                              (object (second transition))
                              (hand (cond ((string= in-hand-left
                                                    object)
                                           :left)
                                          ((string= in-hand-right
                                                    object)
                                           :right)))
                              (goal-place-at
                                (find place-at arrangement-goal
                                      :test
                                      (lambda (subject list-item)
                                        (equal
                                         subject
                                         (first (first list-item))))))
                              (goal-object-at (second goal-place-at)))
                         (and (level-zone-valid place-at)
                              (level-zone-in-reach robot-pose
                                                   torso-height
                                                   place-at
                                                   hand)
                              (or (and (find place-at free-level-zones
                                             :test #'equal)
                                       (string=
                                        (ecase mode-goal
                                          (:explicit object)
                                          (:generic (get-item-class-cached object)))
                                        goal-object-at))
                                  (not (find place-at free-level-zones
                                             :test #'equal))))))
                  (and (eql operation :handover)
                       (or (and (not (not in-hand-left))
                                (not in-hand-right))
                           (and (not (not in-hand-right))
                                (not in-hand-left)))
                       (not *handover-forbidden*))
                  (eql operation :move-base)
                  (eql operation :move-torso))))
        is-valid))))

(defun state-entropy (current-state goal-state)
  (let* ((goal-robot-pose (second (assoc :robot-pose goal-state)))
         (current-robot-pose (second (assoc :robot-pose current-state)))
         (goal-arrangement (cdr (assoc :arrangement goal-state)))
         (current-arrangement (cdr (assoc :arrangement current-state)))
         (goal-mode (second (assoc :mode goal-state)))
         (current-mode (second (assoc :mode current-state))))
    (cond ((eql goal-mode current-mode)
           (loop for set in goal-arrangement
                 as goal-level-zone = (first set)
                 as goal-object = (second set)
                 as current-object = (second
                                      (find goal-level-zone current-arrangement
                                            :test
                                            (lambda (subject list-item)
                                              (equal (first subject)
                                                     (first (first list-item))))))
                 when (not (string= current-object goal-object))
                   counting 1 into wrongly-placed
                 counting 1 into total
                 finally (return (/ (+ wrongly-placed
                                       (cond ((= current-robot-pose goal-robot-pose)
                                              0)
                                             (t 1)))
                                    (+ total 1)))))
          ((and (eql goal-mode :generic)
                (eql current-mode :explicit))
           (loop for set in goal-arrangement
                 as goal-level-zone = (first set)
                 as goal-class = (second set)
                 as current-class = (let ((item
                                            (second
                                             (find goal-level-zone current-arrangement
                                                   :test
                                                   (lambda (subject list-item)
                                                     (equal (first subject)
                                                            (first (first list-item))))))))
                                      (cond (item
                                             (get-item-class-cached item))
                                            (t "")))
                 when (not (string= current-class goal-class))
                   counting 1 into wrongly-placed
                 counting 1 into total
                 finally (return (/ (+ wrongly-placed
                                       (cond ((= current-robot-pose goal-robot-pose)
                                              0)
                                             (t 1)))
                                    (+ total 1)))))
          (t (assert nil)))))

(defun distance-between (state-1 state-2)
  (state-entropy state-1 state-2))

(defun heuristic-cost-estimate (state-1 state-2 &optional transition)
  (+ (distance-between state-1 state-2)
     (cond (transition
            (ecase (first transition)
              (:pick 1)
              (:place 1)
              (:handover 1.5)
              (:move-base 1.25)
              (:move-torso 2)))
           (t 0))))

(defun apply-transition (state transition)
  (let* ((robot-pose (second (assoc :robot-pose state)))
         (torso-height (second (assoc :torso-height state)))
         (in-hand-left
           (second (assoc :left (second
                                 (assoc :in-hand state)))))
         (in-hand-right
           (second (assoc :right (second
                                  (assoc :in-hand state)))))
         (arrangement (cdr (assoc :arrangement state)))
         (operation (first transition)))
    (ecase operation
      (:pick
       (let ((object (second transition))
             (side (third transition)))
         (make-planning-state
          robot-pose
          (remove-if (lambda (set)
                       (string= (second set) object))
                     arrangement)
          :torso-height torso-height
          :in-hand-left (cond ((eql side :left)
                               object)
                              (t in-hand-left))
          :in-hand-right (cond ((eql side :right)
                                object)
                               (t in-hand-right)))))
      (:place
       (let* ((object (second transition))
              (target (third transition)))
         (make-planning-state
          robot-pose
          (append
           arrangement
           `(((,target (0.0 0.0 0.0)) ,object)))
          :torso-height torso-height
          :in-hand-left (cond ((string= in-hand-left object)
                               nil)
                              (t in-hand-left))
          :in-hand-right (cond ((string= in-hand-right object)
                                nil)
                               (t in-hand-right)))))
      (:handover
       (make-planning-state
        robot-pose
        arrangement
        :torso-height torso-height
        :in-hand-left in-hand-right
        :in-hand-right in-hand-left))
      (:move-base
       (let ((position (second transition)))
         (make-planning-state
          position
          arrangement
          :torso-height torso-height
          :in-hand-left in-hand-left
          :in-hand-right in-hand-right)))
      (:move-torso
       (let ((position (second transition)))
         (make-planning-state
          robot-pose
          arrangement
          :torso-height position
          :in-hand-left in-hand-left
          :in-hand-right in-hand-right))))))

(defun reconstruct-path (came-from state &optional inject-into-came-from)
  (let ((first t)
        (total-path `(,state))
        (came-from (cond (inject-into-came-from
                          (let ((came-from-copy came-from))
                            (setstate state came-from-copy
                                      inject-into-came-from)
                            came-from-copy))
                         (t came-from))))
    (loop while (getstate state came-from nil first)
          do (setf state (getstate state came-from nil first))
             (push state total-path)
             (setf first nil))
    total-path))

(defun lowest-score-key (state-map)
  (let ((lowest-score 10000)
        (lowest-key nil))
    (loop for key in (statekeys state-map)
          when (< (getstate key state-map) lowest-score)
            do (setf lowest-score (getstate key state-map))
               (setf lowest-key key))
    lowest-key))

(defun lowest-state-score (states scores)
  (let ((lowest-score most-positive-fixnum)
        (lowest-state nil))
    (cond ((= (length states) 1)
           (setf lowest-score (first scores))
           (setf lowest-state (first states)))
          (t (loop for state in states
                   when (< (getstate state scores) lowest-score)
                     do (setf lowest-score (getstate state scores))
                        (setf lowest-state state))))
    (values lowest-state lowest-score)))

(defun arrangement-sets-equal? (arrangement-set-1 arrangement-set-2)
  (let ((level-zone-1 (first (first arrangement-set-1)))
        (level-zone-2 (first (first arrangement-set-2)))
        (object-1 (second arrangement-set-1))
        (object-2 (second arrangement-set-2)))
    (and (equal level-zone-1 level-zone-2)
         (string= object-1 object-2))))

(defun states-equal? (state-1 state-2 &key relaxed)
  (let* ((robot-pose-1 (second (assoc :robot-pose state-1)))
         (torso-height-1 (second (assoc :torso-height state-1)))
         (in-hand-left-1
           (second (assoc :left (second
                                 (assoc :in-hand state-1)))))
         (in-hand-right-1
           (second (assoc :right (second
                                  (assoc :in-hand state-1)))))
         (arrangement-1 (cdr (assoc :arrangement state-1)))
         (mode-1 (or (second (assoc :mode state-1)) :explicit))
         (robot-pose-2 (second (assoc :robot-pose state-2)))
         (torso-height-2 (second (assoc :torso-height state-2)))
         (in-hand-left-2
           (second (assoc :left (second
                                 (assoc :in-hand state-2)))))
         (in-hand-right-2
           (second (assoc :right (second
                                  (assoc :in-hand state-2)))))
         (arrangement-2 (cdr (assoc :arrangement state-2)))
         (mode-2 (or (second (assoc :mode state-2)) :explicit)))
    (and (or nil;relaxed
             (and (= robot-pose-1 robot-pose-2)
                  (= torso-height-1 torso-height-2)))
         (equal in-hand-left-1 in-hand-left-2)
         (equal in-hand-right-1 in-hand-right-2)
         (block check-arrangement
           (labels ((set-present (set arrangement mode-set mode-arrangement)
                      (cond ((eql mode-set mode-arrangement)
                             (find set arrangement :test #'arrangement-sets-equal?))
                            ((and (eql mode-set :explicit)
                                  (eql mode-arrangement :generic))
                             (find `(,(first set) ,(get-item-class-cached
                                                    (second set)))
                                   arrangement :test #'arrangement-sets-equal?))
                            ((and (eql mode-set :generic)
                                  (eql mode-arrangement :explicit))
                             (find set arrangement
                                   :test
                                   (lambda (subject list-item)
                                     (and (equal (first (first subject)) (first (first list-item)))
                                          (string= (second subject)
                                                   (get-item-class-cached
                                                    (second list-item))))))))))
             (loop for set in arrangement-1
                   when (not (set-present set arrangement-2 mode-1 mode-2))
                     do (return-from check-arrangement nil)))
           t))))

(defun make-state-map ()
  `())

(defun statekeys (state-map)
  (loop for state in state-map
        collect (first state)))

(defun getstate (key-state state-map &optional default relaxed)
  (let ((value (second
                (find key-state state-map
                      :test (lambda (subject list-item)
                              (states-equal? subject (first list-item)
                                             :relaxed relaxed))))))
    (or value default)))

(defun tic ()
  (setf *tic* (/ (get-internal-real-time) 1000.0)))

(defun toc()
  (- (/ (get-internal-real-time) 1000.0) *tic*))

(defun modified-a-star (start-state goal-state &key (number-of-solutions 1))
  (let ((closed-set `())
        (open-set `(,start-state))
        (came-from (make-state-map))
        (g-score (make-state-map))
        (f-score (make-state-map))
        (transitions-map `())
        (solutions `())
        (solver-durations `()))
    (setstate start-state g-score 0)
    (setstate start-state f-score
              (+ (getstate start-state g-score)
                 (heuristic-cost-estimate start-state goal-state)))
    (block a-star-main
      (tic)
      (loop while open-set
            as current-state = (multiple-value-bind (state score)
                                   (lowest-state-score open-set f-score)
                                 (declare (ignore score))
                                 state)
            if (progn
                 (states-equal? current-state goal-state :relaxed t))
              do (push `(,(reconstruct-path
                           came-from
                           current-state)
                         ,transitions-map)
                       solutions)
                 (push (toc) solver-durations)
                 (return-from a-star-main)
            else
              do (setf open-set (remove-if
                                 (lambda (subject-state)
                                   (states-equal? subject-state current-state))
                                 open-set))
                 (push current-state closed-set)
                 (loop for transition in (make-transitions
                                          current-state goal-state)
                       as projected-state = (apply-transition current-state
                                                              transition)
                       do (unless (find projected-state closed-set
                                        :test #'states-equal?)
                            (push `(,current-state ,projected-state ,transition)
                                  transitions-map)
                            (let ((tentative-g-score
                                    (+ (getstate current-state g-score
                                                 most-positive-fixnum)
                                       (distance-between current-state
                                                         projected-state)
                                       (cond ((states-equal? projected-state goal-state
                                                             :relaxed t)
                                              (if (= number-of-solutions 1)
                                                  0
                                                  (progn
                                                    (format t "~a more solution~a to generate~%"
                                                            number-of-solutions
                                                            (cond ((= number-of-solutions 1) "")
                                                                  (t "s")))
                                                    (decf number-of-solutions)
                                                    (push `(,(reconstruct-path
                                                              came-from
                                                              projected-state
                                                              current-state)
                                                            ,transitions-map)
                                                          solutions)
                                                    (push (toc) solver-durations)
                                                    (tic)
                                                    most-positive-fixnum)))
                                             (t 0)))))
                              (block intermediate-check
                                (if (not (find projected-state open-set
                                               :test #'states-equal?))
                                    (push projected-state open-set)
                                    (when (>= tentative-g-score
                                              (getstate projected-state g-score
                                                        most-positive-fixnum))
                                      (return-from intermediate-check)))
                                (setstate projected-state came-from
                                          current-state)
                                (setstate projected-state g-score
                                          tentative-g-score)
                                (setstate projected-state f-score
                                          (+ (getstate projected-state g-score
                                                       most-positive-fixnum)
                                             (heuristic-cost-estimate
                                              projected-state goal-state
                                              transition))))))))
      (format t "FAILURE~%"))
    (mapcar (lambda (solution duration)
              (let ((steps (first solution))
                    (transitions (second solution)))
                (append
                 `(,duration)
                 (loop for i from 0 below (1- (length steps))
                       as step-current = (nth i steps)
                       as step-next = (nth (+ i 1) steps)
                       collect
                       `(,step-current
                         ,(third
                           (find `(,step-current ,step-next)
                                 transitions
                                 :test (lambda (subject list-item)
                                         (and (states-equal?
                                               (first subject)
                                               (first list-item))
                                              (states-equal?
                                               (second subject)
                                     (second list-item))))))))
                 `(,(last steps) nil))))
            (reverse solutions) (reverse solver-durations))))

(defun display-state (state &key highlighted-objects)
  (let* ((arrangement (cdr (assoc :arrangement state)))
         (object-poses (make-hash-table :test 'equal))
         (objects (loop for object-data in arrangement
                        collect
                        (destructuring-bind (((level zone)
                                              (x y theta))
                                             object)
                            object-data
                          (let ((dimensions (get-item-dimensions-cached object)))
                            (setf (gethash object object-poses)
                                  (make-zone-pose
                                   level zone x y
                                   :z (/ (elt dimensions 2) 2)
                                   :orientation (tf:euler->quaternion
                                                 :az theta))))
                          object))))
    (display-objects objects :object-poses object-poses :highlighted-objects highlighted-objects)
    (let ((object-zones (assess-object-zones
                         objects :object-poses object-poses)))
      (display-zones :highlight-zones object-zones))))

(defun display-solution (solution)
  (loop for state-transition in solution
        do (destructuring-bind (state transition)
               state-transition
             (declare (ignore transition))
             (display-state state))))

(defun calculate-all-metrics (solutions)
  (let ((duration-offset 0.0))
    (loop for solution in solutions
          as metrics = (calculate-metrics solution)
          collect
          (prog1
            (calculate-metrics
             solution
             :duration-offset duration-offset)
            (incf duration-offset (first solution))))))

(defun calculate-metrics (solution &key (duration-offset 0))
  (let* ((duration (+ (first solution) duration-offset))
         (solution (rest solution))
         (action-sequence (action-sequence solution))
         (length (length action-sequence))
         (operation-counts
           (let ((operations
                   `(:pick :place :move-torso :move-base :handover)))
             (loop for s-operation in operations
                   collect
                   `(,s-operation
                     ,(loop for step in action-sequence
                            as operation = (first step)
                            when (eql operation
                                      s-operation)
                              sum 1))))))
    `(,duration ,length ,operation-counts)))

(defun pretty-print-metrics (solutions)
  (let ((all-metrics (calculate-all-metrics solutions)))
    (mapcar (lambda (solution metrics)
              (let ((duration (first metrics))
                    (length (second metrics))
                    (components (first metrics)))
                ))
            solutions all-metrics)
    (loop for metrics in all-metrics
          with duration-offset = 0
          collect (prog1 (- (first metrics) duration-offset)
                    (setf duration-offset (first metrics))))))

(defun normalize-durations (durations)
  (let ((largest (loop for i in durations maximizing i)))
    (mapcar (lambda (r) (/ r largest)) durations)))

(defun execute-action-sequence (sequence)
  (loop for step in sequence
        do (execute-action-step step)))

(defun execute-action-step (step)
  (format t "Executing step: ~a~%" step)
  (ecase (first step)
    (:pick
     (let* ((object-name (second step))
            (side (third step))
            (object (get-item-designator object-name)))
       (let ((allowed-arms pr2-manip-pm::*allowed-arms*))
         (setf pr2-manip-pm::*allowed-arms* `(,side))
         (try-forever
           (pick-object object :stationary t))
         (setf pr2-manip-pm::*allowed-arms* allowed-arms))))
    (:place
     (let* ((object-name (second step))
            (object (get-item-designator object-name))
            (level-zone (third step))
            (zone-pose
	     (make-zone-pose
	      (first level-zone)
	      (second level-zone)
	      -0.2 0.0
	      :z 0.02
	      :orientation
	      (tf:euler->quaternion :az (/ pi 2))
	      :elevation
	      (cond (*gazebo*
		     (let ((dimensions
			    (desig-prop-value object 'desig-props::dimensions)))
		       (- (/ (elt dimensions 2) 2))))
		    (t 0.0)))))
       (look-at-level-zone
        (first level-zone) (second level-zone))
       (try-forever
	(place-object object (make-designator 'location `((desig-props:pose ,zone-pose)))
		      :stationary t))))
    (:handover
     ;; TODO: Handover (probably not for the robot experiment)
     )
    (:move-base
     (let* ((position (second step)))
       (go-to-rack-relativ-pose (* position -0.5))))
    (:move-torso
     (let* ((position (second step))
            (rack (first (get-racks)))
            (level (get-rack-on-level rack position))
            (elevation (get-rack-level-elevation level)))
       (move-torso (/ elevation 5.0))))))

(def-top-level-cram-function experiment (&key evaluate-solutions)
  (with-process-modules
    (prepare-settings)
    (move-arms-away)
    ;;(execute-action-step `(:move-torso 1))
    (remove-all-shopping-items)
    (setf *perceived-objects* nil)
    (setf *item-designators* (make-hash-table :test 'equal))
    (setf *object-poses* (make-hash-table :test 'equal))
    (setf *object-dimensions* (make-hash-table :test 'equal))
    (setf *handover-forbidden* t)
    (setf *min-level* 1)
    (setf *max-level* 2)
    (setf *min-zone* 0)
    (setf *max-zone* 3)
    (perceive-rack-full)
    (populate-knowledge-base)
    (execute-action-step `(:move-base 0))
    (let* ((current-state (make-planning-state
                           0 (get-current-arrangement)))
           (target-state (make-planning-state
                          0
                          `((((2 1) (0.0 0.0 0.0)) "TomatoSauce")
                            (((2 2) (0.0 0.0 0.0)) "PancakeMix")
                            (((2 3) (0.0 0.0 0.0)) "PancakeMix")
                            (((1 2) (0.0 0.0 0.0)) "Lion")
                            (((1 3) (0.0 0.0 0.0)) "Kelloggs"))
                          :mode :generic))
           (solutions (modified-a-star current-state target-state))
           (action-sequences
             (mapcar (lambda (solution)
                       (action-sequence (rest solution)))
                     solutions)))
      (cond (evaluate-solutions
             solutions)
            (t (cond (action-sequences
                      (execute-action-sequence
                       (first action-sequences)))
                     (t (format
                         t "No valid action sequence found."))))))))

(defun look-at (pose)
  (with-designators ((look-at-action
                      (action
                       `((desig-props:type desig-props:trajectory)
                         (desig-props:to desig-props:see)
                         (desig-props:pose ,pose)))))
    (perform look-at-action)))

(defun look-at-level-zone (level zone)
  (let ((zone-pose (make-zone-pose level zone 0 0)))
    (look-at zone-pose)))

(defun go-to-rack-relativ-pose (y &key (x -0.85))
  (let ((pose (tf:make-pose-stamped
               "/shopping_rack" 0.0
               (tf:make-3d-vector y x 0.0)
               (tf:euler->quaternion :az (/ pi 2)))))
    (go-to-pose pose)))

(defun back-off ()
  (go-to-rack-relativ-pose 0 :x -1.4))

(defun perceive-rack-full ()
  ;(back-off)
  (look-at-level-zone 1 2)
  (setf *perceived-objects*
        (robosherlock-pm::perceive-object-designator
         (make-designator 'object nil))))

;;
;; The following functions are made for analysing data sets based on a
;; given ground truth
;;

(defvar *data-ground-truth* nil)
(defvar *data-sets* (make-hash-table :test 'equal))

(defun read-dataset (&key ground-truth)
  (let ((data (top-level
                (with-process-modules
                  (robosherlock-pm::perceive-object-designator
                   (make-designator 'object nil))))))
    (when ground-truth
      (setf *data-ground-truth* data))
    data))

(defun generate-action-sequence (dataset)
  (setf *perceived-objects* dataset)
  (solve))

(defun yes-no (yes-no)
  (cond (yes-no "yes")
        (t "no")))

(defun print-metric (metric value)
  (format t " - ~a: ~a~%" metric value))

(defun compare-data (datasets)
  (labels ((structurally-equivalent? (as-1 as-2)
             (block compare
               (when (= (length as-1) (length as-2))
                 (loop for subj1 in as-1
                       for subj2 in as-2
                       unless (eql (first subj1) (first subj2))
                         do (return-from compare))
                 t)))
           (dataset-metrics (dataset &key structural-equivalency-partner)
             (let* ((length-dataset (length dataset))
                    (as-dataset (generate-action-sequence dataset))
                    (length-as-dataset (length as-dataset)))
               (append
                `((:length ,length-dataset)
                  (:action-sequence ,as-dataset)
                  (:length-as-dataset ,length-as-dataset))
                (when structural-equivalency-partner
                  `((:structurally-equivalent
                     ,(structurally-equivalent?
                       as-dataset structural-equivalency-partner)))))))
           (get-detail (symbol dataset)
             (second (assoc symbol dataset)))
           (detail-present (symbol dataset)
             (not (not (find symbol dataset
                             :test (lambda (subject list-item)
                                     (eql subject (first list-item)))))))
           (print-metrics (dataset)
             (print-metric "Objects" (get-detail :length dataset))
             (print-metric "Actions" (get-detail :length-as-dataset dataset))
             (when (detail-present :structurally-equivalent dataset)
               (print-metric "Structurally equivalent to ground truth"
                             (yes-no (get-detail :structurally-equivalent
                                                 dataset))))))
    (let* ((ground-truth-metrics (dataset-metrics (first datasets)))
           (dataset-metrics
             (mapcar (lambda (dataset)
                       (dataset-metrics
                        dataset :structural-equivalency-partner
                        (get-detail :action-sequence ground-truth-metrics)))
                     (rest datasets))))
      (format t "Ground truth~%")
      (print-metrics ground-truth-metrics)
      (loop for i from 0 below (length dataset-metrics)
            as dataset-metric = (nth i dataset-metrics)
            do (format t "Dataset ~a~%" i)
               (print-metrics dataset-metric)))))

(defun displace-middle-objects (id-left id-right)
  (let ((action-sequence `((:move-base 0)
                           (:move-torso 2)
                           (:pick ,id-left :left)
                           (:pick ,id-right :right)
                           (:place ,id-left (2 2))
                           (:place ,id-right (2 1)))))
    (top-level
      (with-process-modules
        (setf *min-level* 1)
        (setf *max-level* 2)
        (move-arms-away)
        (execute-action-sequence action-sequence)))))

(defun take-object-from-behind (front back)
  (let ((action-sequence `(;(:move-base 0)
                           (:move-torso 2)
                           (:pick ,front :left)
                           (:pick ,back :right)
                           (:place ,front (2 3)))))
    (top-level
      (with-process-modules
        (setf *min-level* 1)
        (setf *max-level* 2)
        (move-arms-away)
        (execute-action-sequence action-sequence)))))

(defun moveit-clear-hands ()
  (let ((planning-scene
          (roslisp:call-service
           "/get_planning_scene"
           "moveit_msgs/GetPlanningScene"
           :components
           (roslisp:make-message
            "moveit_msgs/PlanningSceneComponents"
            :components 4))))
    (with-fields (scene) planning-scene
      (with-fields (robot_state) scene
        (with-fields (attached_collision_objects) robot_state
          (loop for i from 0 below (length attached_collision_objects)
                as attached-object = (elt attached_collision_objects i)
                do (with-fields (link_name object) attached-object
                     (with-fields (id) object
                       (moveit:detach-collision-object-from-link
                        (intern id) link_name)))))))))

(defun go-for-it-clean (&key skip-and-go)
  (unless skip-and-go
    (moveit-clear-hands)
    (moveit:clear-collision-environment)
    (prepare-settings)
    (setf *perceived-objects* nil)
    (setf *item-designators* (make-hash-table :test 'equal))
    (populate-knowledge-base))
  (format t "Got these objects:~%")
  (let ((pre-front nil)
        (pre-back nil))
    (loop for h being the hash-keys in *item-designators*
          do (cond ((string= (subseq h 0 4) "Lion")
                    (setf pre-back h))
                   ((string= (subseq h 0 4) "Salt")
                    (setf pre-front h)))
             (format t " - ~a~%" h))
    (format t "Front object~a: " (cond (pre-front
                                        (concatenate
                                         'string
                                         " (\"" pre-front "\")"))
                                       (t "")))
    (let ((front (read-line)))
      (format t "Back object~a: " (cond (pre-back
                                         (concatenate
                                          'string
                                          " (\"" pre-back "\")"))
                                        (t "")))
      (let ((back (read-line)))
        (let ((front (cond ((and (string= front "") pre-front)
                            pre-front)
                           (t front)))
              (back (cond ((and (string= back "") pre-back)
                            pre-back)
                           (t back))))
          (format t "'~a' and '~a'~%" front back)
          (take-object-from-behind front back))))))

(defun solution-states (solution)
  (loop for entry in solution
        collect (first entry)))

(def-top-level-cram-function perception-driven-plan ()
  (with-process-modules
    (setf *handover-forbidden* t)
    (setf *min-level* 1)
    (setf *max-level* 2)
    (let ((perceived-objects
            (setf *perceived-objects*
                  (robosherlock-pm::perceive-object-designator
                   (make-designator 'object nil)))))
      (assert-perceived-objects perceived-objects)
      (populate-knowledge-base)
      (let* ((current-state
               (make-planning-state 0 (get-current-arrangement)))
             (target-state (make-target-state current-state :mode :explicit))
             (solutions (modified-a-star current-state target-state))
             (solution (rest (first solutions)))
             (states (solution-states solution))
             (action-sequence (action-sequence solution)))
        (loop for state in states
              for action in `(,@action-sequence nil)
              when state
                do (format t "~a~%" action)
                   (when (or (eql (first action) :pick)
                             (eql (first action) :place)
                             (not action))
                     (display-state
                      former-state
                      :highlighted-objects (when action `(,(second action))))
                     (sleep 0.5)
                     (display-state
                      state
                      :highlighted-objects (when action `(,(second action))))
                     (sleep 0.5))
                   (setf former-state state)
              with former-state = state)))))

;; (defun perceive-rack-full (&key (back-off t))
;;   (when back-off (back-off))
;;   (look-at-level-zone 1 2)
;;   (with-designators ((generic-obj (object nil))
;;                      (perceive (action `((desig-props:to desig-props:perceive)
;;                                          (desig-props:obj ,generic-obj)))))
;;     (let ((ignorable-objects `("pr2" "ground_plane" "shopping_area" "shopping_rack")))
;;       (setf *perceived-objects*
;;             (mapcar #'rotate-if-necessary
;;                     (remove-if (lambda (subject)
;;                                  (find subject ignorable-objects
;;                                        :test (lambda (subject list-item)
;;                                                (string= (desig-prop-value
;;                                                          subject 'desig-props:name)
;;                                                         list-item))))
;;                                (perform perceive)))))))

;; (defun rotate-if-necessary (object)
;;   (cond (*gazebo*
;;          (let* ((pose (desig-prop-value
;;                        (desig-prop-value
;;                         object
;;                         'desig-props:at)
;;                        'desig-props:pose))
;;                 (transformed-pose
;;                   (tf:make-pose-stamped
;;                    (tf:frame-id pose)
;;                    (tf:stamp pose)
;;                    (tf:origin pose)
;;                    (tf:orientation
;;                     (cl-transforms:transform-pose
;;                      (cl-transforms:pose->transform
;;                       (cl-transforms:make-pose (tf:make-identity-vector)
;;                                                (tf:orientation pose)))
;;                      (tf:make-pose (tf:make-identity-vector)
;;                                    (cl-transforms:euler->quaternion :az (/ pi 2))))))))
;;            (make-effective-designator
;;             object
;;             :new-properties
;;             (update-designator-properties `((desig-props:at
;;                                              ,(make-designator
;;                                                'location
;;                                                `((desig-props:pose ,transformed-pose)))))
;;                                           (description object))
;;             :data-object (slot-value object 'desig:data))))
;;         (t object)))

;; (defun class-of-item (item &key (delimiter "_"))
;;   (let ((delim-pos (position delimiter item :test #'string=)))
;;     (cond (delim-pos
;;            (subseq item 0 delim-pos))
;;           (t item))))

;; (defun perceive-all-objects ()
;;   ;;(remove-all-shopping-items)
;;   (let ((objects (perceive-rack-full)))
;;     (dolist (object objects)
;;       (assert-shopping-item object (class-of-item object))
;;       (set-item-pose-cached (desig-prop-value object 'desig-props:name)
;;                             (desig-prop-value (desig-prop-value object 'desig-props:at)
;;                                               'desig-props:pose)))))
