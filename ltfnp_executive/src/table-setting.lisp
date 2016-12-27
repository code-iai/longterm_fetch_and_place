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
;;; Definitions
;;;

(defvar *scene-context* (make-hash-table))

;;;
;;; Problem Generator Functions
;;;

(defun generate-table-setting-problem ()
  (let* ((pool-guests `(:mary :tim))
         (pool-meal-times `(:breakfast :lunch :dinner))
         (pool-week-days `(:monday :tuesday :wednesday :thursday
                           :friday :saturday :sunday))
         (week-day (nth (random (length pool-week-days))
                        pool-week-days))
         (meal-time (nth (random (length pool-meal-times))
                         pool-meal-times))
         (guests (if (and (not (weekend-p week-day))
                          (eql meal-time :lunch))
                     `(,(nth (random (length pool-guests))
                             pool-guests))
                     (let ((temp-guests
                             (loop for guest in pool-guests
                                   when (>= (random 10)
                                            (if (weekend-p week-day)
                                                2 5))
                                     collect guest)))
                       (if (= (length temp-guests) 0)
                           `(,(nth (random (length pool-guests)) pool-guests))
                           temp-guests)))))
    `(,guests ,meal-time ,week-day)))

;;;
;;; Helper Functions
;;;

(defun clear-scene ()
  (setf *scene-context* (make-hash-table)))

(defun set-scene-detail (detail value)
  (setf (gethash detail *scene-context*) value))

(defun set-random-scene ()
  (destructuring-bind (guests meal-time week-day)
      (generate-table-setting-problem)
    (set-scene-detail :guests guests)
    (set-scene-detail :meal-time meal-time)
    (set-scene-detail :week-day week-day)))

(defun scene-detail (detail)
  (gethash detail *scene-context*))

(defun set-scene-1 ()
  (set-scene-detail :guests `(:tim))
  (set-scene-detail :meal-time :breakfast)
  (set-scene-detail :week-day :saturday))

(defun rectangular-costmap-generator (x-r y-r w h)
  (lambda (x y)
    (if (and (>= x x-r)
             (>= y y-r)
             (< x (+ x-r h))
             (< y (+ y-r w)))
        1.0d0
        0.0d0)))

(defun seat-center (seat)
  ;; TODO: Needs fixing for ltfnp
  (case seat
    (1 '(-0.89 0.85))
    (2 '(-0.89 1.5))))

(defun seat-costmap-generator (seat)
  (let ((seat-width 0.5)
        (seat-height 0.4))
    (or (and (> seat 0)
             (< seat 3)
             (let ((seat-center (seat-center seat))
                   (half-width (/ seat-width 2))
                   (half-height (/ seat-height 2)))
               (rectangular-costmap-generator
                (- (first seat-center) half-height)
                (- (second seat-center) half-width)
                seat-width
                seat-height)))
        (lambda (x y)
          (declare (ignore x y))
          0.0d0))))

(defun seat-relative-costmap-generator (seat &key relation)
  (let ((seat-width 0.5)
        (seat-height 0.4))
    (let ((seat-center (seat-center seat))
          (half-width (/ seat-width 2))
          (half-height (/ seat-height 2)))
      (case relation
        (:near (rectangular-costmap-generator
                (- (first seat-center) half-height)
                (- (second seat-center) half-width)
                seat-width
                seat-height))
        (:left-of (rectangular-costmap-generator
                   (- (first seat-center) half-height)
                   (- (second seat-center) half-width)
                   (* seat-width 0.4)
                   seat-height))
        (:right-of (rectangular-costmap-generator
                    (- (first seat-center) half-height)
                    (+ (* seat-width 0.6)
                       (- (second seat-center) half-width))
                    (* seat-width 0.4)
                    seat-height))
        (:behind-of (rectangular-costmap-generator
                     (- (first seat-center) half-height)
                     (- (second seat-center) half-width)
                     seat-width
                     (* seat-height 0.4)))
        (:center-of (rectangular-costmap-generator
                     (+ (* seat-width 0.4)
                        (- (first seat-center) half-height))
                     (+ (* seat-height 0.3)
                        (- (second seat-center) half-width))
                     (* seat-width 0.4)
                     (* seat-height 0.6)))))))

(defun positions->seat-location (seat positions)
  (make-designator
   :location
   (append `((:seat ,seat)
             (:on CounterTop)
             (:name "meal_table"))
           (mapcar (lambda (position)
                     `(,position seat))
                   positions))))

(defun object-type->object (object-type location guest meal)
  (make-designator
   :object
   `((:at ,location)
     (:type ,object-type)
     (:for-guest ,guest)
     (:for-meal ,meal))))

(defun order-scene-objects (objects)
  (let* ((object-seat-placement-modifiers
           (mapcar
            (lambda (object)
              (let* ((at (desig:desig-prop-value object :at))
                     (seat-placements
                       (cpl:mapcar-clean
                        (lambda (property)
                          (when (eql (cadr property) :seat)
                            (car property)))
                        (desig:description at))))
                (cons object seat-placements)))
            objects)))
    (mapcar
     #'first
     (sort object-seat-placement-modifiers
           (lambda (set-1 set-2)
             (mapcar
              (lambda (modif-1)
                (mapcar
                 (lambda (modif-2)
                   (let ((orderings
                           (force-ll
                            (lazy-mapcar
                             (lambda (bdgs)
                               (with-vars-bound (?order) bdgs
                                 ?order))
                             (cram-prolog:prolog
                              `(seat-place-ordering
                                ,modif-1 ,modif-2
                                ?order))))))
                     (not (find :after orderings))))
                 (rest set-2)))
              (rest set-1)))))))

(defun required-scene-objects ()
  (let* ((lazy-scene-objects (cram-prolog:prolog `(required-object ?object)))
         (scene-objects
           (force-ll
            (lazy-mapcar (lambda (bdgs)
                           (with-vars-bound (?object) bdgs
                             ?object))
                         lazy-scene-objects))))
    scene-objects))

(defun weekend-p (day)
  (not (not (cram-prolog:prolog `(weekend? ,day)))))

(defun object-list (scene-objects)
  (loop for object in scene-objects
        collect (desig:desig-prop-value object :type)))

;;;
;;; Plans
;;;

;; (def-top-level-cram-function set-table ()
;;   (beliefstate:enable-logging nil)
;;   (prepare-settings)
;;   (beliefstate:enable-logging t)
;;   (set-scene-1)
;;   (with-process-modules
;;     (let ((required-objects (required-scene-objects)))
;;       (dolist (required-object required-objects)
;;         (with-designators ((object (object
;;                                     `((desig-props:at
;;                                        ,(make-designator
;;                                          :location
;;                                          `((desig-props::on Cupboard)
;;                                            (desig-props::name "kitchen_sink_block"))))
;;                                       ,@(remove 'at (desig:properties required-object)
;;                                                 :key #'car))))
;;                            (location (location
;;                                       (description (desig-prop-value
;;                                                     required-object 'at)))))
;;           (ensure-arms-up)
;;           (try-forever
;;             (pick-object object)
;;             (place-object object location)))))))

;;;
;;; Facts
;;;

(defmethod costmap-generator-name->score ((name (common-lisp:eql 'seat-distribution))) 18)

(defmethod costmap-generator-name->score ((name (common-lisp:eql 'seat-distribution-relative-left))) 19)
(defmethod costmap-generator-name->score ((name (common-lisp:eql 'seat-distribution-relative-right))) 20)
(defmethod costmap-generator-name->score ((name (common-lisp:eql 'seat-distribution-relative-behind))) 21)
(defmethod costmap-generator-name->score ((name (common-lisp:eql 'seat-distribution-relative-center))) 22)
(defmethod costmap-generator-name->score ((name (common-lisp:eql 'seat-distribution-relative-near))) 23)

(def-fact-group object-validity-facts (perceived-object-invalid)
  
  (<- (invalid-object-link "l_shoulder_pan_link"))
  (<- (invalid-object-link "l_shoulder_lift_link"))
  (<- (invalid-object-link "l_upper_arm_roll_link"))
  (<- (invalid-object-link "l_upper_arm_link"))
  (<- (invalid-object-link "l_elbow_flex_link"))
  (<- (invalid-object-link "l_forearm_roll_link"))
  (<- (invalid-object-link "l_forearm_cam_frame"))
  (<- (invalid-object-link "l_forearm_cam_optical_frame"))
  (<- (invalid-object-link "l_forearm_link"))
  (<- (invalid-object-link "l_wrist_flex_link"))
  (<- (invalid-object-link "l_wrist_roll_link"))
  (<- (invalid-object-link "l_gripper_palm_link"))

  (<- (invalid-object-link "r_shoulder_pan_link"))
  (<- (invalid-object-link "r_shoulder_lift_link"))
  (<- (invalid-object-link "r_upper_arm_roll_link"))
  (<- (invalid-object-link "r_upper_arm_link"))
  (<- (invalid-object-link "r_elbow_flex_link"))
  (<- (invalid-object-link "r_forearm_roll_link"))
  (<- (invalid-object-link "r_forearm_cam_frame"))
  (<- (invalid-object-link "r_forearm_cam_optical_frame"))
  (<- (invalid-object-link "r_forearm_link"))
  (<- (invalid-object-link "r_wrist_flex_link"))
  (<- (invalid-object-link "r_wrist_roll_link"))
  (<- (invalid-object-link "r_gripper_palm_link"))
  
  (<- (perceived-object-invalid ?object)
    (desig-prop ?object (at ?loc))
    (invalid-object-link ?link)
    (cram-prolog:lisp-fun reference ?loc ?pose)
    (cram-prolog:lisp-fun pose-link-origin-distance ?pose ?link ?distance)
    (<= ?distance 0.2)))

(def-fact-group table-setting-costmap-facts (desig-costmap)
  
  (<- (distribution-symbol :left-of seat-distribution-relative-left))
  (<- (distribution-symbol :right-of seat-distribution-relative-right))
  (<- (distribution-symbol :behind-of seat-distribution-relative-behind))
  (<- (distribution-symbol :center-of seat-distribution-relative-center))
  (<- (distribution-symbol :near seat-distribution-relative-near))
  
  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (:seat ?seat))
    (desig-prop ?desig (?relation seat))
    (costmap ?cm)
    (distribution-symbol ?relation ?distrib)
    (costmap-add-function ?distrib
                          (seat-relative-costmap-generator
                           ?seat :relation ?relation)
                          ?cm)))

(def-fact-group table-setting-facts ()
  
  ;; Housekeeping predicates
  (<- (context-prop ?context ?detail ?value)
    (cram-prolog:lisp-fun gethash ?detail ?context ?value))
  
  (<- (context ?context)
    (symbol-value *scene-context* ?context))
  
  (<- (context-prop ?detail ?value)
    (context ?context)
    (context-prop ?context ?detail ?value))
  
  (<- (context-prop-amount ?detail ?amount)
    (context-prop ?detail ?value)
    (length ?value ?amount))
  
  (<- (preference ?context ?person ?preference ?value)
    (cram-prolog:fail))
  
  (<- (guest ?guest)
    (context-prop :guests ?guests)
    (member ?guest ?guests))
  
  (<- (seat-description ?seat (:seat ?seat)))
  
  (<- (weekend? :saturday))
  (<- (weekend? :sunday))
  (<- (workday? ?day)
    (not (weekend? ?day)))
  
  ;; Who likes which meals during what time of the day
  (<- (preference :tim :dish :muesli)
    (context-prop :meal-time :breakfast))
  
  (<- (preference :tim :dish :soup)
    (context-prop :meal-time :lunch))
  
  (<- (preference :tim :dish :bread)
    (context-prop :meal-time :dinner))

  (<- (preference :mary :dish :soup)
    (context-prop :meal-time :dinner))
  
  (<- (preference :mary :dish :coffee)
    (context-prop :meal-time :breakfast))
  
  (<- (preference :mary :dish :bread))

  (<- (preference :tim :dish :wine)
    (context-prop :meal-time :dinner))
  
  (<- (preference :mary :dish :wine)
    (context-prop :meal-time :dinner))
  
  ;; Who sits where
  (<- (preference :tim :seat 2)
    (context-prop-amount :guests ?guest-count)
    (> ?guest-count 1))
  
  (<- (preference :tim :seat 1)
    (context-prop-amount :guests ?guest-count)
    (cram-prolog:== ?guest-count 1))
  
  (<- (preference :mary :seat 1))
  
  ;; Objects for meals
  (<- (required-meal-object :muesli :bowl))
  (<- (required-meal-object :muesli :muesli))
  (<- (required-meal-object :muesli :milkbox))
  (<- (required-meal-object :muesli :spoon))
  
  (<- (required-meal-object :bread :knife))
  (<- (required-meal-object :bread :plate))
  
  (<- (required-meal-object :coffee :cup))
  
  (<- (required-meal-object :wine :glass))
  
  (<- (required-meal-object :soup :bowl))
  (<- (required-meal-object :soup :spoon))
  
  ;; General rules for table setting object placement
  (<- (center-relative-object-table-position :bowl :center-of))
  (<- (center-relative-object-table-position :plate :center-of))
  (<- (center-relative-object-table-position :fork :left-of))
  (<- (center-relative-object-table-position :knife :right-of))
  (<- (center-relative-object-table-position :spoon :left-of))
  (<- (center-relative-object-table-position :cup :left-of))
  (<- (center-relative-object-table-position :cup :behind-of))
  
  (<- (center-relative-object-table-position ?_ :near))
  
  ;; Ordering of costmap-based object placement
  (<- (seat-place-ordering :left-of ?_ :before))
  (<- (seat-place-ordering :right-of ?_ :before))
  (<- (seat-place-ordering :center-of ?_ :after))
  (<- (seat-place-ordering :behind-of ?_ :before))
  (<- (seat-place-ordering :near ?_ :after))
  
  (<- (seat-place-ordering ?a ?b :before)
    (seat-place-ordering ?b ?a :after))
  (<- (seat-place-ordering ?a ?b :after))
  
  ;; Overall collection predicates
  (<- (required-object ?object)
    (guest ?guest)
    (preference ?guest :seat ?seat)
    (seat-description ?seat ?seat-description)
    (preference ?guest :dish ?meal)
    (required-meal-object ?meal ?object-type)
    (cram-prolog:setof ?position (center-relative-object-table-position
                          ?object-type ?position)
               ?positions)
    (cram-prolog:lisp-fun positions->seat-location ?seat ?positions ?location)
    (cram-prolog:lisp-fun object-type->object
                  ?object-type ?location ?guest ?meal ?object)))

(defun look-point (point &key (frame "base_link"))
  (look-at (make-designator :location `((:pose ,(tf:make-pose-stamped frame 0.0 point (tf:make-identity-rotation)))))))

(defun move-arm (side translation rotation &key (frame "base_link"))
  (pr2-manip-pm::execute-move-arm-poses side `(,(tf:make-pose-stamped frame 0.0 translation rotation)) (pr2-manip-pm::make-empty-goal-specification)))

(def-top-level-cram-function mock-perception-run ()
  (setf cram-moveit::*needs-ft-fix* t)
  ;; Old park poses
  (setf pr2-manip-pm::*park-pose-left-default*
        (tf:make-pose-stamped
         "torso_lift_link" (ros-time)
         (cl-transforms:make-3d-vector 0.1 0.45 0.3)
         (cl-transforms:euler->quaternion :ay (/ pi -2))))
  (setf pr2-manip-pm::*park-pose-right-default*
        (tf:make-pose-stamped
         "torso_lift_link" (ros-time)
         (cl-transforms:make-3d-vector 0.1 -0.45 0.3)
         (cl-transforms:euler->quaternion :ay (/ pi -2))))
  (with-process-modules
    ;; Torso up
    (move-torso)
    ;; Arms up
    (move-arms-up-2)
    ;; In front of sink area
    (go-to-pose (tf:make-3d-vector 0.693 0.498 0.052)
                (tf:make-quaternion 0.0 0.0 0.0 1.0)
                :frame "map")
    ;; Look at area
    (look-point (tf:make-3d-vector 0.5 0.0 1.05))
    ;; Open gripper
    (pr2-manip-pm::open-gripper :left)
    ;; Pregrasp plate
    (move-arm :left (tf:make-3d-vector 0.6 0.33 1.02) (tf:euler->quaternion :ax (/ pi 2) :ay (* pi 0.2) :az -0.8))
    ;; Grasp plate
    (move-arm :left (tf:make-3d-vector 0.63 0.27 0.97) (tf:euler->quaternion :ax (/ pi 2) :ay (* pi 0.2) :az -0.8))
    ;; Close gripper
    (pr2-manip-pm::close-gripper :left)
    ;; New park pose -- left
    (setf pr2-manip-pm::*park-pose-left-default*
          (tf:make-pose-stamped
           "torso_lift_link" 0.0
           (tf:make-3d-vector 0.4 0.5 0.1)
           (tf:euler->quaternion :ax (/ pi 2) :ay 0.45 :az (/ pi -2))))
    ;; Arms up
    (move-arms-up-2)
    ;; Look at area
    (look-point (tf:make-3d-vector 0.5 -0.2 1.05))
    ;; Open gripper
    (pr2-manip-pm::open-gripper :right)
    ;; Pregrasp knife
    (move-arm :right (tf:make-3d-vector 0.7 -0.15 1.1) (tf:euler->quaternion :ay (/ pi 2)))
    ;; Grasp knife
    (move-arm :right (tf:make-3d-vector 0.7 -0.15 0.97) (tf:euler->quaternion :ay (/ pi 2)))
    ;; Close gripper
    (pr2-manip-pm::close-gripper :right)
    ;; New park pose -- right
    (setf pr2-manip-pm::*park-pose-right-default*
          (tf:make-pose-stamped
           "torso_lift_link" 0.0
           (tf:make-3d-vector 0.4 -0.3 0.2)
           (tf:euler->quaternion :az (/ pi 2))))
    ;; Arms up
    (move-arms-up-2)
    ;; After pickup, look at scene again
    (sleep 10)
    ;; In front of sink area
    (go-to-pose (tf:make-3d-vector -0.747 -0.352 0.05)
                (tf:make-quaternion 0.0 0.0 0.680 -0.733)
                :frame "map")
    ;; Look at area
    (look-point (tf:make-3d-vector 0.5 -0.3 0.5))
    ;; Putdown plate
    (move-arm :left (tf:make-3d-vector 0.5 -0.0 0.90) (tf:euler->quaternion :ax (/ pi 2) :ay (* pi 0.2) :az -0.8))
    ;; Open gripper
    (pr2-manip-pm::open-gripper :left)
    ;; Correction to release plate
    (move-arm :left (tf:make-3d-vector 0.5 -0.0 1.05) (tf:euler->quaternion :ax (/ pi 2) :ay (/ pi 2)))
    ;; Putdown knife
    (move-arm :right (tf:make-3d-vector 0.6 -0.4 0.90) (tf:euler->quaternion :ay (/ pi 2)))
    ;; Open gripper
    (pr2-manip-pm::open-gripper :right)
    ;; Old park poses
    (setf pr2-manip-pm::*park-pose-left-default*
          (tf:make-pose-stamped
           "torso_lift_link" (ros-time)
           (cl-transforms:make-3d-vector 0.1 0.45 0.3)
           (cl-transforms:euler->quaternion :ay (/ pi -2))))
    (setf pr2-manip-pm::*park-pose-right-default*
          (tf:make-pose-stamped
           "torso_lift_link" (ros-time)
           (cl-transforms:make-3d-vector 0.1 -0.45 0.3)
           (cl-transforms:euler->quaternion :ay (/ pi -2))))
    (move-arms-up-2)))

(defun transform-pose (daisy-chain)
  (let ((len (length daisy-chain)))
    (cond ((= len 0) nil)
          ((= len 1) (first daisy-chain))
          ((= len 2)
           (destructuring-bind (base-pose trafo-pose) daisy-chain
             (let ((transformed
                     (cl-transforms:transform-pose
                      (tf:pose->transform base-pose)
                      trafo-pose)))
               (cond ((eql (class-name (class-of base-pose)) 'tf:pose-stamped)
                      (tf:pose->pose-stamped
                       (tf:frame-id base-pose)
                       (tf:stamp base-pose)
                       transformed))
                     (t transformed)))))
          (t (transform-pose `(,(first daisy-chain) ,(transform-pose (rest daisy-chain))))))))

(defun show-pose (pose)
  (roslisp:publish (roslisp:advertise "/blablabla" "geometry_msgs/PoseStamped")
                   (tf:to-msg pose)))

(defun resolve-seat-location (loc object-type)
  (labels ((translation (vector)
             (tf:make-pose vector (tf:make-identity-rotation)))
           (translate (x y z)
             (translation (tf:make-3d-vector x y z)))
           (rotation (quaternion)
             (tf:make-pose (tf:make-identity-vector) quaternion))
           (rotate (&key (ax 0.0) (ay 0.0) (az 0.0))
             (tf:euler->quaternion :ax ax :ay ay :az az))
           (id-pose ()
             (tf:make-identity-pose)))
    (let* ((base-table-pose
             (let* ((table
                      (first
                       (cram-semantic-map-designators:designator->semantic-map-objects
                        (make-designator
                         :object `((:name "iai_kitchen_meal_table_counter_top"))))))
                    (pose (slot-value table 'cram-semantic-map-utils:pose)))
               (tf:make-pose-stamped
                "map" 0.0
                (tf:v+ (tf:origin pose)
                       (tf:make-3d-vector 0.05 0.25 0.0))
                (tf:euler->quaternion :az (/ pi -2)))))
           (seat-no (desig:desig-prop-value loc :seat))
           (seat-base-offset
             (ecase seat-no
               (1 (translate -0.25 0.4 0.0))
               (2 (translate -0.25 -0.2 0.0))))
           (seat-base-pose (transform-pose `(,base-table-pose ,seat-base-offset)))
           (object-offset-pose
             (or (case object-type
                   (:glass (translate 0.1 -0.2 0.0))
                   (:cup (translate 0.1 -0.2 0.0))
                   (:fork (translate 0.0 0.2 0.0))
                   (:knife (translate 0.0 -0.2 0.0))
                   (:spoon (translate 0.0 0.2 0.0))
                   (:ketchup (translate 0.1 0.2 0.0))
                   (:muesli (translate 0.1 -0.2 0.0))
                   (:milkbox (translate 0.1 0.2 0.0)))
                 (id-pose)))) ;; plate, bowl
      (transform-pose `(,seat-base-pose ,object-offset-pose)))))

(defun setmap-required-scene-objects ()
  (labels ((make-location (type args)
               (make-designator :location (append `((:type ,type)) args)))
             (make-object (mode content)
               (make-designator :object `((,mode ,content)))))
    (mapcar (lambda (obj)
              (let* ((loc (desig:desig-prop-value obj :at))
                     (obj-type (desig:desig-prop-value obj :type))
                     (resolved (resolve-seat-location loc obj-type)))
                `(,(make-object :type (scene-object->object-class obj-type))
                  ,(make-location
                    :absolute
                    `((:pose ,resolved))))))
            (required-scene-objects))))
