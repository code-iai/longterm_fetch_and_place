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
;;; Auxiliary Functions
;;;

(defun do-init (simulated &key headless variance)
  (setf *simulated* simulated)
  (cond (*simulated*
         (setf cram-beliefstate::*kinect-topic-rgb* "/head_mount_kinect/rgb/image_raw");;/compressed")
         (setf pr2-manip-pm::*pregrasp-offset*
               (cl-transforms:make-pose
                (cl-transforms:make-3d-vector
                 -0.30 0.0 0.0)
                (cl-transforms:euler->quaternion :ax (/ pi 2))))
         (setf pr2-manip-pm::*grasp-offset*
               (cl-transforms:make-pose
                (cl-transforms:make-3d-vector -0.20 0.0 0.0)
                (cl-transforms:euler->quaternion :ax (/ pi 2)))))
        (t (setf cram-moveit::*needs-ft-fix* t)
           (setf cram-beliefstate::*kinect-topic-rgb* "/kinect_head/rgb/image_color")))
  (roslisp:ros-info (ltfnp) "Connecting to ROS")
  (roslisp-utilities:startup-ros)
  (prepare-settings :simulated simulated :headless headless :variance variance)
  (roslisp:ros-info (ltfnp) "Putting the PR2 into defined start state")
  (move-arms-up)
  (move-torso))


;;;
;;; Entry Point
;;;

(defun start-scenario (&key (simulated t) (logged nil) skip-init headless (variance "{}"))
  ;; This function is mainly meant as an entry point for external
  ;; runner scripts (for starting the scenario using launch files,
  ;; etc.)
  (let* ((variance (yason:parse variance))
         (exp-mode-string (or (gethash "mode" variance)
                              "tablesetting")))
    (format t "VARIANCE MODE IS ~a~%" exp-mode-string)
    (beliefstate:enable-logging nil)
    (unless skip-init
      (do-init simulated :headless headless :variance variance))
    (roslisp:ros-info (ltfnp) "Running Longterm Fetch and Place")
    (roslisp:ros-info (ltfnp) "Using variance: ~a" variance)
    (prog1
        (labels ((exp-mode (mode)
                   (string= mode exp-mode-string)))
          (cond ((exp-mode "tablesetting")
                 (beliefstate:enable-logging logged)
                 (setf beliefstate::*enable-prolog-logging* logged)
                 (prog1
                     (longterm-fetch-and-place :variance variance)
                   (when logged
                     (beliefstate:extract-files))))
                ((exp-mode "search")
                 (search-object-scenario))
                ((exp-mode "tablesetting2")
                 (tablesetting-scenario))))
      (roslisp:ros-info (ltfnp) "Done with LTFnP"))))


;;;
;;; Top-Level Plans
;;;

(def-top-level-cram-function longterm-fetch-and-place (&key variance)
  (roslisp:ros-info (ltfnp) "Preparation complete, beginning actual scenario")
  (cond (*simulated*
         (roslisp:ros-info (ltfnp) "Environment: Simulated")
         (with-process-modules-simulated
           (fetch-and-place-instance :variance variance)))
        (t
         (roslisp:ros-info (ltfnp) "Environment: Real-World")
         (with-process-modules
           (fetch-and-place-instance :variance variance)))))

(defun make-object-location-goal (name &rest objects+locations)
  (make-instance
   'goal
   :name name
   :preconditions
   (mapcar (lambda (object+location)
             (destructuring-bind (object location) object+location
               `(:object-location ,(make-designator
                                    :object
                                    (enrich-description object))
                                  ,location)))
           objects+locations)))

(defun merge-goals (&rest rest)
  (let* ((names (loop for goal in rest
                      as name = (slot-value goal 'name)
                      collect name))
         (preconditions
           (loop for goal in rest
                 as precondition = (slot-value goal 'preconditions)
                 append precondition))
         (all-merged (reduce (lambda (s0 s1)
                               (concatenate 'string s0 "," s1))
                             names))
         (merged-name
           (concatenate 'string "merged(" all-merged ")")))
    (make-instance
     'goal
     :name merged-name
     :preconditions preconditions)))

(def-cram-function fetch-and-place-instance (&key variance)
  ;; TODO(winkler): Use the variance here to parameterize the
  ;; scenario.
  (let* ((target-table "iai_kitchen_meal_table_counter_top")
         (goal
           (merge-goals
            ;; (make-object-location-goal
            ;;  "fetch-fork"
            ;;  `(((:name "Fork")
            ;;     (:at ,(make-designator
            ;;            :location
            ;;            `((:in "Drawer")
            ;;              (:name "iai_kitchen_sink_area_left_upper_drawer_main")
            ;;              (:handle
            ;;               ,(make-designator
            ;;                 :object
            ;;                 `((:name "drawer_sinkblock_upper_handle")))))))
            ;;     (:location "drawer_sinkblock_upper"))
            ;;    ,(make-designator
            ;;      :location
            ;;      `((:on "CounterTop")
            ;;        (:name "iai_kitchen_meal_table_counter_top")))))
            
            ;; (make-fixed-tabletop-goal
            ;;  `(("RedMetalPlate" ,(tf:make-pose-stamped
            ;;                       "map" 0.0
            ;;                       (tf:make-3d-vector -1.35 -0.90 0.75)
            ;;                       (tf:euler->quaternion)))
            ;;    ("RedMetalCup" ,(tf:make-pose-stamped
            ;;                     "map" 0.0
            ;;                     (tf:make-3d-vector -1.55 -1.05 0.73)
            ;;                     (tf:euler->quaternion))))
            ;;  "iai_kitchen_sink_area_counter_top")
            (make-fixed-tabletop-goal
             `(("RedMetalPlate" ,(tf:make-pose-stamped
                                  "map" 0.0
                                  (tf:make-3d-vector -0.75 -0.90 0.75)
                                  (tf:euler->quaternion)))
               ("RedMetalCup" ,(tf:make-pose-stamped
                                "map" 0.0
                                (tf:make-3d-vector -0.95 -1.05 0.73)
                                (tf:euler->quaternion))))
             "iai_kitchen_sink_area_counter_top")
            ))
         (the-plan (plan (make-empty-state) goal)))
    (spawn-goal-objects goal target-table)
    (roslisp:ros-info (ltfnp) "The plan has ~a step(s)"
                      (length the-plan))
    (dolist (action the-plan)
      (destructuring-bind (type &rest rest) action
        (ecase type
          (:fetch
           (catch-all "fetch"
             (destructuring-bind (object) rest
               (with-designators ((fetch-action :action
                                                `((:to :fetch)
                                                  (:obj ,object))))
                 (perform fetch-action)))))
          (:place
           (catch-all "place"
             (destructuring-bind (object location) rest
               (with-designators ((place-action :action
                                                `((:to :place)
                                                  (:obj ,object)
                                                  (:at ,location))))
                 (go-to-origin :keep-orientation nil)
                 (perform place-action))))))))))

(defun prepare-container-scene ()
  (setf *container-stored-objects* (make-hash-table :test 'equal))
  (maybe-store "Fork" (tf:make-pose (tf:make-3d-vector 0.05 -0.15 0.05) (tf:euler->quaternion :az pi))
         "iai_kitchen_sink_area_left_upper_drawer_handle")
  (maybe-store "Knife" (tf:make-pose (tf:make-3d-vector 0.05 0.0 0.05) (tf:euler->quaternion :az pi))
               "iai_kitchen_sink_area_left_upper_drawer_handle")
  (maybe-store "Spoon" (tf:make-pose (tf:make-3d-vector 0.05 0.15 0.05) (tf:euler->quaternion :az pi))
               "iai_kitchen_sink_area_left_upper_drawer_handle")
  (maybe-store "Muesli" (tf:make-pose (tf:make-3d-vector 0.15 -0.2 0.15) (tf:euler->quaternion :az pi))
               "iai_kitchen_sink_area_counter_top")
  (maybe-store "RedMetalPlate" (tf:make-pose (tf:make-3d-vector 0.15 -0.5 0.05) (tf:euler->quaternion :az pi))
               "iai_kitchen_sink_area_counter_top")
  (maybe-store "RedMetalPlate" (tf:make-pose (tf:make-3d-vector 0.15 1.0 -0.05) (tf:euler->quaternion :ax pi :az 0))
               "iai_kitchen_kitchen_island_counter_top")
  (maybe-store "Milk" (tf:make-pose (tf:make-3d-vector 0.1 0.1 -1.33) (tf:euler->quaternion :az pi))
               "iai_kitchen_fridge_door_handle")
  ;;(maybe-store "Milk" (tf:make-pose (tf:make-3d-vector 0.1 -0.1 -1.33) (tf:euler->quaternion :az pi))
         ;;"iai_kitchen_fridge_door_handle")
  (maybe-store "Buttermilk" (tf:make-pose (tf:make-3d-vector 0.15 0.11 -1.67) (tf:euler->quaternion :az pi))
               "iai_kitchen_fridge_door_handle")
  (maybe-store "Buttermilk" (tf:make-pose (tf:make-3d-vector 0.05 0.025 -1.67) (tf:euler->quaternion :az pi))
               "iai_kitchen_fridge_door_handle")
  (maybe-store "Buttermilk" (tf:make-pose (tf:make-3d-vector -0.05 -0.05 -1.67) (tf:euler->quaternion :az pi))
               "iai_kitchen_fridge_door_handle")
  (maybe-store "Buttermilk" (tf:make-pose (tf:make-3d-vector 0.15 -0.15 -1.67) (tf:euler->quaternion :az pi))
               "iai_kitchen_fridge_door_handle")
  (maybe-store "RedMetalBowl" (tf:make-pose (tf:make-3d-vector 0.17 0.0 0.05) (tf:euler->quaternion :az (/ pi 2)))
               "iai_kitchen_kitchen_island_left_upper_drawer_handle")
  (loop for h being the hash-keys of *container-stored-objects*
        when (eql (container-type h) :countertop)
          do (dolist (item (gethash h *container-stored-objects*))
               (destructuring-bind (nam cls pos) item
                 (let* ((center-pose (countertop-center-pose h))
                        (object-pose (ensure-pose-stamped
                                      (cl-transforms:transform-pose
                                       (tf:pose->transform center-pose) pos))))
                   (roslisp:ros-info (ltfnp) "Add gazebo object model '~a'" nam)
                   (roslisp:publish (roslisp:advertise "/blablabla" "geometry_msgs/PoseStamped")
                                    (tf:to-msg object-pose))
                   (spawn-class nam cls object-pose))))))

(defun cleanup-container-scene ()
  (loop for h being the hash-keys of *container-stored-objects*
        do (dolist (obj (gethash h *container-stored-objects*))
             (destructuring-bind (nam cls pos) obj
               (declare (ignore cls pos))
               (detach-object "ground_plane" "link" nam "link")
               (sleep 0.1)
               (cram-gazebo-utilities::delete-gazebo-model nam))))
  (setf *container-stored-objects* (make-hash-table :test 'equal)))

(def-top-level-cram-function search-object-scenario ()
  (with-process-modules-simulated
    (beliefstate:enable-logging nil)
    (do-init t :variance (make-hash-table :test 'equal))
    ;; Initialize scenario
    (prepare-container-scene)
    (search-object (make-designator :object `((:type "Milk"))))))

(def-cram-function search-object (object)
  (roslisp:ros-info (ltfnp) "Preparation complete, beginning actual scenario")
  ;; These locations could be sorted according to known residence
  ;; probabilities for any given object. Right now, they are just in a
  ;; static order, and will be searched in that order.
  (let ((searchable-locations (location-order-for-object object)))
    (find-and-fetch-object object searchable-locations)))

(def-cram-function find-and-fetch-object (object locations)
  (let ((found-object nil))
    (loop for location in locations until found-object
          as found-objects = (search-location location object
                                              :when-found :leave-accessible)
          when found-objects
            do (setf found-object `(,(first found-objects) ,location)))
    (when found-object
      (destructuring-bind (obj loc) found-object
        (with-failure-handling
            ((cram-plan-failures:location-not-reached-failure (f)
               (declare (ignore f))
               (cpl:retry)))
          (let ((allowed-old pr2-manip-pm::*allowed-arms*))
            (unwind-protect
                 (progn
                   (setf pr2-manip-pm::*allowed-arms*
                         (or (when (context-constraint :open-handle-with-arm)
                               `(,(context-constraint :open-handle-with-arm)))
                             (allowed-hands-for-location loc)))
                   (roslisp:ros-info (ltfnp) "Allowed arms for grasping are: ~a~%"
                                     pr2-manip-pm::*allowed-arms*)
                   (go-to-container-grasping-pose loc)
                   (let ((without-co
                             (cond ((string= loc "iai_kitchen_fridge_door_handle")
                                    `("HTTP://KNOWROB.ORG/KB/IAI-KITCHEN.OWL#IAI_KITCHEN_FRIDGE_AREA-1"
                                      "HTTP://KNOWROB.ORG/KB/IAI-KITCHEN.OWL#IAI_KITCHEN_FRIDGE_MAIN-1"))
                                   ((string= loc "iai_kitchen_kitchen_island_left_upper_drawer_handle")
                                    `("HTTP://KNOWROB.ORG/KB/IAI-KITCHEN.OWL#IAI_KITCHEN_KITCHEN_ISLAND-1"))
                                   (t nil))))
                     (cram-moveit::without-collision-objects without-co
                       (setf found-object
                             (cond ((string= loc "iai_kitchen_fridge_door_handle")
                                    (setf pr2-manip-pm::*allowed-arms* allowed-old)
                                    (cond ((object-is-in-handled-container obj loc)
                                           (prog1 (achieve `(cram-plan-library:object-picked ,obj))
                                             (remove-object-from-handled-container
                                              (desig:desig-prop-value obj :name) loc)))
                                          (t (prog1 (achieve
                                                     `(cram-plan-library:object-in-hand ,obj))
                                               (remove-object-from-handled-container
                                                (desig:desig-prop-value obj :name) loc)))))
                                   (t (prog1 (achieve
                                              `(cram-plan-library:object-in-hand ,obj))
                                        (remove-object-from-handled-container
                                         (desig:desig-prop-value obj :name) loc))))))))
              (setf pr2-manip-pm::*allowed-arms* allowed-old))))
        (when (and (location-closeable loc)
                   (not (string= loc "iai_kitchen_fridge_door_handle")))
          (close-handled-storage-container loc))))
    found-object))

(defun object-is-in-handled-container (obj loc)
  (let ((objs (gethash loc *container-stored-objects*))
        (id (desig:desig-prop-value obj :name)))
    (when id
      (not (not (find id objs :test (lambda (the-id stored-obj)
                                      (destructuring-bind (nam cls relpos) stored-obj
                                        (declare (ignore cls relpos))
                                        (format t "~a ~a~%" nam the-id)
                                        (string= nam the-id)))))))))

(defun make-location-aux-object (object location)
  (make-designator
   :object (append (remove :at (desig:description object)
                           :test (lambda (x pair) (eql x (car pair))))
                   `((:at ,location)))))

(defun find-location-aux-object (object location)
  (block failure-guard
    (let ((aux-object (make-location-aux-object object location)))
      (with-failure-handling
          ((cram-plan-failures:location-not-reached-failure (f)
             (declare (ignore f))
             (cpl:retry))
           (cram-plan-failures:object-not-found (f)
             (declare (ignore f))
             (return-from failure-guard)))
        (find-object aux-object :num-retries 0)))))

(def-cram-function search-location (locname object &key (when-found :return))
  (let ((loctype (container-type locname)))
    (roslisp:ros-info (object search) "Looking for object at '~a'" locname)
    (ecase loctype
      (:countertop
       (let ((aux-found (find-location-aux-object
                         object (make-designator
                                 :location `((:on "CounterTop")
                                             (:name ,locname))))))
         (when aux-found `(,aux-found))))
      (:drawer
       ;; Well-defined starting torso height
       (move-torso)
       ;; Open drawer
       (open-handled-storage-container locname)
       ;; Inspect the contents
       (unwind-protect
            (inspect-container-contents-for-object
             locname object :num-retries 3)
         ;; Close drawer
         (unless (eql when-found :leave-accessible)
           (close-handled-storage-container locname))))
      (:dishwasher
       (move-torso)
       (open-handled-storage-container locname)
       (unwind-protect
            (inspect-container-contents-for-object
             locname object :num-retries 3)
         (unless (eql when-found :leave-accessible)
           (close-handled-storage-container locname))))
      (:fridge
       (move-torso)
       (open-handled-storage-container locname)
       (unwind-protect
            (inspect-container-contents-for-object
             locname object :num-retries 3)
         (unless (eql when-found :leave-accessible)
           (close-handled-storage-container locname)))))))

(def-cram-function inspect-container-contents-for-object (location object &key (num-retries 3))
  ;; We assume that we're looking inside the container right now
  (block failure-guard
    (let ((aux-object (make-location-aux-object
                       object
                       (make-designator :location `((:inside :container)
                                                    (:handle-name ,location))))))
      (with-failure-handling ((cram-plan-failures:object-not-found (f)
                                (declare (ignore f))
                                (when (> num-retries 0)
                                  (decf num-retries)
                                  (cpl:retry))
                                (return-from failure-guard)))
        (format t "LOOKING FOR OBJECTS AT '~a'~%" location)
        (let ((objects (perceive-object :currently-visible aux-object)))
          (loop for obj in objects
                do (format t "FOUND OBJECT: ~a~%" obj)
                   ;; Last object wins
                   (desig:equate object obj))
          objects)))))

(defun scene-object->object-class (so)
  (ecase so
    (:muesli "Muesli")
    (:milkbox "Milk")
    (:buttermilk "Buttermilk")
    (:plate "RedMetalPlate")
    (:cup "RedMetalCup")
    (:bowl "RedMetalBowl")
    (:spoon "Spoon")
    (:knife "Knife")
    (:fork "Fork")
    (:ketchup "Ketchup")
    (:salt "Salt")))

;; (defun common-residence-location (objcls)
;;   (cond ((string= objcls "Muesli")
;;          "iai_kitchen_kitchen_island_left_upper_drawer_handle")
;;         ((string= objcls "Milk")
;;          "iai_kitchen_fridge_door_handle")
;;         ((string= objcls "Buttermilk")
;;          "iai_kitchen_fridge_door_handle")
;;         ((string= objcls "RedMetalPlate")
;;          "iai_kitchen_sink_area_left_middle_drawer_handle")
;;         ((string= objcls "RedMetalCup")
;;          "iai_kitchen_sink_area_left_middle_drawer_handle")
;;         ((string= objcls "RedMetalBowl")
;;          "iai_kitchen_kitchen_island_left_upper_drawer_handle")))

;; (defun random-residence-location ()
;;   (let ((lofo (location-order-for-object
;;                (make-designator :object nil))))
;;     (nth (random (length lofo)) lofo)))

;; (defun populate-scene (scene-objects)
;;   (let ((objects (loop for scene-object in scene-objects
;;                        collect (desig:desig-prop-value
;;                                 scene-object :type)))
;;         (occupied-rel-poses (make-hash-table :test 'equal))
;;         (max-occ-dist 0.2)
;;         (min-occ-dist 0.02))
;;     (labels ((random-occ-dist ()
;;                (+ (/ 1 (random (/ 1 (- max-occ-dist min-occ-dist))))
;;                   min-occ-dist))
;;              (gen-rel-pos (loc)
;;                (let ((occupied-poses (gethash loc occupied-rel-poses)))
;;                  )))
;;       (dolist (object objects)
;;         (let ((objcls (scene-object->object-class object)))
;;           (when objcls
;;             (let ((loc (or (common-residence-location objcls)
;;                            (random-residence-location))))
;;               ;; Place the object there
;;               (cond ((eql (container-type loc) :countertop)
;;                      ;; Just spawn
;;                      )
;;                     (t ;; Handled storage container, store
;;                      (let ((relpos nil))
;;                        ;; TODO: relpos needs to be determined!
;;                        (store objcls relpos loc)))))))))))

(def-top-level-cram-function tablesetting-scenario ()
  (with-process-modules-simulated
    (beliefstate:enable-logging nil)
    (do-init t :variance (make-hash-table :test 'equal))
    ;; Initialize scenario
    (prepare-container-scene)
    (labels ((make-location (type args)
               (make-designator :location (append `((:type ,type)) args)))
             (make-object (mode content)
               (make-designator :object `((,mode ,content)))))
      (set-scene-1)
      (let ((setting-mappings (setmap-required-scene-objects)))
              ;; `((,(make-object :type "RedMetalPlate")
              ;;    ,(make-location
              ;;      :fridge
              ;;      `((:name "iai_kitchen_fridge_door_handle")
              ;;      ;; :dishwasher
              ;;      ;; `((:name "iai_kitchen_sink_area_dish_washer_door_handle")
              ;;        (:pose ,(tf:make-pose
              ;;                 (tf:make-3d-vector -0.1 0.08 0.35)
              ;;                 (tf:euler->quaternion))))))
              ;;   ;; (,(make-object :type "RedMetalPlate")
              ;;   ;;  ,(make-location
              ;;   ;;    :absolute
              ;;   ;;    `((:pose ,(tf:make-pose-stamped
              ;;   ;;               "map" 0.0
              ;;   ;;               (tf:make-3d-vector -1.0 -0.8 0.78)
              ;;   ;;               (tf:euler->quaternion :az (/ pi -2)))))))
              ;;   )))
              ;;   ;; (,(make-object :type "Fork")
              ;;   ;;  ,(make-location
              ;;   ;;    :absolute
              ;;   ;;    `((:pose ,(tf:make-pose-stamped
              ;;   ;;               "map" 0.0
              ;;   ;;               (tf:make-3d-vector -0.75 -0.9 0.78)
              ;;   ;;               (tf:euler->quaternion :az pi))))))
              ;;   ;; (,(make-object :type "Knife")
              ;;   ;;  ,(make-location
              ;;   ;;    :absolute
              ;;   ;;    `((:pose ,(tf:make-pose-stamped
              ;;   ;;               "map" 0.0
              ;;   ;;               (tf:make-3d-vector -1.30 -0.9 0.78)
              ;;   ;;               (tf:euler->quaternion :az pi))))))
              ;;   ;; ;; (,(make-object :type "Milk")
              ;;   ;; ;;  ,(make-location
              ;;   ;; ;;    :absolute
              ;;   ;; ;;    `((:pose ,(tf:make-pose-stamped
              ;;   ;; ;;               "map" 0.0
              ;;   ;; ;;               (tf:make-3d-vector -1.4 -0.9 0.78)
              ;;   ;; ;;               (tf:euler->quaternion :az (/ pi -2)))))))
                
              ;;   ;; ;; ("RedMetalBowl" (make-location
              ;;   ;; ;;                  :countertop
              ;;   ;; ;;                  `((:pose ,(tf:make-pose-stamped
              ;;   ;; ;;                             "map" 0.0
              ;;   ;; ;;                             (tf:make-3d-vector -1.6 -0.8 0.78)
              ;;   ;; ;;                             (tf:euler->quaternion :az (/ pi -2)))))))
              ;;   ;; ;; ("Spoon" (make-location
              ;;   ;; ;;           :countertop
              ;;   ;; ;;           `((:pose ,(tf:make-pose-stamped
              ;;   ;; ;;                      "map" 0.0
              ;;   ;; ;;                      (tf:make-3d-vector -1.5 -0.9 0.78)
              ;;   ;; ;;                      (tf:euler->quaternion :az pi)))))))))
              ;;   ;; (,(make-object :type "Knife")
              ;;   ;;  ,(make-location
              ;;   ;;    :drawer
              ;;   ;;    `((:name "iai_kitchen_sink_area_left_upper_drawer_handle")
              ;;   ;;      (:pose ,(tf:make-pose
              ;;   ;;               (tf:make-3d-vector 0.05 0 0.05)
              ;;   ;;               (tf:euler->quaternion :az (/ pi -2))))))))))
        (process-fetch-and-place setting-mappings)))))

(def-cram-function process-fetch-and-place (setting-mappings)
  (let ((objects-not-found nil)
        (acquired-objects nil))
    (dolist (item setting-mappings)
      (destructuring-bind (orig-object destination-location) item
        (go-to-origin)
        (let* ((loc-type (desig:desig-prop-value destination-location :type))
               (loc-name (desig:desig-prop-value destination-location :name))
               (arm-ok (unless (eql loc-type :absolute)
                         (other-hand (ideal-arm-for-handle loc-name))))
               (object
                 (with-context (when arm-ok `((:open-handle-with-arm ,arm-ok)))
                   (search-object orig-object))))
          (cond (object
                 (go-to-origin)
                 (push object acquired-objects)
                 (ecase loc-type
                   (:absolute
                    (with-designators ((location
                                        :location
                                        `((:pose ,(desig:desig-prop-value
                                                   destination-location :pose))))
                                       (place-action
                                        :action
                                        `((:to :place)
                                          (:obj ,object)
                                          (:at ,location))))
                      (go-to-origin :keep-orientation t)
                      (with-context `((:fix-to-ground-putdown t))
                        (perform place-action))))
                   (:drawer
                    ;; Well-defined starting torso height
                    (move-torso)
                    ;; Open drawer
                    (open-handled-storage-container loc-name)
                    ;; Put object inside
                    (unwind-protect
                         (place-object-into-location object destination-location)
                      ;; Close drawer
                      (close-handled-storage-container loc-name)))
                   (:fridge
                    (move-torso)
                    (open-handled-storage-container loc-name)
                    (unwind-protect
                         (progn
                           (go-to-pose (tf:make-3d-vector -0.1 -0.4 0.0)
                                       (tf:make-identity-rotation))
                           (move-arm-pose :left (tf:make-pose-stamped "base_link" 0.0 (tf:make-3d-vector 0.4 0.0 1.0) (tf:euler->quaternion :ay (/ pi 2) :az (/ pi 2))))
                           (go-to-pose (tf:make-3d-vector 0.15 0.0 0.0)
                                       (tf:make-identity-rotation))
                           (cram-moveit::without-collision-objects
                               `("HTTP://KNOWROB.ORG/KB/IAI-KITCHEN.OWL#IAI_KITCHEN_FRIDGE_AREA-1"
                                 "HTTP://KNOWROB.ORG/KB/IAI-KITCHEN.OWL#IAI_KITCHEN_FRIDGE_MAIN-1")
                             (place-object-into-location object destination-location)))
                      ;; Close drawer
                      (close-handled-storage-container loc-name)))
                   (:dishwasher
                    )))
                (t (push orig-object objects-not-found))))))
    `((:not-found ,objects-not-found)
      (:acquired ,acquired-objects))))

(def-cram-function place-object-into-location (object location)
  (let* ((locname (desig:desig-prop-value location :name))
         (relpos (desig:desig-prop-value location :pose))
         (loc-center-pose
           (or (when (string= locname "iai_kitchen_fridge_door_handle")
                 (tf:transform-pose-stamped
                  *transformer*
                  :pose (tf:pose->pose-stamped (container-frame-for-handle locname)
                                               0.0 (tf:make-identity-pose))
                  :target-frame "map"))
               (container-center-pose locname)))
         (fin-pose (tf:make-pose-stamped
                    "map" 0.0
                    (tf:v+ (tf:origin loc-center-pose) (tf:origin relpos))
                    (tf:orientation relpos))))
    (with-designators ((location
                        :location
                        `((:pose ,fin-pose)))
                       (place-action
                        :action
                        `((:to :place)
                          (:obj ,object)
                          (:at ,location))))
      (roslisp:publish (roslisp:advertise "/blablabla" "geometry_msgs/PoseStamped")
                       (tf:to-msg fin-pose))
      (perform place-action)
      (sleep 0.1)
      (let ((objname (desig:desig-prop-value object :name))
            (objtype (desig:desig-prop-value object :type)))
        (cram-gazebo-utilities::with-physics-paused
          (cram-gazebo-utilities::delete-gazebo-model objname)
          (sleep 0.1)
          (store-object-in-handled-container `(,objname ,objtype ,relpos) locname)
          (spawn-class objname objtype fin-pose))))))


;;;
;;; Override the default mot-man init function
;;;

(defun pr2-manip-pm::make-empty-goal-specification ()
  (mot-man:make-goal-specification :moveit-goal-specification))
