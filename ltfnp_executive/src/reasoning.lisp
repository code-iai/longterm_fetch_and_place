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
;;; Utility predicates
;;;

(defmacro with-first-prolog-vars-bound (vars prolog-query &body body)
  "Evaluates the prolog query `prolog-query' and transforms variables `vars' via `body', returning the result."
  `(with-vars-bound ,vars
       (lazy-car
        (json-prolog:prolog ,prolog-query))
     ,@body))

(defmacro with-prolog-vars-bound (vars prolog-query &body body)
  "Lists all results from the prolog query `prolog-query', each being transformed by `body'. `vars' denotes all variables to make available in `body'."
  `(force-ll
    (lazy-mapcar
     (lambda (bdgs)
       (with-vars-bound ,vars bdgs
         ,@body))
     (json-prolog:prolog ,prolog-query))))

(defun json-symbol->string (symbol)
  "Converts `symbol' as returned from json-prolog to a lisp-usable string by trimming `|' characters at the beginning and the end."
  (let* ((string-symbol (write-to-string symbol)))
    (subseq string-symbol 2 (- (length string-symbol) 2))))

(defun split-prolog-symbol (prolog-symbol &key (delimiter '\#))
  "Splits the namespace from the symbol of a prolog identifier symbol `prolog-symbol'. The two parts must be delimited by the delimiter `delimiter'. Returns a values list, consisting of the symbol, and the namespace."
  (let ((delimiter-position
          (position delimiter prolog-symbol :test #'string=)))
    (when delimiter-position
      (values
       (subseq prolog-symbol (1+ delimiter-position))
       (subseq prolog-symbol 0 delimiter-position)))))

(defun strip-prolog-string (symbol)
  "Combines the functionality of `json-symbol->string' and `split-prolog-symbol', resulting in a namespace-less string representing the value of `symbol'."
  (split-prolog-symbol (json-symbol->string symbol)))

(defun add-prolog-namespace (symbol &key (namespace "http://knowrob.org/kb/knowrob.owl") (delimiter '\#))
  "Concatenates a string that consists of the given `namespace', the `delimiter', and finally the `symbol'. The default namespace represents the base KnowRob OWL namespace, and the default delimiter is `#'."
  (concatenate
   'string
   namespace
   (json-symbol->string (write-to-string delimiter))
   symbol))


;;;
;;; Object and object class related predicates
;;;

(defun object-classes ()
  "Lists all subclasses of the KnowRob `LTFnPObject` class. The
base-class itself does not count towards the enlisted classes."
  (cpl:mapcar-clean
   (lambda (class)
     (let ((stripped-class (strip-prolog-string class)))
       (unless (string= stripped-class "LTFnPObject")
         stripped-class)))
   (with-prolog-vars-bound (?class)
      `("ltfnp_object_class" ?class)
    ?class)))

(defun object-class-p (class)
  "Returns a boolean value denoting whether `class' is a valid object class (i.e., a subclass of `LTFnPObject')."
  (not (not (json-prolog:prolog `("ltfnp_object_class" ,(add-prolog-namespace class))))))

(defun object-instances ()
  (cpl:mapcar-clean
   (lambda (class)
     (let ((stripped-class (strip-prolog-string class)))
       (unless (string= stripped-class "LTFnPObject")
         stripped-class)))
   (with-prolog-vars-bound (?class)
      `("ltfnp_object" ?class)
    ?class)))

(defun register-object (object-id class)
  (assert (object-class-p class)
          ()
          "Class `~a' is not a subclass of `LTFnP'." class)
  (json-prolog:prolog `("ltfnp_register_object" ,(add-prolog-namespace object-id) ,(add-prolog-namespace class))))

(defun instantiate-object (class)
  (assert (object-class-p class)
          ()
          "Class `~a' is not a subclass of `LTFnP'." class)
  (with-first-prolog-vars-bound (?instance)
      `("ltfnp_add_object" ,(add-prolog-namespace class) ?instance)
    (strip-prolog-string ?instance)))

(defun delete-object (object-id)
  (json-prolog:prolog `("ltfnp_remove_object" ,(add-prolog-namespace object-id))))

(defun set-object-pose (object-id translation rotation)
  (json-prolog:prolog `("ltfnp_set_object_pose"
                        ,(add-prolog-namespace object-id)
                        ,(first translation) ,(second translation) ,(third translation)
                        ,(first rotation) ,(second rotation) ,(third rotation) ,(fourth rotation))))

(defun get-object-pose (object-id)
  (json-prolog:prolog `("ltfnp_get_object_pose"
                        ,(add-prolog-namespace object-id) ?tx ?ty ?tz ?qw ?qx ?qy ?qz)))

(defun get-object-class (object-id)
  (with-first-prolog-vars-bound (?class)
      `("ltfnp_instance_of_class" ,(add-prolog-namespace object-id) ?class)
    (strip-prolog-string ?class)))

(defun get-class-urdf-path (class)
  (with-first-prolog-vars-bound (?urdfpath)
      `("ltfnp_get_class_urdf_path" ,(add-prolog-namespace class) ?urdfpath)
    (json-symbol->string ?urdfpath)))

(defun get-object-urdf-path (object-id)
  (get-class-urdf-path (get-object-class object-id)))

(defun make-class-description (class)
  (append
   `((:type ,class))
   (mapcar (lambda (handle-object)
             `(:handle ,handle-object))
           (get-class-semantic-handle-objects class))))

(defun spawn-class (object-id class pose)
  (cram-gazebo-utilities:spawn-gazebo-model
   object-id pose (get-class-urdf-path class)
   :description (make-class-description class)))

(defun spawn-object (object-id pose)
  (cram-gazebo-utilities:spawn-gazebo-model object-id pose (get-object-urdf-path object-id)))

(defun get-countertops ()
  (cram-semantic-map-designators:designator->semantic-map-objects
   (make-designator :object `((:type "CounterTop")))))

(defun pose-on-countertop (countertop)
  (let* ((name (semantic-map-utils:name countertop))
         (type (slot-value countertop 'type))
         (desig (make-designator :location `((:on ,type)
                                             (:name ,name)))))
    (cram-designators:reference desig)))

(defun get-class-semantic-handles (class)
  (cpl:mapcar-clean
   (lambda (handle)
     (get-semantic-handle-details handle))
   (with-prolog-vars-bound (?handle)
       `("ltfnp_class_semantic_handle" ,(add-prolog-namespace class) ?handle)
     (strip-prolog-string ?handle))))

(defun stripped-symbol-name (sym)
  (let ((sym-name (symbol-name sym)))
    (if (and (equal (subseq sym-name 0 1) "'")
             (equal (subseq sym-name (1- (length sym-name)) (length sym-name)) "'"))
        (subseq sym-name 1 (- (length sym-name) 1))
        sym-name)))

(defun get-semantic-handle-details (handle)
  (with-first-prolog-vars-bound (?grasptype ?tx ?ty ?tz ?qw ?qx ?qy ?qz)
      `("ltfnp_semantic_handle_details" ,(add-prolog-namespace handle) ?grasptype ?tx ?ty ?tz ?qw ?qx ?qy ?qz)
    `(,(stripped-symbol-name ?grasptype) ,?tx ,?ty ,?tz ,?qw ,?qx ,?qy ,?qz)))

(defun get-class-semantic-handle-objects (class)
  (mapcar (lambda (handle-data)
            (destructuring-bind (grasp-type tx ty tz qw qx qy qz) handle-data
              (make-designator
               :object
               `((:type :handle)
                 (:grasp-type ,(intern (string-upcase grasp-type) :keyword))
                 (:at ,(make-designator
                        :location
                        `((:pose ,(tf:make-pose (tf:make-3d-vector tx ty tz)
                                                (tf:make-quaternion qw qx qy qz))))))))))
          (get-class-semantic-handles class)))


;;;
;;; Location related reasoning functions (mostly for convenience)
;;;

(defun get-location-type (name)
  ;; Shall return :tabletop, :drawer or :fridge
  )

(defun must-articulate-location (name)
  ;; T or nil
  )

(defun is-location-accessible (name)
  ;; T or nil
  )
