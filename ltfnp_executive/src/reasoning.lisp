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
  "Returns a boolean value denoting whether `class' is a valid shopping item class (i.e., a subclass of `LTFnPObject')."
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

(defun instantiate-object (class)
  (assert (object-class-p class)
          ()
          "Class `~a' is not a subclass of `LTFnP'." class)
  (with-first-prolog-vars-bound (?instance)
      `("ltfnp_add_object" ,(add-prolog-namespace class) ?instance)
    (strip-prolog-string ?instance)))

(defun delete-object (object-id)
  (json-prolog:prolog `("ltfnp_remove_object" ,(add-prolog-namespace object-id))))
