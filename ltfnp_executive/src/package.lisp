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

(in-package :cl-user)

(desig-props:def-desig-package ltfnp-executive
  (:nicknames ltfnp)
  (:use #:common-lisp #:roslisp #:cram-utilities
        #:location-costmap #:cram-plan-occasions-events
        #:cram-plan-library #:cram-language-designator-support
        #:cpl-impl)
  (:import-from :cram-language
                top-level fl-funcall with-tags pursue tag retry-after-suspension
                whenever pulsed value with-task-suspended seq declare-goal def-goal)
  (:import-from :cram-designators
                make-designator
                action
                action-desig?
                desig-prop
                action-desig)
  (:import-from :robosherlock-process-module
                perceived-object-invalid
                object-handle)
  (:import-from :pr2-manipulation-process-module
                reorient-object
                close-radius)
  (:import-from :location-costmap
                desig-costmap)
  (:import-from :cram-language-designator-support with-designators)
  (:import-from :cram-language def-cram-function def-top-level-cram-function)
  (:import-from :cram-prolog def-fact-group <- not)
  (:import-from :cram-tf *transformer*)
  (:export start-scenario)
  (:desig-properties :on :name :grasp-type :handle
                     :pose :shape :box :cylinder
                     :type :navigation :pose :goal :trajectory
                     :perceive :obj :to :at :reach :see :handover))
