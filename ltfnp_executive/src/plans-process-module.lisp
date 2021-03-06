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


(def-fact-group eating-plans (action-desig)
  
  (<- (action-desig ?action-designator (fetch-object ?current-object))
    (desig-prop ?action-designator (:to :fetch))
    (desig-prop ?action-designator (:obj ?object))
    (desig:current-designator ?object ?current-object))
  
  (<- (action-desig ?action-designator (place-object ?current-object ?location))
    (desig-prop ?action-designator (:to :place))
    (desig-prop ?action-designator (:obj ?object))
    (desig-prop ?action-designator (:at ?location))
    (desig:current-designator ?object ?current-object))
  
  (<- (action-desig ?action-designator (find-object ?current-object))
    (desig-prop ?action-designator (:to :find))
    (desig-prop ?action-designator (:obj ?object))
    (desig:current-designator ?object ?current-object))
  
  (<- (action-desig ?action-designator (pick-object ?current-object))
    (desig-prop ?action-designator (:to :pick))
    (desig-prop ?action-designator (:obj ?object))
    (desig:current-designator ?object ?current-object)))
