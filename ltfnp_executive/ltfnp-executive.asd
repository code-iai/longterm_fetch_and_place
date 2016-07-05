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

(asdf:defsystem ltfnp-executive
  :name "ltfnp-executive"
  :author "Jan Winkler <winkler@cs.uni-bremen.de>"
  :version "0.1"
  :maintainer "Jan Winkler <winkler@cs.uni-bremen.de>"
  :licence "BSD"
  :description "Shopping Scenario Executive"
  :depends-on (cram-prediction
               roslisp
               designators-ros
               cram-roslisp-common
               cram-plan-library
               cram-reasoning
               cram-pr2-knowledge
               cram-plan-knowledge
               cram-environment-representation
               pr2-manipulation-knowledge
               pr2-manipulation-process-module
               pr2-reachability-costmap
               pr2-navigation-process-module
               pr2-reachability-costmap
               point-head-process-module
               object-location-designators
               alexandria
               physics-utils
               occupancy-grid-costmap
               location-costmap
               semantic-map-costmap
               cram-language
               bullet-reasoning-designators
               robosherlock-process-module
               cram-beliefstate
               gazebo-perception-process-module
               attache_msgs-srv
               pr2_controllers_msgs-msg)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "facts" :depends-on ("package"))
     (:file "costmap-metadata" :depends-on ("package"))
     (:file "sem-map-config" :depends-on ("package"))
     (:file "reasoning" :depends-on ("package"))
     (:file "utils" :depends-on ("package" "reasoning"))
     (:file "planner" :depends-on ("package" "reasoning"))
     (:file "planner_v2" :depends-on ("package" "reasoning"))
     (:file "plan-library" :depends-on ("package"
                                        "utils"
                                        "reasoning"
                                        "costmap-metadata"
                                        "sem-map-config"
                                        "facts"
                                        "planner"
                                        "planner_v2"))
     (:file "top-level-plans" :depends-on ("package"
                                           "utils"
                                           "reasoning"
                                           "costmap-metadata"
                                           "sem-map-config"
                                           "facts"
                                           "plan-library"
                                           "planner"
                                           "planner_v2"))))))
