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

(asdf:defsystem ltfnp-executive
  :name "ltfnp-executive"
  :author "Jan Winkler <winkler@cs.uni-bremen.de>"
  :version "0.1"
  :maintainer "Jan Winkler <winkler@cs.uni-bremen.de>"
  :licence "BSD"
  :description "Longterm Fetch and Place Executive"
  :depends-on (roslisp
               cram-tf
               cram-plan-library
               cram-prolog
               cram-pr2-description
               cram-plan-occasions-events
               cram-occasions-events
               cram-pr2-designators
               pr2-manipulation-process-module
               pr2-reachability-costmap
               pr2-navigation-process-module
               point-head-process-module
               alexandria
               cram-physics-utils
               cram-occupancy-grid-costmap
               cram-location-costmap
               cram-semantic-map-costmap
               cram-language
               cram-bullet-reasoning-designators
               robosherlock-process-module
               cram-beliefstate
               cram-robot-pose-gaussian-costmap
               gazebo-perception-process-module
               attache_msgs-srv
               pr2_controllers_msgs-msg)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "costmap-metadata" :depends-on ("package"))
     (:file "reasoning" :depends-on ("package" "costmap-metadata"))
     (:file "utils" :depends-on ("package" "costmap-metadata" "reasoning"))
     (:file "sem-map-config" :depends-on ("package"))
     (:file "facts" :depends-on ("package"))
     (:file "plan-library" :depends-on ("package"
                                        "utils"
                                        "costmap-metadata"
                                        "reasoning"
                                        "sem-map-config"
                                        "facts"))
     (:file "plans-process-module" :depends-on ("package"
                                                "utils"
                                                "costmap-metadata"
                                                "reasoning"
                                                "sem-map-config"
                                                "facts"
                                                "plan-library"))
     (:file "top-level-plans" :depends-on ("package"
                                           "utils"
                                           "costmap-metadata"
                                           "reasoning"
                                           "sem-map-config"
                                           "plan-library"
                                           "facts"
                                           "plans-process-module"))))))
