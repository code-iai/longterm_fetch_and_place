#!/usr/bin/env sh
"true"; exec /usr/bin/env /usr/bin/sbcl --noinform --end-runtime-options --noprint --no-userinit --disable-debugger --script "$0" "$@"


;; ASDF
(REQUIRE :ASDF)

;; Swank
(load "/opt/ros/indigo/share/common-lisp/source/slime/swank-loader.lisp")
(swank-loader:init)


;; Utility functions
(labels ((get-roslisp-path ()
           ;; calls rospack to find path to roslisp
           (let ((rospack-process
                   (run-program "rospack" '("find" "roslisp")
                                :search t
                                :output :stream)))
             (when rospack-process
               (unwind-protect
                    (with-open-stream (o (process-output rospack-process))
                      (concatenate 'string (car (loop
                                                  for line := (read-line o nil nil)
                                                  while line
                                                  collect line)) "/load-manifest/"))
                 (process-close rospack-process)))))
         (load-ros-lookup ()
           ;; make sure roslisp is in asdf central registry
           (PUSH (get-roslisp-path) ASDF:*CENTRAL-REGISTRY*)
           ;; load ros-load-manifest, defining e.g. "ros-load:load-system"
           (ASDF:OPERATE 'ASDF:LOAD-OP :ROS-LOAD-MANIFEST :VERBOSE NIL)))
  (load-ros-lookup))


(PUSH :ROSLISP-STANDALONE-EXECUTABLE *FEATURES*)


;; Roslisp
(ros-load:load-system "roslisp" "roslisp")
(ros-load:load-system "ltfnp_executive" "ltfnp-executive")
