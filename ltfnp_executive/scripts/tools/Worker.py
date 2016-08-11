#  Software License Agreement (BSD License)
#  
#  Copyright (c) 2016, Institute for Artificial Intelligence,
#  Universitaet Bremen.
#  All rights reserved.
#  
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#  
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

# Author: Jan Winkler


import os
import sys
import time
import subprocess
from Queue import Queue, Empty
from threading import Thread, Timer


def enqueueWorkerOutput(source, queue):
    for line in iter(source.readline, b''):
        queue.put(line.rstrip())


class Worker(object):
    def __init__(self, executable):
        self.executable = executable
        self.done = True
        self.lines = []
        self.process = None
    
    def fullName(self):
        return self.executable
    
    def run(self, args = []):
        if self.done:
            try:
                self.process = subprocess.Popen([self.executable] + args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=False)
                self.done = False
                self.lines = []
                self.lineQueue = Queue()
                
                self.waitDoneThread = Thread(target=self.waitDone)
                self.waitDoneThread.daemon = True
                self.waitDoneThread.start()
                
                self.enqueueOutputThreadOut = Thread(target=enqueueWorkerOutput, args=(self.process.stdout, self.lineQueue))
                self.enqueueOutputThreadOut.daemon = True
                self.enqueueOutputThreadOut.start()
                
                self.enqueueOutputThreadErr = Thread(target=enqueueWorkerOutput, args=(self.process.stderr, self.lineQueue))
                self.enqueueOutputThreadErr.daemon = True
                self.enqueueOutputThreadErr.start()
                
                while not self.done:
                    try:
                        line = self.lineQueue.get_nowait()
                        
                        self.lines.append(line)
                    except Empty:
                        pass
                
                self.lineQueue.put("no-error")
            except OSError:
                self.lineQueue.put("fail-popen")
        else:
            print "A process is already running on this worker:", [self.executable] + args
    
    def nextLine(self):
        if len(self.lines) > 0:
            line = self.lines[0]
            self.lines = self.lines[1:]
            
            return line
        else:
            return None
    
    def hasLines(self):
        return len(self.lines) > 0
    
    def waitDone(self):
        self.process.wait()
        self.done = True
    
    def isDone(self):
       return self.done 
    
    def kill(self):
        self.process.terminate()
        print "\rTerminated '" + self.executable + "', wait for it to shut down"
        self.process.wait()
        print "Shutdown complete for " + self.executable
