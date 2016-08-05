#!/usr/bin/python

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


import time
import signal
import sys
from threading import Thread
from tools.Worker import Worker


workers = []


def run(w, args):
    w.run(args)


def signalHandler(signal, frame):
    for w in workers:
        w.kill()


def addToChecklist(checklist, item, matchmode, template, message):
    checklist[item] = {}
    checklist[item]["matchmode"] = matchmode
    checklist[item]["template"] = template
    checklist[item]["message"] = message
    checklist[item]["matched"] = False


def maintainChecklist(checklist, line):
    for item in checklist:
        if not checklist[item]["matched"]:
            match = False
            
            if checklist[item]["matchmode"] == "match":
                if line == checklist[item]["template"]:
                    match = True
            elif checklist[item]["matchmode"] == "contains":
                if checklist[item]["template"] in line:
                    match = True
            
            if match:
                checklist[item]["matched"] = True
                print checklist[item]["message"]
                
                return True
    
    return False


def isChecklistDone(checklist):
    all_match = True
    
    for item in checklist:
        if not checklist[item]["matched"]:
            all_match = False
            break
    
    return all_match


def runWorker(w, args = []):
    workers.append(w)
    
    thrdRun = Thread(target=run, args=(w, args))
    thrdRun.daemon = True
    
    thrdRun.start()
    time.sleep(1)
    
    checklist = {}
    addToChecklist(checklist, "moveit", "match",
                   "All is well! Everyone is happy! You can start planning now!",
                   "MoveIt! launched successfully")
    addToChecklist(checklist, "attache", "contains",
                   "Attache plugin loaded",
                   "Attache plugin present")
    addToChecklist(checklist, "spawn_model", "match",
                   "spawn_model script started",
                   "Gazebo accepts spawn_model requests")
    addToChecklist(checklist, "reasoning", "contains",
                   "ltfnp_reasoning/prolog/init.pl compiled",
                   "Reasoning started successfully")
    addToChecklist(checklist, "gzclient", "contains",
                   "Connected to gazebo master",
                   "Gazebo Client successfully connected to gzserver")
    
    while w.hasLines() or not w.isDone():
        line = w.nextLine()
        
        if line != None:
            if maintainChecklist(checklist, line):
                if isChecklistDone(checklist):
                    # TODO: Do something once the checklist is
                    # complete; probably running the next worker
                    pass

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signalHandler)
    
    w = Worker("roslaunch")
    runWorker(w, ["ltfnp_executive", "ltfnp_simulation.launch"])
