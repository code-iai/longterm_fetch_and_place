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


import os.path
import time
import signal
import sys
import yaml
from threading import Thread
from tools.Worker import Worker


workers_schedule = []
workers = []
killed = False
last_message_did_newline = True


def message(sender, subject, msg, do_newline = True):
    global last_message_did_newline
    max_width_no_newline = 60
    
    sys.stdout.write('\r')
    
    for i in range(0, max_width_no_newline):
        sys.stdout.write(" ")
    
    sys.stdout.write('\r')
    
    last_message_did_newline = do_newline
    
    color = ""
    if not do_newline:
        color = "\033[0m"
    
    fullmsg = color + "[" + sender + "] " + subject + ": " + msg
    
    if not do_newline:
        if len(fullmsg) > 30:
            fullmsg = fullmsg[:max_width_no_newline]
    
    sys.stdout.write(fullmsg)
    
    if do_newline:
        sys.stdout.write('\n')
    
    sys.stdout.flush()


def addWorker(cmd, args, checklist, timeout = None):
    workers_schedule.append([cmd, args, checklist, timeout])


def run(w, args):
    w.run(args)


def signalHandler(signal, frame):
    killed = True
    
    for w in workers:
        w.kill()


def addToChecklist(checklist, item, matchmode, template, message):
    checklist[item] = {}
    checklist[item]["matchmode"] = matchmode
    checklist[item]["template"] = template
    checklist[item]["message"] = message
    checklist[item]["matched"] = False


def maintainChecklist(w, checklist, line):
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
                message(w.fullName(), "Checklist", checklist[item]["message"])
                
                return True
    
    if len(checklist) == 0:
        return True
    
    return False


def isChecklistDone(checklist):
    all_match = True
    
    for item in checklist:
        if not checklist[item]["matched"]:
            all_match = False
            break
    
    return all_match


def runWorker(w, args = [], checklist = {}):
    global killed
    
    workers.append(w)
    
    thrdRun = Thread(target=run, args=(w, args))
    thrdRun.daemon = True
    
    thrdRun.start()
    time.sleep(1)
    
    while (w.hasLines() or not w.isDone()) and not killed:
        line = w.nextLine()
        
        if line != None:
            message("Line", "Out", line, False)
            
            if maintainChecklist(w, checklist, line):
                if isChecklistDone(checklist):
                    message(w.fullName(), "Run complete", "Moving into background")
                    break


def runNextWorker():
    global workers_schedule
    
    if len(workers_schedule) > 0:
        current_worker = workers_schedule[0]
        workers_schedule = workers_schedule[1:]
        
        w = Worker(current_worker[0])
        
        message(w.fullName(), "Run worker with parameters", str(current_worker[1]))
        runWorker(w, current_worker[1], current_worker[2])
        message(w.fullName(), "Run complete", "Advancing pipeline")
        
        return True
    
    return False


def loadWorkersFromYaml(doc):
    print doc


if __name__ == "__main__":
    doc = None
    
    if len(sys.argv) > 1:
        yamlfile = sys.argv[1]
        
        if os.path.isfile(yamlfile):
            with open(yamlfile, 'r') as f:
                doc = yaml.load(f)
    
    if doc:
        loadWorkersFromYaml(doc)
        
        signal.signal(signal.SIGINT, signalHandler)
        
        # From here, the actual parameterization of the scenario starts;
        # you can put whatever you want in these worker checklists.
        
        # cl1 = {}
        # addToChecklist(cl1, "moveit", "match",
        #                "All is well! Everyone is happy! You can start planning now!",
        #                "MoveIt! launched successfully")
        # addToChecklist(cl1, "attache", "contains",
        #                "Attache plugin loaded",
        #                "Attache plugin present")
        # addToChecklist(cl1, "spawn_model", "match",
        #                "spawn_model script started",
        #                "Gazebo accepts spawn_model requests")
        # addToChecklist(cl1, "reasoning", "contains",
        #                "ltfnp_reasoning/prolog/init.pl compiled",
        #                "Reasoning started successfully")
        # addToChecklist(cl1, "gzclient", "contains",
        #                "Connected to gazebo master",
        #                "Gazebo Client successfully connected to gzserver")
        
        # addWorker("roslaunch", ["ltfnp_executive", "ltfnp_simulation.launch"], cl1)
        
        # cl2 = {}
        # addToChecklist(cl2, "initialize", "contains",
        #                "Initialization complete. You can start using the system now.",
        #                "Semrec Initialized")
        
        # addWorker("rosrun", ["semrec", "semrec"], cl2)
        ## continue here
        # cl3 = {}
        # addToChecklist(cl3, "prep_db", "contains",
        #                "MongoDB shell",
        #                "MongoDB Logging started")
        
        # addWorker("rosrun", ["ltfnp_executive", "prep_mongodb.sh"], cl3)
        
        # cl4 = {}
        # addToChecklist(cl4, "connect_ros", "contains",
        #                "Connecting to ROS",
        #                "Connecting to ROS")
        # addToChecklist(cl4, "running", "contains",
        #                "Running Longterm Fetch and Place",
        #                "Started scenario execution")
        # addToChecklist(cl4, "done", "contains",
        #                "Done with LTFnP",
        #                "Scenario completed")
        
        # addWorker("rosrun", ["ltfnp_executive", "start.sh"], cl4)
        
        # cl5 = {}
        # addToChecklist(cl5, "connect_ros", "contains",
        #                "Connecting to ROS",
        #                "Connecting to ROS")
        # addToChecklist(cl5, "running", "contains",
        #                "Running Longterm Fetch and Place",
        #                "Started scenario execution")
        # addToChecklist(cl5, "done", "contains",
        #                "Done with LTFnP",
        #                "Scenario completed")
        
        # addWorker("rosrun", ["ltfnp_executive", "package_log.sh"], cl5)
        
        while runNextWorker() and not killed:
            pass
        
        message("Core", "All tasks completed", "Tearing down workers")
        
        signalHandler(None, None)
    else:
        message("Core", "Invalid", "No or no valid yaml configuration supplied")
