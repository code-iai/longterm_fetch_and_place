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
import multiprocessing
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
    
    w = []


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


def runWorker(w, args, checklist):
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


def runWorkerWithTimeout(w, args = [], checklist = {}, timeout = None):
    global killed
    
    if timeout:
        p = multiprocessing.Process(target=runWorker, args=(w, args, checklist))
        p.start()
        
        p.join(timeout)
        
        if p.is_alive():
            message(w.fullName(), "Run", "Timeout reached, shutting down.")
            killed = True
            signalHandler(None, None)
    else:
        runWorker(w, args, checklist)


def runNextWorker():
    global workers_schedule
    
    if len(workers_schedule) > 0:
        current_worker = workers_schedule[0]
        workers_schedule = workers_schedule[1:]
        
        w = Worker(current_worker[0])
        
        message(w.fullName(), "Run worker with parameters", str(current_worker[1]))
        runWorkerWithTimeout(w, current_worker[1], current_worker[2], current_worker[3])
        message(w.fullName(), "Run complete", "Advancing pipeline")
        
        return True
    
    return False


def loadWorker(worker):
    details = {}
    
    for detail in worker:
        key = detail.keys()[0]
        details[key] = detail[key]
    
    if not "command" in details:
        details["command"] = ""
    
    if not "parameters" in details:
        details["parameters"] = []
    
    if not "checklist" in details:
        details["checklist"] = {}
    
    if not "timeout" in details:
        details["timeout"] = None
    
    checklist = {}
    
    for item in details["checklist"]:
        data = item["item"]
        checklistitem = {}
        
        for detail in data:
            key = detail.keys()[0]
            checklistitem[key] = detail[key]
        
        addToChecklist(checklist, checklistitem["name"], checklistitem["matchmode"], checklistitem["template"], checklistitem["message"])
    
    addWorker(details["command"], details["parameters"], checklist, details["timeout"])


def loadWorkersFromYaml(doc):
    for worker_wrap in doc:
        loadWorker(worker_wrap["worker"])


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
        
        while runNextWorker() and not killed:
            pass
        
        message("Core", "All tasks completed", "Tearing down workers")
        signalHandler(None, None)
        
        print workers_schedule
    else:
        message("Core", "Invalid", "No or no valid yaml configuration supplied")
