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
    
    def run(self, args = []):
        if self.done:
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
