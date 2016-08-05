#!/usr/bin/python


import time
from threading import Thread
from tools.Worker import Worker


def run(w, args):
    w.run(args)


def runWorker(w, args = []):
    thrdRun = Thread(target=run, args=(w, args))
    thrdRun.daemon = True
    
    thrdRun.start()
    time.sleep(1)
    
    while w.hasLines() or not w.isDone():
        line = w.nextLine()
        
        if line != None:
            if not line[0] == '%':
                print "Line:", line


if __name__ == "__main__":
    w = Worker("roslaunch")
    runWorker(w, ["ltfnp_reasoning", "ltfnp_reasoning.launch"])
