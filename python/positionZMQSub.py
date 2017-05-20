'''

 All components of this library are licensed under the BSD 3-Clause
 License.

 Copyright (c) 2015-, Algorithmic Robotics and Control Group @Rutgers
 (http://arc.cs.rutgers.edu). All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are
 met:

 Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.  Redistributions
 in binary form must reproduce the above copyright notice, this list of
 conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution. Neither the name of
 Rutgers University nor the names of the contributors may be used to
 endorse or promote products derived from this software without specific
 prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

'''
import sys
import zmq
import time
import threading
from threading import Thread
import copy

import utils

# Dictionary
carPosiDict = dict()

# Create a threading lock for safe access to dictionary
lock = threading.Lock()

def _pull_zmq_data():
    #Connect to zmq publisher 
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket

    print "Collecting update from server: tcp://%s:%s" \
        %(utils.zmqPublisherIP, utils.zmqPublisherPort)
    socket.connect ("tcp://%s:%s" %(utils.zmqPublisherIP,
        utils.zmqPublisherPort))
    print "Connected..."

    socket.setsockopt(zmq.SUBSCRIBE, "")

    #Continuous update of car position, if available
    while True:
        string = socket.recv()
        firstSpaceAt = 1
        while string[firstSpaceAt] != " ":
            firstSpaceAt += 1
        carID, rest = string[:firstSpaceAt], string[(firstSpaceAt + 1):]
        with lock:
            carPosiDict[int(carID)] = rest 
        
def _get_all_car_position_data():
    with lock:
        tempData = copy.deepcopy(carPosiDict)
    return tempData
        
def _get_car_position_data(carID):
    tempData = ""
    with lock:
        if carPosiDict.has_key(carID):
            tempData = carPosiDict[carID]
    return tempData

t = Thread(target = _pull_zmq_data)
t.setDaemon(True)

def _initialize_zmq():
    t.start()
    time.sleep(0.3)

def _stop_zmq():
    return
