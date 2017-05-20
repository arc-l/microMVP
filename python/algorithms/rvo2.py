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
"""
Path Planning Using RVO solver.
"""
import subprocess
from subprocess import STDOUT,PIPE
import os

pointGrid = list()

def GetPath(locs, goals, wb, bound):
    paths = [list() for x in range(len(locs))]

    # Generate the problem
    file = open("tempData", "w")
    file.write(str(len(locs)) + " ")
    file.write(str(wb * 2) + " ") 
    for loc in locs:
        file.write(str(loc[0]) + " " + str(loc[1]) + " ")
    for goal in goals:
        file.write(str(goal[0]) + " " + str(goal[1]) + " ")
    file.close()

    # Call RvoCaller.exe
    try:
        proc = subprocess.Popen(["algorithms/rvobin/RvoCaller.exe", "tempData"], stdin = PIPE, stdout=PIPE, stderr=STDOUT)
        stdout, stderr = proc.communicate()
        data = stdout.split()
    except WindowsError:
        print "RvoCaller.exe nor found!"
        return [[x] for x in locs]
    iterator = 0

    # Extract path
    while True:
        try:
            for i in range(len(locs)):
                paths[i].append((float(data[iterator]), float(data[iterator + 1])))
                iterator += 2
        except:
            break 

    return paths