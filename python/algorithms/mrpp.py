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
Path Planning Using MRPP solver.
"""
import utils
import math
import sys
import os
import os.path, subprocess
from subprocess import STDOUT,PIPE

def GetDist(x1, y1, x2, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

def generateGrid(wb, bound):
    newLine = []
    stepSize = (bound.height) / 5.0 * (2 / math.sqrt(3))
    xSize = 6
    ySize = 6
    newLine.append((bound.l + stepSize / 2, bound.u))    
    while True:
        if len(newLine) % 2 != len(utils.gridCopy) % 2:
            stepSpace = 1
        else:
            stepSpace = 2
        newLine.append((newLine[-1][0] + stepSize * stepSpace, newLine[0][1]))
        if len(newLine) > xSize:
            newLine.pop(-1)
            utils.gridCopy.append(newLine)
            nextLine = []
            if len(utils.gridCopy) % 2 == 1:
                nextLineMove = -1
            else:
                nextLineMove = 1
            nextLine.append((newLine[0][0] + nextLineMove * stepSize / 2, newLine[0][1] + math.sqrt(3) * stepSize / 2))
            if len(utils.gridCopy) == ySize:
                break
            newLine = nextLine

def assignStart(locs):
    startVertex = []
    # Get the start vertices 
    for x, y in locs:
        closestVertex = -1
        shortestDistace = 10000.0
        # Get the closest vertices
        for i in range(len(utils.gridCopy[0])):
            for j in range(len(utils.gridCopy)):
                thisVertexDist = GetDist(x, y, utils.gridCopy[i][j][0], utils.gridCopy[i][j][1])
                if thisVertexDist < shortestDistace:
                    # If it's already occupied by another car, don't use it
                    if i * 6 + j in startVertex:
                        continue
                    closestVertex = i * 6 + j
                    shortestDistace = thisVertexDist
        startVertex.append(closestVertex)
    return startVertex

def execute_java(startVertex):
    """ Run Path Planning Algorithm, Return path """
    cmd = ['java','projects.multipath.ILP.Main', '11', str(len(utils.gridCopy[0])), str(len(utils.gridCopy))]
    for vertex in startVertex:
        cmd.append(str(vertex))
    proc = subprocess.Popen(cmd, stdin=PIPE, stdout=PIPE, stderr=STDOUT)
    stdout, stderr = proc.communicate()
    return stdout

def extractPath(stdout, startVertex):
    lines = stdout.splitlines()
    pointList = []
    for i in range(len(lines)):
        lines[i] = str(lines[i])
        if len(lines[i]) == 0:
            continue
        if lines[i][0] != 'A':
            continue
        pointList.append([])
        for j in range(len(lines[i])):
            if lines[i][j] == "(":
                pointList[-1].append(utils.gridCopy[int(lines[i][j + 3])][int(lines[i][j + 1])])
    return pointList

def GetPath(locs, goals, wb, bound):
    # paths = [list() for x in range(num)]
    if len(utils.gridCopy) == 0: 
        generateGrid(wb, bound)
    startVertex = assignStart(locs)
    stdout = execute_java(startVertex)
    paths = extractPath(stdout, startVertex) 
    if len(paths) == 0:
        print "MRPP solver not working properly, did you installed gurobi correctly?"
        return [[x] for x in locs]
    return paths