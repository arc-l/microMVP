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
Pattern Path Generator Template
Modify this file to generate your own pattern.
Make copies of this file if you want to generate multiple different patterns.
The files will be automatically loaded into the gui.
The paths are represented as lists of points. 
The points are represented as tuples, e.g. (x, y)
Please make sure that there is no collision in your paths if the cars run in the same speed.
Check circle1.py or circle2.py if you have more questions.

Parameters:
num: number of cars
bound: class defines the boundary
    bound.u : upper boundary (y_min)
    bound.d : lower boundary (y_max)
    bound.l : left boundary (x_min)
    bound.r : right boundary (x_max)
    bound.width : bound.r - bound.l
    bound.height : bound.d - bound.u
"""

import math

def GetPath(num, bound):
    paths = [list() for x in range(num)]
    return paths