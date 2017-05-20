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
microMVP useful functions.

"""

# The arduino ID and the respect tag of car (ID, tag), max 10
carInfo = [(2, 2), (3, 3), (4, 4), (5, 0), (6, 11),(7, 7),(8, 8),(9, 9),(10, 10)]
# IP address of the computer running position_.exe (local: zmqPublisherIP = "localhost")
zmqPublisherIP = "localhost"
# Port of xBee sender, can be found in Device Manager, Ports
xBeeSocket = 'COM3'
# Simulation Speed, lower value gives a more precise simulation
simSpeed = 5.0
# Sidelength of tag / Vehicle wheelbase
tagRatio = 0.90


"""
-------------------------------------------------------
Dont make changes under this line
-------------------------------------------------------
"""
# Part 0: configs
zmqPublisherPort = "5556"
wheelBase = 30.0
tagRatio = 0.90
container_width = 1420
container_height = 780
painter_width = 1280
painter_height = 720
spacer = 8
gridCopy = []

# Part 1: Colors
RGB_WHITE = (255, 255, 255)
RGB_BLACK = (0, 0, 0)
RGB_RED = (255, 0, 0)
RGB_GREEN = (0, 255, 0)
RGB_BLUE = (0, 0, 255)
RGB_PINK = (255, 0, 255)
RGB_YELLOW = (255, 255, 0)
RGB_GREY = (128, 128, 128)
RGB_DEEPBLUE = (0, 0, 255)
RGB_PURPLE = (127, 0, 255)

# High contrast colors, from http://godsnotwheregodsnot.blogspot.ru/2012/09/color-distribution-methodology.html
RGB_PATH_COLORS = [(0, 0, 0),(1, 0, 103),(213, 255, 0),(255, 0, 86),(158, 0, 142),(14, 76, 161),\
(255, 229, 2),(0, 95, 57),(0, 255, 0),(149, 0, 58),(255, 147, 126),(164, 36, 0),\
(0, 21, 68),(145, 208, 203),(98, 14, 0),(107, 104, 130),(0, 0, 255),(0, 125, 181),\
(106, 130, 108),(0, 174, 126),(194, 140, 159),(190, 153, 112),(0, 143, 156),(95, 173, 78),\
(255, 0, 0),(255, 0, 246),(255, 2, 157),(104, 61, 59),(255, 116, 163),(150, 138, 232),\
(152, 255, 82),(167, 87, 64),(1, 255, 254),(255, 238, 232),(254, 137, 0),(189, 198, 255),\
(1, 208, 255),(187, 136, 0),(117, 68, 177),(165, 255, 210),(255, 166, 254),(119, 77, 0),\
(122, 71, 130),(38, 52, 0),(0, 71, 84),(67, 0, 44),(181, 0, 255),(255, 177, 103),\
(255, 219, 102),(144, 251, 146),(126, 45, 210),(189, 211, 147),(229, 111, 254),(222, 255, 116),\
(0, 255, 120),(0, 155, 255),(0, 100, 1),(0, 118, 255),(133, 169, 0),(0, 185, 23),\
(120, 130, 49),(0, 255, 198),(255, 110, 65),(232, 94, 190)]


# Part n: Class for a car
class UnitCar:
    def __init__(self, tag = "0", ID = "0"):
        self.tag = tag
        self.ID = ID
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.lSpeed = 0.0
        self.rSpeed = 0.0
        self.path = list()

class Boundary:
    def __init__(self):
        self.u = 2 * wheelBase
        self.d = painter_height - 2 * wheelBase
        self.l = 2 * wheelBase
        self.r = painter_width - 2 * wheelBase
        self.width = self.r - self.l
        self.height = self.d - self.u

# Part 1: Dialogs in GUI
from pgu import gui

class AboutDialog(gui.Dialog):
    def __init__(self, **params):
        title = gui.Label("About microMVP")
        
        width = 400
        height = 200
        doc = gui.Document(width = width)
        
        space = title.style.font.size(" ")
        
        doc.block(align=0)
        for word in """microMVP v2.0""".split(" "): 
            doc.add(gui.Label(word))
            doc.space(space)
        doc.br(space[1])
        
        doc.block(align=-1)
        for word in """microMVP v2.0""".split(" "): 
            doc.add(gui.Label(word))
            doc.space(space)
        doc.br(space[1])
                          
        gui.Dialog.__init__(self,title,gui.ScrollArea(doc,width,height))

class HelpDialog(gui.Dialog):
    def __init__(self, **params):
        title = gui.Label("Help")
        
        doc = gui.Document(width=400)
        
        space = title.style.font.size(" ")
        
        doc.br(space[1])
        
        doc.block(align=-1)    
        for word in """Please refer to http://arc.cs.rutgers.edu/mvp/""".split(" "): 
            doc.add(gui.Label(word))
            doc.space(space)

        gui.Dialog.__init__(self,title,doc)     

class QuitDialog(gui.Dialog):
    def __init__(self, **params):
        title = gui.Label("Quit")
        
        t = gui.Table()
        
        t.tr()
        t.add(gui.Label("Are you sure you want to quit?"),colspan=2)
        
        t.tr()
        e = gui.Button("Okay")
        e.connect(gui.CLICK,self.send,gui.QUIT)
        t.td(e)
        
        e = gui.Button("Cancel")
        e.connect(gui.CLICK,self.close,None)
        t.td(e)
        
        gui.Dialog.__init__(self,title,t)

import math

def CheckCollosion(thresh, x1, y1, x2, y2):
        if math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2)) <= thresh:
            return True
        return False    