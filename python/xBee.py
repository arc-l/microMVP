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
Basic XBee functions
"""
import serial
import time
import sys
import utils

class Bee:
    def __init__(self, comPort):
        self.bee = serial.Serial(comPort, 57600)

    def xBeeSendSingle(self, id, left, right):
        carID = int(id)
        if(self.bee.writable()):
            cmd = bytearray(b'\x43\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\x4D')
            cmd[2*carID-1] = int(abs(right*127))
            cmd[2*carID] = int(abs(left*127))
            if right < 0:
                cmd[2*carID-1] = cmd[2*carID-1] | 0x80;
            if left < 0:
                cmd[2*carID] = cmd[2*carID] | 0x80;
            self.bee.write(cmd)


    def xBeeFlush7(self):
        # Send some garbage to force xBee refresh 
        if(self.bee.writable()):
            cmd = bytearray(b'\x45\x45\x45\x45\x45\x45\x45\x45\x45\x45\x45\x45\x45\x45\x45\x45\x45\x45\x45\x45\x45\x45')
            self.bee.write(cmd)
            time.sleep(0.5)

    def xBeeSend7(self, left, right):
        if(self.bee.writable()):
            cmd = bytearray(b'\x43\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\x4D')
            for carID in range(1, 8):
                cmd[2*carID-1] = int(abs(right*127))&0x7F
                cmd[2*carID] = int(abs(left*127))&0x7F
                if right < 0:
                    cmd[2*carID-1] = cmd[2*carID-1] | 0x80;
                if left < 0:
                    cmd[2*carID] = cmd[2*carID] | 0x80;
            self.bee.write(cmd)

    def xBeeSend(self, idList, lefts, rights):
        if(self.bee.writable()):
            cmd = bytearray(b'\x43\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\x4D')
            for index, carID in enumerate(idList):
                cmd[2*carID-1] = int(abs(rights[index]*127))&0x7F
                cmd[2*carID] = int(abs(lefts[index]*127))&0x7F
                if rights[index] < 0:
                    cmd[2*carID-1] = cmd[2*carID-1] | 0x80;
                if lefts[index] < 0:
                    cmd[2*carID] = cmd[2*carID] | 0x80;
            self.bee.write(cmd)

    # def xBeeSend(self, idList, lefts, rights):
    #     if(self.bee.writable()):
    #         cmd = bytearray(b'\x43\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\x4D')
    #         for carID in idList:
    #             cmd[2*carID-1] = int(abs(rights[carID]*127))&0x7F
    #             cmd[2*carID] = int(abs(lefts[carID]*127))&0x7F
    #             if rights[carID] < 0:
    #                 cmd[2*carID-1] = cmd[2*carID-1] | 0x80;
    #             if lefts[carID] < 0:
    #                 cmd[2*carID] = cmd[2*carID] | 0x80;
    #         self.bee.write(cmd)

if __name__ == "__main__":
    b = Bee(utils.xBeeSocket)
    b.xBeeSendSingle(sys.argv[1], 0.9, 0.9)
