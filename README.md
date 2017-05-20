# microMVP
microMVP software platform

Packages Needed for Pure Simulation:

pygame:		> pip install pygame

pyserial: 	> pip install pyserial

zmq: 		> pip install pyzmq

pgu:		https://github.com/parogers/pgu

munkres:	http://software.clapper.org/munkres/

Try to test if pure simulation works:

	> python gui.py -s

The next step is to control the hardware.
Get chilitag source file from https://github.com/chili-epfl/chilitags.
Find and replace detect-live.cpp with the file in our directory chilitags/detect-live.cpp
Build chilitag, now you can use the generated detect-live executable to detect the tags on the cars.
From commandline, run:

	> detect-live.exe 0 5556
	
	(replace 0 with the camera you want to use if you have multiple cameras)
	
Setup your own parameters for the cars:

	open utils.py and modify the parameters:
	
		carInfo, zmqPublisherIP, xBeeSocket, tagRatio
		
Try to test if hardware control works:

	> python gui.py

--------------------------------------------------------
(optional)

To run rvo2, first get the binaries from http://gamma.cs.unc.edu/RVO2/downloads/,
then build it with the file in our directory: rvo2/RvoCaller.cpp.

This generates an executable, rename it as RvoCaller.exe and place it in 

	python\algorithms\rvobin

To run the MRPP path planning solver, you need to install Java and Gurobi on your computer:

	http://www.oracle.com/technetwork/java/javase/downloads
	
	http://www.gurobi.com/
	
	And make sure gurobi is linked to your java.

You can also design your own robot dance patterns or path planning algorithms, 
please refer to file template.py in directory patterns/ or algorithms/.

--------------------------------------------------------
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
