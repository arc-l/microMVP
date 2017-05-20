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
This is the differential drive model.
"""
import utils
import math

def GetDist(x1, y1, x2, y2):
	return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

def NormalizeAngle(angle):
	""" Convert angle to [0, 2pi) """
	if angle < 0:
		angle += 2 * math.pi
	elif angle >= 2 * math.pi:
		angle -= 2 * math.pi
	if angle >= 0 and angle < 2 * math.pi:
		return angle
	else:
		return NormalizeAngle(angle)

def calculateATan(diff_x, diff_y):
	""" Given the difference of the coordinate, calculate the angle """
	diff_x += 0.0
	diff_y += 0.0
	if diff_x == 0:
		if diff_y > 0:
			return math.pi / 2
		else:
			return math.pi * 3 / 2
	if diff_y == 0:
		if diff_x > 0:
			return 0
		else:
			return math.pi
	angle = math.atan(diff_y / diff_x)
	if diff_x > 0 and diff_y > 0:
		return angle
	elif diff_x < 0 and diff_y > 0:
		return angle + math.pi
	elif diff_x < 0 and diff_y < 0:
		return angle + math.pi
	else:
		return angle + 2 * math.pi	

def Calculate(x, y, theta, path, v, wb):
	# Delete points close to car
	while True:
		try:
			if GetDist(x, y , path[0][0], path[0][1]) <= wb * 0.9:
				path.pop(0)
			else:
				break
		except:
			if len(path) == 0:
				return 0.0, 0.0
	(px, py) = path[0]
	(vx, vy) = px - x, py - y
	v = v / 2
	theta = NormalizeAngle(theta)
	# Check backward
	direction = calculateATan(vx, vy)
	diffAngle = math.fabs(direction - theta)
	if diffAngle > math.pi / 2 and diffAngle < math.pi * 3 / 2:
		if direction > theta and direction - theta > math.pi:
			return v, -v
		elif direction < theta and theta - direction < math.pi:
			return v, -v
		else:
			return -v, v
	# Forward, Calculate speed
	dist = GetDist(x, y, px, py)
	angle_at_center = NormalizeAngle(direction - theta)
	if angle_at_center != 0:
		radius = (dist / 2) / math.sin(angle_at_center)
		if radius < wb / 2:
			vL = v * (-(wb / 2 - radius) / radius)
			vR = v * ((radius + wb / 2) / radius)
		else:
			vL = v * ((radius - wb / 2) / radius)
			vR = v * ((radius + wb / 2) / radius)
		if vL > 1.0:
			return 1.0, vR / vL
		elif vR > 1.0:
			return vL / vR, 1.0
		else:
			return vL, vR
	else:
		return v, v

def Simulate(x, y, theta, vL, vR, wb):
	if vL == 0 and vR == 0:
		pass
	elif vL == -vR:
		if vL > 0:
			theta = NormalizeAngle(theta - (vL / (wb / 2)))
		else:
			theta = NormalizeAngle(theta + (vR / (wb / 2)))		
	elif vL < 0 or vR < 0:
		total = math.fabs(vL - vR)
		if vL > 0:
			radius = wb * (vL / total - 0.5)
			step = vL * (vL / total - 0.5)
			angle = step / radius
			direction = theta - angle / 2
			theta = NormalizeAngle(theta - angle)
		if vL < 0:
			radius = wb * (vR / total - 0.5)
			step = vR * (vR / total - 0.5)
			angle = step / radius
			direction = theta + angle / 2
			theta = NormalizeAngle(theta + angle)
		x += math.cos(direction) * step
		y += math.sin(direction) * step
	else:
		if vL == vR:
			x += math.cos(theta) * vL
			y += math.sin(theta) * vR
		elif vL > vR:
			bigR = wb / (vL - vR)
			radius = wb / (vL - vR) - wb / 2
			step = vL * (radius / bigR)
			angle = step / radius
			direction = theta - angle / 2
			theta = NormalizeAngle(theta - angle)
			x += math.cos(direction) * step
			y += math.sin(direction) * step
		else:
			bigR = wb / (vR - vL)
			radius = wb / (vR - vL) - wb / 2
			step = vR * (radius / bigR)	
			angle = step / radius
			direction = theta + angle / 2	
			theta = NormalizeAngle(theta + angle)
			x += math.cos(direction) * step
			y += math.sin(direction) * step
	return (x, y, theta)



# from random import random
# from time import sleep
# while True:
# 	vL, vR = Calculate(random() * 1000, random() * 1000, 1.1, [(random() * 1000, random() * 1000)], 1.0)
# 	if vL > 1.0 or vR > 1.0:
# 		print vL, vR
# 		sleep(1)
# 	pass