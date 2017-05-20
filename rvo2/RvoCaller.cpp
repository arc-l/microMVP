/*
All components of this library are licensed under the BSD 3 - Clause
License.

Copyright(c) 2015 - , Algorithmic Robotics and Control Group @Rutgers
(http://arc.cs.rutgers.edu). All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met :

Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.Redistributions
in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.Neither the name of
Rutgers University nor the names of the contributors may be used to
endorse or promote products derived from this software without specific
prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT
	LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include<iostream>
#include<fstream>
#include "RVO.h"
std::vector<RVO::Vector2> goals;

void updateVisualization(RVO::RVOSimulator* sim) {
	// Output the current global time.
	// std::cout << sim->getGlobalTime() << " ";

	// Output the position for all the agents.
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		// std::cout << sim->getAgentPosition(i) << " ";
		std::cout << sim->getAgentPosition(i).x() << " " << sim->getAgentPosition(i).y() << " ";
	}

	// std::cout << std::endl;
}

bool reachedGoal(RVO::RVOSimulator* sim) {
	// Check whether all agents have arrived at their goals.
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		if (absSq(goals[i] - sim->getAgentPosition(i)) > sim->getAgentRadius(i) * sim->getAgentRadius(i)) {
			// Agent is further away from its goal than one radius.
			return false;
		}
	}
	return true;
}

void setPreferredVelocities(RVO::RVOSimulator* sim) {
	// Set the preferred velocity for each agent.
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		if (absSq(goals[i] - sim->getAgentPosition(i)) < sim->getAgentRadius(i) * sim->getAgentRadius(i)) {
			// Agent is within one radius of its goal, set preferred velocity to zero
			sim->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
		}
		else {
			// Agent is far away from its goal, set preferred velocity as unit vector towards agent's goal.
			sim->setAgentPrefVelocity(i, normalize(goals[i] - sim->getAgentPosition(i)));
		}
	}
}

int main(int argc, char *argv[])
{
	int numCar;
	float carRatio;

	std::fstream fs;
	fs.open(argv[1], std::fstream::in | std::fstream::out | std::fstream::app);
	fs >> numCar >> carRatio;

	// Create a new simulator instance.
	RVO::RVOSimulator* sim = new RVO::RVOSimulator(1.0f, 1000.0f, 5, carRatio * 2, carRatio * 2, carRatio, 5.0);

	// Add agents, specifying their start position.
	for (int i = 0; i < numCar; i++)
	{
		float tempX, tempY;
		fs >> tempX >> tempY;
		sim->addAgent(RVO::Vector2(tempX, tempY));
	}

	// Create goals (simulator is unaware of these).
	for (size_t i = 0; i < sim->getNumAgents(); ++i) 
	{
		float tempX, tempY;
		fs >> tempX >> tempY;
		goals.push_back(RVO::Vector2(tempX, tempY));
	}

	// Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.
	std::vector<RVO::Vector2> vertices;
	vertices.push_back(RVO::Vector2(0.0f, 0.0f));
	vertices.push_back(RVO::Vector2(0.0f, 720.0f));
	vertices.push_back(RVO::Vector2(1280.0f, 720.0f));
	vertices.push_back(RVO::Vector2(1280.0f, 0.0f));
	sim->addObstacle(vertices);
	// Process obstacles so that they are accounted for in the simulation.
	sim->processObstacles();

	// Perform (and manipulate) the simulation.
	int maxIter = 0;
	do {
		updateVisualization(sim);
		setPreferredVelocities(sim);
		sim->doStep();
	} while (!reachedGoal(sim) && maxIter++ < 3500);

	delete sim;
	fs.close();
}

