'''
 * Copyright (c) 2014, 2015 Entertainment Intelligence Lab, Georgia Institute of Technology.
 * Originally developed by Mark Riedl.
 * Last edited by Mark Riedl 05/2015
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
'''

import sys, pygame, math, numpy, random, time, copy, operator
from pygame.locals import *

from constants import *
from utils import *
from core import *

# Creates the path network as a list of lines between all path nodes that are traversable by the agent.
def myBuildPathNetwork(pathnodes, world, agent = None):
	lines = []
	### YOUR CODE GOES BELOW HERE ###

	for i in range(len(pathnodes)):
		point1 = pathnodes[i]
		for j in range(i + 1, len(pathnodes)):
			point2 = pathnodes[j]
			line = (point1, point2)
			if not rayTraceWorld(point1, point2, world.getLines()):
				collision = False
				for obstacle in world.getObstacles():
					for p in obstacle.getPoints():
						collision = collision or minimumDistance(line, p) < agent.getMaxRadius()
				if not collision:
					lines.append(line)

	### YOUR CODE GOES ABOVE HERE ###
	return lines
