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

import sys, pygame, math, numpy, random, time, copy
from pygame.locals import * 

from constants import *
from utils import *
from core import *



###############################
### AStarNavigator2
###
### Creates a path node network and implements the A* algorithm to create a path to the given destination.
			
class AStarNavigator2(PathNetworkNavigator):

				
	### Finds the shortest path from the source to the destination using A*.
	### self: the navigator object
	### source: the place the agent is starting from (i.e., its current location)
	### dest: the place the agent is told to go to
	def computePath(self, source, dest):
		self.setPath(None)
		### Make sure the next and dist matrices exist
		if self.agent != None and self.world != None: 
			self.source = source
			self.destination = dest
			### Step 1: If the agent has a clear path from the source to dest, then go straight there.
			### Determine if there are no obstacles between source and destination (hint: cast rays against world.getLines(), check for clearance).
			### Tell the agent to move to dest
			if clearShot(source, dest, self.world.getLinesWithoutBorders(), self.world.getPoints(), self.agent):
				self.agent.moveToTarget(dest)
			else:
				### Step 2: If there is an obstacle, create the path that will move around the obstacles.
				### Find the path nodes closest to source and destination.
				start = getOnPathNetwork(source, self.pathnodes, self.world.getLinesWithoutBorders(), self.agent)
				end = getOnPathNetwork(dest, self.pathnodes, self.world.getLinesWithoutBorders(), self.agent)
				if start != None and end != None:
					### Remove edges from the path network that intersect gates
					newnetwork = unobstructedNetwork(self.pathnetwork, self.world.getGates(), self.world)
					closedlist = []
					### Create the path by traversing the pathnode network until the path node closest to the destination is reached
					path, closedlist = astar(start, end, newnetwork)
					if path is not None and len(path) > 0:
						### Determine whether shortcuts are available
						path = shortcutPath(source, dest, path, self.world, self.agent)
						### Store the path by calling self.setPath()
						self.setPath(path)
						if self.path is not None and len(self.path) > 0:
							### Tell the agent to move to the first node in the path (and pop the first node off the path)
							first = self.path.pop(0)
							self.agent.moveToTarget(first)
		return None
		
	### Called when the agent gets to a node in the path.
	### self: the navigator object
	def checkpoint(self):
		myCheckpoint(self)
		return None

	### This function gets called by the agent to figure out if some shortcuts can be taken when traversing the path.
	### This function should update the path and return True if the path was updated.
	def smooth(self):
		return mySmooth(self)

	def update(self, delta):
		myUpdate(self, delta)


### Removes any edge in the path network that intersects a worldLine (which should include gates).
def unobstructedNetwork(network, worldLines, world):
	newnetwork = []
	for l in network:
		hit = rayTraceWorld(l[0], l[1], worldLines)
		if hit == None:
			newnetwork.append(l)
	return newnetwork



### Returns true if the agent can get from p1 to p2 directly without running into an obstacle.
### p1: the current location of the agent
### p2: the destination of the agent
### worldLines: all the lines in the world
### agent: the Agent object
def clearShot(p1, p2, worldLines, worldPoints, agent):
	### YOUR CODE GOES BELOW HERE ###

	return not any(rayTrace(p1, p2, line) for line in worldLines) and not any(minimumDistance((p1, p2), point) < agent.getMaxRadius() for point in worldPoints)

	### YOUR CODE GOES ABOVE HERE ###
	return False

### Given a location, find the closest pathnode that the agent can get to without collision
### agent: the agent
### location: the location to check from (typically where the agent is starting from or where the agent wants to go to) as an (x, y) point
### pathnodes: a list of pathnodes, where each pathnode is an (x, y) point
### world: pointer to the world
def getOnPathNetwork(location, pathnodes, worldLines, agent):
	node = None
	### YOUR CODE GOES BELOW HERE ###

	dist = INFINITY
	for n in pathnodes:
		if rayTraceWorld(location, n, worldLines) == None:
			d = distance(location, n)
			if (node == None or d < dist) and d > agent.getMaxRadius():
				node = n
				dist = d

	### YOUR CODE GOES ABOVE HERE ###
	return node



### Implement the a-star algorithm
### Given:
### Init: a pathnode (x, y) that is part of the pathnode network
### goal: a pathnode (x, y) that is part of the pathnode network
### network: the pathnode network
### Return two values: 
### 1. the path, which is a list of states that are connected in the path network
### 2. the closed list, the list of pathnodes visited during the search process
def astar(init, goal, network):
	path = []
	open = []
	closed = []
	### YOUR CODE GOES BELOW HERE ###

	open.append({
		'node': init,
		'path': [init],
		'f': 0
	})

	end = False

	while open and not end:
		q = min(open, key=lambda n: n['f'])
		open.remove(q)

		neighbors = [n[1] for n in network if n[0] == q['node']] + [n[0] for n in network if n[1] == q['node']]

		for node in neighbors:
			if not end and not any(c['node'] == node for c in closed):
				if node == goal:
					path = q['path'] + [node]
					end = True

				else:
					f = distance(q['node'], node) + distance(node, goal)

					if not any((o['node'] == node and o['f'] < f) for o in open):
						open.append({'node': node, 'path': q['path'] + [node], 'f': f})

		closed.append(q)

	closed = [c['node'] for c in closed]

	### YOUR CODE GOES ABOVE HERE ###
	return path, closed




def myUpdate(nav, delta):
	### YOUR CODE GOES BELOW HERE ###

	# if (nav.getDestination() is not None and any(rayTrace(nav.agent.getLocation(), nav.agent.moveTarget, gate) is None or minimumDistance(gate, nav.agent.getLocation()) <= nav.agent.getMaxRadius() for gate in nav.world.getGates())):
	# 	nav.agent.stopMoving()

	### YOUR CODE GOES ABOVE HERE ###
	return None

def myCheckpoint(nav):
	### YOUR CODE GOES BELOW HERE ###
	
	### YOUR CODE GOES ABOVE HERE ###
	return None







### This function optimizes the given path and returns a new path
### source: the current position of the agent
### dest: the desired destination of the agent
### path: the path previously computed by the A* algorithm
### world: pointer to the world
def shortcutPath(source, dest, path, world, agent):
	path = copy.deepcopy(path)
	### YOUR CODE GOES BELOW HERE ###

	if clearShot(source, dest, world.getLines(), world.getPoints(), agent):
		return []

	path = [source] + path + [dest]

	i = 0
	while i < len(path):
		j = i + 2
		while j < len(path):
			if clearShot(path[i], path[j], world.getLines(), world.getPoints(), agent):
				path.pop(j - 1)
			else:
				j += 1
		i += 1

	path.remove(source)
	path.remove(dest)

	### YOUR CODE GOES BELOW HERE ###
	return path


### This function changes the move target of the agent if there is an opportunity to walk a shorter path.
### This function should call nav.agent.moveToTarget() if an opportunity exists and may also need to modify nav.path.
### nav: the navigator object
### This function returns True if the moveTarget and/or path is modified and False otherwise
def mySmooth(nav):
	### YOUR CODE GOES BELOW HERE ###

	if nav.agent.getLocation() and nav.getDestination() and clearShot(nav.agent.getLocation(), nav.getDestination(), nav.world.getLines(), nav.world.getPoints(), nav.agent):
		nav.agent.moveToTarget(nav.getDestination())
		nav.path = []
		return True

	### YOUR CODE GOES ABOVE HERE ###
	return False

