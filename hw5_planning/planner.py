from constants import *
from utils import *
from core import *

import pdb
import copy
from functools import reduce

from statesactions import *

############################
## HELPERS

### Return true if the given state object is a goal. Goal is a State object too.
def is_goal(state, goal):
  return len(goal.propositions.difference(state.propositions)) == 0

### Return true if the given state is in a set of states.
def state_in_set(state, set_of_states):
  for s in set_of_states:
    if s.propositions != state.propositions:
      return False
  return True

### For debugging, print each state in a list of states
def print_states(states):
  for s in states:
    ca = None
    if s.causing_action is not None:
      ca = s.causing_action.name
    print(s.id, s.propositions, ca, s.get_g(), s.get_h(), s.get_f())


############################
### Planner 
###
### The planner knows how to generate a plan using a-star and heuristic search planning.
### It also knows how to execute plans in a continuous, time environment.

class Planner():

  def __init__(self):
    self.running = False              # is the planner running?
    self.world = None                 # pointer back to the world
    self.the_plan = []                # the plan (when generated)
    self.initial_state = None         # Initial state (State object)
    self.goal_state = None            # Goal state (State object)
    self.actions = []                 # list of actions (Action objects)

  ### Start running
  def start(self):
    self.running = True
    
  ### Stop running
  def stop(self):
    self.running = False

  ### Called every tick. Executes the plan if there is one
  def update(self, delta = 0):
    result = False # default return value
    if self.running and len(self.the_plan) > 0:
      # I have a plan, so execute the first action in the plan
      self.the_plan[0].agent = self
      result = self.the_plan[0].execute(delta)
      if result == False:
        # action failed
        print("AGENT FAILED")
        self.the_plan = []
      elif result == True:
        # action succeeded
        done_action = self.the_plan.pop(0)
        print("ACTION", done_action.name, "SUCCEEDED")
        done_action.reset()
    # If the result is None, the action is still executing
    return result

  ### Call back from Action class. Pass through to world
  def check_preconditions(self, preconds):
    if self.world is not None:
      return self.world.check_preconditions(preconds)
    return False

  ### Call back from Action class. Pass through to world
  def get_x_y_for_label(self, label):
    if self.world is not None:
      return self.world.get_x_y_for_label(label)
    return None

  ### Call back from Action class. Pass through to world
  def trigger(self, action):
    if self.world is not None:
      return self.world.trigger(action)
    return False

  ### Generate a plan. Init and goal are State objects. Actions is a list of Action objects
  ### Return the plan and the closed list
  def astar(self, init, goal, actions):
      plan = []    # the final plan
      open = []    # the open list (priority queue) holding State objects
      closed = []  # the closed list (already visited states). Holds state objects
      ### YOUR CODE GOES HERE

      if (is_goal(init, goal)):
        return plan, closed

      open.append(init)

      end = None

      while open and not end:
        q = min(open, key=lambda n: n.get_f())
        open.remove(q)

        successors = []
        for action in actions:
          if all(p in q.propositions for p in action.preconditions):
            state = State(q.propositions.union(action.add_list).difference(action.delete_list))
            state.parent = q
            state.causing_action = action
            state.g = q.g + action.cost
            state.h = self.compute_heuristic(state, goal, actions)
            successors.append(state)

        for node in successors:
          if not end and not any(c.propositions == node.propositions for c in closed):
            if is_goal(node, goal):
              end = node
            elif not any(o.propositions == node.propositions and o.get_f() < node.get_f() for o in closed):
                open.append(node)

        closed.append(q)

      while end and end.causing_action:
        plan.insert(0, end.causing_action)
        end = end.parent

      ### CODE ABOVE
      return plan, closed

  ### Compute the heuristic value of the current state using the HSP technique.
  ### Current_state and goal_state are State objects.
  def compute_heuristic(self, current_state, goal_state, actions):
    actions = copy.deepcopy(actions)  # Make a deep copy just in case
    h = 0                             # heuristic value to return
    ### YOUR CODE BELOW

    start = Action('start', [], current_state.propositions, [])
    end = Action('end', goal_state.propositions, [], [])

    actions.append(start)
    actions.append(end)

    dists = {}
    graph = {}
    for action in actions:
      incoming = []
      for action2 in actions:
        for prop in action.preconditions:
          if prop in action2.add_list:
            incoming.append((prop, action2))
      graph[action] = incoming
      dists[action] = 1

    visited = set()
    queue = [start]
    while queue:
      node = queue.pop(0)
      visited.add(node)
      if graph[node]:
        dist = max(dists[edge[1]] for edge in graph[node])
        if dist > dists[node]:
          dists[node] = dist
      for action in graph:
        for edge in graph[action]:
          new_dist = dists[node] + action.cost
          if edge[1] == node and new_dist > dists[action]:
            dists[action] = new_dist
        if action not in visited and all(any(prop in act.add_list for act in visited) for prop in action.preconditions):
          queue.append(action)

    h = dists[end] - 2

    ### YOUR CODE ABOVE
    return h

