import heapq
import sys
from environment import Agent, Environment
from searchUtils import searchUtils
from simulator import Simulator


def manhattan_distance(prev, cur):
    return abs(prev[0] - cur[0]) + abs(prev[1] - cur[1])


def heuristic_cost_estimate(start, goal):
    return manhattan_distance(start, goal) / 2


def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        total_path.append(current)
        current = came_from[current]
    total_path.append(current)
    return total_path[::-1][1:]


def get_lowest_f_score(f_score):
    min_score = min(f_score.values())
    return [(k, v) for k, v in f_score.items() if v == min_score]


def a_star(start, goal):
    actions = [None, 'forward-3x', 'forward-2x', 'forward', 'left', 'right']
    closed_set = {start}
    open_set = {start}
    came_from = dict()
    g_score = {start: 0}
    f_score = {start: heuristic_cost_estimate(start, goal)}
    current = start
    while open_set:
        lowest_f_score = get_lowest_f_score(f_score)
        for current, cost in lowest_f_score:
            if current == goal:
                return True, reconstruct_path(came_from, current)
            open_set.remove(current)
            del f_score[current]
            closed_set.add(current)
            for action in actions:
                neighbor = the_env.applyAction(agent=the_agent, state={'location': current}, action=action)
                neighbor = neighbor['location']
                if neighbor == current:
                    continue
                distance = manhattan_distance(start, neighbor)
                tentative_g_score = g_score[current] + distance
                if neighbor not in open_set:
                    open_set.add(neighbor)
                elif tentative_g_score > g_score[neighbor]:
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic_cost_estimate(neighbor, goal)

    if current == goal:
        return True, reconstruct_path(came_from, current)
    else:
        return False, reconstruct_path(came_from, current)


class SearchAgent(Agent):
    def __init__(self, env, location=None):
        super(SearchAgent, self).__init__(env)
        self.valid_actions = self.env.valid_actions  # The set of valid actions
        self.action_sequence = []
        self.searchutil = searchUtils(env)

    def choose_action(self):
        print(self.action_sequence)
        action = None
        if len(self.action_sequence) >= 1:
            action = self.action_sequence[0]
        if len(self.action_sequence) >= 2:
            self.action_sequence = self.action_sequence[1:]
        else:
            self.action_sequence = []
        return action

    def drive(self, goal_states, grid):
        start_state = self.state['location']
        goal_states = [x['location'] for x in goal_states]
        """Write your algorithm for self driving car"""
        ok, act_sequence = a_star(start_state, goal_states[0])
        print(ok, act_sequence)
        return act_sequence

    def update(self):
        """ The update function is called when a time step is completed in the
            environment for a given trial. This function will build the agent
            state, choose an action, receive a reward, and learn if enabled. """
        startstate = self.state
        goalstates = self.env.getGoalStates()
        inputs = self.env.sense(self)
        self.action_sequence = self.drive(goalstates, inputs)
        action = self.choose_action()  # Choose an action
        self.state = self.env.act(self, action)
        return


def run(filename):
    """ Driving function for running the simulation.
        Press ESC to close the simulation, or [SPACE] to pause the simulation. """

    env = Environment(config_file=filename, fixmovement=False)

    agent = env.create_agent(SearchAgent)

    global the_env
    global the_agent
    the_env = env
    the_agent = agent

    def apply_action_from_env(current, action):
        env.applyAction(agent, current, action)

    env.set_primary_agent(agent)

    ##############
    # Create the simulation
    # Flags:
    #   update_delay - continuous time (in seconds) between actions, default is 2.0
    #   seconds
    #   display      - set to False to disable the GUI if PyGame is enabled
    sim = Simulator(env, update_delay=2)

    ##############
    # Run the simulator
    ##############
    sim.run()


if __name__ == '__main__':
    run(sys.argv[1])
