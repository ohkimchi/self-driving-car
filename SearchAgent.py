import sys
from environment import Agent, Environment
from searchUtils import searchUtils
from simulator import Simulator


def manhattan_distance(prev, cur):
    return abs(prev[0] - cur[0]) + abs(prev[1] - cur[1])


def heuristic_cost_estimate(start, goal):
    return manhattan_distance(start, goal) / 2


def reconstruct_path(came_from, current):

    if current not in came_from:
        total_path = [current]
    else:
        total_path = []
        while current in came_from:
            total_path.append(came_from[current][-1])
            current = came_from[current][0]
        total_path.append(current)

    ans = total_path[::-1][1:]
    return ans


def get_lowest_f_score(f_score):
    min_score = min(f_score.values())
    return [(k, v) for k, v in f_score.items() if v == min_score]


def a_star(start, goals):
    start_tuple = start['location']
    actions = [None, 'forward-3x', 'forward-2x', 'forward', 'left', 'right']
    for goal in goals:
        goal_tuple = goal['location']
        closed_set = [start_tuple]
        open_set = [start_tuple]
        came_from = dict()
        g_score = {start_tuple: 0}
        f_score = {start_tuple: heuristic_cost_estimate(start_tuple, goal_tuple)}
        current = start
        current_tuple = start['location']
        while open_set:
            lowest_f_score = get_lowest_f_score(f_score)
            for current_tuple, _ in lowest_f_score:
                if current_tuple == goal_tuple:
                    return True, reconstruct_path(came_from, current_tuple)
                open_set.remove(current_tuple)
                del f_score[current_tuple]
                closed_set.append(current_tuple)
                for action in actions:
                    neighbor = the_env.applyAction(agent=the_agent, state={'location': current_tuple}, action=action)
                    prev = neighbor['previous']
                    prev = prev['location']
                    neighbor = neighbor['location']
                    if neighbor == prev:
                        continue
                    neighbor_tuple = neighbor
                    distance = manhattan_distance(current_tuple, neighbor_tuple)
                    tentative_g_score = g_score[current_tuple] + distance

                    if neighbor_tuple not in open_set:
                        open_set.append(neighbor_tuple)
                    elif tentative_g_score >= g_score[neighbor_tuple]:
                        continue
                    came_from[neighbor] = (current_tuple, action)
                    g_score[neighbor_tuple] = tentative_g_score
                    f_score[neighbor_tuple] = g_score[neighbor_tuple] + heuristic_cost_estimate(neighbor_tuple, goal_tuple)

        if current == goal:
            return True, reconstruct_path(came_from, current_tuple)
        else:
            return False, reconstruct_path(came_from, current_tuple)


class SearchAgent(Agent):
    def __init__(self, env, location=None):
        super(SearchAgent, self).__init__(env)
        self.valid_actions = self.env.valid_actions  # The set of valid actions
        self.action_sequence = []
        self.searchutil = searchUtils(env)

    def choose_action(self):
        action = None
        if len(self.action_sequence) >= 1:
            action = self.action_sequence[0]
        if len(self.action_sequence) >= 2:
            self.action_sequence = self.action_sequence[1:]
        else:
            self.action_sequence = []
        return action

    def drive(self, goal_states, grid):
        start_state = self.state
        goal_states = [x for x in goal_states]
        """Write your algorithm for self driving car"""
        print('start:', start_state, 'goal:', goal_states)
        ok, act_sequence = a_star(start_state, goal_states)
        return act_sequence

    def update(self):
        """ The update function is called when a time step is completed in the
            environment for a given trial. This function will build the agent
            state, choose an action, receive a reward, and learn if enabled. """
        startstate = self.state
        goalstates = self.env.getGoalStates()
        inputs = self.env.sense(self)
        self.action_sequence = self.drive(goalstates, inputs)
        print('self.action_sequence', self.action_sequence)
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
