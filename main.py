import sys
import heapq
import time

class GridWorld():
    ACTIONS = ['U', 'D', 'L', 'R']

    def __init__(self):
        self.n_rows = 9
        self.n_cols = 9

        # indexing is top to bottom, left to right (matrix indexing)
        init_r = 8
        init_c = 0
        self.init_state = (init_r, init_c)
        goal_r = 0
        goal_c = 8
        self.goal_state = (goal_r, goal_c)
        self.obstacles = [[0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 1, 1, 1, 1, 1, 0, 0],
                          [0, 0, 0, 0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 1, 1, 1, 1, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0]]

        self.costs = [[1, 1, 1, 5, 5, 5, 5, 1, 1],
                      [1, 1, 1, 5, 5, 5, 5, 1, 1],
                      [1, 1, 10, 10, 10, 10, 10, 1, 1],
                      [1, 1, 1, 10, 10, 10, 10, 1, 1],
                      [1, 1, 1, 1, 1, 10, 10, 1, 1],
                      [1, 1, 1, 1, 1, 10, 10, 1, 1],
                      [1, 1, 1, 1, 10, 10, 10, 1, 1],
                      [1, 1, 1, 10, 10, 10, 10, 1, 1],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1]]

    def step(self, state, action):
        """
        :param state: (row, column) tuple representing the current position of the agent
        :param action: One of 'U', 'D', 'L' or 'R'
        :return: Success (True or False), new state, action cost
        """
        r, c = state
        new_r = r
        new_c = c

        if action == 'U':
            new_r -= 1
        elif action == 'D':
            new_r += 1
        elif action == 'L':
            new_c -= 1
        elif action == 'R':
            new_c += 1
        else:
            assert False, "Invalid action"

            if (not (0 <= new_r < 9) or (not 0 <= new_c < 9)) or self.obstacles[new_r][new_c] == 1:
                return False, (r,v), self.costs[r][c]
            else:
                return True, (new_r, new_c), self.costs[new_r][new_c]

    def is_goal(self, state):
        """
        :param state: (row, col) tuple
        :return: True or False value, indicating whether the current state is the goal state
        """
        return state == self.goal_state

    def get_state_cost(self, state):
        r, c = state
        return self.costs[r][c]

    # Less than function required for heapq
    def __lt__(self, other):
        return True

class StateNode:
    def __init__(self, env, state, actions, path_cost):
        self.env = env
        self.state = state
        self.actions = actions # Full path taken to get here
        self.path_cost = path_cost

    def get_successors(self):
        successors = []
        for a in GridWorld.ACTIONS:
            success, new_state, a_cost = self.env.step(self.state, a)
            if success:
                successors.append(StateNode(self.env,
                                            new_state,
                                            self.actions + [a],
                                            self.path_cost + self.env.get_state_cost(new_state)))
        return successors

def ucs(env):
        container = [(0, StateNode(env, env.init_state, [], 0))]
        heapq.heapify(container)
        visited = {env.init_state : 0}
        n_expanded = 0

        while container: # Exhaust container
            n_expanded += 1
            _, node = heapq.heappop(container)

            # Did we reach the goal?
            if env.is_goal(node.state):
                print("Nodes expanded:", n_expanded)
                return node.actions

            # Add unvisited successors to the queue
            successors = node.get_successors()
            for s in successors:
                if s.state not in visited.keys() or s.path_cost < visited[s.state]:
                    visited[s.state] = s.path_cost
                    heapq.heappush(container, (s.path_cost,s))
        return None

def manhat(env, state):
    return abs(env.goal_state[0] - state[0]) + abs(env.goal_state[1] - state[1])

def heuristic(env, state):
    return manhat(env, state)

def a_star(env):
    container = [(0, StateNode(env, env.init_state, [], 0))]
    heapq.heapify(container)
    visited = {env.init_state: 0}
    n_expanded = 0

    while container:  # Exhaust container
        n_expanded += 1
        _, node = heapq.heappop(container)

        # Did we reach the goal?
        if env.is_goal(node.state):
            print("Nodes expanded:", n_expanded)
            return node.actions

        # Add unvisited successors to the queue
        successors = node.get_successors()
        for s in successors:
            if s.state not in visited.keys() or s.path_cost < visited[s.state]:
                visited[s.state] = s.path_cost
                heapq.heappush(container, (s.path_cost + heuristic(env, s.state), s))
    return None



def main(arglist):
    n_trials = 50
    gridworld = GridWorld()

    print("=== UCS ===")
    t0 = time.time()
    for i in range(n_trials):
        actions_ucs = ucs(gridworld)
    t_ucs = (time.time() - t0) / n_trials
    print(f"Num actions: {len(actions_ucs)},\t\tActions: {actions_ucs}")
    print(f"Time: {t_ucs}")
    print()

    print("=== A* ===")
    t0 = time.time()
    for i in range(n_trials):
        actions_a_star = a_star(gridworld)
    t_a_star = (time.time() - t0) / n_trials
    print(f"Num actions: {len(actions_ucs)},\t\tActions: {actions_a_star}")
    print(f"Time: {t_a_star}")
    print()


if __name__ == '__main__':
    main(sys.argv[1:])