import math
import time
from collections import deque
from queue import PriorityQueue

class Graph:
    def __init__(self):
        self.edges = {}
        self.locations = {}

    def add_location(self, name, x, y, z):
        self.locations[name] = (x, y, z)
        self.edges[name] = []

    def add_edge(self, src, dest):
        self.edges[src].append(dest)
        self.edges[dest].append(src)

    def get_neighbors(self, location):
        return self.edges[location]

    def get_location(self, name):
        return self.locations[name]

def parse_input(file_path):
    graph = Graph()
    with open(file_path, 'r') as file:
        algorithm = file.readline().strip()
        energy_limit = int(file.readline().strip())
        num_locations = int(file.readline().strip())

        for _ in range(num_locations):
            name, x, y, z = file.readline().strip().split()
            graph.add_location(name, int(x), int(y), int(z))

        num_edges = int(file.readline().strip())
        for _ in range(num_edges):
            src, dest = file.readline().strip().split()
            graph.add_edge(src, dest)

    return algorithm, energy_limit, graph

def calculate_energy(graph, src, dest):
    x1, y1, z1 = graph.get_location(src)
    x2, y2, z2 = graph.get_location(dest)
    return z2 - z1

def is_valid_move(graph, energy_limit, momentum, src, dest):
    energy_needed = calculate_energy(graph, src, dest)
    return energy_needed <= energy_limit + momentum

def euclidean_distance_2d(graph, src, dest):
    x1, y1, _ = graph.get_location(src)
    x2, y2, _ = graph.get_location(dest)
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def euclidean_distance_3d(graph, src, dest):
    x1, y1, z1 = graph.get_location(src)
    x2, y2, z2 = graph.get_location(dest)
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)


# Breadth-first search (BFS)
def bfs(graph, start, goal, energy_limit):
    queue = deque([(start, 0, [start])])
    visited = set()

    while queue:
        current, current_momentum, path = queue.popleft()

        if current == goal:
            return path
        
        for neighbor in graph.get_neighbors(current):
            energy_needed = calculate_energy(graph, current, neighbor)
            next_momentum = -energy_needed if energy_needed < 0 else 0

            new_state = (neighbor, next_momentum)
            if new_state not in visited and is_valid_move(graph, energy_limit, current_momentum, current, neighbor):
                visited.add(new_state)
                queue.append((neighbor, next_momentum, path + [neighbor]))

    return "FAIL"


# Uniform-cost search (UCS)
def ucs(graph, start, goal, energy_limit):
    queue = PriorityQueue()  # (path_cost, current_node, momentum, path)
    queue.put((0, start, 0, [start]))
    visited = {}  # Maps (node, momentum) to the lowest path cost at which it was visited

    while not queue.empty():
        path_cost, current, current_momentum, path = queue.get()

        # Check if this state has been visited with a lower cost; if so, skip it
        if (current, current_momentum) in visited and visited[(current, current_momentum)] <= path_cost:
            continue

        visited[(current, current_momentum)] = path_cost

        if current == goal:
            return path
        
        for neighbor in graph.get_neighbors(current):
            energy_needed = calculate_energy(graph, current, neighbor)
            next_momentum = -energy_needed if energy_needed < 0 else 0

            if is_valid_move(graph, energy_limit, current_momentum, current, neighbor):
                new_path_cost = path_cost + euclidean_distance_2d(graph, current, neighbor)
                new_state = (neighbor, next_momentum)

                # Only enqueue if this is a new state or if this path to the state is cheaper
                if new_state not in visited or new_path_cost < visited[new_state]:
                    new_path = path + [neighbor]
                    queue.put((new_path_cost, neighbor, next_momentum, new_path))

    return "FAIL"


def a_star(graph, start, goal, energy_limit):
    queue = PriorityQueue()  # (estimated_total_cost, path_cost, current_node, momentum, path)
    visited = {}  # Maps (node, momentum) pairs to the lowest path cost at which they were visited

    start_heuristic = euclidean_distance_3d(graph, start, goal)
    queue.put((start_heuristic, 0, start, 0, [start]))  # Initialize with start node and heuristic

    while not queue.empty():
        estimated_total_cost, path_cost, current, current_momentum, path = queue.get()

        if (current, current_momentum) in visited and visited[(current, current_momentum)] <= path_cost:
            continue

        visited[(current, current_momentum)] = path_cost

        if current == goal:
            return path

        for neighbor in graph.get_neighbors(current):
            energy_needed = calculate_energy(graph, current, neighbor)
            next_momentum = -energy_needed if energy_needed < 0 else 0

            if is_valid_move(graph, energy_limit, current_momentum, current, neighbor):
                new_path_cost = path_cost + euclidean_distance_3d(graph, current, neighbor)
                heuristic_cost = euclidean_distance_3d(graph, neighbor, goal)
                new_estimated_total_cost = new_path_cost + heuristic_cost

                new_state = (neighbor, next_momentum)
                if new_state not in visited or new_path_cost < visited.get(new_state, float('inf')):
                    queue.put((new_estimated_total_cost, new_path_cost, neighbor, next_momentum, path + [neighbor]))

    return "FAIL"



def write_output(path, output_file="output.txt"):
    with open(output_file, "w") as file:
        if path == "FAIL":
            file.write(path)
        else:
            file.write(" ".join(path))

def main():
    algorithm, energy_limit, graph = parse_input('input.txt')

    # start_time = time.time()  # Start time measurement

    if algorithm == "BFS":
        start, goal = "start", "goal"
        path = bfs(graph, start, goal, energy_limit)
        write_output(path)
    
    elif algorithm == "UCS":
        start, goal = "start", "goal"
        path = ucs(graph, start, goal, energy_limit)
        write_output(path)

    elif algorithm == "A*":
        start, goal = "start", "goal"
        path = a_star(graph, start, goal, energy_limit)
        write_output(path)

    # end_time = time.time()  # End time measurement
    # runtime = end_time - start_time  # Calculate runtime
    # print(f"Runtime of {algorithm}: {runtime:.4f} seconds")

if __name__ == "__main__":
    main()
