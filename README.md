# Mars Rover Path Planning

## Project Description
This project focuses on developing a path planning algorithm for a Mars rover tasked with collecting soil samples. The objective is to guide the rover from its current location to a new location using the shortest safe path possible. The terrain of Mars is represented as a graph with 3D coordinates for each location and path segments connecting them. The rover must consider terrain elevation changes and has limited energy for uphill moves, which can be offset by downhill momentum.

Three search algorithms are implemented:
- **BFS (Breadth-First Search)**: Treats each move as a unit step without considering elevation.
- **UCS (Uniform-Cost Search)**: Uses 2D Euclidean distance for step costs, ignoring elevation.
- **A\* Search**: Incorporates 3D Euclidean distance, considering elevation differences.


