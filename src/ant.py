#! /usr/bin/env python3

# ant.py
#==============================================================================
# Author: Carl Larsson
# Description: Ant colony optimization for finding optimal path through a maze
# Date: 24-01-2024

# This software is licensed under the MIT License
# Refer to the LICENSE file for details
#==============================================================================


#------------------------------------------------------------------------------
# Libraries
from antsys import AntWorld
from antsys import AntSystem
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import random
#------------------------------------------------------------------------------


#------------------------------------------------------------------------------
# Variables
#------------------------------------------------------------------------------


#==============================================================================
# Ant colony optimization
def ant_optimization():

#==============================================================================

#==============================================================================
# Maze generation
def generate_maze():
    maze_graph = nx.grid_2d_graph(20, 20)

    # First coodrinate is y, second is x (rows and columns)

    # All outer walls
    for x in range(0, 20):
        maze_graph.remove_node((0, x))
        maze_graph.remove_node((19, x))
    for y in range(1, 19):
        maze_graph.remove_node((y, 0))
    for y in range(1, 16):
        maze_graph.remove_node((y, 19))
    for y in range(17, 19):
        maze_graph.remove_node((y, 19))

    # Inner walls left horizontal
    for x in range(1, 7):
        maze_graph.remove_node((2, x))
    for x in range(1, 5):
        maze_graph.remove_node((4, x))
    for x in range(2, 5):
        maze_graph.remove_node((6, x))
    for x in range(2, 5):
        maze_graph.remove_node((8, x))
    for x in range(1, 3):
        maze_graph.remove_node((11, x))

    # Inner walls middle horizontal
    for x in range(8, 13):
        maze_graph.remove_node((4, x))
    for x in range(6, 13):
        maze_graph.remove_node((6, x))
    for x in range(4, 9):
        maze_graph.remove_node((10, x))
    for x in range(4, 8):
        maze_graph.remove_node((15, x))
    for x in range(6, 10):
        maze_graph.remove_node((17, x))
    for x in range(11, 15):
        maze_graph.remove_node((17, x))
    for x in range(7, 12):
        maze_graph.remove_node((12, x))
    for x in range(11, 14):
        maze_graph.remove_node((14, x))
    maze_graph.remove_node((8, 10))

    # Inner walls right horizontal
    for x in range(12, 18):
        maze_graph.remove_node((2, x))
    for x in range(16, 19):
        maze_graph.remove_node((4, x))
    for x in range(14, 18):
        maze_graph.remove_node((6, x))
    for x in range(13, 16):
        maze_graph.remove_node((8, x))
    for x in range(15, 18):
        maze_graph.remove_node((9, x))
    for x in range(13, 19):
        maze_graph.remove_node((11, x))
    for x in range(15, 18):
        maze_graph.remove_node((13, x))
    for x in range(15, 17):
        maze_graph.remove_node((14, x))
    for x in range(15, 19):
        maze_graph.remove_node((15, x))
    for x in range(16, 19):
        maze_graph.remove_node((17, x))

    # Inner walls left vertical
    maze_graph.remove_node((7, 4))
    for y in range(9, 11):
        maze_graph.remove_node((y, 2))
    for y in range(13, 19):
        maze_graph.remove_node((y, 2))
    for y in range(16, 18):
        maze_graph.remove_node((y, 4))
    maze_graph.remove_node((18, 6))
    for y in range(11, 14):
        maze_graph.remove_node((y, 4))
    for y in range(11, 14):
        maze_graph.remove_node((y, 5))
    for y in range(4, 6):
        maze_graph.remove_node((y, 6))
    for y in range(7, 9):
        maze_graph.remove_node((y, 6))

    # Inner walls middle vertical
    for y in range(2, 4):
        maze_graph.remove_node((y, 8))
    for y in range(1, 3):
        maze_graph.remove_node((y, 10))
    for y in range(7, 10):
        maze_graph.remove_node((y, 8))
    for y in range(8, 11):
        maze_graph.remove_node((y, 11))
    for y in range(13, 15):
        maze_graph.remove_node((y, 7))
    maze_graph.remove_node((16, 7))
    for y in range(13, 16):
        maze_graph.remove_node((y, 9))
    for y in range(15, 17):
        maze_graph.remove_node((y, 11))
    maze_graph.remove_node((3, 12))

    # Inner walls right vertical
    maze_graph.remove_node((15, 13))
    for y in range(12, 14):
        maze_graph.remove_node((y, 13))
    maze_graph.remove_node((10, 13))
    for y in range(4, 6):
        maze_graph.remove_node((y, 14))
    maze_graph.remove_node((5, 17))
    for y in range(7, 9):
        maze_graph.remove_node((y, 17))
    maze_graph.remove_node((1, 17))
    maze_graph.remove_node((5, 12))

    return maze_graph
#==============================================================================

#==============================================================================
# Find optimal path through maze using ant colony optimization
def find_optimal_path():
    # Create reference maze
    ref_maze = generate_maze()

    # Ant colony parameters
    num_ants = 5
    num_iterations = 10
    start_node = (9, 9)
    goal_node = (16, 19)
    maze_ant, optimal_path = ant_colony_optimization_dynamic(ref_maze, start_node, goal_node, num_ants, num_iterations)

    # Visualize the reference maze and the ant map, and optimal path
    pos = {(x, y): (y, -x) for x, y in ref_maze.nodes()}
    nx.draw(ref_maze, pos, with_labels=True, font_size=8, node_size=755, node_color='lightgray', font_color='black', node_shape="s", label="Reference Maze")
    nx.draw(maze_ant, pos, with_labels=True, font_size=8, node_size=755, node_color='lightblue', font_color='black', node_shape="s", label="Dynamic Ant Maze Graph")
    nx.draw_networkx_edges(maze_ant, pos, edgelist=optimal_path, edge_color='red', width=2, label="Optimal Path")
    plt.title('Reference Maze and Ant Map')
    plt.legend()
    plt.show()

    return optimal_path
#==============================================================================

#==============================================================================
# Main
def main():
    optimal_path = find_optimal_path()
#==============================================================================

#==============================================================================
if __name__ == "__main__":
    main()
#==============================================================================
