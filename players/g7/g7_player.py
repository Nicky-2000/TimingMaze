import os
import pickle
import numpy as np
import logging
import random
from utils import get_divisors
from dataclasses import dataclass
import networkx as nx # pip install networkx
import matplotlib.pyplot as plt # pip install matplotlib
from math import lcm
from players.g7.player_helper_code import build_graph_from_memory, MazeGraph, PlayerMemory, findShortestPathsToEachNode, reconstruct_path


import constants
from timing_maze_state import TimingMazeState
from collections import deque

class Player:
    def __init__(self, rng: np.random.Generator, logger: logging.Logger,
                 precomp_dir: str, maximum_door_frequency: int, radius: int) -> None:
        """Initialise the player with the basic amoeba information

            Args:
                rng (np.random.Generator): numpy random number generator, use this for same player behavior across run
                logger (logging.Logger): logger use this like logger.info("message")
                maximum_door_frequency (int): the maximum frequency of doors
                radius (int): the radius of the drone
                precomp_dir (str): Directory path to store/load pre-computation
        """

        # precomp_path = os.path.join(precomp_dir, "{}.pkl".format(map_path))

        # # precompute check
        # if os.path.isfile(precomp_path):
        #     # Getting back the objects:
        #     with open(precomp_path, "rb") as f:
        #         self.obj0, self.obj1, self.obj2 = pickle.load(f)
        # else:
        #     # Compute objects to store
        #     self.obj0, self.obj1, self.obj2 = _

        #     # Dump the objects
        #     with open(precomp_path, 'wb') as f:
        #         pickle.dump([self.obj0, self.obj1, self.obj2], f)

        self.rng = rng
        self.logger = logger
        self.maximum_door_frequency = maximum_door_frequency
        self.radius = radius
        self.memory = PlayerMemory()
        self.turn = 0
        self.starting_position_set = False #check

    
    def move(self, current_percept) -> int:
        """Function which retrieves the current state of the amoeba map and returns an amoeba movement

            Args:
                current_percept(TimingMazeState): contains current state information
            Returns:
                int: This function returns the next move of the user:
                    WAIT = -1
                    LEFT = 0
                    UP = 1
                    RIGHT = 2
                    DOWN = 3
        """
        # if not self.starting_position_set: #setting starting position if not set
        #     self.memory.pos = (current_percept.player_x, current_percept.player_y)
        #     self.starting_position_set = True
        self.turn += 1
        move = constants.WAIT

        # Decide on the next move based on the current percept.
        self.memory.update_memory(current_percept.maze_state, self.turn)
        
        # Build the graph from the updated memory
        currentGraph = build_graph_from_memory(self.memory)
        
        # Determine the go
        # l node
        if current_percept.is_end_visible:
            # target_node = (current_percept.end_x, current_percept.end_y)
            target_node = (current_percept.end_y, current_percept.end_x)
            print("End is visible. Target node set to {target_node}")

        else:
            # If the end is not visible, choosing an intermediate node
            target_node = self.choose_intermediate_target_node(current_percept)
            #target_node = (-40, -40)
            print("End not visible. Intermediate target node set to {target_node}")

        # Find shortest paths to the target node
        minDistanceArray, parent = findShortestPathsToEachNode(currentGraph, self.memory.pos, self.turn)

        path = reconstruct_path(parent, self.memory.pos, target_node)

        if path and len(path) > 1:

            next_move = self.get_move_direction(path)
            print(path)
            print("Want to make next move: ", next_move)
            if self.memory.is_move_valid(next_move, current_percept.maze_state):
                self.memory.update_pos(next_move)
                print("New Pos: ", self.memory.pos)
                move = next_move
            else: 
                print("Desired Next Move is Invalid")
                move = constants.WAIT
        
            print(f"Move: {move}")
            return move
            # Get the next move from the path
            # need to ensure invalid turns don't happen

        else:
            # If no path found, explore
            print("No valid path found. Initiating exploration.")
            exploration_move = self.explore(current_percept)
            if exploration_move is not None:
                if self.memory.is_move_valid(exploration_move, current_percept.maze_state):
                    self.memory.update_pos(exploration_move)
                    print("Exploration move accepted. New position: {self.memory.pos}")
                    move = exploration_move
                else:
                    print("Exploration move is invalid. Waiting.")
            else:
                print("No exploration moves available. Waiting.")

        self.logger.info(f"Turn {self.turn}: Move selected - {move}")
        return move


    def choose_intermediate_target_node(self, current_percept): #when goal node is not visible

        unexplored_nodes = self.get_unexplored_nodes(current_percept)
        if unexplored_nodes:
            return random.choice(unexplored_nodes)
        else:
            # Fallback to current position or some default strategy
            return self.memory.pos

    def get_move_direction(self, path): #current to next position
        """        
        Args:
            current_pos (tuple): Current position (x, y).
            next_pos (tuple): Next position (x, y).
        
        Returns:
            int: Direction of movement (LEFT, UP, RIGHT, DOWN).
        """

        # this is the DY, DX from the Min distance array
        dy = path[1][0] - path[0][0]
        dx = path[1][1] - path[0][1]
        # Convert this to the direction

        # THIS IS HOW IT SHOULD BE BUT OUR SEARCH IS FLIPPED FOR SOME REASON
        if dx == -1 and dy == 0:
            return constants.LEFT
        elif dx == 1 and dy == 0:
            return constants.RIGHT
        elif dx == 0 and dy == -1:
            return constants.UP
        elif dx == 0 and dy == 1:
            return constants.DOWN
        else:
            return constants.WAIT
        # if dx == -1 and dy == 0:
        #     return constants.DOWN
        # elif dx == 1 and dy == 0:
        #     return constants.UP
        # elif dx == 0 and dy == -1:
        #     return constants.LEFT
        # elif dx == 0 and dy == 1:
        #     return constants.RIGHT
        # else:
        #     return constants.WAIT

    #Placeholder
    #def get_unexplored_nodes(self, current_percept):  
    #    unexplored_nodes = []
    #    return unexplored_nodes

    def get_unexplored_nodes(self): #based on current memory.

        unexplored = []
        for y in range(len(self.memory.memory)):
            for x in range(len(self.memory.memory[0])):
                node = (y, x)
                if node not in self.memory.visited and node in self.memory.map:
                    unexplored.append(node)
        print("Unexplored nodes found: {unexplored}")
        return unexplored
    
    def explore(self, current_percept) -> int:
        """Exploration strategy when the target is not visible or reachable.

        Args:
            current_percept (TimingMazeState): Current state information.

        Returns:
            int: Direction to move (LEFT, UP, RIGHT, DOWN) or None if no move found.
        """
        directions = [constants.LEFT, constants.UP, constants.RIGHT, constants.DOWN]
        direction_offsets = {
            constants.LEFT: (0, -1),
            constants.UP: (-1, 0),
            constants.RIGHT: (0, 1),
            constants.DOWN: (1, 0),
        }

        start_pos = self.memory.pos
        exploration_queue = deque([start_pos])
        local_visited = set([start_pos])

        while exploration_queue:
            current_pos = exploration_queue.popleft()

            for direction in directions:
                dy, dx = direction_offsets[direction]
                next_pos = (current_pos[0] + dy, current_pos[1] + dx)

                if next_pos in local_visited or next_pos in self.memory.visited:
                    continue

                # Check if the move to the next position is valid based on current door state
                if self.memory.is_move_valid(direction, current_percept.maze_state):
                    # Mark the node as visited and move towards it
                    self.memory.visited.add(next_pos)
                    self.memory.update_pos(direction)
                    print("Exploration move selected: {direction}, New position: {next_pos}")
                    return direction

                # If move is not valid, but position is known, add to queue for further exploration
                if next_pos in self.memory.map and next_pos not in self.memory.visited:
                    exploration_queue.append(next_pos)
                    local_visited.add(next_pos)

        # If no exploration moves are possible
        print("No valid exploration moves found.")
        return None

    
    #def move(self, current_percept) -> int:
        """Function which retrieves the current state of the amoeba map and returns an amoeba movement

            Args:
                current_percept(TimingMazeState): contains current state information
            Returns:
                int: This function returns the next move of the user:
                    WAIT = -1
                    LEFT = 0
                    UP = 1
                    RIGHT = 2
                    DOWN = 3
        """

        #self.memory.update_memory(current_percept.maze_state, self.turn)
        # currentGraph = build_graph_from_memory(self.memory)

        # we want to build graph with PlayerMemory (self.memory)
        #currentGraph = build_graph_from_memory(self.memory)
        #minDistanceArray, parent = findShortestPathsToEachNode(currentGraph, (100, 100), turnNumber=self.turn)

        ## Look at minDistance Array and see where we can get too.. And decide where to go.
        #currentGraph.reconstruct_path(parent, (100, 100), ##"Node we want to go to"##)


        
        # currentGraph.visualize_graph_in_grid()
        #print("yay")
        # if self.turn % 10 == 0 and self.turn != 0:
            

        #self.turn += 1
        #return constants.WAIT

        # direction = [0, 0, 0, 0]
        # for maze_state in current_percept.maze_state:
        #     if maze_state[0] == 0 and maze_state[1] == 0:
        #         direction[maze_state[2]] = maze_state[3]

        # if current_percept.is_end_visible:
        #     if abs(current_percept.end_x) >= abs(current_percept.end_y):
        #         if current_percept.end_x > 0 and direction[constants.RIGHT] == constants.OPEN:
        #             for maze_state in current_percept.maze_state:
        #                 if (maze_state[0] == 1 and maze_state[1] == 0 and maze_state[2] == constants.LEFT
        #                         and maze_state[3] == constants.OPEN):
        #                     return constants.RIGHT
        #         if current_percept.end_x < 0 and direction[constants.LEFT] == constants.OPEN:
        #             for maze_state in current_percept.maze_state:
        #                 if (maze_state[0] == -1 and maze_state[1] == 0 and maze_state[2] == constants.RIGHT
        #                         and maze_state[3] == constants.OPEN):
        #                     return constants.LEFT
        #         if current_percept.end_y < 0 and direction[constants.UP] == constants.OPEN:
        #             for maze_state in current_percept.maze_state:
        #                 if (maze_state[0] == 0 and maze_state[1] == -1 and maze_state[2] == constants.DOWN
        #                         and maze_state[3] == constants.OPEN):
        #                     return constants.UP
        #         if current_percept.end_y > 0 and direction[constants.DOWN] == constants.OPEN:
        #             for maze_state in current_percept.maze_state:
        #                 if (maze_state[0] == 0 and maze_state[1] == 1 and maze_state[2] == constants.UP
        #                         and maze_state[3] == constants.OPEN):
        #                     return constants.DOWN
        #         return constants.WAIT
        #     else:
        #         if current_percept.end_y < 0 and direction[constants.UP] == constants.OPEN:
        #             for maze_state in current_percept.maze_state:
        #                 if (maze_state[0] == 0 and maze_state[1] == -1 and maze_state[2] == constants.DOWN
        #                         and maze_state[3] == constants.OPEN):
        #                     return constants.UP
        #         if current_percept.end_y > 0 and direction[constants.DOWN] == constants.OPEN:
        #             for maze_state in current_percept.maze_state:
        #                 if (maze_state[0] == 0 and maze_state[1] == 1 and maze_state[2] == constants.UP
        #                         and maze_state[3] == constants.OPEN):
        #                     return constants.DOWN
        #         if current_percept.end_x > 0 and direction[constants.RIGHT] == constants.OPEN:
        #             for maze_state in current_percept.maze_state:
        #                 if (maze_state[0] == 1 and maze_state[1] == 0 and maze_state[2] == constants.LEFT
        #                         and maze_state[3] == constants.OPEN):
        #                     return constants.RIGHT
        #         if current_percept.end_x < 0 and direction[constants.LEFT] == constants.OPEN:
        #             for maze_state in current_percept.maze_state:
        #                 if (maze_state[0] == -1 and maze_state[1] == 0 and maze_state[2] == constants.RIGHT
        #                         and maze_state[3] == constants.OPEN):
        #                     return constants.LEFT
        #         return constants.WAIT
        # else:
        #     if direction[constants.LEFT] == constants.OPEN:
        #         for maze_state in current_percept.maze_state:
        #             if (maze_state[0] == -1 and maze_state[1] == 0 and maze_state[2] == constants.RIGHT
        #                     and maze_state[3] == constants.OPEN):
        #                 return constants.LEFT
        #     if direction[constants.DOWN] == constants.OPEN:
        #         for maze_state in current_percept.maze_state:
        #             if (maze_state[0] == 0 and maze_state[1] == 1 and maze_state[2] == constants.UP
        #                     and maze_state[3] == constants.OPEN):
        #                 return constants.DOWN
        #     if direction[constants.RIGHT] == constants.OPEN:
        #         for maze_state in current_percept.maze_state:
        #             if (maze_state[0] == 1 and maze_state[1] == 0 and maze_state[2] == constants.LEFT
        #                     and maze_state[3] == constants.OPEN):
        #                 return constants.RIGHT
        #     if direction[constants.UP] == constants.OPEN:
        #         for maze_state in current_percept.maze_state:
        #             if (maze_state[0] == 0 and maze_state[1] == -1 and maze_state[2] == constants.DOWN
        #                     and maze_state[3] == constants.OPEN):
        #                 return constants.UP
            # return constants.WAIT

def print_min_dist_array(minDistanceArray, start_row, end_row, start_col, end_col, width=4):
    for y in range(len(minDistanceArray)):
        if y >= start_row and y <= end_row:
            row = minDistanceArray[y]
            for x in range(len(row)):
                if x >= start_col and x <= end_col:
                    # Print each element with a fixed width
                    print(f"{row[x]:>{width}}", end=" ")
            print()