import numpy as np
from typing import List, Tuple, Optional, Union
import heapq

class Node:
    def __init__(self, position: np.ndarray):
        self.position = position
        self.g_cost = float('inf')
        self.h_cost = 0.0
        self.f_cost = float('inf')
        self.parent = None

    def __lt__(self, other):
        return self.f_cost < other.f_cost

class AStarBasicPlanner:
    def __init__(self, 
                 start: np.ndarray,
                 goal: np.ndarray,
                 bounds: np.ndarray,
                 obstacles: List[Union[np.ndarray, Tuple, dict]]):
        
        self.start = start
        self.goal = goal
        self.bounds = bounds
        self.obstacles = obstacles
        self.nodes = []

    def check_collision(self, point: np.ndarray) -> bool:
        for obs in self.obstacles:
            if isinstance(obs, np.ndarray):
                x, y, z = point
                ox_min, oy_min, oz_min, ox_max, oy_max, oz_max = obs
                if (ox_min <= x <= ox_max and
                    oy_min <= y <= oy_max and
                    oz_min <= z <= oz_max):
                    return True
            else:
                if isinstance(obs, dict):
                    center = obs['position']
                    radius = obs['radius']
                else:
                    center, radius = obs
                if np.linalg.norm(point - np.array(center)) <= radius:
                    return True
        return False

    def get_neighbors(self, point: np.ndarray) -> List[np.ndarray]:
        step_size = 1.0
        neighbors = []
        
        # Basic 6-directional movement
        directions = [
            [step_size, 0, 0], [-step_size, 0, 0],
            [0, step_size, 0], [0, -step_size, 0],
            [0, 0, step_size], [0, 0, -step_size]
        ]
        
        for dx, dy, dz in directions:
            new_point = point + np.array([dx, dy, dz])
            
            # Check bounds
            if not all(self.bounds[i][0] <= new_point[i] <= self.bounds[i][1] for i in range(3)):
                continue
                
            # Check collision
            if self.check_collision(new_point):
                continue
                
            neighbors.append(new_point)
            
        return neighbors

    def heuristic(self, point: np.ndarray) -> float:
        return np.linalg.norm(point - self.goal)

    def plan_path(self) -> Optional[np.ndarray]:
        start_node = Node(self.start)
        start_node.g_cost = 0
        start_node.h_cost = self.heuristic(self.start)
        start_node.f_cost = start_node.g_cost + start_node.h_cost
        
        open_set = []
        heapq.heappush(open_set, start_node)
        closed_set = set()
        node_dict = {tuple(self.start): start_node}
        
        iteration = 0
        max_iterations = 20000
        
        while open_set and iteration < max_iterations:
            iteration += 1
            current = heapq.heappop(open_set)
            current_pos = tuple(current.position)
            
            if current_pos in closed_set:
                continue
                
            closed_set.add(current_pos)
            self.nodes.append(current)
            
            # Check if goal is reached
            if np.linalg.norm(current.position - self.goal) < 1.0:
                path = []
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return np.array(path[::-1])
            
            # Explore neighbors
            for neighbor_pos in self.get_neighbors(current.position):
                neighbor_tuple = tuple(neighbor_pos)
                
                if neighbor_tuple in closed_set:
                    continue
                    
                new_g_cost = current.g_cost + np.linalg.norm(neighbor_pos - current.position)
                
                if neighbor_tuple not in node_dict:
                    neighbor = Node(neighbor_pos)
                    node_dict[neighbor_tuple] = neighbor
                else:
                    neighbor = node_dict[neighbor_tuple]
                    
                if new_g_cost >= neighbor.g_cost:
                    continue
                    
                neighbor.parent = current
                neighbor.g_cost = new_g_cost
                neighbor.h_cost = self.heuristic(neighbor_pos)
                neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                
                heapq.heappush(open_set, neighbor)
        
        return None