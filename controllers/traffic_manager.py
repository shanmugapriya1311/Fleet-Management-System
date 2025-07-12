from typing import Dict, Tuple, List, Set, Optional
from collections import defaultdict
import time
import logging
import math
from src.models.robot import RobotStatus

class TrafficManager:
    def __init__(self):
        self.vertex_locks = {}  # vertex_id: (robot_id, timestamp)
        self.edge_locks = {}    # (from_vertex, to_vertex): (robot_id, timestamp)
        self.waiting_queues = defaultdict(list)  # vertex_id: [robot_ids]
        self.lock_duration = 5.0  # seconds
        self.reservation_timeout = 2.0  # seconds for path reservation
        self.blocked_vertices = set()  # Track blocked vertices
        self.blocked_edges = set()     # Track blocked edges
        self.safety_radius = 2.5       # Increased safety radius around robots
        self.robot_positions = {}      # robot_id: (x, y, vertex_id, edge)
        self.vertex_safety_radius = 2.0  # Increased safety radius around vertices
        self.vertex_occupants = {}     # vertex_id: robot_id - Track which robot is at each vertex
        self.edge_waypoints = {}       # (from_vertex, to_vertex): List of (x, y) waypoints
        self.robot_edge_progress = {}  # robot_id: (current_edge, progress)
        self.backtrack_history = {}    # robot_id: List of (vertex, timestamp)
        self.nav_graph = None          # Reference to navigation graph
        self.vertex_approach_zones = defaultdict(set)  # vertex_id: set of approaching robot_ids
        self.fleet_manager = None      # Reference to fleet manager
        
    def set_nav_graph(self, nav_graph):
        """Set the navigation graph reference"""
        self.nav_graph = nav_graph

    def set_fleet_manager(self, fleet_manager):
        """Set the fleet manager reference"""
        self.fleet_manager = fleet_manager

    def reserve_edge(self, robot_id: int, from_vertex: int, to_vertex: int) -> Tuple[bool, Optional[int]]:
        """Reserve an edge for robot movement with collision checking"""
        current_time = time.time()
        edge = (from_vertex, to_vertex)
        reverse_edge = (to_vertex, from_vertex)
        
        # Check if edge is already locked
        if edge in self.edge_locks:
            lock_robot, lock_time = self.edge_locks[edge]
            if lock_robot != robot_id and current_time - lock_time < self.lock_duration:
                return False, lock_robot
        if reverse_edge in self.edge_locks:
            lock_robot, lock_time = self.edge_locks[reverse_edge]
            if lock_robot != robot_id and current_time - lock_time < self.lock_duration:
                return False, lock_robot
                
        # Lock the edge
        self.edge_locks[edge] = (robot_id, current_time)
        self.blocked_edges.add(edge)
        
        # Generate waypoints if not already cached
        if edge not in self.edge_waypoints:
            self.edge_waypoints[edge] = self._generate_waypoints(from_vertex, to_vertex)
            
        return True, None
        
    def _generate_waypoints(self, from_vertex: int, to_vertex: int, num_points: int = 5) -> List[Tuple[float, float]]:
        """Generate intermediate waypoints along an edge"""
        if not self.nav_graph:
            return []
            
        start_pos = self.nav_graph.get_vertex_position(from_vertex)
        end_pos = self.nav_graph.get_vertex_position(to_vertex)
        
        waypoints = []
        for i in range(num_points):
            t = (i + 1) / (num_points + 1)  # Exclude start and end vertices
            x = start_pos[0] + t * (end_pos[0] - start_pos[0])
            y = start_pos[1] + t * (end_pos[1] - start_pos[1])
            waypoints.append((x, y))
        return waypoints
        
    def update_robot_position(self, robot_id: int, x: float, y: float, current_vertex: int, 
                            current_edge: Optional[Tuple[int, int]] = None, progress: float = 0.0) -> Tuple[bool, Optional[int]]:
        """Update robot position with continuous tracking and edge progress"""
        # Check for collisions with other robots
        collision, blocking_robot = self.check_collision(robot_id, x, y)
        if collision:
            self._handle_collision(robot_id, blocking_robot)
            return False, blocking_robot
            
        # Update position and edge progress
        self.robot_positions[robot_id] = (x, y, current_vertex, current_edge)
        if current_edge is not None:
            self.robot_edge_progress[robot_id] = (current_edge, progress)
            
            # Update approach zones
            next_vertex = current_edge[1]
            if progress > 0.7:  # If robot is more than 70% along the edge
                self.vertex_approach_zones[next_vertex].add(robot_id)
            else:
                self.vertex_approach_zones[next_vertex].discard(robot_id)
            
        # Update vertex occupancy if at a vertex
        if current_edge is None:
            if current_vertex not in self.vertex_occupants:
                self.vertex_occupants[current_vertex] = robot_id
            # Clear approach zones when robot reaches vertex
            for vertex in self.vertex_approach_zones:
                self.vertex_approach_zones[vertex].discard(robot_id)
            
        return True, None
        
    def _handle_collision(self, robot_id: int, blocking_robot_id: int) -> None:
        """Handle collision by updating backtrack history"""
        current_time = time.time()
        if robot_id not in self.backtrack_history:
            self.backtrack_history[robot_id] = []
            
        # Get current vertex
        if robot_id in self.robot_positions:
            current_vertex = self.robot_positions[robot_id][2]
            self.backtrack_history[robot_id].append((current_vertex, current_time))
            
            # Keep only last 5 positions
            if len(self.backtrack_history[robot_id]) > 5:
                self.backtrack_history[robot_id].pop(0)
                
    def should_backtrack(self, robot_id: int) -> bool:
        """Determine if robot should backtrack based on collision history"""
        if robot_id not in self.backtrack_history:
            return False
            
        history = self.backtrack_history[robot_id]
        if len(history) < 3:
            return False
            
        # Check if robot has been stuck at same vertex
        current_time = time.time()
        recent_vertices = set(vertex for vertex, _ in history[-3:])
        if len(recent_vertices) == 1:
            last_collision_time = history[-1][1]
            if current_time - last_collision_time > 2.0:  # If stuck for more than 2 seconds
                return True
                
        return False
        
    def get_backtrack_vertex(self, robot_id: int) -> Optional[int]:
        """Get vertex to backtrack to"""
        if robot_id not in self.backtrack_history:
            return None
            
        history = self.backtrack_history[robot_id]
        if len(history) < 2:
            return None
            
        # Return second-to-last vertex
        return history[-2][0]
        
    def check_collision(self, robot_id: int, x: float, y: float) -> Tuple[bool, Optional[int]]:
        """Enhanced collision checking with improved shared path handling"""
        # Check collision with other robots
        for other_id, (other_x, other_y, other_vertex, other_edge) in self.robot_positions.items():
            if other_id != robot_id:
                # Skip collision check if other robot has completed its task
                if other_id in self.robot_positions and self.fleet_manager is not None:
                    other_robot = self.fleet_manager.robots[other_id]
                    if other_robot.status == RobotStatus.TASK_COMPLETE:
                        continue
                
                # First check if paths are actually shared
                robot_edge = None
                if robot_id in self.robot_edge_progress:
                    robot_edge = self.robot_edge_progress[robot_id][0]
                
                paths_shared = False
                if robot_edge and other_edge:
                    # Check if edges are the same or reversed
                    if (robot_edge == other_edge or 
                        robot_edge == (other_edge[1], other_edge[0])):
                        paths_shared = True
                        # Check progress of both robots
                        robot_progress = self.robot_edge_progress[robot_id][1]
                        other_progress = self.robot_edge_progress[other_id][1]
                        
                        # If other robot is almost done, don't consider it a collision
                        if other_progress > 0.9:
                            paths_shared = False
                        # If this robot just started and other robot is far ahead, let it proceed
                        elif robot_progress < 0.1 and other_progress > 0.5:
                            paths_shared = False
                        # If robot has been waiting (check waiting queue), be more lenient
                        elif (robot_id in self.waiting_queues.get(robot_edge[0], []) and 
                              time.time() - self.vertex_locks.get(robot_edge[0], (None, 0.0))[1] > 3.0):
                            paths_shared = False
                    
                    # Check if edges share a vertex and robots are approaching it
                    elif (robot_edge[1] == other_edge[1] and  # Same target vertex
                          self.robot_edge_progress[robot_id][1] > 0.7 and
                          self.robot_edge_progress[other_id][1] > 0.7):
                        paths_shared = True
                        # If robot has been waiting, be more lenient
                        if (robot_id in self.waiting_queues.get(robot_edge[0], []) and 
                              time.time() - self.vertex_locks.get(robot_edge[0], (None, 0.0))[1] > 3.0):
                            paths_shared = False
                
                # Only check physical collision if paths are shared
                if paths_shared:
                    distance = math.sqrt((x - other_x)**2 + (y - other_y)**2)
                    if distance < self.safety_radius * 2:
                        return True, other_id
                        
        return False, None

    def is_path_clear(self, robot_id: int, path: List[int]) -> Tuple[bool, Optional[int]]:
        """Enhanced path clearance check with shared path detection"""
        current_time = time.time()
        
        # Get all edges in the requested path
        requested_edges = set()
        for i in range(len(path) - 1):
            edge = (path[i], path[i + 1])
            reverse_edge = (path[i + 1], path[i])
            requested_edges.add(edge)
            requested_edges.add(reverse_edge)
            
        # Check for actual path conflicts with other robots
        for other_robot_id, (_, _, _, other_edge) in self.robot_positions.items():
            if other_robot_id != robot_id and other_edge:
                # Skip check if other robot has completed its task
                if other_robot_id in self.robot_positions and self.fleet_manager is not None:
                    other_robot = self.fleet_manager.robots[other_robot_id]
                    if other_robot.status == RobotStatus.TASK_COMPLETE:
                        continue
                
                # Only consider it blocked if the paths actually intersect
                if other_edge in requested_edges or (other_edge[1], other_edge[0]) in requested_edges:
                    # Check if the other robot is moving away from the shared segment
                    if other_robot_id in self.robot_edge_progress:
                        other_progress = self.robot_edge_progress[other_robot_id][1]
                        # If other robot is almost done with the shared segment, don't block
                        if other_progress > 0.8:
                            continue
                    return False, other_robot_id
                    
        # Check each vertex and edge in the path
        for i in range(len(path) - 1):
            current_vertex = path[i]
            next_vertex = path[i + 1]
            
            # Check if vertices are already occupied
            if current_vertex in self.vertex_locks:
                lock_robot, _ = self.vertex_locks[current_vertex]
                if lock_robot != robot_id:
                    # Skip check if locking robot has completed its task
                    if lock_robot in self.robot_positions and self.fleet_manager is not None:
                        locking_robot = self.fleet_manager.robots[lock_robot]
                        if locking_robot.status == RobotStatus.TASK_COMPLETE:
                            continue
                    
                    # Allow if the other robot is moving away and not sharing path
                    if lock_robot in self.robot_edge_progress:
                        other_edge, other_progress = self.robot_edge_progress[lock_robot]
                        if (other_edge and other_edge[0] == current_vertex and 
                            other_progress > 0.3 and 
                            other_edge not in requested_edges):
                            continue
                        return False, lock_robot
                    
            # Check if edge is locked by a robot that's actually using it
            edge = (current_vertex, next_vertex)
            reverse_edge = (next_vertex, current_vertex)
            if edge in self.edge_locks or reverse_edge in self.edge_locks:
                lock_robot = self.edge_locks.get(edge, (None, None))[0] or self.edge_locks.get(reverse_edge, (None, None))[0]
                if lock_robot != robot_id:
                    # Skip check if locking robot has completed its task
                    if lock_robot in self.robot_positions and self.fleet_manager is not None:
                        locking_robot = self.fleet_manager.robots[lock_robot]
                        if locking_robot.status == RobotStatus.TASK_COMPLETE:
                            continue
                            
                    # Only block if the locking robot is actually on this edge
                    if lock_robot in self.robot_positions:
                        _, _, _, other_edge = self.robot_positions[lock_robot]
                        if other_edge in [edge, reverse_edge]:
                            return False, lock_robot
                    
        return True, None

    def get_edge_points(self, v1: int, v2: int, num_points: int) -> List[Tuple[float, float]]:
        """Get points along an edge for collision checking"""
        if not self.nav_graph:
            return []
            
        pos1 = self.nav_graph.get_vertex_position(v1)
        pos2 = self.nav_graph.get_vertex_position(v2)
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = pos1[0] + t * (pos2[0] - pos1[0])
            y = pos1[1] + t * (pos2[1] - pos1[1])
            points.append((x, y))
        return points

    def get_vertex_position(self, vertex_id: int) -> Tuple[float, float]:
        """Get vertex position"""
        if self.nav_graph and vertex_id in self.nav_graph.vertices:
            return self.nav_graph.vertices[vertex_id][:2]
        return (0, 0)  # Default if vertex not found

    def request_path(self, robot_id: int, path: List[int]) -> Tuple[bool, Optional[int]]:
        """Enhanced path request with better shared path handling and deadlock prevention"""
        current_time = time.time()
        
        # First check if path is clear
        is_clear, blocking_robot = self.is_path_clear(robot_id, path)
        if not is_clear:
            # Before adding to waiting queue, check if the blocking robot is also waiting
            if blocking_robot in self.robot_positions:
                blocking_robot_edge = self.robot_positions[blocking_robot][3]
                if blocking_robot_edge:
                    blocking_robot_progress = self.robot_edge_progress.get(blocking_robot, (None, 0.0))[1]
                    # If blocking robot is making progress, wait
                    if blocking_robot_progress > 0.0 and blocking_robot_progress < 0.95:
                        current_vertex = path[0]
                        if robot_id not in self.waiting_queues[current_vertex]:
                            self.waiting_queues[current_vertex].append(robot_id)
                            logging.info(f"Robot {robot_id} waiting at vertex {current_vertex} (blocked by Robot {blocking_robot})")
                            self.blocked_vertices.add(current_vertex)
                        return False, blocking_robot
                    # If blocking robot is stuck or almost done, try to find alternative
                    else:
                        return True, None

        # Before reserving new path, release any existing reservations
        if robot_id in self.robot_positions:
            current_vertex = self.robot_positions[robot_id][2]
            self.release_path(robot_id, current_vertex)

        # Path is clear, make reservations
        for i in range(len(path) - 1):
            current_vertex = path[i]
            next_vertex = path[i + 1]
            
            # Lock current vertex and edge with timeout
            self.vertex_locks[current_vertex] = (robot_id, current_time)
            self.edge_locks[(current_vertex, next_vertex)] = (robot_id, current_time)
            
            # Add to blocked sets for visualization
            if current_vertex in self.blocked_vertices:
                self.blocked_vertices.remove(current_vertex)
            self.blocked_edges.add((current_vertex, next_vertex))
            
            logging.info(f"Robot {robot_id} reserved vertex {current_vertex} and edge {current_vertex}->{next_vertex}")

        return True, None

    def release_path(self, robot_id: int, vertex_id: int) -> Optional[List[int]]:
        """Enhanced path release with immediate and complete lock cleanup"""
        logging.info(f"Releasing path for Robot {robot_id} at vertex {vertex_id}")
        
        # Store waiting robots before clearing
        waiting_robots = set()
        
        # Check if robot has completed its task
        robot_completed = False
        if self.fleet_manager and robot_id in self.fleet_manager.robots:
            robot = self.fleet_manager.robots[robot_id]
            robot_completed = robot.status == RobotStatus.TASK_COMPLETE
        
        # If robot has completed its task, clear ALL its locks
        if robot_completed:
            # Clear all vertex locks held by this robot
            for v in list(self.vertex_locks.keys()):
                if self.vertex_locks[v][0] == robot_id:
                    del self.vertex_locks[v]
                    if v in self.blocked_vertices:
                        self.blocked_vertices.remove(v)
                    # Check for waiting robots
                    if v in self.waiting_queues and self.waiting_queues[v]:
                        waiting_robots.update(self.waiting_queues[v])
                        self.waiting_queues[v] = []
            
            # Clear all edge locks held by this robot
            for edge in list(self.edge_locks.keys()):
                if self.edge_locks[edge][0] == robot_id:
                    del self.edge_locks[edge]
                    if edge in self.blocked_edges:
                        self.blocked_edges.remove(edge)
                    # Check both vertices of the edge for waiting robots
                    for v in edge:
                        if v in self.waiting_queues and self.waiting_queues[v]:
                            waiting_robots.update(self.waiting_queues[v])
                            self.waiting_queues[v] = []
            
            # Clear from all vertex occupants
            for v in list(self.vertex_occupants.keys()):
                if self.vertex_occupants[v] == robot_id:
                    del self.vertex_occupants[v]
            
            # Clear from all waiting queues
            for v in list(self.waiting_queues.keys()):
                if robot_id in self.waiting_queues[v]:
                    self.waiting_queues[v].remove(robot_id)
            
            # Clear from approach zones
            for v in self.vertex_approach_zones:
                self.vertex_approach_zones[v].discard(robot_id)
            
            # Clear from positions and progress tracking
            if robot_id in self.robot_positions:
                del self.robot_positions[robot_id]
            if robot_id in self.robot_edge_progress:
                del self.robot_edge_progress[robot_id]
                
        else:
            # Regular path release for non-completed robots
            # Clear vertex occupancy
            if vertex_id in self.vertex_occupants and self.vertex_occupants[vertex_id] == robot_id:
                del self.vertex_occupants[vertex_id]
            
            # Remove vertex lock
            if vertex_id in self.vertex_locks and self.vertex_locks[vertex_id][0] == robot_id:
                del self.vertex_locks[vertex_id]
                if vertex_id in self.blocked_vertices:
                    self.blocked_vertices.remove(vertex_id)
                if vertex_id in self.waiting_queues and self.waiting_queues[vertex_id]:
                    waiting_robots.update(self.waiting_queues[vertex_id])
                    self.waiting_queues[vertex_id] = []

            # Remove edge locks for this vertex
            edges_to_remove = []
            for edge, (lock_robot, _) in self.edge_locks.items():
                if lock_robot == robot_id and (edge[0] == vertex_id or edge[1] == vertex_id):
                    edges_to_remove.append(edge)
                    if edge in self.blocked_edges:
                        self.blocked_edges.remove(edge)
                    # Check both vertices of the edge for waiting robots
                    for v in edge:
                        if v in self.waiting_queues and self.waiting_queues[v]:
                            waiting_robots.update(self.waiting_queues[v])
                            self.waiting_queues[v] = []
            
            for edge in edges_to_remove:
                del self.edge_locks[edge]

            # Update position and progress tracking for this vertex
            if robot_id in self.robot_positions and self.robot_positions[robot_id][2] == vertex_id:
                del self.robot_positions[robot_id]
            if robot_id in self.robot_edge_progress:
                current_edge = self.robot_edge_progress[robot_id][0]
                if current_edge[0] == vertex_id or current_edge[1] == vertex_id:
                    del self.robot_edge_progress[robot_id]

            # Clear from approach zones for this vertex
            self.vertex_approach_zones[vertex_id].discard(robot_id)
        
        if waiting_robots:
            return list(waiting_robots)
        return None

    def get_waiting_robots(self, vertex_id: int) -> List[int]:
        """Get list of robots waiting at a vertex"""
        return self.waiting_queues.get(vertex_id, [])
        
    def get_blocked_vertices(self) -> Set[int]:
        """Get set of currently blocked vertices"""
        return self.blocked_vertices
        
    def get_blocked_edges(self) -> Set[Tuple[int, int]]:
        """Get set of currently blocked edges"""
        return self.blocked_edges