from enum import Enum, auto
from typing import List, Optional, Tuple
import time
import math
import colorsys
import random

class RobotStatus(Enum):
    IDLE = "Idle"
    MOVING = "Moving"
    WAITING = "Waiting"
    TASK_COMPLETE = "Task Complete"
    BLOCKED = "Blocked"

    def __str__(self):
        return self.value

class Robot:
    def __init__(self, robot_id: int, spawn_vertex: int):
        self.robot_id = robot_id
        self.current_vertex = spawn_vertex
        self.target_vertex = None
        self.status = RobotStatus.IDLE
        self.path = []
        self.current_edge_index = 0
        
        # Position and movement
        self.current_x = 0
        self.current_y = 0
        self.next_x = 0
        self.next_y = 0
        self.move_progress = 0.0
        self.move_speed = 0.5 # Further reduced speed for smoother movement
        
        # Visual properties
        hue = random.random()
        self.color = tuple(int(x * 255) for x in colorsys.hsv_to_rgb(hue, 0.8, 0.9))
        self.size = 15
        
        # Status tracking
        self.waiting_time = 0
        self.task_start_time = None
        self.blocked_by = None
        self.last_vertex = None
        self.last_pos = None
        self.current_edge = None
        self.backtracking = False
        self.backtrack_target = None
        self.consecutive_blocks = 0
        
    def start_task(self, target: int, path: List[int], start_pos: Tuple[float, float]) -> None:
        """Initialize a new task with position setup"""
        self.target_vertex = target
        self.path = path
        self.current_edge_index = 0
        self.status = RobotStatus.MOVING
        self.current_x, self.current_y = start_pos
        self.move_progress = 0.0
        
        # Initialize next position if path exists
        if len(path) > 1:
            self.next_x, self.next_y = start_pos  # Will be updated in first position update
        
    def set_blocked(self, blocking_robot_id: Optional[int] = None) -> None:
        """Set robot to blocked state"""
        self.status = RobotStatus.BLOCKED
        self.blocked_by = blocking_robot_id
        print(f"Robot {self.robot_id} blocked by Robot {blocking_robot_id}")
        
    def set_waiting(self) -> None:
        """Set robot to waiting state"""
        self.status = RobotStatus.WAITING
        print(f"Robot {self.robot_id} waiting")
        
    def update_position(self, delta_time: float, get_vertex_pos) -> None:
        """Enhanced position updates with improved simultaneous movement handling"""
        if self.status != RobotStatus.MOVING or not self.path:
            return

        # Handle backtracking if needed
        if hasattr(self, 'fleet_manager') and self.fleet_manager.traffic_manager.should_backtrack(self.robot_id):
            if not self.backtracking:
                self.backtrack_target = self.fleet_manager.traffic_manager.get_backtrack_vertex(self.robot_id)
                if self.backtrack_target is not None:
                    self.backtracking = True
                    self.move_progress = 0.0
                    print(f"Robot {self.robot_id} initiating backtrack to vertex {self.backtrack_target}")

        # Check if we've reached the end of the path
        if self.current_edge_index >= len(self.path) - 1:
            if self.backtracking:
                self.backtracking = False
                self.backtrack_target = None
                self.status = RobotStatus.MOVING
                return

            self._complete_task(get_vertex_pos)
            return

        # Get current and next vertex positions
        current_vertex = self.path[self.current_edge_index]
        next_vertex = self.path[self.current_edge_index + 1]
        
        # Try to reserve the edge if not already reserved
        if hasattr(self, 'fleet_manager') and self.current_edge != (current_vertex, next_vertex):
            can_reserve, blocking_robot = self.fleet_manager.traffic_manager.reserve_edge(
                self.robot_id, current_vertex, next_vertex
            )
            if not can_reserve:
                self._handle_movement_blocked(blocking_robot)
                return
            self.current_edge = (current_vertex, next_vertex)
            self.waiting_start_time = None

        # Update movement with dynamic speed adjustment
        self._update_movement(delta_time, get_vertex_pos(current_vertex), get_vertex_pos(next_vertex))

    def _update_movement(self, delta_time: float, current_pos: Tuple[float, float], next_pos: Tuple[float, float]) -> None:
        """Handle movement updates with shared path awareness"""
        # Update progress with dynamic speed
        old_progress = self.move_progress
        base_speed = self.move_speed
        
        # Adjust speed based on situation
        if hasattr(self, 'fleet_manager'):
            # Slow down if approaching intersection or on shared path
            if self.move_progress > 0.7:
                base_speed *= 0.7
            elif self._is_on_shared_path():
                base_speed *= 0.8  # Slower on shared paths
            # Speed up if other robots are waiting and path is not shared
            elif self.current_vertex in self.fleet_manager.traffic_manager.waiting_queues and not self._is_on_shared_path():
                base_speed *= 1.3
        
        self.move_progress += base_speed * delta_time * self._smooth_acceleration(self.move_progress)
        
        # Calculate new position with cubic easing
        t = self._cubic_ease(self.move_progress)
        new_x = self._lerp(current_pos[0], next_pos[0], t)
        new_y = self._lerp(current_pos[1], next_pos[1], t)

        # Update position with collision checking
        if hasattr(self, 'fleet_manager'):
            can_move, blocking_robot = self.fleet_manager.traffic_manager.update_robot_position(
                self.robot_id, new_x, new_y, self.current_vertex,
                current_edge=self.current_edge,
                progress=self.move_progress
            )
            if not can_move:
                self._handle_movement_blocked(blocking_robot)
                self.move_progress = max(old_progress - 0.1, 0.0)
                return

        # Update position if movement is significant
        if abs(new_x - self.current_x) > 0.001 or abs(new_y - self.current_y) > 0.001:
            self.current_x = new_x
            self.current_y = new_y
            self.last_pos = (self.current_x, self.current_y)

        # Handle edge completion
        if self.move_progress >= 1.0:
            self._complete_edge()

    def _is_on_shared_path(self) -> bool:
        """Enhanced check for shared path with actual path conflict detection"""
        if not hasattr(self, 'fleet_manager') or not self.current_edge:
            return False
            
        traffic_manager = self.fleet_manager.traffic_manager
        current_edge = self.current_edge
        reverse_edge = (self.current_edge[1], self.current_edge[0])
        
        # Check other robots' positions and paths
        for other_id, (_, _, _, other_edge) in traffic_manager.robot_positions.items():
            if other_id != self.robot_id and other_edge:
                # Check if edges are the same or reversed
                if other_edge == current_edge or other_edge == reverse_edge:
                    return True
                    
                # Check if edges share target vertex and both robots are approaching
                if (other_edge[1] == current_edge[1] and  # Same target vertex
                    self.robot_id in traffic_manager.robot_edge_progress and
                    other_id in traffic_manager.robot_edge_progress):
                    robot_progress = traffic_manager.robot_edge_progress[self.robot_id][1]
                    other_progress = traffic_manager.robot_edge_progress[other_id][1]
                    if robot_progress > 0.7 and other_progress > 0.7:
                        return True
                        
        return False

    def _handle_movement_blocked(self, blocking_robot_id: Optional[int]) -> None:
        """Handle movement being blocked with improved waiting time management"""
        # Only wait if there's an actual path conflict
        if blocking_robot_id is not None and hasattr(self, 'fleet_manager'):
            traffic_manager = self.fleet_manager.traffic_manager
            
            # First check if blocking robot has completed its task
            if blocking_robot_id in self.fleet_manager.robots:
                blocking_robot = self.fleet_manager.robots[blocking_robot_id]
                if blocking_robot.status == RobotStatus.TASK_COMPLETE:
                    print(f"Robot {self.robot_id} proceeding as Robot {blocking_robot_id} has completed its task")
                    self.status = RobotStatus.MOVING
                    self.blocked_by = None
                    self.waiting_start_time = None
                    # Force release of blocking robot's locks
                    if blocking_robot.current_vertex is not None:
                        traffic_manager.release_path(blocking_robot_id, blocking_robot.current_vertex)
                    return
            
            # Get current edges for both robots
            current_edge = None
            blocking_edge = None
            
            if self.robot_id in traffic_manager.robot_edge_progress:
                current_edge = traffic_manager.robot_edge_progress[self.robot_id][0]
            if blocking_robot_id in traffic_manager.robot_edge_progress:
                blocking_edge = traffic_manager.robot_edge_progress[blocking_robot_id][0]
            
            # Check if paths actually conflict
            if current_edge and blocking_edge:
                # Get progress of both robots
                robot_progress = traffic_manager.robot_edge_progress[self.robot_id][1]
                blocking_progress = traffic_manager.robot_edge_progress[blocking_robot_id][1]
                
                # Check if blocking robot is almost done or stuck
                if blocking_progress > 0.9:
                    print(f"Robot {self.robot_id} continuing as blocking Robot {blocking_robot_id} is almost done")
                    return
                elif blocking_progress < 0.1 and robot_progress > 0.5:
                    print(f"Robot {self.robot_id} continuing as it has made more progress than blocking Robot {blocking_robot_id}")
                    return
                
                # Check waiting time - only wait for 2 seconds maximum
                current_time = time.time()
                if hasattr(self, 'waiting_start_time') and self.waiting_start_time:
                    waiting_time = current_time - self.waiting_start_time
                    if waiting_time > 2.0:  # Reduced from 3.0 to 2.0 seconds
                        print(f"Robot {self.robot_id} waited for {waiting_time:.1f}s, attempting to proceed")
                        # Try to find alternate path first
                        self._attempt_alternate_path()
                        if self.status == RobotStatus.WAITING:  # If no alternate path found
                            # Reset waiting time and try to proceed on current path
                            self.waiting_start_time = None
                            self.status = RobotStatus.MOVING
                            self.blocked_by = None
                        return
        
        # If we get here, we need to wait
        self.status = RobotStatus.WAITING
        self.blocked_by = blocking_robot_id
        self.consecutive_blocks += 1
        
        # Start waiting timer if not already started
        if not hasattr(self, 'waiting_start_time') or self.waiting_start_time is None:
            self.waiting_start_time = time.time()
            
        # If blocked too many times, try alternate path sooner
        if self.consecutive_blocks >= 2:  # Reduced from 3 to 2
            self._attempt_alternate_path()

    def _complete_edge(self) -> None:
        """Handle edge completion"""
        if hasattr(self, 'fleet_manager'):
            # Release current edge and vertex
            self.fleet_manager.traffic_manager.release_path(self.robot_id, self.current_vertex)
            self.current_edge = None

        self.current_edge_index += 1
        self.move_progress = 0.0
        self.current_vertex = self.path[self.current_edge_index]
        self.last_pos = (self.current_x, self.current_y)
        
        # Resume movement
        self.status = RobotStatus.MOVING
        self.blocked_by = None
        self.consecutive_blocks = 0
        self.waiting_start_time = None

    def _complete_task(self, get_vertex_pos) -> None:
        """Handle task completion with immediate lock release"""
        if hasattr(self, 'fleet_manager'):
            traffic_manager = self.fleet_manager.traffic_manager
            
            # First update status to TASK_COMPLETE
            self.status = RobotStatus.TASK_COMPLETE
            
            # Release all vertex locks held by this robot
            for vertex_id in list(traffic_manager.vertex_locks.keys()):
                if traffic_manager.vertex_locks[vertex_id][0] == self.robot_id:
                    waiting_robots = traffic_manager.release_path(self.robot_id, vertex_id)
                    self._handle_waiting_robots(waiting_robots)
            
            # Release current vertex and target vertex specifically
            if self.current_vertex is not None:
                waiting_robots = traffic_manager.release_path(self.robot_id, self.current_vertex)
                self._handle_waiting_robots(waiting_robots)

            if self.target_vertex is not None:
                waiting_robots = traffic_manager.release_path(self.robot_id, self.target_vertex)
                self._handle_waiting_robots(waiting_robots)
            
            # Clear from all waiting queues
            for vertex_id in list(traffic_manager.waiting_queues.keys()):
                if self.robot_id in traffic_manager.waiting_queues[vertex_id]:
                    traffic_manager.waiting_queues[vertex_id].remove(self.robot_id)

            # Force unblock any robots that were blocked by this one
            for robot in self.fleet_manager.robots.values():
                if robot.blocked_by == self.robot_id:
                    print(f"Force unblocking Robot {robot.robot_id} after Robot {self.robot_id} completed task")
                    robot.status = RobotStatus.MOVING
                    robot.blocked_by = None
                    robot.waiting_start_time = None
                    robot.consecutive_blocks = 0
                    # Immediately try to reassign the task
                    if robot.target_vertex is not None:
                        self.fleet_manager.assign_task(robot.robot_id, robot.target_vertex)

        # Update position and status
        final_pos = get_vertex_pos(self.target_vertex)
        self.current_x = final_pos[0]
        self.current_y = final_pos[1]
        self.current_vertex = self.target_vertex
        self.last_vertex = self.current_vertex
        self.current_edge = None
        self.waiting_start_time = None
        self.consecutive_blocks = 0
        print(f"Robot {self.robot_id} completed task at vertex {self.target_vertex}")

    def _handle_waiting_robots(self, waiting_robots: Optional[List[int]]) -> None:
        """Handle waiting robots after releasing a path"""
        if not waiting_robots:
            return
            
        for waiting_robot_id in waiting_robots:
            if waiting_robot_id in self.fleet_manager.robots:
                waiting_robot = self.fleet_manager.robots[waiting_robot_id]
                if waiting_robot.status in [RobotStatus.BLOCKED, RobotStatus.WAITING] and waiting_robot.target_vertex is not None:
                    print(f"Attempting to unblock Robot {waiting_robot_id} after Robot {self.robot_id} completed task")
                    self.fleet_manager.assign_task(waiting_robot_id, waiting_robot.target_vertex)

    def _handle_blocking(self, blocking_robot_id: int) -> None:
        """Handle being blocked by another robot"""
        self.status = RobotStatus.WAITING
        self.blocked_by = blocking_robot_id
        self.consecutive_blocks += 1
        
        if self.consecutive_blocks >= 5:  # If blocked too many times
            self._attempt_alternate_path()

    def _attempt_alternate_path(self) -> None:
        """Enhanced alternate path finding with better path selection"""
        if hasattr(self, 'fleet_manager') and self.target_vertex is not None:
            traffic_manager = self.fleet_manager.traffic_manager
            
            # Get current occupied edges and vertices to avoid
            occupied_edges = set()
            occupied_vertices = set()
            
            for robot_id, (_, _, vertex, edge) in traffic_manager.robot_positions.items():
                if robot_id != self.robot_id:
                    if edge:
                        occupied_edges.add(edge)
                        occupied_edges.add((edge[1], edge[0]))  # Add reverse edge
                    occupied_vertices.add(vertex)
            
            # Try to find path avoiding occupied areas
            path = self.fleet_manager.nav_graph.get_shortest_path(
                self.current_vertex,
                self.target_vertex,
                occupied_vertices=occupied_vertices
            )
            
            if path and path != self.path:
                print(f"Robot {self.robot_id} found alternate path to vertex {self.target_vertex}")
                # Check if new path is significantly different
                shared_vertices = set(path) & set(self.path)
                if len(shared_vertices) < len(self.path) * 0.7:  # If less than 70% overlap
                    if self.fleet_manager.assign_task(self.robot_id, self.target_vertex):
                        self.consecutive_blocks = 0
                        self.waiting_start_time = None
                        print(f"Robot {self.robot_id} successfully switched to alternate path")
                        return
            
            # If no good alternate path found, try to backtrack
            if self.consecutive_blocks >= 5:
                self._initiate_backtrack()

    def _initiate_backtrack(self) -> None:
        """Initiate backtracking to find a better path"""
        if hasattr(self, 'fleet_manager'):
            traffic_manager = self.fleet_manager.traffic_manager
            
            # Find a previous vertex that's not blocked
            for i in range(max(0, self.current_edge_index - 1), -1, -1):
                backtrack_vertex = self.path[i]
                if backtrack_vertex not in traffic_manager.blocked_vertices:
                    print(f"Robot {self.robot_id} backtracking to vertex {backtrack_vertex}")
                    self.backtracking = True
                    self.backtrack_target = backtrack_vertex
                    self.move_progress = 0.0
                    self.consecutive_blocks = 0
                    self.waiting_start_time = None
                    return
                    
            print(f"Robot {self.robot_id} could not find suitable backtrack vertex")

    def _smooth_acceleration(self, progress: float) -> float:
        """Smooth acceleration function to prevent sudden movements"""
        # Gradually increase speed from 0.5 to 1.0
        return 0.5 + 0.5 * self._smooth_step(progress)

    def _cubic_ease(self, t: float) -> float:
        """Cubic easing function for smoother movement"""
        t = max(0.0, min(1.0, t))
        return t * t * (3 - 2 * t)

    def _smooth_step(self, x: float) -> float:
        """Smooth step function for interpolation"""
        x = max(0.0, min(1.0, x))
        return x * x * (3 - 2 * x)
    
    def _lerp(self, start: float, end: float, t: float) -> float:
        """Linear interpolation with clamping"""
        t = max(0.0, min(1.0, t))
        return start + (end - start) * t
    
    def get_position(self) -> Tuple[float, float]:
        """Get current interpolated position"""
        return (self.current_x, self.current_y)