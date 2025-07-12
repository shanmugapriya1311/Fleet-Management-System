from typing import Dict, List, Optional
from src.models.robot import Robot, RobotStatus
from src.models.nav_graph import NavGraph
from .traffic_manager import TrafficManager
import logging
import time

class FleetManager:
    def __init__(self, nav_graph: NavGraph):
        self.nav_graph = nav_graph
        self.robots: Dict[int, Robot] = {}
        self.traffic_manager = TrafficManager()
        self.traffic_manager.set_nav_graph(nav_graph)  # Set nav_graph reference
        self.next_robot_id = 1
        self.active_tasks = set()  # Track active tasks

    def spawn_robot(self, vertex_id: int) -> Optional[Robot]:
        """Spawn a new robot with proper position initialization"""
        if vertex_id not in self.nav_graph.vertices:
            return None
            
        robot = Robot(self.next_robot_id, vertex_id)
        # Initialize robot position to vertex position
        x, y, _ = self.nav_graph.vertices[vertex_id]
        robot.current_x = x
        robot.current_y = y
        robot.fleet_manager = self  # Set fleet manager reference
        
        self.robots[self.next_robot_id] = robot
        self.next_robot_id += 1
        return robot

    def assign_task(self, robot_id: int, target_vertex: int) -> bool:
        """Assign task with collision checking"""
        if robot_id not in self.robots:
            logging.warning(f"Cannot assign task: Robot {robot_id} does not exist")
            return False
            
        robot = self.robots[robot_id]
        if robot.status not in [RobotStatus.IDLE, RobotStatus.TASK_COMPLETE, RobotStatus.BLOCKED]:
            logging.warning(f"Cannot assign task: Robot {robot_id} is {robot.status}")
            return False
            
        if target_vertex not in self.nav_graph.vertices:
            logging.warning(f"Cannot assign task: Target vertex {target_vertex} does not exist")
            return False
            
        path = self.nav_graph.get_shortest_path(robot.current_vertex, target_vertex)
        if not path:
            logging.warning(f"Cannot assign task: No path found from vertex {robot.current_vertex} to {target_vertex}")
            return False
            
        # Try to reserve path
        can_move, blocking_robot = self.traffic_manager.request_path(robot_id, path)
        if not can_move:
            robot.set_blocked(blocking_robot)
            logging.info(f"Robot {robot_id} blocked by Robot {blocking_robot} while attempting to start task")
            return False
            
        # Start the task
        start_pos = self.nav_graph.get_vertex_position(robot.current_vertex)
        robot.start_task(target_vertex, path, start_pos)
        logging.info(f"Robot {robot_id} assigned task: Move from vertex {robot.current_vertex} to {target_vertex}")
        return True

    def update_robots(self) -> None:
        """Update robots with aggressive unblocking mechanism"""
        current_time = time.time()
        delta_time = min(0.1, current_time - getattr(self, 'last_update', current_time))
        self.last_update = current_time
        
        # First pass: Check for completed robots and force unblock their blocked robots
        for robot in self.robots.values():
            if robot.status == RobotStatus.TASK_COMPLETE:
                # Force unblock any robots that were blocked by this one
                for other_robot in self.robots.values():
                    if other_robot.blocked_by == robot.robot_id:
                        print(f"Force unblocking Robot {other_robot.robot_id} from completed Robot {robot.robot_id}")
                        # Reset all blocking flags
                        other_robot.status = RobotStatus.MOVING
                        other_robot.blocked_by = None
                        other_robot.waiting_start_time = None
                        other_robot.consecutive_blocks = 0
                        other_robot.current_edge = None
                        # Immediately try to reassign task
                        if other_robot.target_vertex is not None:
                            print(f"Immediately reassigning task for unblocked Robot {other_robot.robot_id}")
                            self.assign_task(other_robot.robot_id, other_robot.target_vertex)
        
        # Second pass: Update robot positions and handle blocking
        for robot in self.robots.values():
            if robot.status == RobotStatus.MOVING:
                # Update position
                old_vertex = robot.current_vertex
                robot.update_position(delta_time, self.nav_graph.get_vertex_position)
                
                # If robot moved to new vertex, update traffic management
                if robot.current_vertex != old_vertex:
                    print(f"Robot {robot.robot_id} moved from vertex {old_vertex} to {robot.current_vertex}")
                    # Release old vertex and try to unblock waiting robots
                    waiting_robots = self.traffic_manager.release_path(robot.robot_id, old_vertex)
                    
                    # Try to start movement for waiting robots
                    if waiting_robots:
                        print(f"Processing waiting robots at vertex {old_vertex}: {waiting_robots}")
                        for waiting_robot_id in waiting_robots:
                            if waiting_robot_id in self.robots:
                                waiting_robot = self.robots[waiting_robot_id]
                                if waiting_robot.status in [RobotStatus.BLOCKED, RobotStatus.WAITING]:
                                    if waiting_robot.target_vertex is not None:
                                        print(f"Attempting to reassign task for waiting Robot {waiting_robot_id}")
                                        if self.assign_task(waiting_robot_id, waiting_robot.target_vertex):
                                            print(f"Successfully unblocked Robot {waiting_robot_id}")
                                            waiting_robot.consecutive_blocks = 0
                                        else:
                                            print(f"Robot {waiting_robot_id} still blocked")
                            
            elif robot.status == RobotStatus.BLOCKED:
                # Check if blocking robot has completed its task
                if robot.blocked_by in self.robots:
                    blocking_robot = self.robots[robot.blocked_by]
                    if blocking_robot.status == RobotStatus.TASK_COMPLETE:
                        print(f"Unblocking Robot {robot.robot_id} - blocking Robot {robot.blocked_by} completed task")
                        # Reset all blocking flags
                        robot.status = RobotStatus.MOVING
                        robot.blocked_by = None
                        robot.waiting_start_time = None
                        robot.consecutive_blocks = 0
                        robot.current_edge = None
                        # Immediately try to reassign task
                        if robot.target_vertex is not None:
                            print(f"Reassigning task for unblocked Robot {robot.robot_id}")
                            if self.assign_task(robot.robot_id, robot.target_vertex):
                                print(f"Successfully reassigned task for Robot {robot.robot_id}")
                                continue
                
                # Periodically try to reassign blocked robots
                if hasattr(robot, 'last_retry_time') and current_time - robot.last_retry_time < 1.0:
                    continue
                    
                robot.last_retry_time = current_time
                if robot.target_vertex is not None:
                    print(f"Attempting periodic reassignment for blocked Robot {robot.robot_id}")
                    if self.assign_task(robot.robot_id, robot.target_vertex):
                        print(f"Successfully reassigned blocked Robot {robot.robot_id}")
                        robot.consecutive_blocks = 0
                    else:
                        print(f"Failed to reassign blocked Robot {robot.robot_id}")
                        robot.consecutive_blocks += 1
                        
                        # If robot has been blocked too many times, try to find alternate path
                        if robot.consecutive_blocks >= 5:
                            print(f"Robot {robot.robot_id} blocked too many times, attempting alternate path")
                            self._attempt_alternate_path(robot)

    def cancel_task(self, robot_id: int) -> bool:
        """Cancel current task for robot"""
        if robot_id not in self.robots:
            logging.warning(f"Cannot cancel task: Robot {robot_id} does not exist")
            return False
        
        robot = self.robots[robot_id]
        if robot.status in [RobotStatus.MOVING, RobotStatus.WAITING, RobotStatus.BLOCKED]:
            # Release all path reservations
            if robot.current_vertex is not None:
                self.traffic_manager.release_path(robot_id, robot.current_vertex)
            robot.status = RobotStatus.IDLE
            robot.target_vertex = None
            robot.path = []
            robot.current_edge = None
            robot.waiting_start_time = None
            robot.consecutive_blocks = 0
            logging.info(f"Cancelled task for Robot {robot_id}")
            return True
            
        logging.warning(f"Cannot cancel task: Robot {robot_id} is {robot.status}")
        return False