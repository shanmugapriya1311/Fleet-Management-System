import pygame
import pygame.freetype
from typing import Dict, Tuple, Optional, Set
import logging
from src.models.nav_graph import NavGraph
from src.models.robot import Robot, RobotStatus
import time
import math

class FleetGUI:
    def __init__(self, nav_graph: NavGraph, fleet_manager):
        pygame.init()
        self.nav_graph = nav_graph
        self.fleet_manager = fleet_manager
        self.robots = fleet_manager.robots
        self.screen = pygame.display.set_mode((1200, 800))
        pygame.display.set_caption("Fleet Management System")
        self.clock = pygame.time.Clock()
        
        # Setup fonts
        pygame.font.init()
        self.font = pygame.font.SysFont('Arial', 16)
        self.alert_font = pygame.font.SysFont('Arial', 20)
        self.small_font = pygame.font.SysFont('Arial', 12)
        self.legend_font = pygame.font.SysFont('Arial', 14)
        
        # Initialize state variables
        self.spawn_mode = False
        self.task_mode = False
        self.selected_robot = None
        self.hover_vertex = None
        self.alerts = []
        self.zoom_robot = None
        self.pulse_alpha = 128  # For pulsing effects
        self.pulse_increasing = True
        self.blink_state = True  # For blinking effects
        self.last_blink_time = time.time()
        
        # Colors with high contrast scheme
        self.colors = {
            'background': (42, 42, 42),         # Dark grey
            'vertex': (0, 180, 255),            # Neon blue
            'vertex_hover': (255, 255, 0),      # Yellow glow
            'vertex_selected': (255, 255, 0),   # Yellow
            'edge': (204, 204, 204),           # Light grey
            'edge_blocked': (255, 0, 0),       # Red
            'edge_reserved': (255, 165, 0),    # Orange
            'vertex_blocked': (255, 0, 0),     # Red
            'vertex_reserved': (255, 165, 0),   # Orange
            'vertex_approach': (255, 255, 0),   # Yellow
            'text': (255, 255, 255),           # White
            'panel_bg': (30, 30, 30),          # Darker grey
            'status_panel': (35, 35, 35),      # Slightly lighter grey
            'robot_panel': (35, 35, 35),       # Same as status panel
            'button_normal': (60, 60, 60),     # Medium grey
            'button_hover': (80, 80, 80),      # Lighter grey
            'button_active': (100, 100, 100),  # Even lighter grey
            'alert': (255, 0, 0),             # Red
            'warning': (255, 255, 0),         # Yellow
            'critical': (255, 0, 0),          # Red
            'success': (0, 255, 0),           # Green
            'charger': (0, 255, 0),           # Neon green
            'charger_border': (0, 200, 0),    # Darker green
            'panel_header': (0, 180, 255),    # Neon blue
            'waiting': (255, 165, 0),         # Orange for waiting status
        }
        
        # Robot specific colors
        self.robot_colors = {
            1: (0, 255, 255),    # Electric Blue
            2: (0, 255, 0),      # Neon Green
            3: (255, 165, 0),    # Bright Orange
            4: (255, 0, 255),    # Magenta
            5: (255, 255, 0),    # Yellow
        }
        
        # Status effects
        self.status_effects = {
            RobotStatus.MOVING: {
                'glow': True,
                'trail': True,
                'color': self.colors['success']
            },
            RobotStatus.WAITING: {
                'blink': True,
                'color': self.colors['warning']
            },
            RobotStatus.BLOCKED: {
                'flash': True,
                'color': self.colors['critical']
            },
            RobotStatus.TASK_COMPLETE: {
                'checkmark': True,
                'color': self.colors['success']
            }
        }
        
        # Trail effect data
        self.robot_trails = {}  # robot_id: list of (pos, alpha) tuples
        
        # Button setup
        self.buttons = {
            'spawn': {
                'rect': pygame.Rect(925, 20, 250, 40),
                'text': 'Spawn Robot',
                'active': False
            },
            'assign': {
                'rect': pygame.Rect(925, 80, 250, 40),
                'text': 'Assign Task',
                'active': False
            },
            'cancel': {
                'rect': pygame.Rect(925, 140, 250, 40),
                'text': 'Cancel',
                'active': False
            }
        }
        
        # GUI Layout
        self.main_area = pygame.Rect(0, 0, 900, 800)
        self.side_panel = pygame.Rect(900, 0, 300, 800)
        self.traffic_panel = pygame.Rect(925, 200, 250, 250)
        self.robot_panel = pygame.Rect(925, 470, 250, 250)
        self.legend_panel = pygame.Rect(10, 700, 200, 90)  # New legend panel
        
        # Critical alerts banner
        self.critical_alerts = []
        self.critical_banner_height = 30
        
        # Setup logging
        logging.basicConfig(
            filename='logs/fleet_logs.txt',
            level=logging.INFO,
            format='%(asctime)s - %(message)s'
        )
        
        # Add new visual elements
        self.status_messages = []
        self.selected_robot = None
        self.hover_vertex = None
        
        # Status colors
        self.status_colors = {
            RobotStatus.IDLE: (100, 100, 100),     # Gray
            RobotStatus.MOVING: (0, 255, 0),       # Green
            RobotStatus.WAITING: (255, 165, 0),    # Orange
            RobotStatus.BLOCKED: (255, 0, 0),      # Red
            RobotStatus.TASK_COMPLETE: (128, 0, 128) # Purple
        }

    def _scale_position(self, x: float, y: float) -> Tuple[int, int]:
        """Convert graph coords to screen coords"""
        margin = 50
        scale = min(
            (self.screen.get_width() - 2 * margin) / (self.nav_graph.max_x - self.nav_graph.min_x),
            (self.screen.get_height() - 2 * margin) / (self.nav_graph.max_y - self.nav_graph.min_y)
        )
        
        screen_x = int((x - self.nav_graph.min_x) * scale + margin)
        screen_y = int((y - self.nav_graph.min_y) * scale + margin)
        return (screen_x, screen_y)

    def _get_vertex_at_pos(self, mouse_pos: Tuple[int, int]) -> Optional[int]:
        """Return vertex id if mouse is over a vertex, None otherwise"""
        for vertex_id, (x, y, _) in self.nav_graph.vertices.items():
            pos = self._scale_position(x, y)
            if ((mouse_pos[0] - pos[0])**2 + 
                (mouse_pos[1] - pos[1])**2) < 100:  # 10px radius
                return vertex_id
        return None

    def _render_text(self, text: str, color, font=None, bold=False):
        """Helper method to render text"""
        if font is None:
            font = self.font
        if bold:
            font.set_bold(True)
        text_surface = font.render(text, True, color)
        if bold:
            font.set_bold(False)
        return text_surface

    def _draw_edges(self):
        """Draw edges with enhanced traffic visualization"""
        traffic_manager = self.fleet_manager.traffic_manager
        
        # Draw all edges first (background)
        for u, v in self.nav_graph.graph.edges():
            u_pos = self._scale_position(*self.nav_graph.vertices[u][:2])
            v_pos = self._scale_position(*self.nav_graph.vertices[v][:2])
            pygame.draw.line(self.screen, self.colors['edge'], u_pos, v_pos, 2)

        # Draw reserved edges
        for edge, (robot_id, _) in traffic_manager.edge_locks.items():
            u_pos = self._scale_position(*self.nav_graph.vertices[edge[0]][:2])
            v_pos = self._scale_position(*self.nav_graph.vertices[edge[1]][:2])
            pygame.draw.line(self.screen, self.colors['edge_reserved'], u_pos, v_pos, 3)

            # Draw waypoints along reserved edges
            if edge in traffic_manager.edge_waypoints:
                for wx, wy in traffic_manager.edge_waypoints[edge]:
                    pos = self._scale_position(wx, wy)
                    pygame.draw.circle(self.screen, self.colors['edge_reserved'], pos, 3)

        # Draw blocked edges
        for edge in traffic_manager.get_blocked_edges():
            u_pos = self._scale_position(*self.nav_graph.vertices[edge[0]][:2])
            v_pos = self._scale_position(*self.nav_graph.vertices[edge[1]][:2])
            pygame.draw.line(self.screen, self.colors['edge_blocked'], u_pos, v_pos, 4)

    def _draw_vertices(self):
        """Draw vertices with enhanced visual effects"""
        traffic_manager = self.fleet_manager.traffic_manager
        
        # Draw edges first
        for edge in self.nav_graph.graph.edges():
            start_pos = self._scale_position(*self.nav_graph.vertices[edge[0]][:2])
            end_pos = self._scale_position(*self.nav_graph.vertices[edge[1]][:2])
            
            # Determine edge color and style
            color = self.colors['edge']
            width = 2
            if edge in traffic_manager.edge_locks:
                color = self.colors['edge_blocked']
                width = 3
            
            # Draw dashed line for paths
            length = math.hypot(end_pos[0] - start_pos[0], end_pos[1] - start_pos[1])
            dash_length = 10
            dash_gap = 5
            num_dashes = int(length / (dash_length + dash_gap))
            
            for i in range(num_dashes):
                start_t = i * (dash_length + dash_gap) / length
                end_t = start_t + dash_length / length
                dash_start = (
                    start_pos[0] + (end_pos[0] - start_pos[0]) * start_t,
                    start_pos[1] + (end_pos[1] - start_pos[1]) * start_t
                )
                dash_end = (
                    start_pos[0] + (end_pos[0] - start_pos[0]) * end_t,
                    start_pos[1] + (end_pos[1] - start_pos[1]) * end_t
                )
                pygame.draw.line(self.screen, color, dash_start, dash_end, width)
        
        # Draw approach zones with glow effect
        for vertex_id, approaching_robots in traffic_manager.vertex_approach_zones.items():
            if approaching_robots:
                pos = self._scale_position(*self.nav_graph.vertices[vertex_id][:2])
                # Draw multiple circles for glow effect
                for radius in range(int(traffic_manager.vertex_safety_radius * 10), 
                                 int(traffic_manager.vertex_safety_radius * 8), -2):
                    alpha = int(128 * (radius / (traffic_manager.vertex_safety_radius * 10)))
                    glow_surface = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
                    pygame.draw.circle(glow_surface, (*self.colors['vertex_approach'], alpha), 
                                    (radius, radius), radius)
                    self.screen.blit(glow_surface, (pos[0] - radius, pos[1] - radius))

        # Draw vertices with enhanced effects
        for vertex_id, (x, y, attrs) in self.nav_graph.vertices.items():
            pos = self._scale_position(x, y)
            
            # Determine vertex appearance
            color = self.colors['vertex']
            radius = 12
            glow = False
            
            if vertex_id == self.hover_vertex:
                color = self.colors['vertex_hover']
                glow = True
            if vertex_id in traffic_manager.vertex_occupants:
                color = self.colors['vertex_reserved']
            if vertex_id in traffic_manager.get_blocked_vertices():
                color = self.colors['vertex_blocked']
                glow = True
            
            # Draw glow effect if needed
            if glow:
                for r in range(radius + 6, radius, -2):
                    alpha = int(128 * (r / (radius + 6)))
                    glow_surface = pygame.Surface((r * 2, r * 2), pygame.SRCALPHA)
                    pygame.draw.circle(glow_surface, (*color, alpha), (r, r), r)
                    self.screen.blit(glow_surface, (pos[0] - r, pos[1] - r))
            
            # Draw vertex
            if self.nav_graph.is_charger(vertex_id):
                # Draw charger with pulsing effect
                alpha = self.pulse_alpha if self.pulse_increasing else 255 - self.pulse_alpha
                glow_surface = pygame.Surface((30, 30), pygame.SRCALPHA)
                pygame.draw.circle(glow_surface, (*self.colors['charger'], alpha), (15, 15), 15)
                self.screen.blit(glow_surface, (pos[0] - 15, pos[1] - 15))
                pygame.draw.circle(self.screen, self.colors['charger'], pos, 15)
                pygame.draw.circle(self.screen, self.colors['charger_border'], pos, 15, 2)
                # Draw lightning bolt
                text_surface = self._render_text("⚡", self.colors['text'])
                text_rect = text_surface.get_rect(center=pos)
                self.screen.blit(text_surface, text_rect)
            else:
                pygame.draw.circle(self.screen, color, pos, radius)
            
            # Draw vertex label with white text
            label = f"{vertex_id}"
            if attrs.get('name'):
                label += f" ({attrs['name']})"
            text_surface = self._render_text(label, self.colors['text'], bold=True)
            self.screen.blit(text_surface, (pos[0] - text_surface.get_width()//2, 
                                          pos[1] + 20))

    def _draw_robots(self):
        """Draw robots with enhanced visual effects"""
        current_time = time.time()
        
        # Update pulsing effect
        if self.pulse_increasing:
            self.pulse_alpha = min(255, self.pulse_alpha + 5)
            if self.pulse_alpha >= 255:
                self.pulse_increasing = False
        else:
            self.pulse_alpha = max(128, self.pulse_alpha - 5)
            if self.pulse_alpha <= 128:
                self.pulse_increasing = True
        
        # Update blinking effect (every 0.5 seconds)
        if current_time - self.last_blink_time > 0.5:
            self.blink_state = not self.blink_state
            self.last_blink_time = current_time
        
        for robot_id, robot in self.robots.items():
            pos = self._scale_position(robot.current_x, robot.current_y)
            
            # Get robot color from specific colors or generate one
            robot_color = self.robot_colors.get(robot_id, robot.color)
            
            # Update and draw trail if moving
            if robot.status == RobotStatus.MOVING:
                if robot_id not in self.robot_trails:
                    self.robot_trails[robot_id] = []
                self.robot_trails[robot_id].append((pos, 255))
                
                # Draw trail
                for trail_pos, alpha in self.robot_trails[robot_id]:
                    trail_surface = pygame.Surface((10, 10), pygame.SRCALPHA)
                    pygame.draw.circle(trail_surface, (*robot_color, alpha), (5, 5), 5)
                    self.screen.blit(trail_surface, (trail_pos[0] - 5, trail_pos[1] - 5))
                
                # Update trail alphas and remove old points
                self.robot_trails[robot_id] = [(p, max(0, a - 10)) for p, a in self.robot_trails[robot_id]]
                self.robot_trails[robot_id] = [(p, a) for p, a in self.robot_trails[robot_id] if a > 0]
            
            # Draw robot with status effects
            status_effect = self.status_effects.get(robot.status, {})
            
            # Draw glow effect for moving robots
            if status_effect.get('glow', False):
                glow_surface = pygame.Surface((40, 40), pygame.SRCALPHA)
                glow_alpha = self.pulse_alpha if self.pulse_increasing else 255 - self.pulse_alpha
                pygame.draw.circle(glow_surface, (*robot_color, glow_alpha), (20, 20), 20)
                self.screen.blit(glow_surface, (pos[0] - 20, pos[1] - 20))
            
            # Draw robot body
            pygame.draw.circle(self.screen, robot_color, pos, robot.size)
            
            # Draw blinking border for waiting robots
            if status_effect.get('blink', False) and self.blink_state:
                pygame.draw.circle(self.screen, self.colors['text'], pos, robot.size + 2, 2)
            
            # Draw checkmark for completed tasks
            if status_effect.get('checkmark', False):
                check_surface = self._render_text("✓", self.colors['success'])
                check_rect = check_surface.get_rect(center=pos)
                self.screen.blit(check_surface, check_rect)
            
            # Draw status text with enhanced visibility
            status_text = f"R{robot_id}: {robot.status.value}"
            if robot.status == RobotStatus.TASK_COMPLETE:
                status_text += f" at {robot.target_vertex}"
            elif robot.status == RobotStatus.MOVING:
                if robot.current_edge:
                    progress = self.fleet_manager.traffic_manager.robot_edge_progress.get(
                        robot_id, (None, 0.0))[1]
                    status_text += f" → {robot.target_vertex} ({int(progress*100)}%)"
            elif robot.status == RobotStatus.BLOCKED:
                status_text += f" (by R{robot.blocked_by})"
            
            # Draw status text with background for better visibility
            text_surface = self._render_text(status_text, self.colors['text'])
            text_bg = pygame.Surface((text_surface.get_width() + 4, text_surface.get_height() + 4))
            text_bg.fill(self.colors['panel_bg'])
            text_bg.set_alpha(200)
            self.screen.blit(text_bg, (pos[0] + 15, pos[1] - 15))
            self.screen.blit(text_surface, (pos[0] + 17, pos[1] - 13))
            
            # Draw selection highlight
            if robot_id == self.selected_robot:
                pygame.draw.circle(self.screen, self.colors['vertex_selected'], pos, robot.size + 4, 2)

    def _draw_alerts(self):
        """Draw alerts with corrected text rendering"""
        current_time = time.time()
        y_offset = 10
        
        active_alerts = []
        for alert in self.alerts:
            if current_time - alert['time'] < alert['duration']:
                text_surface = self._render_text(alert['message'], self.colors['alert'], self.alert_font)
                self.screen.blit(text_surface, (10, y_offset))
                y_offset += 30
                active_alerts.append(alert)
        self.alerts = active_alerts

    def add_alert(self, message: str, duration: float = 3.0):
        """Add alert with logging"""
        self.alerts.append({
            'message': message,
            'time': time.time(),
            'duration': duration
        })
        logging.info(message)

    def _draw_side_panel(self):
        """Draw side panel with both traffic and robot status information"""
        # Draw panel background
        pygame.draw.rect(self.screen, self.colors['panel_bg'], self.side_panel)
        
        # Draw buttons
        self._draw_buttons()
        
        # Draw traffic status panel
        self._draw_traffic_panel()
        
        # Draw robot status panel
        self._draw_robot_panel()

    def _draw_traffic_panel(self):
        """Draw traffic status panel"""
        pygame.draw.rect(self.screen, self.colors['status_panel'], self.traffic_panel)
        
        traffic_manager = self.fleet_manager.traffic_manager
        
        # Draw panel header with background
        header_rect = pygame.Rect(self.traffic_panel.left, self.traffic_panel.top, 
                                self.traffic_panel.width, 30)
        pygame.draw.rect(self.screen, self.colors['panel_header'], header_rect)
        title_surface = self._render_text("Traffic Status", (255, 255, 255))  # White text
        self.screen.blit(title_surface, (self.traffic_panel.centerx - title_surface.get_width()//2, 
                                       self.traffic_panel.top + 5))
        
        y_offset = self.traffic_panel.top + 40
        
        # Show blocked vertices
        blocked_vertices = traffic_manager.get_blocked_vertices()
        if blocked_vertices:
            text = f"Blocked: {sorted(list(blocked_vertices))}"
            text_surface = self._render_text(text, self.colors['vertex_blocked'])
            self.screen.blit(text_surface, (self.traffic_panel.left + 10, y_offset))
            y_offset += 20
        
        # Show reserved vertices
        reserved_vertices = traffic_manager.vertex_occupants
        if reserved_vertices:
            text = f"Reserved: {[(v, f'R{r}') for v, r in reserved_vertices.items()]}"
            text_surface = self._render_text(text, self.colors['vertex_reserved'])
            self.screen.blit(text_surface, (self.traffic_panel.left + 10, y_offset))
            y_offset += 20
        
        # Show approach zones
        approach_zones = traffic_manager.vertex_approach_zones
        if approach_zones:
            text = f"Approaching: {[(v, len(r)) for v, r in approach_zones.items()]}"
            text_surface = self._render_text(text, self.colors['vertex_approach'])
            self.screen.blit(text_surface, (self.traffic_panel.left + 10, y_offset))
            y_offset += 20
        
        # Show edge locks
        edge_locks = traffic_manager.edge_locks
        if edge_locks:
            text = f"Reserved Edges: {len(edge_locks)}"
            text_surface = self._render_text(text, self.colors['edge_reserved'])
            self.screen.blit(text_surface, (self.traffic_panel.left + 10, y_offset))
            y_offset += 20
        
        # Show waiting queues
        waiting_queues = {v: q for v, q in traffic_manager.waiting_queues.items() if q}
        if waiting_queues:
            text = f"Waiting: {[(v, len(q)) for v, q in waiting_queues.items()]}"
            text_surface = self._render_text(text, self.colors['waiting'])
            self.screen.blit(text_surface, (self.traffic_panel.left + 10, y_offset))

    def _draw_robot_panel(self):
        """Draw robot status panel with detailed information"""
        pygame.draw.rect(self.screen, self.colors['robot_panel'], self.robot_panel)
        
        # Draw panel header with background
        header_rect = pygame.Rect(self.robot_panel.left, self.robot_panel.top, 
                                self.robot_panel.width, 30)
        pygame.draw.rect(self.screen, self.colors['panel_header'], header_rect)
        title_surface = self._render_text("Robot Status", (255, 255, 255))  # White text
        self.screen.blit(title_surface, (self.robot_panel.centerx - title_surface.get_width()//2, 
                                       self.robot_panel.top + 5))
        
        y_offset = self.robot_panel.top + 40
        
        # Group robots by status
        robots_by_status = {status: [] for status in RobotStatus}
        for robot_id, robot in self.robots.items():
            robots_by_status[robot.status].append(robot)
        
        # Display robots grouped by status
        for status, robots in robots_by_status.items():
            if not robots:
                continue
                
            # Status header
            status_color = self.status_colors[status]
            status_text = f"{status.value} ({len(robots)})"
            text_surface = self._render_text(status_text, status_color, self.alert_font)
            self.screen.blit(text_surface, (self.robot_panel.left + 10, y_offset))
            y_offset += 25
            
            # Robot details
            for robot in robots:
                details = f"R{robot.robot_id}: "
                if status == RobotStatus.MOVING:
                    if robot.current_edge:
                        progress = self.fleet_manager.traffic_manager.robot_edge_progress.get(
                            robot.robot_id, (None, 0.0))[1]
                        details += f"→ {robot.target_vertex} ({int(progress*100)}%)"
                    else:
                        details += f"→ {robot.target_vertex}"
                elif status == RobotStatus.BLOCKED:
                    details += f"by R{robot.blocked_by}"
                elif status == RobotStatus.WAITING:
                    if robot.waiting_start_time:
                        wait_time = time.time() - robot.waiting_start_time
                        details += f"for {int(wait_time)}s"
                elif status == RobotStatus.TASK_COMPLETE:
                    details += f"at {robot.target_vertex}"
                
                text_surface = self._render_text(details, self.colors['text'], self.small_font)
                self.screen.blit(text_surface, (self.robot_panel.left + 20, y_offset))
                y_offset += 20
            
            y_offset += 5  # Add space between status groups

    def handle_click(self, pos: Tuple[int, int]) -> None:
        """Enhanced click handling with error prevention"""
        try:
            print(f"Click at position: {pos}")  # Debug print
            
            # Check if click is in side panel area
            if pos[0] > 900:  # Side panel starts at x=900
                # Handle button clicks
                for button_name, button in self.buttons.items():
                    if button['rect'].collidepoint(pos):
                        print(f"Clicked {button_name} button")  # Debug print
                        self._handle_button_click(button_name)
                        return
            else:
                # Handle vertex clicks in main area
                clicked_vertex = self._get_vertex_at_pos(pos)
                if clicked_vertex is not None:
                    print(f"Clicked vertex: {clicked_vertex}")  # Debug print
                    self._handle_vertex_click(clicked_vertex)
        except Exception as e:
            print(f"Error handling click: {e}")  # Debug print
            logging.error(f"Error handling click: {e}")
            # Don't let the error crash the program
            self.add_alert(f"Error: {str(e)}")

    def _handle_button_click(self, button_name: str):
        """Handle button clicks with error prevention"""
        try:
            # Reset other modes first
            self.spawn_mode = False
            self.task_mode = False
            
            # Update button states
            for name in self.buttons:
                self.buttons[name]['active'] = False
            
            if button_name == 'spawn':
                self.spawn_mode = True
                self.buttons['spawn']['active'] = True
                self.selected_robot = None
                print("Spawn mode activated")  # Debug print
                self.add_alert("Spawn mode: Click any vertex to spawn a robot")
            
            elif button_name == 'assign':
                self.task_mode = True
                self.buttons['assign']['active'] = True
                self.selected_robot = None
                print("Task mode activated")  # Debug print
                self.add_alert("Task mode: First click a robot, then click destination")
            
            elif button_name == 'cancel':
                self.selected_robot = None
                print("All modes cancelled")  # Debug print
                self.add_alert("Cancelled current action")
            
        except Exception as e:
            print(f"Error in button click: {e}")  # Debug print
            logging.error(f"Error in button click: {e}")
            self.add_alert(f"Error: {str(e)}")

    def _handle_vertex_click(self, vertex_id: int):
        """Handle vertex clicks with error prevention"""
        try:
            if self.spawn_mode:
                print(f"Attempting to spawn robot at vertex {vertex_id}")  # Debug print
                new_robot = self.fleet_manager.spawn_robot(vertex_id)
                if new_robot:
                    print(f"Successfully spawned Robot {new_robot.robot_id}")  # Debug print
                    self.add_alert(f"Spawned Robot {new_robot.robot_id} at vertex {vertex_id}")
                else:
                    print(f"Failed to spawn robot at vertex {vertex_id}")  # Debug print
                    self.add_alert(f"Cannot spawn robot at vertex {vertex_id}")

            elif self.task_mode:
                if self.selected_robot is None:
                    # Try to select a robot at this vertex
                    for robot_id, robot in self.robots.items():
                        if robot.current_vertex == vertex_id:
                            self.selected_robot = robot_id
                            print(f"Selected Robot {robot_id}")  # Debug print
                            self.add_alert(f"Selected Robot {robot_id}")
                            break
                else:
                    # Assign task to selected robot
                    print(f"Attempting to assign task: Robot {self.selected_robot} to vertex {vertex_id}")  # Debug print
                    if self.fleet_manager.assign_task(self.selected_robot, vertex_id):
                        print(f"Successfully assigned task")  # Debug print
                        self.add_alert(f"Assigned Robot {self.selected_robot} to navigate to vertex {vertex_id}")
                    else:
                        print(f"Failed to assign task")  # Debug print
                        self.add_alert(f"Cannot assign task - path blocked or invalid")
                    self.selected_robot = None
                
        except Exception as e:
            print(f"Error in vertex click: {e}")  # Debug print
            logging.error(f"Error in vertex click: {e}")
            self.add_alert(f"Error: {str(e)}")

    def _draw_buttons(self):
        """Draw buttons with corrected text rendering"""
        for button_name, button in self.buttons.items():
            # Determine button color based on state
            color = self.colors['button_active'] if button['active'] else self.colors['button_normal']
            
            # Draw button
            pygame.draw.rect(self.screen, color, button['rect'])
            
            # Draw button text with corrected rendering
            text_surface = self._render_text(button['text'], self.colors['text'])
            text_rect = text_surface.get_rect(center=button['rect'].center)
            self.screen.blit(text_surface, text_rect)

    def _draw_legend(self):
        """Draw legend panel with robot colors and status indicators"""
        pygame.draw.rect(self.screen, self.colors['panel_bg'], self.legend_panel)
        pygame.draw.rect(self.screen, self.colors['panel_header'], self.legend_panel, 2)
        
        # Draw title
        title = self._render_text("Legend", self.colors['text'], bold=True)
        self.screen.blit(title, (self.legend_panel.x + 10, self.legend_panel.y + 5))
        
        # Draw robot colors
        y_offset = 25
        for robot_id, color in self.robot_colors.items():
            pygame.draw.circle(self.screen, color, 
                             (self.legend_panel.x + 20, self.legend_panel.y + y_offset), 8)
            text = self._render_text(f"Robot {robot_id}", self.colors['text'])
            self.screen.blit(text, (self.legend_panel.x + 35, self.legend_panel.y + y_offset - 8))
            y_offset += 20
        
        # Draw status indicators
        x_offset = 100
        y_offset = 25
        for status, effect in self.status_effects.items():
            color = effect['color']
            text = self._render_text(status.value, self.colors['text'])
            pygame.draw.circle(self.screen, color,
                             (self.legend_panel.x + x_offset, self.legend_panel.y + y_offset), 8)
            self.screen.blit(text, (self.legend_panel.x + x_offset + 15, 
                                  self.legend_panel.y + y_offset - 8))
            y_offset += 20

    def _draw_critical_banner(self):
        """Draw critical alerts banner at the top of the screen"""
        if self.critical_alerts:
            banner_rect = pygame.Rect(0, 0, self.screen.get_width(), self.critical_banner_height)
            pygame.draw.rect(self.screen, self.colors['critical'], banner_rect)
            
            # Draw the most recent critical alert
            alert_text = self._render_text(self.critical_alerts[-1], self.colors['text'], bold=True)
            text_pos = ((self.screen.get_width() - alert_text.get_width()) // 2,
                       (self.critical_banner_height - alert_text.get_height()) // 2)
            self.screen.blit(alert_text, text_pos)

    def add_critical_alert(self, message):
        """Add a critical alert to be displayed in the banner"""
        self.critical_alerts.append(message)
        if len(self.critical_alerts) > 5:  # Keep only the 5 most recent alerts
            self.critical_alerts.pop(0)

    def update(self):
        """Update the display with all visual elements"""
        # Fill background
        self.screen.fill(self.colors['background'])
        
        # Draw main elements
        self._draw_vertices()
        self._draw_robots()
        self._draw_panels()
        self._draw_buttons()
        self._draw_legend()
        self._draw_critical_banner()
        
        # Update display
        pygame.display.flip()
        self.clock.tick(60)  # 60 FPS

    def run(self):
        """Main GUI loop with error handling"""
        running = True
        while running:
            try:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        self.handle_click(event.pos)
                    elif event.type == pygame.MOUSEMOTION:
                        self.hover_vertex = self._get_vertex_at_pos(event.pos)

                # Draw everything
                self.screen.fill(self.colors['background'])
                self._draw_edges()
                self._draw_vertices()
                self._draw_robots()
                self._draw_buttons()
                self._draw_side_panel()
                self._draw_alerts()
                
                pygame.display.flip()
                self.clock.tick(60)
                
            except Exception as e:
                print(f"Error in main loop: {e}")  # Debug print
                logging.error(f"Error in main loop: {e}")
                self.add_alert(f"Error: {str(e)}")

        pygame.quit()