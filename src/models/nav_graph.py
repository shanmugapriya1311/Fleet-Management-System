import json
import networkx as nx
from typing import Dict, List, Tuple, Optional, Set

class NavGraph:
    def __init__(self):
        self.graph = nx.Graph()
        self.vertices: Dict[int, Tuple[float, float, Dict]] = {}
        self.building_name = ""
        self.max_x = 0
        self.min_x = 0
        self.max_y = 0
        self.min_y = 0
        self.charger_vertices = set()  # Track charger vertices

    def load_from_json(self, file_path: str) -> None:
        with open(file_path) as f:
            data = json.load(f)
        
        self.building_name = data.get("building_name", "")
        level_data = next(iter(data["levels"].values()))  # Use first level
        
        # Add vertices
        for idx, vertex in enumerate(level_data["vertices"]):
            x, y, attrs = vertex
            self.vertices[idx] = (x, y, attrs)
            self.graph.add_node(idx, pos=(x, y), **attrs)
            
            # Track charger vertices
            if attrs.get("is_charger", False):
                self.charger_vertices.add(idx)
            
            # Update bounds
            if idx == 0:  # First vertex
                self.max_x = self.min_x = x
                self.max_y = self.min_y = y
            else:
                self.max_x = max(self.max_x, x)
                self.min_x = min(self.min_x, x)
                self.max_y = max(self.max_y, y)
                self.min_y = min(self.min_y, y)
        
        # Add edges
        for u, v, attrs in level_data["lanes"]:
            self.graph.add_edge(u, v, **attrs)

    def get_shortest_path(self, start: int, end: int, occupied_vertices: Set[int] = None) -> List[int]:
        """Find shortest path avoiding occupied vertices"""
        if occupied_vertices is None:
            occupied_vertices = set()
        
        try:
            # Create a copy of the graph excluding occupied vertices
            temp_graph = self.graph.copy()
            for vertex in occupied_vertices:
                if vertex in temp_graph and vertex != start and vertex != end:
                    temp_graph.remove_node(vertex)
                
            return nx.astar_path(temp_graph, start, end, 
                               heuristic=self._euclidean_heuristic)
        except nx.NetworkXNoPath:
            return []

    def _euclidean_heuristic(self, u, v):
        (x1, y1, _), (x2, y2, _) = self.vertices[u], self.vertices[v]
        return ((x2-x1)**2 + (y2-y1)**2)**0.5

    def get_vertex_position(self, vertex_id: int) -> Tuple[float, float]:
        """Get the position of a vertex"""
        x, y, _ = self.vertices[vertex_id]
        return (x, y)

    def is_charger(self, vertex_id: int) -> bool:
        """Check if a vertex is a charger point"""
        return vertex_id in self.charger_vertices