import math

class Node:
    """Node class for Dijkstra Path Planning tracking coordinates and cost."""
    def __init__(self, x: int, y: int, cost: float, parent_index: int):
        self.x = x  # Grid index X
        self.y = y  # Grid index Y
        self.cost = cost  # Accumulative travel cost
        self.parent_index = parent_index  # Index of the previous node

    def __str__(self):
        return f"Node({self.x}, {self.y}, cost={self.cost}, parent={self.parent_index})"


class DijkstraPlanner:
    def __init__(self, ox: list[float], oy: list[float], resolution: float, robot_radius: float, map_bounds: tuple[float, float, float, float]):
        """
        Initialize grid map for Dijkstra planning.
        :param ox: Obstacle X positions list [m]
        :param oy: Obstacle Y positions list [m]
        :param resolution: Grid resolution [m]
        :param robot_radius: Robot safety radius [m]
        :param map_bounds: Tuple of (min_x, min_y, max_x, max_y)
        """
        self.resolution = resolution
        self.robot_radius = robot_radius
        
        # Unpack absolute map constraints requested by the API
        self.min_x, self.min_y, self.max_x, self.max_y = map_bounds
        
        # Calculate grid size (cells) from world dimensions
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        
        # Initialize 2D obstacle map grid with False (default free space)
        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]
        self.calc_obstacle_map(ox, oy)
        
        # 8-connected Grid Motion Model: [dx, dy, step_cost]
        self.motion = [
            [1, 0, 1.0],        # Right
            [0, 1, 1.0],        # Up
            [-1, 0, 1.0],       # Left
            [0, -1, 1.0],       # Down
            [1, 1, math.sqrt(2)],   # Top-Right Diagonal
            [1, -1, math.sqrt(2)],  # Bottom-Right Diagonal
            [-1, 1, math.sqrt(2)],  # Top-Left Diagonal
            [-1, -1, math.sqrt(2)]  # Bottom-Left Diagonal
        ]

    def calc_position(self, index: int, min_pos: float) -> float:
        """Convert matrix grid index back to real world meter coordinate."""
        return index * self.resolution + min_pos

    def calc_xy_index(self, position: float, min_pos: float) -> int:
        """Convert real world meter coordinate to matrix grid index."""
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node: Node) -> int:
        """Generate unique 1D hash index identifier for tracking in dictionaries."""
        return node.y * self.x_width + node.x

    def calc_obstacle_map(self, ox: list[float], oy: list[float]):
        """Builds inflated obstacle grid map. Prevents diagonal clipping."""
        # Safety padding formula: Robot size + maximum diagonal reach within a cell
        safety_buffer = self.robot_radius + (self.resolution / math.sqrt(2))

        for ix in range(self.x_width):
            x = self.calc_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y)
                
                # If current cell hits ANY obstacle buffer zone, mark it as blocked
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= safety_buffer:
                        self.obstacle_map[ix][iy] = True
                        break

    def verify_node(self, node: Node) -> bool:
        """Check if node is within map bounds and sits outside obstacle zones."""
        if node.x < 0 or node.x >= self.x_width:
            return False
        if node.y < 0 or node.y >= self.y_width:
            return False
        if self.obstacle_map[node.x][node.y]:
            return False
        return True

    def planning(self, sx: float, sy: float, gx: float, gy: float) -> tuple[list[float], list[float], float, list[list[float]]]:
        """
        Dijkstra path planning execution.
        :return: (path_x, path_y, final_cost, search_history)
        """
        # Map start/goal coordinates to matrix grid nodes
        start_node = Node(self.calc_xy_index(sx, self.min_x),
                          self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = Node(self.calc_xy_index(gx, self.min_x),
                         self.calc_xy_index(gy, self.min_y), 0.0, -1)

        if not self.verify_node(start_node) or not self.verify_node(goal_node):
            return [], [], 0.0, []

        open_set = dict()   # Nodes to explore
        closed_set = dict() # Visited nodes
        
        open_set[self.calc_grid_index(start_node)] = start_node
        
        # Track history of explored [x, y] coordinates for API GIF animation
        search_history = []

        while open_set:
            # Dijkstra chooses node with the lowest overall accumulated cost
            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            # Add current coordinate to visualization history
            search_history.append([
                self.calc_position(current.x, self.min_x),
                self.calc_position(current.y, self.min_y)
            ])

            # Destination check reached
            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Move current node from Open to Closed set
            del open_set[c_id]
            closed_set[c_id] = current

            # Expand search to 8 neighbors
            for move in self.motion:
                node = Node(current.x + move[0],
                            current.y + move[1],
                            current.cost + move[2], c_id)
                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id in open_set:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id].cost = node.cost
                        open_set[n_id].parent_index = c_id
                else:
                    open_set[n_id] = node

        # Backtrack route to compile final coordinates lists
        if goal_node.parent_index == -1:
            return [], [], 0.0, search_history  # Path not found

        rx, ry = [], []
        parent_node = current
        while parent_node.parent_index != -1:
            rx.append(self.calc_position(parent_node.x, self.min_x))
            ry.append(self.calc_position(parent_node.y, self.min_y))
            parent_node = closed_set[parent_node.parent_index]
            
        rx.append(self.calc_position(start_node.x, self.min_x))
        ry.append(self.calc_position(start_node.y, self.min_y))

        # Reverse coordinates to read from Start -> Goal
        return rx[::-1], ry[::-1], goal_node.cost, search_history