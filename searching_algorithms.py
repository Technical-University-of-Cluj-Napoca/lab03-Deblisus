from utils import *
from collections import deque
from queue import PriorityQueue
from grid import Grid
from spot import Spot
from math import sqrt

def bfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Breadth-First Search (BFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    if start is None or end is None:
        return False
    from collections import deque
    queue = deque()
    queue.append(start)
    visited = {start}
    came_from = {}

    while len(queue):
        # event
        current = queue.popleft()
        if current == end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        
        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                came_from[neighbor] = current
                queue.append(neighbor)
                neighbor.make_open()
        
        draw()
        if current != start:
            current.make_closed()
    
    return False

def dfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Depdth-First Search (DFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    if start is None or end is None:
        return False
    stack = [start]
    visited = {start}
    came_from = {}

    while len(stack):
        # UI event
        current = stack.pop()
        if current == end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        
        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                came_from[neighbor] = current
                stack.append(neighbor)
                neighbor.make_open()
        
        draw()
        if current != start:
            current.make_closed()

def h_manhattan_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """
    Heuristic function for A* algorithm: uses the Manhattan distance between two points.
    Args:
        p1 (tuple[int, int]): The first point (x1, y1).
        p2 (tuple[int, int]): The second point (x2, y2).
    Returns:
        float: The Manhattan distance between p1 and p2.
    """
    return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])

def h_euclidian_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """
    Heuristic function for A* algorithm: uses the Euclidian distance between two points.
    Args:
        p1 (tuple[int, int]): The first point (x1, y1).
        p2 (tuple[int, int]): The second point (x2, y2).
    Returns:
        float: The Manhattan distance between p1 and p2.
    """
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


def astar(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    A* Pathfinding Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    count = 0
    open_heap = PriorityQueue()
    open_heap.put((0, count, start))
    came_from = {}
    g_score = {}
    f_score = {}
    for row in grid.grid:
        for col in row:
            g_score[col] = 9999999999
            f_score[col] = 9999999999
    g_score[start] = 0
    f_score[start] = h_euclidian_distance(start.get_position(), end.get_position())
    lookup_set = {start}

    while not open_heap.empty():
        # UI events
        current = open_heap.get()[2]
        lookup_set.remove(current)

        if current is end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        
        for neighbor in current.neighbors:
            tentative_g = g_score[current] + 1 # 1 == cost betweenn moves
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + h_euclidian_distance(neighbor.get_position(), end.get_position())
                if neighbor not in lookup_set:
                    count += 1
                    open_heap.put((f_score[neighbor], count, neighbor))
                    lookup_set.add(neighbor)
                    neighbor.make_open()
        
        draw()
        if current != start:
            current.make_closed()
    
    return False

def ucs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    open_heap = PriorityQueue()
    open_heap.put((0, start))
    visited = {start: (0, None)}
    came_from = {}

    while not open_heap.empty():
        current_cost, current = open_heap.get()
        if current is end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:
            total_cost = current_cost + 1
            if neighbor not in visited or total_cost < visited[neighbor][0]:
                visited[neighbor] = (total_cost, current)
                open_heap.put((total_cost, neighbor))
                neighbor.make_open()
        
        draw()
        if current != start:
            current.make_closed()
    return False

# and the others algorithms...
# ▢ Depth-Limited Search (DLS)
# ▢ Uninformed Cost Search (UCS)
# ▢ Greedy Search
# ▢ Iterative Deepening Search/Iterative Deepening Depth-First Search (IDS/IDDFS)
# ▢ Iterative Deepening A* (IDA)
# Assume that each edge (graph weight) equalss