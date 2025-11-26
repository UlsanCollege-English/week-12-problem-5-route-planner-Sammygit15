import heapq
from collections import deque

def route_planner(graph, start, goal, weighted):
    """
    Plan a route between start and goal.

    If weighted is False:
        - graph: dict node -> list of neighbor nodes (unweighted).
        - Use BFS to find a path with the fewest edges.
        - Return (path, steps) where steps = number of edges.

    If weighted is True:
        - graph: dict node -> list of (neighbor, weight) pairs (positive weights).
        - Use Dijkstra to find a path with the smallest total weight.
        - Return (path, total_cost).

    In both cases:
        - If start or goal not in graph, or no path exists, return ([], None).
    """
    if start not in graph or goal not in graph:
        return [], None

    if start == goal:
        return [start], 0

    if weighted:
        return _dijkstra_shortest_path(graph, start, goal)
    else:
        return _bfs_shortest_path(graph, start, goal)


def _bfs_shortest_path(graph, start, goal):
    """Return shortest path in number of edges using BFS."""
    visited = set([start])
    queue = deque([[start]])

    while queue:
        path = queue.popleft()
        node = path[-1]

        if node == goal:
            return path, len(path) - 1  # steps = edges

        for neighbor in graph.get(node, []):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(path + [neighbor])

    return [], None


def _dijkstra_shortest_path(graph, start, goal):
    """Return shortest path and total cost using Dijkstra."""
    heap = [(0, start, [start])]  # (cost, node, path)
    visited = {}

    while heap:
        cost, node, path = heapq.heappop(heap)

        if node in visited and visited[node] <= cost:
            continue
        visited[node] = cost

        if node == goal:
            return path, cost

        for neighbor, weight in graph.get(node, []):
            if neighbor not in visited or cost + weight < visited.get(neighbor, float('inf')):
                heapq.heappush(heap, (cost + weight, neighbor, path + [neighbor]))

    return [], None


if __name__ == "__main__":
    # Manual tests (optional)
    unweighted_graph = {
        "S": ["A", "B"],
        "A": ["S", "C"],
        "B": ["S", "D"],
        "C": ["A", "E"],
        "D": ["B", "E"],
        "E": ["C", "D", "F"],
        "F": ["E"],
    }

    weighted_graph = {
        "S": [("A", 1), ("B", 5)],
        "A": [("S", 1), ("C", 2)],
        "B": [("S", 5), ("C", 1), ("D", 7)],
        "C": [("A", 2), ("B", 1), ("E", 3)],
        "D": [("B", 7), ("E", 1)],
        "E": [("C", 3), ("D", 1)],
    }

    print("Unweighted path S->F:", route_planner(unweighted_graph, "S", "F", weighted=False))
    print("Weighted path S->E:", route_planner(weighted_graph, "S", "E", weighted=True))
