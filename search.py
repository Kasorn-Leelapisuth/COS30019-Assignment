import sys # Import system to read commands from the terminal
import math
import heapq

from collections import deque #from week 3
import numpy as np

from utils import * # Uses PriorityQueue and distance from the provided utils.py


def parse_file(filename): 
    nodes = {} # Stores node coordinates
    edges = {} # Stores paths between nodes as adjacency list
    origin = None # Stores the starting point
    destinations = [] # Stores the goal points
    
    try:
        with open(filename, 'r',  encoding="utf-8-sig") as f: 
            lines = [line.strip() for line in f.readlines() if line.strip()] # Read lines and remove empty spaces

    except FileNotFoundError: # If file is missing, show error
        print(f"Error: File '{filename}' not found.")
        sys.exit()

    section = None # Marker to know which part of the file we are reading
        
    for line in lines: # Loop through each line
        if "Nodes:" in line: 
                    section = "nodes"
        elif "Edges:" in line: 
                    section = "edges"
        elif "Origin:" in line: 
                    section = "origin"
        elif "Destinations:" in line: 
                    section = "dest"
        else:
            if section == "nodes":
                    node_id, coord_part = line.split(":") # Separate node name and numbers
                    x, y = coord_part.strip().strip("()").split(",")
                    nodes[node_id] = (int(x), int(y))
                       
            elif section == "edges":
                    edge_part, cost_part = line.split(":") # Separate path and cost
                    u, v = edge_part.strip().strip("()").split(",")
                    u, v = u.strip(), v.strip()
                    cost = int(cost_part.strip())

                    if u not in edges: 
                        edges[u] = []
                    edges[u].append((v, cost)) # Add the path to our list
                    
            elif section == "origin":
                origin = line.strip() # Set the start node
            elif section == "dest":
                destinations = [d.strip() for d in line.split(';') if d.strip()] # Set goal nodes
    
    # make sure every node exists in edges dict, even if it has no outgoing edges
    for node in nodes:
        if node not in edges:
            edges[node] = []

    return nodes, edges, origin, destinations

# Helper function to rebuild the path from the parent dictionary
def reconstruct_path(parent_map, current): 
    path = []
    while current is not None:
        path.append(current)
        current = parent_map.get(current)
    return path[::-1]

def heuristic(node, goals, nodes):
    x1, y1 = nodes[node] # Get current node coordinates
    best = float("inf") # Start with a very large distance

    for goal in goals: # Check distance to each goal
        x2, y2 = nodes[goal]
        dist = int(math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)) # Euclidean distance, rounded down like Week 4
        if dist < best:
            best = dist # Keep the smallest distance

    return best

# =========================
# Depth-First Search section
# =========================

def dfs_search(edges, start, goals): 
    stack = [(start, [start])] # Stack stores (current node, path taken)
    visited = [] # List of nodes we have already checked
    nodes_created = 0 # Counter
    
    while stack: # Keep searching while there are nodes in the stack
        current, path = stack.pop() # Take the last node added to the stack
        nodes_created += 1
        
        if current in goals: 
            return current, nodes_created, path # Return results
            
        if current not in visited:
            visited.append(current) # Mark this node as checked
            # smaller numbers are picked first
            neighbors = sorted(edges.get(current, []), reverse=True) 
            for n in neighbors:
                if n not in visited:
                    stack.append((n, path + [n])) # Add next nodes to the stack
                    
    return None, nodes_created, None # Return nothing if no path found

# ==============================
# Greedy Best-First Search section
# ==============================

def greedy_best_first_search(nodes, edges, start, goals):
    frontier = [] # Priority queue storing nodes to explore
    visited = set() # Set of nodes already explored
    frontier_nodes = set() # Nodes already sitting in the frontier
    nodes_created = 1 # Counter for created nodes, starting with the start node
    order = 0 # To keep track of the order of node creation

    heapq.heappush(
        frontier,
        (heuristic(start, goals, nodes), int(start), order, start, [start])
    ) # Add the start node with its heuristic value
    frontier_nodes.add(start) # Mark the start node as being in the frontier

    while frontier: # Keep searching while there are nodes in the frontier
        h_value, node_num, insert_order, current, path = heapq.heappop(frontier)
        frontier_nodes.discard(current) # Remove current node from the frontier set

        if current in goals:
            return current, nodes_created, path # Return results if goal is found

        if current not in visited:
            visited.add(current) # Mark current node as explored

            # Sort neighbours numerically in ascending order
            neighbors = sorted(edges.get(current, []), key=lambda x: int(x[0]))

            for neighbor, cost in neighbors:
                # Only add if not already expanded and not already waiting in frontier
                if neighbor not in visited and neighbor not in frontier_nodes:
                    order += 1
                    heapq.heappush(
                        frontier,
                        (heuristic(neighbor, goals, nodes), int(neighbor), order, neighbor, path + [neighbor])
                    )
                    frontier_nodes.add(neighbor)
                    nodes_created += 1

    return None, nodes_created, None # Return nothing if no path is found

# =========================
# Breadth First Search section
# =========================

def bfs_search(edges, start, goals):
    frontier = deque([(start, [start])])  # queue stores (current node, path)
    visited = {start}                     # mark visited when enqueued
    nodes_created = 1                     # Start node counts as created

    while frontier:
        current, path = frontier.popleft()  # first in first out for breadth first expansion

        if current in goals:
            return current, nodes_created, path

        # Expand smaller node numbers first
        neighbors = sorted(edges.get(current, []), key=lambda x: int(x[0]))

        for neighbor, cost in neighbors:
            if neighbor not in visited:
                visited.add(neighbor)
                frontier.append((neighbor, path + [neighbor]))
                nodes_created += 1

    return None, nodes_created, None


# =========================
# A* Search (AS)
# =========================

def astar_search(nodes, edges, start, goals):
    
    goals = set(goals)

    # g-best and parents for reconstruction
    g_best = {start: 0}
    parent = {start: None}

    # (f, node_id_int, insertion_order, node_str, g_snapshot)
    frontier = []
    order = 0

    h0 = heuristic(start, goals, nodes)
    heapq.heappush(frontier, (h0, int(start), order, start, 0))
    order += 1

    nodes_created = 1  # count unique nodes discovered

    while frontier:
        f, _nid_int, _ord, current, g_snap = heapq.heappop(frontier)

        # skip stale entries
        if g_snap != g_best.get(current, None):
            continue

        if current in goals:
            return current, nodes_created, reconstruct_path(parent, current)

        # expand neighbors in ascending node order
        neighbors = sorted(edges.get(current, []), key=lambda x: int(x[0]))
        for neighbor, cost in neighbors:
            new_g = g_snap + cost

            # relax
            if new_g < g_best.get(neighbor, float("inf")):
                first_time = neighbor not in g_best
                g_best[neighbor] = new_g
                parent[neighbor] = current

                if first_time:
                    nodes_created += 1
                    

                new_f = new_g + heuristic(neighbor, goals, nodes)
                heapq.heappush(frontier, (new_f, int(neighbor), order, neighbor, new_g))
                order += 1

    return None, nodes_created, None

# =========================
# Custom Algorithm (CUS2)
# Bidirectional + alternating expansions + dynamic heuristic target
# =========================

def cus2_bidirectional_astar(nodes, edges, start, goals, w=1.9, beta=0.85, starve_ratio=3.0):
    """
    Bidirectional Weighted A*-style search with dynamic heuristic targets.

    - Forward search from start
    - Backward search from ALL goals using reverse edges
    - Alternates expansions front and backward, but breaks alternation if one side is 'starving'
    - Each side's heuristic aims partly at its fixed end and partly at the other frontier's best node

    Returns: (goal_reached, nodes_created, path_list)
    Not guaranteed optimal, but usually faster than plain A*.
    """
    goals = set(goals)
    goal_list = sorted(goals, key=lambda x: int(x))

    # Distance (same style as your heuristic: floored Euclidean)
    def dist(a, b):
        x1, y1 = nodes[a]
        x2, y2 = nodes[b]
        return int(math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2))

    # Reverse adjacency for backward search (handles one-way edges correctly)
    rev = {}
    for u, nbrs in edges.items():
        for v, c in nbrs:
            rev.setdefault(v, []).append((u, c))
    for v in rev:
        rev[v].sort(key=lambda t: int(t[0]))

    # Fixed heuristics
    def h_to_closest_goal(n):
        return heuristic(n, goals, nodes)

    def h_to_start(n):
        return dist(n, start)

    # Mixed heuristic: fixed end + dynamic target
    def h_forward(n, dyn_target):
        return int((1.0 - beta) * h_to_closest_goal(n) + beta * dist(n, dyn_target))

    def h_backward(n, dyn_target):
        return int((1.0 - beta) * h_to_start(n) + beta * dist(n, dyn_target))

    # PQ helpers (skip stale)
    def peek_best(open_heap, g_best):
        while open_heap:
            f, _nid_int, _ord, n, g_snap = open_heap[0]
            if g_snap != g_best.get(n, None):
                heapq.heappop(open_heap)
                continue
            return n, f, g_snap
        return None, float("inf"), None

    def pop_best(open_heap, g_best):
        while open_heap:
            f, _nid_int, _ord, n, g_snap = heapq.heappop(open_heap)
            if g_snap != g_best.get(n, None):
                continue
            return n, f, g_snap
        return None, None, None

    # Forward side
    gF = {start: 0}
    parentF = {start: None}
    openF = []
    ordF = 0

    # Backward side (multi-source)
    gB = {}
    parentB = {}  # parentB[p] = next node toward goal (in original direction)
    openB = []
    ordB = 0

    # Created count = unique nodes discovered across both searches
    created_set = set([start])

    # init forward
    heapq.heappush(openF, (w * h_to_closest_goal(start), int(start), ordF, start, 0))
    ordF += 1

    # init backward from all goals
    for g in goal_list:
        gB[g] = 0
        parentB[g] = None
        created_set.add(g)
        heapq.heappush(openB, (w * h_to_start(g), int(g), ordB, g, 0))
        ordB += 1

    mu = float("inf")
    meet = None

    forward_turn = True
    max_iters = 600000
    it = 0

    while it < max_iters:
        it += 1
        if not openF and not openB:
            break

        bestF, bestFf, _ = peek_best(openF, gF)
        bestB, bestBf, _ = peek_best(openB, gB)

        if bestF is None and bestB is None:
            break

        # Stop if we already have a complete path and both frontiers can't beat it
        if mu < float("inf") and bestFf >= mu and bestBf >= mu:
            break

        # decide which side to expand: alternate unless starving
        if bestF is None:
            expand_forward = False
        elif bestB is None:
            expand_forward = True
        else:
            if len(openF) * starve_ratio < len(openB):
                expand_forward = True
            elif len(openB) * starve_ratio < len(openF):
                expand_forward = False
            else:
                expand_forward = forward_turn
                forward_turn = not forward_turn

        if expand_forward:
            u, _, gu = pop_best(openF, gF)
            if u is None:
                continue

            # meeting check
            if u in gB:
                cand = gu + gB[u]
                if cand < mu:
                    mu = cand
                    meet = u

            # dynamic target = best backward open node, fallback to a goal
            dynB, _, _ = peek_best(openB, gB)
            if dynB is None:
                dynB = goal_list[0]

            for v, c in sorted(edges.get(u, []), key=lambda x: int(x[0])):
                new_g = gu + c
                if new_g < gF.get(v, float("inf")):
                    gF[v] = new_g
                    parentF[v] = u
                    if v not in created_set:
                        created_set.add(v)

                    # update meet
                    if v in gB:
                        cand = new_g + gB[v]
                        if cand < mu:
                            mu = cand
                            meet = v

                    fv = new_g + w * h_forward(v, dynB)
                    if fv < mu:
                        heapq.heappush(openF, (fv, int(v), ordF, v, new_g))
                        ordF += 1

        else:
            u, _, gu = pop_best(openB, gB)
            if u is None:
                continue

            if u in gF:
                cand = gF[u] + gu
                if cand < mu:
                    mu = cand
                    meet = u

            dynF, _, _ = peek_best(openF, gF)
            if dynF is None:
                dynF = start

            for p, c in rev.get(u, []):
                new_g = gu + c
                if new_g < gB.get(p, float("inf")):
                    gB[p] = new_g
                    parentB[p] = u
                    if p not in created_set:
                        created_set.add(p)

                    if p in gF:
                        cand = gF[p] + new_g
                        if cand < mu:
                            mu = cand
                            meet = p

                    fp = new_g + w * h_backward(p, dynF)
                    if fp < mu:
                        heapq.heappush(openB, (fp, int(p), ordB, p, new_g))
                        ordB += 1

    if meet is None:
        return None, len(created_set), None

    # reconstruct start -> meet (forward parents)
    pathF = reconstruct_path(parentF, meet)

    # reconstruct meet -> goal (follow parentB until hitting a goal)
    tail = []
    cur = meet
    if cur not in parentB and cur not in goals:
        return None, len(created_set), None

    while cur not in goals:
        cur = parentB.get(cur)
        if cur is None:
            return None, len(created_set), None
        tail.append(cur)

    reached_goal = cur
    return reached_goal, len(created_set), pathF + tail

# =========================
# Main
# =========================
def main():
    if len(sys.argv) < 3: # Check if user typed enough arguments
        print("Error: Please type: python search.py <file> <method>")
        return
        
    fname, method = sys.argv[1], sys.argv[2].upper() # Get filename and search method
    nodes, edges, origin, destinations = parse_file(fname) # Get data from file
    
    if method == "DFS": # Run only if user asks for DFS
        goal, count, path = dfs_search(edges, origin, destinations) # Start search
        if goal:
            print(f"{fname} {method}")
            print(f"{goal} {count}")
            print(" ".join(path))
        else:
            print("No path found.")

    elif method == "GBFS": # Run Greedy Best-First Search
        goal, count, path = greedy_best_first_search(nodes, edges, origin, destinations)
        if goal:
            print(f"{fname} {method}")
            print(f"{goal} {count}")
            print(" ".join(path))
        else:
            print("No path found.")

    elif method == "BFS":
        goal, count, path = bfs_search(edges, origin, destinations)
        if goal:
            print(f"{fname} {method}")
            print(f"{goal} {count}")
            print(" ".join(path))
        else:
            print("No path found.")

    elif method == "AS":
        goal, count, path = astar_search(nodes, edges, origin, destinations)
        if goal:
            print(f"{fname} {method}")
            print(f"{goal} {count}")
            print(" ".join(path))
        else:
            print("No path found.")

    elif method == "CUS2":
        goal, count, path = cus2_bidirectional_astar(nodes, edges, origin, destinations)
        if goal:
            print(f"{fname} {method}")
            print(f"{goal} {count}")
            print(" ".join(path))
        else:
            print("No path found.")

    else:
        print("Error: method must be DFS, GBFS or BFS")

if __name__ == "__main__":
    main()
