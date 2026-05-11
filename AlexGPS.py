import csv
import math
from collections import deque

# =============================Loading the Roads file====================================
def load_roads(roads_file):
    rgraph = {}
    with open(roads_file, mode='r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            s = row["NodeA"]
            e = row["NodeB"]
            dist = float(row["Distance_Km"])
            speed = int(row["Speed_Limit"])
            travel_time = (dist / speed) * 60
            if s not in rgraph: rgraph[s] = []
            if e not in rgraph: rgraph[e] = []
            rgraph[s].append({"node": e, "time": travel_time, "distance": dist})
            rgraph[e].append({"node": s, "time": travel_time, "distance": dist})
    return rgraph
#==============================Loading the heuristic==========================

def load_heuristics(heuristic_file):
    hgraph = {}
    with open(heuristic_file, mode='r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            hgraph[row["Name"]] = (float(row["X_Coord"]), float(row["Y_Coord"]))
    return hgraph


#============================Implementing The Euclidean distance formula to get the heuristic value======================
def heuristic_value(NodeA, NodeB, coords):
    if NodeA not in coords or NodeB not in coords:
        return 0
    x1, y1 = coords[NodeA]
    x2, y2 = coords[NodeB]
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)*111

# =========================A star========================================================
def Astar(graph, start, goal, coords):
    expanded_nodes = 0 
    queue = [[(start, 0, 0)]]
    visited = []

    while queue:
        queue.sort(key=lambda path: path[-1][1] + heuristic_value(path[-1][0], goal, coords))
        
        path = queue.pop(0)
        node, g, d = path[-1]
        
        if node in visited: 
            continue
        expanded_nodes += 1
        visited.append(node)
        
        if node == goal:
            station_names = [step[0] for step in path]
            return station_names, g, d, expanded_nodes
        
        for neighbor in graph.get(node, []):
            if neighbor['node'] not in visited:
                new_path = list(path)
                new_path.append((neighbor['node'], g + neighbor['time'], d + neighbor['distance']))
                queue.append(new_path)
    return None, 0, 0, expanded_nodes

# =======================BFS===================================================================
def bfs_path(graph, start, goal):
    visited = {start}
    queue = deque([[start]])
    expanded_nodes = 0
    while queue:
        path = queue.popleft()
        current_node = path[-1]
        expanded_nodes += 1
        if current_node == goal:
            return path, expanded_nodes
        if current_node in graph:
            for neighbor_dict in graph[current_node]:
                neighbor = neighbor_dict['node']
                if neighbor not in visited:
                    visited.add(neighbor)
                    new_path = list(path)
                    new_path.append(neighbor)
                    queue.append(new_path)
    return None, expanded_nodes

# -----------------
alex_roads = load_roads("Roads.csv")
alex_coords = load_heuristics("Heuristic_data.csv")

queries = [
    ("Alagami", "Abo_Qeer"),
    ("Miami", "Almanshia"),
    ("Sidi_Gaber", "Sanstefano"),
    ("Almalaha", "Kafr_abdo"),
    ("Bacos", "Alibrahimia"),
    ("Miami", "Smouha")
]

comparison_data = []

print("===  DETAILED NAVIGATION RESULTS ===\n")
for start_node, end_node in queries:
    # Get results from both algorithms
    a_path, a_time, a_dist, a_nodes = Astar(alex_roads, start_node, end_node, alex_coords)
    b_path, b_nodes = bfs_path(alex_roads, start_node, end_node)
    
    if a_path and b_path:
        print(f"Query: From {start_node} to {end_node}")
        
        # Print A* Result
        print(f"  [A* Path]  : {' -> '.join(a_path)}")
        
        # Print BFS Result
        print(f"  [BFS Path] : {' -> '.join(b_path)}")
        
        print("-" * 50)
        comparison_data.append([f"{start_node}->{end_node}", a_nodes, b_nodes, f"{a_dist:.2f} km"])
        
# -------------------------------------COMPARISON TABLE----------------------
print("\n=== COMPARISON TABLE ===")
print("-" * 80)
print(f"{'Route':<35} | {'A* Nodes':<12} | {'BFS Nodes':<12} | {'Distance'}")
print("-" * 80)
for row in comparison_data:
    print(f"{row[0]:<35} | {row[1]:<12} | {row[2]:<12} | {row[3]}")
print("-" * 80)
