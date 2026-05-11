import csv # To load the CSV files
import math # To implement the Euclidean distance formula
from collections import deque #queue library
from tkinter import ttk # implementing the GUI
import tkinter as tk #implementing the GUI

# ---
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

def load_heuristics(heuristic_file):
    hgraph = {}
    with open(heuristic_file, mode='r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            hgraph[row["Name"]] = (float(row["X_Coord"]), float(row["Y_Coord"]))
    return hgraph
#-------
def heuristic_value(NodeA, NodeB, coords):
    if NodeA not in coords or NodeB not in coords:
        return 0
    x1, y1 = coords[NodeA]
    x2, y2 = coords[NodeB]
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# ----- A*------------------
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

# --- BFS ---
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
        
# ---final table----
print("\n=== COMPARISON TABLE ===")
print("-" * 80)
print(f"{'Route':<35} | {'A* Nodes':<12} | {'BFS Nodes':<12} | {'Distance'}")
print("-" * 80)
for row in comparison_data:
    print(f"{row[0]:<35} | {row[1]:<12} | {row[2]:<12} | {row[3]}")
print("-" * 80)

class AstarGui:
    def __init__(self, root, roads, coords):
        self.root = root
        self.roads = roads
        self.coords = coords
        self.locations = sorted(list(self.roads.keys()))

        # Window Setup
        self.root.title("Alexandria Navigation - A* Focus")
        self.root.geometry("800x650")

        # Top Control Panel
        top_panel = tk.Frame(root, pady=10)
        top_panel.pack()

        tk.Label(top_panel, text="Start:").grid(row=0, column=0, padx=5)
        self.start_cb = ttk.Combobox(top_panel, values=self.locations)
        self.start_cb.grid(row=0, column=1)

        tk.Label(top_panel, text="Goal:").grid(row=0, column=2, padx=5)
        self.goal_cb = ttk.Combobox(top_panel, values=self.locations)
        self.goal_cb.grid(row=0, column=3)

        self.btn = tk.Button(top_panel, text="Search & Map", command=self.display, bg="#c0392b", fg="white", font=("Arial", 10, "bold"))
        self.btn.grid(row=0, column=4, padx=15)

        # The Map Canvas
        self.canvas = tk.Canvas(root, width=700, height=450, bg="#fdfefe", highlightthickness=1, relief="ridge")
        self.canvas.pack(pady=10)

        # Bottom Results Panel
        self.stats_label = tk.Label(root, text="Select nodes and press Search", font=("Arial", 11, "italic"), fg="#34495e")
        self.stats_label.pack(pady=5)

        self.draw_base_map()

    def get_canvas_coords(self, node_name):
        raw_x, raw_y = self.coords[node_name]
    
        # 1. Scaling X (Longitude): Stretching it across the width
        # (raw_x - min_x) * scale + offset
        canvas_x = (raw_x - 31.10) * 1800 + 100 
    
        # 2. Scaling Y (Latitude): Inverting it so north is UP
        # height - ((raw_y - min_y) * scale) - offset
        canvas_y = 450 - ((raw_y - 29.90) * 1800 + 100)
        return canvas_x, canvas_y

    def draw_base_map(self):
        self.canvas.delete("all")
        # Draw all roads in light gray
        for node, neighbors in self.roads.items():
            x1, y1 = self.get_canvas_coords(node)
            for n_dict in neighbors:
                x2, y2 = self.get_canvas_coords(n_dict['node'])
                self.canvas.create_line(x1, y1, x2, y2, fill="#ecf0f1", width=1)
        
        # Draw all nodes as small dots
        for loc in self.locations:
            x, y = self.get_canvas_coords(loc)
            self.canvas.create_oval(x-2, y-2, x+2, y+2, fill="#bdc3c7")
            self.canvas.create_text(x, y-8, text=loc, font=("Arial", 7 , "bold"), fill="black")

    def display(self):
        start = self.start_cb.get()
        goal = self.goal_cb.get()

        if start not in self.roads or goal not in self.roads:
            return

        # 1. Run the logic
        a_path, a_time, a_dist, a_nodes = Astar(self.roads, start, goal, self.coords)
        b_path, b_nodes = bfs_path(self.roads, start, goal)

        # 2. Clear the canvas and draw ONLY the gray background roads
        self.canvas.delete("all")
        for node, neighbors in self.roads.items():
            x1, y1 = self.get_canvas_coords(node)
            for n_dict in neighbors:
                x2, y2 = self.get_canvas_coords(n_dict['node'])
                self.canvas.create_line(x1, y1, x2, y2, fill="#ecf0f1", width=1)

        # 3. Draw the A* Path (The Red Line)
        if a_path:
            for i in range(len(a_path)-1):
                x1, y1 = self.get_canvas_coords(a_path[i])
                x2, y2 = self.get_canvas_coords(a_path[i+1])
                self.canvas.create_line(x1, y1, x2, y2, fill="#e74c3c", width=4)

        # 4. DRAW NODES LAST (This puts them on top of the red line)
        for loc in self.locations:
            x, y = self.get_canvas_coords(loc)
            # Use a slightly larger circle for the nodes on the path if you want
            node_color = "green" if loc == start else "red" if loc == goal else "#bdc3c7"
            node_size = 4 if (loc == start or loc == goal) else 2
        
            self.canvas.create_oval(x-node_size, y-node_size, x+node_size, y+node_size, fill=node_color)
            self.canvas.create_text(x, y-10, text=loc, font=("Arial", 7, "bold"), fill="black")

        # 5. Update Label
        result_text = (f"A* Path: {a_dist:.2f}km in {a_time:.2f}m | "
                   f"Nodes Expanded: A* [{a_nodes}] vs BFS [{b_nodes}]")
        self.stats_label.config(text=result_text)

# --- EXECUTION ---
if __name__ == "__main__":
    
    alex_roads = load_roads("Roads.csv")
    alex_coords = load_heuristics("Heuristic_data.csv")

    root = tk.Tk()
    app = AstarGui(root, alex_roads, alex_coords)
    root.mainloop()