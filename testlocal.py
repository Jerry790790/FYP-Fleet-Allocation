from pyrosm import OSM

# demo sample, ignore this file

# 1. Initialize the parser with your exact binary file
file_path = "data/malaysia-singapore-brunei-260715.osm.pbf"
print(f"Reading {file_path}...")

# 2. Pyrosm reads the binary data
osm = OSM(file_path)

# 3. Extract only the driving network (ignores walking paths, buildings, etc.)
print("Extracting driving network nodes and edges...")
nodes, edges = osm.get_network(network_type="driving", nodes=True)

# 4. Convert it directly into a NetworkX graph (the same format OSMnx uses)
print("Building the graph matrix...")
G = osm.to_graph(nodes, edges, graph_type="networkx")

print(f"Success! Graph loaded with {len(G.nodes)} nodes.")