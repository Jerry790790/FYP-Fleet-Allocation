import osmnx as ox
import pyrosm

# Convert downloaded osm file into graphml format. Treat this like updating
# once in a while because the osm file is updated every few months. This is a one-time operation.
file_path = "data/malaysia-singapore-brunei-260715.osm.pbf"
output_path = "data/sungai_long_driving.graphml"

# Scope down the area to a small bounding box for Bandar Sungai Long, Malaysia. 
# This is a small area, so the graph will be very small and fast to load.
# Format for Pyrosm: [min_longitude, min_latitude, max_longitude, max_latitude]
# (West, South, East, North)
sungai_long_bbox = [101.7900, 3.0250, 101.8150, 3.0550]

print("1. Initializing local parser for the Sungai Long boundary...")
osm = pyrosm.OSM(file_path, bounding_box=sungai_long_bbox)

print("2. Extracting the local driving network from the binary file...")
nodes, edges = osm.get_network(network_type="driving", nodes=True)

print("3. Building the NetworkX graph matrix...")
G = osm.to_graph(nodes, edges, graph_type="networkx")

print(f"4. Saving optimized spatial graph with {len(G.nodes)} road intersections...")
ox.save_graphml(G, filepath=output_path)

print(f"\n=== SUCCESS! Map pre-baked successfully to {output_path} ===")