import networkx as nx
import pyrosm

# Convert downloaded osm file into graphml format. Treat this like updating
# once in a while because the osm file is updated every few months. This is a one-time operation.
file_path = "data/malaysia-singapore-brunei-260715.osm.pbf"
output_path = "data/sungai_long_driving_claude.graphml"

# Scope down the area to a small bounding box for Bandar Sungai Long, Malaysia.
# This is a small area, so the graph will be very small and fast to load.
# Format for Pyrosm: [min_longitude, min_latitude, max_longitude, max_latitude]
# (West, South, East, North)
sungai_long_bbox = [101.7900, 3.0250,101.8150, 3.0550]

print("1. Initializing local parser for the Sungai Long boundary...")
osm = pyrosm.OSM(file_path, bounding_box=sungai_long_bbox)

print("2. Extracting the local driving network from the binary file...")
nodes, edges = osm.get_network(network_type="driving", nodes=True)

print("3. Building the NetworkX graph matrix...")
G = osm.to_graph(nodes, edges, graph_type="networkx")

print("4. Normalizing attribute types (oneway -> bool, length/maxspeed -> numeric)...")
for _, _, data in G.edges(data=True):
    if "oneway" in data:
        data["oneway"] = str(data["oneway"]).strip().lower() in ("yes", "true", "1", "-1")
    if "length" in data:
        try:
            data["length"] = float(data["length"])
        except (TypeError, ValueError):
            data["length"] = 0.0

for _, data in G.nodes(data=True):
    data["x"] = float(data["x"])
    data["y"] = float(data["y"])


def sanitize_attrs(d):
    """GraphML can't serialize None or list/other non-primitive values, and
    pyrosm leaves many optional OSM tags (maxspeed, bridge, tunnel, name...)
    as None when absent. Drop None entries and stringify anything else that
    isn't a plain str/int/float/bool."""
    clean = {}
    for k, v in d.items():
        if v is None:
            continue
        if isinstance(v, (list, tuple, set)):
            v = ";".join(str(x) for x in v)
        elif not isinstance(v, (str, int, float, bool)):
            v = str(v)
        clean[k] = v
    return clean


print("5. Sanitizing attributes (dropping None values, stringifying lists)...")
for _, data in G.nodes(data=True):
    cleaned = sanitize_attrs(dict(data))
    data.clear()
    data.update(cleaned)

for _, _, data in G.edges(data=True):
    cleaned = sanitize_attrs(dict(data))
    data.clear()
    data.update(cleaned)

G.graph.update(sanitize_attrs(dict(G.graph)))

print(f"6. Saving graph with {len(G.nodes)} road intersections to '{output_path}'...")
nx.write_graphml(G, output_path)

print(f"\n=== SUCCESS! Map pre-baked successfully to {output_path} ===")