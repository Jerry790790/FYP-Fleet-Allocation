"""
Sungai Long Fleet Route Planner - pure networkx version (no osmnx)
--------------------------------------------------------------------
1. Loads a road network exported to GraphML by build_graph.py (pyrosm + networkx).
2. Randomly drops NUM_LOCATIONS stops anywhere inside that network's extent.
3. Orders the stops with a Greedy Nearest-Neighbour heuristic (network
   shortest-path distance, not straight-line distance).
4. Prints the resulting route with expected finish time, total distance,
   and estimated fuel cost.

Requirements: networkx only
    pip install networkx
"""

import random
from datetime import datetime, timedelta
from math import radians, sin, cos, sqrt, atan2

import networkx as nx

# ---------------------------------------------------------------------------
# CONFIG - adjust these to match your setup
# ---------------------------------------------------------------------------
GRAPHML_PATH = "data/sungai_long_driving_claude.graphml"
NUM_LOCATIONS = 10
START_TIME = datetime(2026, 7, 16, 8, 0)

FUEL_CONSUMPTION_L_PER_100KM = 8.0       # avg fuel consumption of the vehicle
FUEL_PRICE_PER_LITRE = 2.05              # RM/litre (RON95) - adjust as needed
DEFAULT_SPEED_KMH = 30                    # fallback speed for edges with no usable speed data

RANDOM_SEED = 42                          # set to None for a different route each run

# Rough typical speeds (km/h) by OSM 'highway' road class, used when a
# usable 'maxspeed' tag isn't present on an edge.
HWY_SPEEDS_KMH = {
    "motorway": 90, "motorway_link": 60,
    "trunk": 80, "trunk_link": 50,
    "primary": 60, "primary_link": 40,
    "secondary": 50, "secondary_link": 40,
    "tertiary": 40, "tertiary_link": 30,
    "unclassified": 30,
    "residential": 30,
    "living_street": 20,
    "service": 20,
}


# ---------------------------------------------------------------------------
# 1. LOAD THE GRAPH
# ---------------------------------------------------------------------------
def parse_maxspeed(value):
    """Try to extract a numeric km/h value from a maxspeed tag; None if unparseable."""
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    s = str(value).strip().lower()
    if s.startswith("["):  # e.g. "['30', '50']" - take the first value
        s = s.strip("[]").split(",")[0].strip(" '\"")
    s = s.replace("km/h", "").replace("kmh", "").strip()
    try:
        return float(s)
    except ValueError:
        return None


def infer_speed_kph(edge_data, default_speed):
    speed = parse_maxspeed(edge_data.get("maxspeed"))
    if speed:
        return speed
    highway = edge_data.get("highway")
    if isinstance(highway, list):  # some edges tag multiple highway values
        highway = highway[0] if highway else None
    if highway in HWY_SPEEDS_KMH:
        return HWY_SPEEDS_KMH[highway]
    return default_speed


def load_graph(path):
    print(f"Loading graph from '{path}' ...")
    # force_multigraph-style guarantee: read directly into a MultiDiGraph so
    # parallel/one-way edges behave consistently regardless of how the file
    # was written
    G = nx.read_graphml(path, node_type=int, force_multigraph=True)
    if not G.is_directed() or not G.is_multigraph():
        G = nx.MultiDiGraph(G)

    # Defensive type-casting: GraphML round-trips can leave numeric fields
    # as strings depending on which tool wrote the file
    for _, data in G.nodes(data=True):
        data["x"] = float(data["x"])
        data["y"] = float(data["y"])

    for _, _, data in G.edges(data=True):
        try:
            data["length"] = float(data.get("length", 0) or 0)
        except (TypeError, ValueError):
            data["length"] = 0.0

        speed_kph = infer_speed_kph(data, DEFAULT_SPEED_KMH)
        data["speed_kph"] = speed_kph
        data["travel_time"] = (
            data["length"] / (speed_kph * 1000 / 3600) if speed_kph > 0 else 0.0
        )

    print(f"Graph loaded: {len(G.nodes)} nodes, {len(G.edges)} edges")
    return G


# ---------------------------------------------------------------------------
# 2. GENERATE RANDOM LOCATIONS WITHIN THE LOADED NETWORK'S EXTENT
# ---------------------------------------------------------------------------
def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000  # metres
    phi1, phi2 = radians(lat1), radians(lat2)
    dphi = radians(lat2 - lat1)
    dlambda = radians(lon2 - lon1)
    a = sin(dphi / 2) ** 2 + cos(phi1) * cos(phi2) * sin(dlambda / 2) ** 2
    return 2 * R * atan2(sqrt(a), sqrt(1 - a))


def nearest_node(G, lat, lon):
    """Brute-force nearest node by straight-line distance. Fine for a small
    extract like Sungai Long; swap for a KD-tree if the graph gets large."""
    best_node, best_dist = None, float("inf")
    for node, data in G.nodes(data=True):
        d = haversine_m(lat, lon, data["y"], data["x"])
        if d < best_dist:
            best_dist, best_node = d, node
    return best_node


def generate_random_locations(G, n, seed=None):
    rng = random.Random(seed)

    lats = [data["y"] for _, data in G.nodes(data=True)]
    lons = [data["x"] for _, data in G.nodes(data=True)]
    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons), max(lons)

    locations, seen = [], set()
    attempts = 0
    while len(locations) < n and attempts < n * 50:
        attempts += 1
        lat = rng.uniform(min_lat, max_lat)
        lon = rng.uniform(min_lon, max_lon)
        node_id = nearest_node(G, lat, lon)
        if node_id in seen:
            continue
        seen.add(node_id)
        locations.append(node_id)

    if len(locations) < n:
        raise RuntimeError("Could not find enough distinct nodes - try a bigger graph area")

    return locations


# ---------------------------------------------------------------------------
# 3. GREEDY NEAREST-NEIGHBOUR ORDERING
# ---------------------------------------------------------------------------
def greedy_order(G, node_ids):
    remaining = node_ids[1:]
    route = [node_ids[0]]
    current = node_ids[0]

    while remaining:
        best_node, best_dist = None, float("inf")
        for candidate in remaining:
            try:
                d = nx.shortest_path_length(G, current, candidate, weight="length")
            except nx.NetworkXNoPath:
                d = float("inf")
            if d < best_dist:
                best_dist, best_node = d, candidate
        route.append(best_node)
        remaining.remove(best_node)
        current = best_node

    return route


# ---------------------------------------------------------------------------
# 4. BUILD THE FULL ROUTE AND COMPUTE METRICS
# ---------------------------------------------------------------------------
def edge_attr_min(G, u, v, attr):
    """Smallest value of `attr` among parallel edges u->v (MultiDiGraph-safe)."""
    data = G.get_edge_data(u, v)
    return min(d.get(attr, 0) for d in data.values())


def evaluate_route(G, ordered_nodes):
    legs = []
    total_distance_m = 0.0
    total_time_s = 0.0

    for a, b in zip(ordered_nodes[:-1], ordered_nodes[1:]):
        path = nx.shortest_path(G, a, b, weight="length")
        leg_distance_m = nx.shortest_path_length(G, a, b, weight="length")
        leg_time_s = sum(
            edge_attr_min(G, u, v, "travel_time")
            for u, v in zip(path[:-1], path[1:])
        )
        legs.append(
            {"from": a, "to": b, "distance_m": leg_distance_m, "time_s": leg_time_s}
        )
        total_distance_m += leg_distance_m
        total_time_s += leg_time_s

    return legs, total_distance_m, total_time_s


# ---------------------------------------------------------------------------
# 5. REPORT
# ---------------------------------------------------------------------------
def print_report(G, ordered_nodes, legs, total_distance_m, total_time_s):
    def coords(node):
        return G.nodes[node]["y"], G.nodes[node]["x"]

    print("\n=== ROUTE (Greedy Nearest-Neighbour) ===")
    lat, lon = coords(ordered_nodes[0])
    print(f"Start (Stop 1): node {ordered_nodes[0]}  ({lat:.6f}, {lon:.6f})")

    for i, leg in enumerate(legs, start=1):
        lat, lon = coords(leg["to"])
        print(
            f"  -> Stop {i + 1}: node {leg['to']}  ({lat:.6f}, {lon:.6f})  "
            f"| leg distance: {leg['distance_m'] / 1000:.2f} km "
            f"| leg time: {leg['time_s'] / 60:.1f} min"
        )

    total_distance_km = total_distance_m / 1000
    fuel_used_l = total_distance_km * FUEL_CONSUMPTION_L_PER_100KM / 100
    fuel_cost = fuel_used_l * FUEL_PRICE_PER_LITRE
    finish_time = START_TIME + timedelta(seconds=total_time_s)

    print("\n=== SUMMARY ===")
    print(f"Departure time     : {START_TIME.strftime('%Y-%m-%d %H:%M')}")
    print(f"Expected finish time: {finish_time.strftime('%Y-%m-%d %H:%M')}")
    print(f"Total travel time   : {total_time_s / 60:.1f} min")
    print(f"Total distance      : {total_distance_km:.2f} km")
    print(f"Fuel used           : {fuel_used_l:.2f} L")
    print(f"Estimated fuel cost : RM {fuel_cost:.2f}")


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------
def main():
    G = load_graph(GRAPHML_PATH)
    stops = generate_random_locations(G, NUM_LOCATIONS, seed=RANDOM_SEED)
    ordered_stops = greedy_order(G, stops)
    legs, total_distance_m, total_time_s = evaluate_route(G, ordered_stops)
    print_report(G, ordered_stops, legs, total_distance_m, total_time_s)


if __name__ == "__main__":
    main()