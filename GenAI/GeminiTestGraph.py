import osmnx as ox
import networkx as nx
import random
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# ==========================================
# 1. PARCEL GENERATION & ROAD SNAPPING
# ==========================================
def generate_and_snap_parcels(G, num_parcels=5):
    """Generates random coordinates in Sungai Long and snaps them to the nearest road."""
    # Bounding box for Bandar Sungai Long
    north, south = 3.0550, 3.0250
    east, west = 101.8150, 101.7900
    
    # Depot is always Node 0
    # Let's set UTAR Sungai Long roughly as the depot coordinate
    depot_lat, depot_lng = 3.0400, 101.7950
    
    # Note: OSMnx expects coordinates in (Longitude, Latitude) order for the nearest_nodes function
    lats = [depot_lat]
    lngs = [depot_lng]
    demands = [0] # Depot has 0 weight
    
    print("Generating random parcels...")
    for _ in range(num_parcels):
        lats.append(random.uniform(south, north))
        lngs.append(random.uniform(west, east))
        demands.append(round(random.uniform(2.0, 15.0))) # Random weight 2kg-15kg
        
    print("Snapping GPS coordinates to the physical road network...")
    # This instantly finds the closest valid road intersections to your random points
    road_nodes = ox.distance.nearest_nodes(G, X=lngs, Y=lats)
    
    return road_nodes, demands

# ==========================================
# 2. THE REAL DISTANCE MATRIX BUILDER
# ==========================================
def build_real_distance_matrix(G, road_nodes):
    """Calculates the physical driving distance between all points using the OSM map."""
    print(f"Building {len(road_nodes)}x{len(road_nodes)} physical route matrix...")
    matrix = []
    
    for origin in road_nodes:
        row = []
        for destination in road_nodes:
            if origin == destination:
                row.append(0)
            else:
                try:
                    # Calculate the physical road distance using the 'length' attribute of the edges
                    distance = nx.shortest_path_length(G, source=origin, target=destination, weight='length')
                    
                    # OR-Tools STRICTLY requires integers. We round the meters to the nearest whole number.
                    row.append(int(round(distance)))
                except nx.NetworkXNoPath:
                    # If two roads are completely disconnected (e.g., a gated community with no exit), penalize heavily
                    row.append(999999)
        matrix.append(row)
        
    return matrix

# ==========================================
# 3. OR-TOOLS CVRP SOLVER
# ==========================================
def solve_cvrp(distance_matrix, demands, num_vehicles, vehicle_capacities):
    """Runs the Google OR-Tools optimization engine."""
    # 1. Initialize the Routing Index Manager and Model
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicles, 0)
    routing = pywrapcp.RoutingModel(manager)

    # 2. Define the Distance Callback
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # 3. Define the Weight/Capacity Constraints
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacities,
        True,  # start cumul to zero
        'Capacity'
    )

    # 4. Set Parameters and Solve
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.time_limit.seconds = 10 # Prevent infinite loops

    print("Running Google OR-Tools Optimizer...")
    solution = routing.SolveWithParameters(search_parameters)

    # 5. Output the Results
    if solution:
        total_distance = 0
        for vehicle_id in range(num_vehicles):
            index = routing.Start(vehicle_id)
            plan_output = f'\nVehicle {vehicle_id} (Max Cap: {vehicle_capacities[vehicle_id]}kg):\nRoute:\n'
            route_dist = 0
            route_load = 0
            
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                route_load += demands[node_index]
                plan_output += f' Node {node_index} Load({route_load}kg) -> '
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_dist += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
                
            plan_output += f' Depot\nDistance: {route_dist} meters'
            print(plan_output)
            total_distance += route_dist
            
        print(f'\nTotal Fleet Distance Driven: {total_distance} meters')
    else:
        print("No mathematical solution found. You may need more vehicles or higher capacity.")

# ==========================================
# MAIN EXECUTION THREAD
# ==========================================
if __name__ == '__main__':
    map_file = "data/sungai_long_driving.graphml"
    
    print(f"Loading local map from {map_file}...")
    # Load the map strictly as a directed graph so NetworkX can route it properly
    G = ox.load_graphml(map_file)
    
    # 1. Generate 5 random parcels and map them to physical roads
    nodes, demands = generate_and_snap_parcels(G, num_parcels=8)
    
    # 2. Build the exact physical driving matrix
    matrix = build_real_distance_matrix(G, nodes)
    
    # 3. Configure your fleet (e.g., 2 Motorcycles at 30kg capacity each)
    fleet_size = 2
    fleet_capacities = [30, 30] 
    
    # 4. Execute the routing!
    solve_cvrp(matrix, demands, fleet_size, fleet_capacities)