from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve_cvrp():
    data = {}
    # Time/Distance Matrix between 5 points
    data["distance_matrix"] = [
        [0, 15, 20, 25, 30],  # 0: Depot
        [15, 0, 10, 20, 25],  # 1: Stop 1
        [20, 10, 0, 12, 18],  # 2: Stop 2
        [25, 20, 12, 0, 10],  # 3: Stop 3
        [30, 25, 18, 10, 0],  # 4: Stop 4
    ]
    # Cargo weight requirements for each stop (Depot requires 0kg)
    data["demands"] = [0, 150, 200, 300, 100] 
    
    data["num_vehicles"] = 2          # 2 active drivers available
    data["vehicle_capacities"] = [400, 400]  # Each vehicle can carry a max of 400kg
    data["depot"] = 0

    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )
    routing = pywrapcp.RoutingModel(manager)

    # Distance/Time Callback
    def distance_callback(from_index, to_index):
        return data["distance_matrix"][manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Demand/Weight Callback
    def demand_callback(from_index):
        return data["demands"][manager.IndexToNode(from_index)]
    
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    
    # Add Capacity Constraints to the solver
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities list
        True,  # start cumul to zero
        "Capacity"
    )

    # Search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.seconds = 2 # Prevent infinite mathematical loops

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        for vehicle_id in range(data["num_vehicles"]):
            index = routing.Start(vehicle_id)
            plan_output = f"Route for Vehicle {vehicle_id}:\n"
            route_dist = 0
            route_load = 0
            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                route_load += data["demands"][node]
                plan_output += f" {node} (Load: {route_load}kg) ->"
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_dist += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            plan_output += f" {manager.IndexToNode(index)}\n"
            plan_output += f"Distance: {route_dist}m | Total Route Load: {route_load}kg\n"
            print(plan_output)

if __name__ == "__main__":
    solve_cvrp();