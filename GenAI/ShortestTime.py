from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve_tsp():
    # 1. Instantiate the data model. 
    # This matrix represents travel times (in seconds) between 4 points.
    data = {}
    data["distance_matrix"] = [
        [0, 240, 360, 450],   # Depot (Hub)
        [240, 0, 180, 300],   # Stop 1
        [360, 180, 0, 210],   # Stop 2
        [450, 300, 210, 0],   # Stop 3
    ]
    data["num_vehicles"] = 1
    data["depot"] = 0  # Index of the start/end location

    # 2. Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # 3. Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # 4. Create and register a transit callback (cost function evaluator).
    def distance_callback(from_index, to_index):
        # Convert routing variable token to matrix node index.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # 5. Define cost of each travel arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # 6. Setting first solution heuristics.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # 7. Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # 8. Print solution on console.
    if solution:
        index = routing.Start(0)
        plan_output = "Optimal Route for Driver:\n"
        route_time = 0
        while not routing.IsEnd(index):
            plan_output += f" {manager.IndexToNode(index)} ->"
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_time += routing.GetArcCostForVehicle(previous_index, index, 0)
        plan_output += f" {manager.IndexToNode(index)}\n"
        print(plan_output)
        print(f"Total Travel Time: {route_time} seconds")

if __name__ == "__main__":
    solve_tsp()