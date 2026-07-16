import networkx as nx

def find_best_insertion_point(matrix: list, current_route: list, new_stop_index: int):
    """
    Finds the cheapest place to insert a new on-demand order into an active route.
    matrix: The 2D array of travel times between all points.
    current_route: List of stop indices, e.g., [0, 4, 2, 7] (where 0 is the driver's current location).
    """
    best_cost = float('inf')
    best_index = -1
    
    # We start checking from index 1, because index 0 is where the driver currently is 
    # (you can't insert a stop in the past or interrupt their current driving leg).
    for i in range(1, len(current_route)):
        stop_a = current_route[i - 1]
        stop_b = current_route[i]
        
        # Calculate the "detour" cost: 
        # Time to go (A -> New -> B) MINUS the original time (A -> B)
        detour_cost = (matrix[stop_a][new_stop_index] + 
                       matrix[new_stop_index][stop_b] - 
                       matrix[stop_a][stop_b])
        
        if detour_cost < best_cost:
            best_cost = detour_cost
            best_index = i
            
    # Also check appending it to the very end of the route
    end_cost = matrix[current_route[-1]][new_stop_index]
    if end_cost < best_cost:
        best_index = len(current_route)
        
    # Return the modified route
    new_route = current_route.copy()
    new_route.insert(best_index, new_stop_index)
    
    return new_route

# Example: current_route = [0, 1, 2], new_stop = 3
# If inserting between 1 and 2 is cheapest, returns [0, 1, 3, 2]