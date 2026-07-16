from contextlib import asynccontextmanager
from fastapi import FastAPI
import pyrosm

# sample fastapi server, ignore this file

@asynccontextmanager
async def lifespan(app: FastAPI):
    # --- [STARTUP LOGIC] ---
    print("=== [SERVER BOOT] Ingesting OSM Data ===")
    
    file_path = "data/malaysia-singapore-brunei-260715.osm.pbf"
    
    # Define a bounding box covering the greater Selangor, KL, Cheras, and Kajang regions
    # Format: [min_longitude, min_latitude, max_longitude, max_latitude]
    selangor_kl_bbox = [101.4000, 2.7000, 101.9500, 3.4000]
    
    # 1. Initialize the parser ONLY for this box
    osm = pyrosm.OSM(file_path, bounding_box=selangor_kl_bbox)
    
    print("Extracting regional driving network nodes and edges...")
    # 2. This will extract data only within your coordinates
    nodes, edges = osm.get_network(network_type="driving", nodes=True)
    
    print("Building the spatial matrix graph...")
    # 3. This will now complete in seconds instead of freezing
    app.state.graph = osm.to_graph(nodes, edges, graph_type="networkx")
    
    print(f"=== [SERVER READY] Loaded {len(app.state.graph.nodes)} nodes into RAM. ===")
    
    yield
    
    # --- [SHUTDOWN LOGIC] ---
    print("=== [SERVER SHUTDOWN] Cleaning up resources ===")
    del app.state.graph

app = FastAPI(lifespan=lifespan)

@app.get("/calculate-route")
async def get_route(origin_id: int, destination_id: int):
    return {"status": "success", "nodes_in_memory": len(app.state.graph.nodes)}