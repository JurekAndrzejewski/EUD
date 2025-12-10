"""
FastAPI application for robot control
API endpoints for robot mission execution
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .models import MissionsRequest, MissionsResponse, MissionResponse, validate_mission
from .mission_service import (
    execute_mission, 
    start_simulation, 
    stop_simulation, 
    get_simulation_status
)
import sys
from pathlib import Path
import requests

# Add parent directory to import robot_api
sys.path.append(str(Path(__file__).parent.parent))

from robot_interface.robot_api import robot

app = FastAPI(
    title="Robot Control API",
    description="API for controlling UR5e robot arm via mission JSON",
    version="0.1.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

LATEST_QUESTIONS = []

def validate(actions):
    names = []
    for action in actions:
        names.append(action.name)

    return validate_mission(names)

@app.get("/questions")
async def get_questions():
    global LATEST_QUESTIONS
    to_return = []
    for key in LATEST_QUESTIONS:
        to_return += LATEST_QUESTIONS[key]
    print(to_return)
    return to_return

@app.get("/clear_questions")
def clear_questions():
    global LATEST_QUESTIONS
    LATEST_QUESTIONS = []
    return []

@app.post("/missions", response_model=MissionsResponse, tags=["Mission Control"])
async def execute_missions(batch: MissionsRequest):
    """
    Execute multiple missions sequentially in a single request.
    Returns a per-mission result list.
    """
    global LATEST_QUESTIONS
    LATEST_QUESTIONS = []
    results: list[MissionResponse] = []
    for mission in batch.missions:
        x=''
        #res = await execute_mission(mission)
        #results.append(res)

    overall_status = "success" if all(r.status == "success" for r in results) else "error"
    generated = validate(batch.missions[0].actions)
    LATEST_QUESTIONS = generated
    return MissionsResponse(status=overall_status, results=results)

@app.get("/object/{object_name}/position", tags=["Objects"])
async def get_object_position(object_name: str):
    """
    Get the current position of an object in the simulation
    
    Args:
        object_name: Name of the object ("red_box", "blue_box", "drop_bucket", "table_top")
    
    Returns:
        JSON with object position or error message
        
    Example:
        GET /object/red_box/position
        Response: {
            "status": "success",
            "position": {"x": 0.3, "y": 0.3, "z": 0.52},
            "object_name": "red_box"
        }
    """
    try:
        # Call the robot API's get_object_position method
        result = robot.get_object_position(object_name)
        
        if result["status"] == "success":
            return {
                "status": "success",
                "position": result["position"],
                "object_name": object_name
            }
        else:
            return {
                "status": "error",
                "message": result["message"],
                "object_name": object_name
            }
            
    except Exception as e:
        return {
            "status": "error",
            "message": f"Failed to get position for {object_name}: {str(e)}",
            "object_name": object_name
        }

@app.post("/objects/reset-boxes", tags=["Objects"])
async def reset_boxes_endpoint():
    """
    Reset the positions of red_box and blue_box to their default coordinates.
    Returns the status dictionary from robot.reset_boxes().
    """
    try:
        result = robot.reset_boxes()
        return result
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.post("/simulation/start", tags=["Simulation Control"])
async def start_simulation_endpoint():
    """Manually start simulation loop"""
    start_simulation()
    return {"status": "success", "message": "Simulation started"}

@app.post("/simulation/stop", tags=["Simulation Control"])
async def stop_simulation_endpoint():
    """Manually stop simulation loop"""
    stop_simulation()
    return {"status": "success", "message": "Simulation stopped"}

@app.get("/status", tags=["Status"])
async def get_program_status():
    """Get current robot and simulation status"""
    return get_simulation_status()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
