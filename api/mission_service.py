"""
Mission execution service
Handles robot mission execution logic separate from API endpoints
"""
import asyncio
import time
import threading
from typing import Dict, Any
import sys
from pathlib import Path

# Add parent directory to import robot_api
sys.path.append(str(Path(__file__).parent.parent))

from robot_interface.robot_api import robot
from .models import Mission, MissionResponse, RobotAction, MissionStep

# Global state
simulation_running = False
simulation_thread = None
current_mission = None

def simulation_loop():
    """Background simulation loop - must run continuously for robot movement"""
    global simulation_running
    while simulation_running:
        try:
            # Step the simulation - this is crucial for robot movement
            robot.step_simulation()
            # Sleep for one timestep (usually 0.005s for 200Hz)
            time.sleep(robot.model.opt.timestep)
        except Exception as e:
            print(f"Simulation error: {e}")
            break

def start_simulation():
    """Start the background simulation loop"""
    global simulation_running, simulation_thread
    if not simulation_running:
        robot.launch_viewer()
        robot.reset_boxes()
        robot.initialize()
        simulation_running = True
        simulation_thread = threading.Thread(target=simulation_loop, daemon=True)
        simulation_thread.start()
        print("Simulation loop started")

def stop_simulation():
    """Stop the background simulation loop"""
    global simulation_running
    robot.close_viewer()
    simulation_running = False
    print("Simulation loop stopped")

async def execute_action(action: RobotAction) -> Dict[str, Any]:
    """Execute a single robot action"""
    try:
        if action.action == "initialize":
            result = robot.initialize()

        elif action.action == "reset_boxes":
            result = robot.reset_boxes()
            
        elif action.action == "move_to_position":
            pos = action.position
            result = robot.move_to_position(pos.x, pos.y, pos.z, action.duration)
            
        elif action.action == "move_to_object":
            result = robot.move_to_object(action.object_name, action.height_offset)
            
        elif action.action == "pick_object":
            result = robot.pick_object(action.object_name)
            
        elif action.action == "grasp":
            result = robot.grasp()
            
        elif action.action == "release":
            result = robot.release()
            
        elif action.action == "wait":
            result = robot.wait(action.seconds)
        else:
            result = {"status": "error", "message": f"Unknown action: {action.action}"}
        return result
        
    except Exception as e:
        return {"status": "error", "message": str(e)}

async def execute_mission(mission: Mission):
    """
    Execute a complete robot mission from JSON
    
    The mission JSON should contain:
    - mission_id: Optional identifier
    - actions: List of robot actions to execute in sequence
    """
    global current_mission
    start_time = time.time()
    
    try:
        current_mission = mission
        
        # Ensure simulation is running
        if not simulation_running:
            start_simulation()
        
        # Execute each action in sequence
        mission_steps = []
        for i, action in enumerate(mission.actions):
            step_start_time = time.time()
            try:
                print(f"Executing action {i+1}/{len(mission.actions)}: {action.action} {action.position} {action.name}")
            except:
                print(f"Executing action {i+1}/{len(mission.actions)}: {action.action}")
            
            # Execute the action
            result = await execute_action(action)
            step_execution_time = time.time() - step_start_time
            
            # Create step record
            step = MissionStep(
                step_number=i+1,
                action=action.action,
                status=result["status"],
                message=result["message"],
                execution_time=step_execution_time
            )
            mission_steps.append(step)
            
            # Check if action failed
            if result["status"] == "error":
                print(f"Action {i+1} failed: {result['message']}")
                return MissionResponse(
                    status="error",
                    message=f"Action {i+1} failed: {result['message']}",
                    mission_id=mission.mission_id,
                    execution_time=time.time() - start_time,
                    mission_steps=mission_steps
                )
            
            # Wait for trajectory to complete if it's a movement action
            if action.action in ["move_to_position", "move_to_object", "pick_object"]:
                # Wait for trajectory to complete
                while robot.current_trajectory and len(robot.current_trajectory) > 0:
                    await asyncio.sleep(0.1)  # Check every 100ms
        
        execution_time = time.time() - start_time
        
        return MissionResponse(
            status="success",
            message=f"Mission completed successfully. Executed {len(mission.actions)} actions.",
            mission_id=mission.mission_id,
            execution_time=execution_time,
            trajectory_length=len(robot.current_trajectory) if robot.current_trajectory else 0,
            mission_steps=mission_steps
        )
        
    except Exception as e:
        print(e)
        return MissionResponse(
            status="error",
            message=f"Mission execution failed: {str(e)}",
            mission_id=mission.mission_id,
            execution_time=time.time() - start_time,
            mission_steps=[]
        )

def get_simulation_status():
    """Get current simulation status"""
    return {
        "simulation_running": simulation_running,
        "viewer_running": robot.viewer_running,
        "current_trajectory_length": len(robot.current_trajectory) if robot.current_trajectory else 0,
        "current_mission": current_mission.mission_id if current_mission else None
    }
