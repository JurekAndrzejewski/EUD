from pydantic import BaseModel, Field
from typing import List, Optional, Union, Literal

class Position(BaseModel):
    """3D position coordinates"""
    x: float = Field(..., description="X coordinate in meters")
    y: float = Field(..., description="Y coordinate in meters")
    z: float = Field(..., description="Z coordinate in meters")

class MoveToPositionAction(BaseModel):
    """Move robot to specific position"""
    action: Literal["move_to_position"] = "move_to_position"
    position: Position
    duration: Optional[float] = Field(default=2.0, description="Movement duration in seconds")
    name: Optional[str] = Field(default=None, description="Optional name")

class MoveToObjectAction(BaseModel):
    """Move robot above an object"""
    action: Literal["move_to_object"] = "move_to_object"
    object_name: str = Field(..., description="Name of object (red_box, blue_box, drop_bucket)")
    height_offset: Optional[float] = Field(default=0.05, description="Height above object in meters")

class PickObjectAction(BaseModel):
    """Pick an object (complete pick sequence)"""
    action: Literal["pick_object"] = "pick_object"
    object_name: str = Field(..., description="Name of object to pick")

class GraspAction(BaseModel):
    """Close gripper"""
    action: Literal["grasp"] = "grasp"

class ReleaseAction(BaseModel):
    """Open gripper"""
    action: Literal["release"] = "release"

class WaitAction(BaseModel):
    """Wait for specified time"""
    action: Literal["wait"] = "wait"
    seconds: float = Field(..., description="Time to wait in seconds")

class InitializeAction(BaseModel):
    """Initialize robot to home position"""
    action: Literal["initialize"] = "initialize"

class ResetBoxesAction(BaseModel):
    """Reset the positions of red_box and blue_box to their default coordinates"""
    action: Literal["reset_boxes"] = "reset_boxes"

# Union type for all possible actions
RobotAction = Union[
    MoveToPositionAction,
    MoveToObjectAction,
    PickObjectAction,
    GraspAction,
    ReleaseAction,
    WaitAction,
    InitializeAction,
    ResetBoxesAction
]

class Mission(BaseModel):
    """Complete mission containing sequence of actions"""
    mission_id: Optional[str] = Field(default=None, description="Optional mission identifier")
    actions: List[RobotAction] = Field(..., description="Sequence of robot actions to execute")

class MissionStep(BaseModel):
    """Individual step result within a mission"""
    step_number: int = Field(..., description="Step number (1-based)")
    action: str = Field(..., description="Action that was executed")
    status: str = Field(..., description="success or error")
    message: str = Field(..., description="Step result message")
    execution_time: Optional[float] = Field(default=None, description="Time taken for this step in seconds")

class MissionResponse(BaseModel):
    """Response after mission execution"""
    status: str = Field(..., description="success or error")
    message: str = Field(..., description="Status message")
    mission_id: Optional[str] = Field(default=None, description="Mission identifier")
    execution_time: Optional[float] = Field(default=None, description="Total execution time in seconds")
    trajectory_length: Optional[int] = Field(default=None, description="Remaining trajectory waypoints")
    mission_steps: List[MissionStep] = Field(default=[], description="List of completed mission steps")

class MissionsRequest(BaseModel):
    """Batch missions payload containing multiple missions"""
    missions: List[Mission] = Field(..., description="List of missions to execute sequentially")

class MissionsResponse(BaseModel):
    """Batch missions response with per-mission results"""
    status: str = Field(..., description="success or error")
    results: List[MissionResponse] = Field(..., description="Results for each mission in order")
