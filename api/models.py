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
    name: Optional[str] = Field(default=None, description="Optional name")

class PickObjectAction(BaseModel):
    """Pick an object (complete pick sequence)"""
    action: Literal["pick_object"] = "pick_object"
    object_name: str = Field(..., description="Name of object to pick")
    name: Optional[str] = Field(default=None, description="Optional name")

class GraspAction(BaseModel):
    """Close gripper"""
    action: Literal["grasp"] = "grasp"
    name: Optional[str] = Field(default=None, description="Optional name")

class ReleaseAction(BaseModel):
    """Open gripper"""
    action: Literal["release"] = "release"
    name: Optional[str] = Field(default=None, description="Optional name")

class WaitAction(BaseModel):
    """Wait for specified time"""
    action: Literal["wait"] = "wait"
    seconds: float = Field(..., description="Time to wait in seconds")
    name: Optional[str] = Field(default=None, description="Optional name")

class InitializeAction(BaseModel):
    """Initialize robot to home position"""
    action: Literal["initialize"] = "initialize"
    name: Optional[str] = Field(default=None, description="Optional name")

class ResetBoxesAction(BaseModel):
    """Reset the positions of red_box and blue_box to their default coordinates"""
    action: Literal["reset_boxes"] = "reset_boxes"
    name: Optional[str] = Field(default=None, description="Optional name")

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


# ---------------------------------------------------------
# GOLD STANDARD FLOW TEMPLATE
# (names/positions vary by flow, so template is about structure)
# ---------------------------------------------------------

RED_FLOW_TEMPLATE = [
    "move to red cap",
    "grasp",
    "move to red bucket",
    "release"
]

GREEN_FLOW_TEMPLATE = [
    "move to green cap",
    "grasp",
    "move to green bucket",
    "release"
]

INITIAL_TEMPLATE = [
    "initialize",
    "reset boxes"
]

def match_any_template(actions, index, templates):
    """
    Check if any template matches actions starting at 'index'.
    Returns (template_name, template_length) or (None, 0)
    """
    for name, tpl in templates.items():
        T = len(tpl)
        if actions[index:index+T] == tpl:
            return name, T
    return None, 0

def replace_templates(actions, templates):
    """
    Replace any matching template with a single element = template name.
    Returns a list like ["initialize", "reset_boxes", "FLOW_RED_CAP1", "FLOW_RED_CAP2", ...]
    """
    result = []
    i = 0
    L = len(actions)

    while i < L:
        name, size = match_any_template(actions, i, templates)
        if name is not None:
            # Found a template match
            result.append(name)
            i += size  # skip entire matched template
        else:
            # Not a template start → keep original element
            result.append(actions[i])
            i += 1

    return result

def split_into_flows(names: List[str]):
    """
    Correct flow slicing:
    - First two actions: initialize + reset_boxes
    - Everything else is cut into flows ending with 'release'
    """
    initial_block = names[:2]
    remaining = names[2:]
    templates = {
        "move to red cap": ['red_middle', 'red_cap', 'red_cap'],
        "move to green cap": ['green_middle', 'green_cap', 'green_cap'],
        "move to red bucket": ['red_middle', 'red_bucket'],
        "move to green bucket": ['red_middle', 'green_bucket']
    }
    flows = []
    current = []
    print(remaining)
    remaining = replace_templates(remaining, templates)
    for action_name in remaining:
        current.append(action_name)

        # flow ends exactly at 'release'
        if action_name == "release":
            flows.append(current)
            current = []

    # If leftover without final release → broken mission
    if current:
        flows.append(current)
    return initial_block, flows

def compare_flow(given: List[str], template: List[str], index):
    number_to_str = {0: 'first red', 1: 'second red', 2: 'third red', 3: 'first green', 4: 'second green', 5: 'third green'}
    questions_answers = []
    # --- check length first ---
    if len(given) > len(template):
        if len(set(given)-set(template)) == 1:
            questions_answers.append({
                "question": f"Why was the {number_to_str[index]} cap dropped not in the bucket?",
                "answer": f"There is an additional, unnecessary node in {number_to_str[index]} flow. {str(set(given)-set(template)).replace('{', '').replace('}', '')} is additional in {number_to_str[index]} flow.",
            })
        else:
            questions_answers.append({
                "question": f"Why was the {number_to_str[index]} cap dropped not in the bucket in {number_to_str[index]} flow?",
                "answer": f"There are additional, unnecessary nodes. {str(set(given)-set(template)).replace('{', '').replace('}', '')} are additional in {number_to_str[index]} flow.",
            })
    elif len(given) < len(template):
        if len(set(template)-set(given)) == 1:
            questions_answers.append({
                "question": f"Why the robot did not pick up the {number_to_str[index]} cap?",
                "answer": f"Node {str(set(template)-set(given)).replace('{', '').replace('}', '')} is missing in {number_to_str[index]} flow.",
            })
        else:
            questions_answers.append({
                "question": f"Why the robot did not pick up the {number_to_str[index]} cap?",
                "answer": f"Nodes {str(set(template)-set(given)).replace('{', '').replace('}', '')} are missing in {number_to_str[index]} flow.",
            })
    given_counter = 0
    for i, expected_action_type in enumerate(template):
        actual = given[given_counter] if given_counter < len(given) else None
        if expected_action_type != actual:
            if given_counter < len(template)-1:
                if actual == template[given_counter+1]:
                    questions_answers.append({
                        "question": f"Why the robot did not place the {number_to_str[index]} cap into the bucket?",
                        "answer": f"Node {expected_action_type} is missing at position {i+1} in {number_to_str[index]} flow.",
                    })
                    continue
                else:
                    questions_answers.append({
                        "question": f"Why the robot did not place the {number_to_str[index]} cap into the bucket?",
                        "answer": f"Node at position {given_counter+1} should be '{expected_action_type}' in {number_to_str[index]} flow.",
                    })
            else:
                questions_answers.append({
                    "question": f"Why the robot did not place the {number_to_str[index]} cap into the bucket?",
                    "answer": f"Node at position {given_counter+1} should be '{expected_action_type}' in {number_to_str[index]} flow.",
                })
        given_counter += 1
    return questions_answers

def validate_mission(names):
    errors = {'additional': []}
    # ---------- 1. split ----------
    given_initial, flows = split_into_flows(names)

    # ---------- 2. validate initial actions ----------
    expected_initial = [a for a in INITIAL_TEMPLATE]

    if given_initial != expected_initial:
        errors['additional'].append({
            "block": "initial",
            "error": "initial_block_mismatch",
        })

    # ---------- 3. ensure 6 flows ----------
    if len(flows) < 6:
        errors['additional'].append({
            "question": "Why there is one cap left on the table?",
            "answer": "One flow is missing, ensure all 6 caps are handled.",
        })
        # still allow partial compare

    # ---------- 4. compare each flow to template ----------
    for i, flow in enumerate(flows):
        print("i, flow", i, flow)
        if i <= 2:
            flow_template = RED_FLOW_TEMPLATE
        else:
            flow_template = GREEN_FLOW_TEMPLATE
        flow_errors = compare_flow(flow, flow_template, i)
        if flow_errors:
            errors[i] = flow_errors

    return errors
