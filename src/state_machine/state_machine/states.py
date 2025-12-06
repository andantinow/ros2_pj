from __future__ import annotations

from typing import List, TYPE_CHECKING
from f110_msgs.msg import Wpnt
from state_machine.state_types import StateType
if TYPE_CHECKING:
    from state_machine.state_machine import StateMachine

def DefaultStateLogic(state_machine: StateMachine) -> List[Wpnt]:
    """
    This is a global state that incorporates the other states.
    
    Handles both legacy and modern mode types.
    """
    match state_machine.state:
        # === Modern Modes ===
        case StateType.CRUISE:
            return Cruise(state_machine)
        case StateType.FOLLOW:
            return Follow(state_machine)
        case StateType.OVERTAKE_CANDIDATE:
            return OvertakeCandidate(state_machine)
        case StateType.OVERTAKE:
            return Overtaking(state_machine)
        case StateType.OBSTACLE_STOP:
            return ObstacleStop(state_machine)
        
        # === Legacy Modes (for backward compatibility) ===
        case StateType.GB_TRACK:
            return GlobalTracking(state_machine)
        case StateType.TRAILING:
            return Trailing(state_machine)
        case StateType.FTGONLY:
            return FTGOnly(state_machine)
        case StateType.TRAILING_TO_GBTRACK:
            return Trailing_to_gbtrack(state_machine)

        case _:
            raise NotImplementedError(f"State {state_machine.state} not recognized")

"""
Here we define the behaviour in the different states.
Every function should be fairly concise, and output an array of f110_msgs.Wpnt
"""

# === Modern Mode Implementations ===

def Cruise(state_machine: StateMachine) -> List[Wpnt]:
    """
    CRUISE Mode: Normal driving along global raceline at target speed.
    
    Uses the global raceline without any modifications.
    This is the default state when no obstacles are present.
    """
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def Follow(state_machine: StateMachine) -> List[Wpnt]:
    """
    FOLLOW Mode: Safe following of preceding vehicle within safe distance.
    
    Follows the global raceline but speed is controlled by the controller
    to maintain safe following distance.
    """
    # Use avoidance waypoints if available (for lateral adjustments)
    if state_machine.last_valid_avoidance_wpnts is not None:
        splini_wpts = state_machine.get_splini_wpts()
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        return [splini_wpts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]
    else:
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def OvertakeCandidate(state_machine: StateMachine) -> List[Wpnt]:
    """
    OVERTAKE_CANDIDATE Mode: In OVERTAKE ZONE, evaluating overtake path candidates.
    
    Continues following while the upper-level evaluates pre-computed 
    overtake trajectories. Does NOT initiate overtake on its own.
    """
    # Same as FOLLOW - wait for upper level to select trajectory
    return Follow(state_machine)

def ObstacleStop(state_machine: StateMachine) -> List[Wpnt]:
    """
    OBSTACLE_STOP Mode: Safely stopped due to obstacle/collision.
    
    Requirements:
    - v_ref = 0 (speed target is zero)
    - Steering does not change abruptly (stable control)
    - Wait for upper level to re-plan route or manual intervention
    
    IMPORTANT: This mode does NOT use "repulsion" or "bouncing" logic.
    The vehicle simply stops safely and waits.
    """
    # Return minimal waypoints - controller will handle stopping
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    # Return fewer waypoints as we're stopping
    n_wpnts = min(5, state_machine.params.n_loc_wpnts)
    return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(n_wpnts)]

# === Legacy Mode Implementations (preserved for backward compatibility) ===

def GlobalTracking(state_machine: StateMachine) -> List[Wpnt]:
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def Trailing_to_gbtrack(state_machine: StateMachine) -> List[Wpnt]:
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def Trailing(state_machine: StateMachine) -> List[Wpnt]:
    # This allows us to trail on the last valid spline if necessary
    if state_machine.last_valid_avoidance_wpnts is not None:
        splini_wpts = state_machine.get_splini_wpts()
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        return [splini_wpts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]
    else:
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def Overtaking(state_machine: StateMachine) -> List[Wpnt]:
    splini_wpts = state_machine.get_splini_wpts()
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    return [splini_wpts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def FTGOnly(state_machine: StateMachine) -> List[Wpnt]:
    """No waypoints are generated in this follow the gap only state, all the 
    control inputs are generated in the control node.
    """
    return []