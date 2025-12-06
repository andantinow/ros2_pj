from __future__ import annotations

from typing import TYPE_CHECKING

from state_machine.state_types import StateType

if TYPE_CHECKING:
    from state_machine.state_machine import StateMachine


def dummy_transition(state_machine: StateMachine)->str:
    match state_machine.state:
        case StateType.GB_TRACK:
            if state_machine._low_bat:
                return StateType.LOW_BAT
            else:
                return StateType.GB_TRACK
        case StateType.LOW_BAT:
            return StateType.LOW_BAT
        case default:
            return StateType.GB_TRACK
        
        
def timetrials_transition(state_machine: StateMachine)->str:
    return StateType.GB_TRACK


def head_to_head_transition(state_machine: StateMachine)->str:
    """
    Head-to-head racing transition logic.
    
    Supports both legacy modes (GB_TRACK, TRAILING, etc.) and 
    modern modes (CRUISE, FOLLOW, OVERTAKE_CANDIDATE, OVERTAKE, OBSTACLE_STOP).
    """
    match state_machine.state:
        # === Modern Modes ===
        case StateType.CRUISE:
            return CruiseTransition(state_machine)
        case StateType.FOLLOW:
            return FollowTransition(state_machine)
        case StateType.OVERTAKE_CANDIDATE:
            return OvertakeCandidateTransition(state_machine)
        case StateType.OVERTAKE:
            # OVERTAKE handles both modern and legacy transitions
            return OvertakeTransition(state_machine)
        case StateType.OBSTACLE_STOP:
            return ObstacleStopTransition(state_machine)
        
        # === Legacy Modes ===
        case StateType.GB_TRACK:
            return SpliniGlobalTrackingTransition(state_machine)
        case StateType.TRAILING:
            return SpliniTrailingTransition(state_machine)
        case StateType.TRAILING_TO_GBTRACK:
            return SpliniTrailingToGbtrackTransition(state_machine)
        case StateType.FTGONLY:
            return SpliniFTGOnlyTransition(state_machine)
        case default:
            raise ValueError(f"Invalid state {state_machine.state}")


# === Modern Mode Transitions ===

def CruiseTransition(state_machine: StateMachine) -> StateType:
    """
    Transitions from CRUISE mode.
    
    CRUISE -> FOLLOW: When preceding vehicle detected within follow distance
    CRUISE -> OBSTACLE_STOP: When collision/obstacle risk detected
    CRUISE -> CRUISE: Otherwise continue cruising
    """
    # Check for obstacle stop condition first (highest priority)
    if _should_stop_for_obstacle(state_machine):
        return StateType.OBSTACLE_STOP
    
    # Check if we should start following
    if not state_machine._check_gbfree:
        return StateType.FOLLOW
    
    return StateType.CRUISE


def FollowTransition(state_machine: StateMachine) -> StateType:
    """
    Transitions from FOLLOW mode.
    
    FOLLOW -> OBSTACLE_STOP: When collision/obstacle risk detected
    FOLLOW -> CRUISE: When no preceding vehicle and close to raceline
    FOLLOW -> OVERTAKE_CANDIDATE: When in OVERTAKE ZONE and conditions met
    FOLLOW -> FOLLOW: Otherwise continue following
    
    IMPORTANT: FOLLOW mode does NOT spontaneously start overtaking.
    Only transitions to OVERTAKE_CANDIDATE in pre-defined OVERTAKE ZONEs.
    """
    # Check for obstacle stop condition first (highest priority)
    if _should_stop_for_obstacle(state_machine):
        return StateType.OBSTACLE_STOP
    
    # FTG fallback check
    if state_machine._check_only_ftg_zone:
        return StateType.FTGONLY
    
    if state_machine._check_ftg:
        return StateType.FTGONLY
    
    gb_free = state_machine._check_gbfree
    ot_sector = state_machine._check_ot_sector
    
    # If global track is free and close to raceline, return to cruise
    if gb_free and state_machine._check_close_to_raceline:
        return StateType.CRUISE
    
    # If in overtake sector with valid conditions, consider overtake
    if (not gb_free and ot_sector and 
        state_machine._check_availability_splini_wpts and 
        state_machine._check_ofree):
        return StateType.OVERTAKE_CANDIDATE
    
    return StateType.FOLLOW


def OvertakeCandidateTransition(state_machine: StateMachine) -> StateType:
    """
    Transitions from OVERTAKE_CANDIDATE mode.
    
    This mode evaluates pre-computed overtake trajectory candidates.
    
    OVERTAKE_CANDIDATE -> OBSTACLE_STOP: If collision risk detected
    OVERTAKE_CANDIDATE -> FOLLOW: If no longer in OVERTAKE ZONE
    OVERTAKE_CANDIDATE -> OVERTAKE: If valid trajectory selected (handled by upper level)
    OVERTAKE_CANDIDATE -> CRUISE: If preceding vehicle clears
    """
    # Check for obstacle stop condition first
    if _should_stop_for_obstacle(state_machine):
        return StateType.OBSTACLE_STOP
    
    ot_sector = state_machine._check_ot_sector
    
    # If not in overtake sector anymore, return to follow/cruise
    if not ot_sector:
        if state_machine._check_gbfree:
            return StateType.CRUISE
        return StateType.FOLLOW
    
    # Check if overtake conditions are met and trajectory is valid
    if (state_machine._check_availability_splini_wpts and 
        state_machine._check_ofree):
        # Upper level will select trajectory - transition to OVERTAKE
        return StateType.OVERTAKE
    
    # Stay in candidate mode while evaluating
    return StateType.OVERTAKE_CANDIDATE


def OvertakeTransition(state_machine: StateMachine) -> StateType:
    """
    Transitions from OVERTAKE mode.
    
    OVERTAKE -> OBSTACLE_STOP: If collision risk during overtake
    OVERTAKE -> CRUISE: When overtake complete and GB free
    OVERTAKE -> FOLLOW: When overtake complete but still have preceding vehicle
    OVERTAKE -> OVERTAKE: Otherwise continue overtake
    """
    # Check for obstacle stop - abort overtake if collision risk
    if _should_stop_for_obstacle(state_machine):
        return StateType.OBSTACLE_STOP
    
    if state_machine._check_only_ftg_zone:
        return StateType.FTGONLY
    
    in_ot_sector = state_machine._check_ot_sector
    spline_valid = state_machine._check_availability_splini_wpts
    o_free = state_machine._check_ofree
    
    # If spline is on an obstacle we trail
    if not o_free:
        return StateType.FOLLOW
    
    if in_ot_sector and o_free and spline_valid:
        return StateType.OVERTAKE
    
    # If spline becomes invalid while overtaking, we trail
    if in_ot_sector and not spline_valid and not o_free:
        return StateType.FOLLOW
    
    # Go to CRUISE if not in ot_sector and the GB is free
    if not in_ot_sector and state_machine._check_gbfree:
        return StateType.CRUISE
    
    # Go to Following if not in ot_sector and the GB is not free
    return StateType.FOLLOW


def ObstacleStopTransition(state_machine: StateMachine) -> StateType:
    """
    Transitions from OBSTACLE_STOP mode.
    
    This mode is entered when:
    - Wall/obstacle collision risk detected
    - Emergency stop required
    
    In this mode:
    - v_ref = 0 (target speed is zero)
    - Steering does not change abruptly
    - NO "repulsion" or "bouncing" logic is used
    
    OBSTACLE_STOP -> CRUISE: When obstacle cleared and path is safe
    OBSTACLE_STOP -> FOLLOW: When obstacle cleared but preceding vehicle present
    OBSTACLE_STOP -> OBSTACLE_STOP: Otherwise remain stopped
    """
    # Check if obstacle is cleared (front is clear beyond safe distance)
    obstacle_cleared = not _should_stop_for_obstacle(state_machine)
    
    if obstacle_cleared:
        if state_machine._check_gbfree:
            return StateType.CRUISE
        else:
            return StateType.FOLLOW
    
    # Remain in OBSTACLE_STOP
    return StateType.OBSTACLE_STOP


def _should_stop_for_obstacle(state_machine: StateMachine) -> bool:
    """
    Helper function to check if obstacle stop is required.
    
    This replaces any "repulsion" or "bouncing" logic with a simple stop.
    """
    # Check for very close obstacles (below minimum safe distance)
    # This is the emergency stop threshold
    MIN_STOP_DISTANCE = 0.3  # meters
    
    # Check if obstacles exist and are within emergency stop distance
    for obs in state_machine.obstacles:
        gap = (obs.s_center - state_machine.cur_s) % state_machine.track_length
        if gap < MIN_STOP_DISTANCE:
            return True
    
    return False


# === Legacy Mode Transitions (preserved for backward compatibility) ===

def SpliniGlobalTrackingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.GB_TRACK`"""
    if not state_machine._check_only_ftg_zone:
        if state_machine._check_gbfree:
            return StateType.GB_TRACK
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY

def SpliniTrailingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.TRAILING`"""
    gb_free = state_machine._check_gbfree
    ot_sector = state_machine._check_ot_sector

    if not state_machine._check_only_ftg_zone:
        # If we have been sitting around in TRAILING for a while then FTG
        if state_machine._check_ftg:
            return StateType.FTGONLY
        if not gb_free and not ot_sector:
            return StateType.TRAILING

        # 아래와 같이 바로 GB_TRACK로 전환하지 않고 TRAILING_TO_GBTRACK로 전환하도록 수정
        elif gb_free and state_machine._check_close_to_raceline:
            return StateType.TRAILING_TO_GBTRACK


        elif (
            not gb_free
            and ot_sector
            and state_machine._check_availability_splini_wpts
            and state_machine._check_ofree
        ):
            return StateType.OVERTAKE
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY

def SpliniTrailingToGbtrackTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.TRAILING_TO_GBTRACK`"""
    # GB_TRACK 이외의 다른 상태로 return 할 때에는 trailing_to_gbtrack_count를 0으로 리셋해주기
    
    gb_free = state_machine._check_gbfree
    ot_sector = state_machine._check_ot_sector

    if not state_machine._check_only_ftg_zone:
        # If we have been sitting around in TRAILING for a while then FTG
        if state_machine._check_ftg:
            state_machine.trailing_to_gbtrack_count = 0
            return StateType.FTGONLY
        if not gb_free and not ot_sector:
            state_machine.trailing_to_gbtrack_count = 0
            return StateType.TRAILING


        elif gb_free and state_machine._check_close_to_raceline:

            state_machine.trailing_to_gbtrack_count += 1

            # gb_free의 횟수가 threshold를 넘기면 그때는 진짜로 gbtrack으로 전환
            if state_machine.trailing_to_gbtrack_count >= state_machine.trailing_to_gbtrack_counting_threshold:
                state_machine.trailing_to_gbtrack_count = 0
                return StateType.GB_TRACK

            else:
                return StateType.TRAILING_TO_GBTRACK

        elif (
            not gb_free
            and ot_sector
            and state_machine._check_availability_splini_wpts
            and state_machine._check_ofree
        ):
            state_machine.trailing_to_gbtrack_count = 0
            return StateType.OVERTAKE
        else:
            state_machine.trailing_to_gbtrack_count = 0
            return StateType.TRAILING
    else:
        state_machine.trailing_to_gbtrack_count = 0
        return StateType.FTGONLY



def SpliniOvertakingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.OVERTAKE`"""
    if not state_machine._check_only_ftg_zone:
        in_ot_sector = state_machine._check_ot_sector
        spline_valid = state_machine._check_availability_splini_wpts
        o_free = state_machine._check_ofree

        # If spline is on an obstacle we trail
        if not o_free:
            return StateType.TRAILING
        if in_ot_sector and o_free and spline_valid:
            return StateType.OVERTAKE
        # If spline becomes unvalid while overtaking, we trail
        elif in_ot_sector and not spline_valid and not o_free:
            return StateType.TRAILING
        # go to GB_TRACK if not in ot_sector and the GB is free
        elif not in_ot_sector and state_machine._check_gbfree:
            return StateType.GB_TRACK
        # go to Trailing if not in ot_sector and the GB is not free
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniFTGOnlyTransition(state_machine: StateMachine) -> StateType:
    if state_machine._check_only_ftg_zone:
        return StateType.FTGONLY
    else:
        if state_machine._check_close_to_raceline and state_machine._check_gbfree:
            return StateType.GB_TRACK
        else:
            return StateType.FTGONLY
        