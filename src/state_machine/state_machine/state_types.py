import enum

class StateType(enum.Enum):
    """
    Racing Agent State Types
    
    Mode Structure (as per requirements):
    - CRUISE (GB_TRACK): Normal driving along global raceline at target speed
    - FOLLOW (TRAILING): Safe following of preceding vehicle within safe distance
    - OVERTAKE_CANDIDATE: In OVERTAKE ZONE, evaluating overtake path candidates
    - OVERTAKE: Executing pre-computed overtake trajectory
    - OBSTACLE_STOP: Safely stopped due to obstacle/collision (v_ref = 0, stable steering)
    
    Legacy modes are preserved for backward compatibility:
    - GB_TRACK -> CRUISE
    - TRAILING -> FOLLOW
    - FTGONLY -> Follow-The-Gap only mode (fallback)
    """
    
    # === Primary Modes (New Architecture) ===
    CRUISE = 'CRUISE'                     # Normal driving along global raceline
    FOLLOW = 'FOLLOW'                     # Safe following of preceding vehicle
    OVERTAKE_CANDIDATE = 'OVERTAKE_CANDIDATE'  # Evaluating overtake candidates in OVERTAKE ZONE
    OVERTAKE = 'OVERTAKE'                 # Executing pre-computed overtake trajectory
    OBSTACLE_STOP = 'OBSTACLE_STOP'       # Safely stopped due to obstacle/collision
    
    # === Legacy/Compatibility Modes ===
    GB_TRACK = 'GB_TRACK'                 # Legacy: equivalent to CRUISE
    TRAILING = 'TRAILING'                 # Legacy: equivalent to FOLLOW  
    FTGONLY = 'FTGONLY'                   # Follow-The-Gap only mode
    TRAILING_TO_GBTRACK = 'TRAILING_TO_GBTRACK'  # Transition state

    # === Mode Mapping Helpers ===
    @classmethod
    def to_modern_mode(cls, legacy_mode: 'StateType') -> 'StateType':
        """Convert legacy mode to modern equivalent"""
        mapping = {
            cls.GB_TRACK: cls.CRUISE,
            cls.TRAILING: cls.FOLLOW,
            cls.TRAILING_TO_GBTRACK: cls.FOLLOW,
        }
        return mapping.get(legacy_mode, legacy_mode)
    
    @classmethod
    def to_legacy_mode(cls, modern_mode: 'StateType') -> 'StateType':
        """Convert modern mode to legacy equivalent"""
        mapping = {
            cls.CRUISE: cls.GB_TRACK,
            cls.FOLLOW: cls.TRAILING,
        }
        return mapping.get(modern_mode, modern_mode)
