# Adaptive vehicle model parameters estimated online
# Units:
# - mu: friction coefficient [-]
# - Cf: front cornering stiffness [N/rad]
# - Cr: rear cornering stiffness [N/rad] (optional, may be 0 if unused)

builtin_interfaces/Time stamp
string frame_id

float64 mu
float64 Cf
float64 Cr
