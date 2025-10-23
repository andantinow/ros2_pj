# AdaptiveVehicleModel.msg
# Defines the real-time estimated vehicle dynamic parameters for NMPC (Pillar 2 output).

std_msgs/Header header

# 1. Estimated Tire Grip Coefficient (Friction Circle limit)
float64 tire_mu

# 2. Estimated Tire Stiffness (Cornering stiffness, key for dynamic model)
float64 tire_cf
float64 tire_cr

# 3. Estimated vehicle mass (for robustness against varying loads)
float64 mass
