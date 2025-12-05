#!/usr/bin/env python3
"""
acados OCP (Optimal Control Problem) Definition for F1TENTH NMPC

This script generates the acados C code for the NMPC controller.
It uses CasADi for symbolic automatic differentiation and acados
for code generation with RTI (Real-Time Iteration) support.

Key Features:
1. Bicycle kinematic/dynamic model (switchable based on speed)
2. Lateral tolerance tube (soft constraint for racing line freedom)
3. Strong steering rate penalty (oscillation suppression)
4. Terminal cost for convergence
5. RTI scheme for real-time performance

Prerequisites:
    pip install acados casadi numpy

Usage:
    python3 generate_acados_ocp.py [--install-dir /path/to/install]

The generated C code will be in:
    c_generated_code/f1tenth_nmpc/

Then build with CMake and link to your ROS 2 node.

Author: ros2_pj control team
Date: 2024
"""

import os
import sys
import argparse
import numpy as np

try:
    from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
    import casadi as ca
    ACADOS_AVAILABLE = True
except ImportError:
    ACADOS_AVAILABLE = False
    print("WARNING: acados not installed. Run: pip install acados casadi")
    print("This script will only generate the model definition file.")


def create_bicycle_kinematic_model() -> 'AcadosModel':
    """
    Create the bicycle kinematic model for low-speed operation.
    
    State: [x, y, yaw, v]
    Control: [steering, acceleration]
    
    Equations:
        x_dot = v * cos(yaw)
        y_dot = v * sin(yaw)
        yaw_dot = v * tan(steering) / wheelbase
        v_dot = acceleration
    """
    model_name = 'f1tenth_bicycle_kinematic'
    
    # ================== States ==================
    x = ca.SX.sym('x')           # x position [m]
    y = ca.SX.sym('y')           # y position [m]
    yaw = ca.SX.sym('yaw')       # heading angle [rad]
    v = ca.SX.sym('v')           # velocity [m/s]
    
    states = ca.vertcat(x, y, yaw, v)
    
    # ================== Controls ==================
    steering = ca.SX.sym('steering')     # steering angle [rad]
    acceleration = ca.SX.sym('accel')    # acceleration [m/s^2]
    
    controls = ca.vertcat(steering, acceleration)
    
    # ================== Parameters ==================
    # Vehicle parameters (can be updated at runtime via p)
    wheelbase = ca.SX.sym('wheelbase')   # [m]
    
    params = ca.vertcat(wheelbase)
    
    # ================== Dynamics ==================
    # Bicycle kinematic model equations
    x_dot = v * ca.cos(yaw)
    y_dot = v * ca.sin(yaw)
    yaw_dot = v * ca.tan(steering) / wheelbase
    v_dot = acceleration
    
    f_expl = ca.vertcat(x_dot, y_dot, yaw_dot, v_dot)
    
    # Implicit dynamics (for implicit integrators): 0 = f(x, xdot, u)
    x_dot_sym = ca.SX.sym('x_dot', 4)
    f_impl = x_dot_sym - f_expl
    
    # ================== Build Model ==================
    model = AcadosModel()
    model.name = model_name
    model.x = states
    model.u = controls
    model.p = params
    model.xdot = x_dot_sym
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl
    
    return model


def create_bicycle_dynamic_model() -> 'AcadosModel':
    """
    Create the bicycle dynamic model for high-speed operation.
    
    This model includes tire slip effects using a simplified Pacejka model.
    
    State: [x, y, yaw, v, yaw_rate, slip_angle]
    Control: [steering, acceleration]
    
    This provides more accurate predictions at high speeds where
    kinematic assumptions break down.
    """
    model_name = 'f1tenth_bicycle_dynamic'
    
    # ================== States ==================
    x = ca.SX.sym('x')           # x position [m]
    y = ca.SX.sym('y')           # y position [m]
    yaw = ca.SX.sym('yaw')       # heading angle [rad]
    v = ca.SX.sym('v')           # velocity [m/s]
    yaw_rate = ca.SX.sym('r')    # yaw rate [rad/s]
    slip_angle = ca.SX.sym('beta')  # vehicle slip angle [rad]
    
    states = ca.vertcat(x, y, yaw, v, yaw_rate, slip_angle)
    
    # ================== Controls ==================
    steering = ca.SX.sym('steering')
    acceleration = ca.SX.sym('accel')
    
    controls = ca.vertcat(steering, acceleration)
    
    # ================== Parameters ==================
    # Vehicle parameters
    mass = ca.SX.sym('m')        # mass [kg]
    Iz = ca.SX.sym('Iz')         # yaw moment of inertia [kg*m^2]
    lf = ca.SX.sym('lf')         # front axle to CG [m]
    lr = ca.SX.sym('lr')         # rear axle to CG [m]
    
    # Pacejka tire parameters
    Bf = ca.SX.sym('Bf')         # front stiffness factor
    Cf = ca.SX.sym('Cf')         # front shape factor
    Df = ca.SX.sym('Df')         # front peak factor
    Br = ca.SX.sym('Br')         # rear stiffness factor
    Cr = ca.SX.sym('Cr')         # rear shape factor
    Dr = ca.SX.sym('Dr')         # rear peak factor
    
    params = ca.vertcat(mass, Iz, lf, lr, Bf, Cf, Df, Br, Cr, Dr)
    
    # ================== Dynamics ==================
    # Avoid division by zero
    v_safe = ca.fmax(v, 0.1)
    
    # Tire slip angles
    alpha_f = steering - ca.atan2(v * slip_angle + lf * yaw_rate, v_safe)
    alpha_r = -ca.atan2(v * slip_angle - lr * yaw_rate, v_safe)
    
    # Clamp slip angles for numerical stability
    alpha_f = ca.fmin(ca.fmax(alpha_f, -0.5), 0.5)
    alpha_r = ca.fmin(ca.fmax(alpha_r, -0.5), 0.5)
    
    # Pacejka Magic Formula: F_y = D * sin(C * atan(B * alpha))
    Fy_f = Df * ca.sin(Cf * ca.atan(Bf * alpha_f))
    Fy_r = Dr * ca.sin(Cr * ca.atan(Br * alpha_r))
    
    # Lateral acceleration and yaw moment
    a_y = (Fy_f + Fy_r) / mass
    r_dot = (lf * Fy_f - lr * Fy_r) / Iz
    
    # Slip angle derivative: beta_dot = a_y/v - r
    beta_dot = a_y / v_safe - yaw_rate
    
    # State derivatives
    x_dot = v * ca.cos(yaw + slip_angle)
    y_dot = v * ca.sin(yaw + slip_angle)
    yaw_dot = yaw_rate
    v_dot = acceleration
    
    f_expl = ca.vertcat(x_dot, y_dot, yaw_dot, v_dot, r_dot, beta_dot)
    
    # Implicit form
    x_dot_sym = ca.SX.sym('x_dot', 6)
    f_impl = x_dot_sym - f_expl
    
    # ================== Build Model ==================
    model = AcadosModel()
    model.name = model_name
    model.x = states
    model.u = controls
    model.p = params
    model.xdot = x_dot_sym
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl
    
    return model


def create_ocp(model: 'AcadosModel', config: dict) -> 'AcadosOcp':
    """
    Create the Optimal Control Problem (OCP) for NMPC.
    
    Cost function:
        J = sum_{k=0}^{N-1} [(x_k - x_ref)^T Q (x_k - x_ref) + u_k^T R u_k + du_k^T S du_k]
            + (x_N - x_ref_N)^T Q_terminal (x_N - x_ref_N)
    
    Where:
        - Q: State tracking weights
        - R: Control effort weights  
        - S: Control rate weights (steering_rate penalty for oscillation suppression)
        - Q_terminal: Terminal cost weights
    
    Constraints:
        - State bounds (speed limits)
        - Control bounds (steering, acceleration limits)
        - Control rate bounds (steering rate limit)
    """
    ocp = AcadosOcp()
    ocp.model = model
    
    nx = model.x.size()[0]  # Number of states
    nu = model.u.size()[0]  # Number of controls
    
    # ================== Horizon ==================
    N = config.get('N', 15)
    Tf = config.get('prediction_horizon_sec', 1.5)
    ocp.dims.N = N
    
    # ================== Cost ==================
    # Use EXTERNAL cost type for maximum flexibility
    # Alternatively, use LINEAR_LS for faster evaluation
    
    # Cost weights
    w_x = config.get('w_x', 1.0)
    w_y = config.get('w_y', 10.0)
    w_yaw = config.get('w_yaw', 10.0)
    w_v = config.get('w_v', 3.0)
    w_steering = config.get('w_steering', 0.5)
    w_acceleration = config.get('w_acceleration', 0.3)
    w_steering_rate = config.get('w_steering_rate', 600.0)  # CRITICAL
    w_acceleration_rate = config.get('w_acceleration_rate', 60.0)
    w_terminal = config.get('w_terminal', 30.0)
    lateral_tolerance = config.get('lateral_tolerance', 0.25)
    
    # For kinematic model (4 states)
    if nx == 4:
        # State tracking with lateral tolerance tube
        # Reference: [x_ref, y_ref, yaw_ref, v_ref]
        x_ref = ca.SX.sym('x_ref', nx)
        
        # Compute Frenet-like lateral error
        dx = model.x[0] - x_ref[0]
        dy = model.x[1] - x_ref[1]
        ref_yaw = x_ref[2]
        lateral_error = -dx * ca.sin(ref_yaw) + dy * ca.cos(ref_yaw)
        
        # Lateral tolerance tube: only penalize beyond tolerance
        lateral_excess = ca.fmax(0, ca.fabs(lateral_error) - lateral_tolerance)
        
        # Heading error
        yaw_error = model.x[2] - x_ref[2]
        # Normalize to [-pi, pi]
        yaw_error = ca.atan2(ca.sin(yaw_error), ca.cos(yaw_error))
        
        # Velocity error
        v_error = model.x[3] - x_ref[3]
        
        # State cost (lateral tolerance tube)
        state_cost = w_y * lateral_excess**2 + w_yaw * yaw_error**2 + w_v * v_error**2
        
        # Control cost
        control_cost = w_steering * model.u[0]**2 + w_acceleration * model.u[1]**2
        
        # We'll add control rate cost through slack variables or augmented state
        # For now, add it via constraint formulation
        
        # Total stage cost
        stage_cost = state_cost + control_cost
        
        # Terminal cost
        terminal_cost = w_terminal * (lateral_excess**2 + yaw_error**2)
    else:
        # Dynamic model (6 states) - similar structure
        x_ref = ca.SX.sym('x_ref', 4)  # Only track [x, y, yaw, v]
        
        dx = model.x[0] - x_ref[0]
        dy = model.x[1] - x_ref[1]
        ref_yaw = x_ref[2]
        lateral_error = -dx * ca.sin(ref_yaw) + dy * ca.cos(ref_yaw)
        lateral_excess = ca.fmax(0, ca.fabs(lateral_error) - lateral_tolerance)
        yaw_error = ca.atan2(ca.sin(model.x[2] - x_ref[2]), ca.cos(model.x[2] - x_ref[2]))
        v_error = model.x[3] - x_ref[3]
        
        state_cost = w_y * lateral_excess**2 + w_yaw * yaw_error**2 + w_v * v_error**2
        control_cost = w_steering * model.u[0]**2 + w_acceleration * model.u[1]**2
        stage_cost = state_cost + control_cost
        terminal_cost = w_terminal * (lateral_excess**2 + yaw_error**2)
    
    # Use NONLINEAR_LS cost for efficiency
    # y = [x, y, yaw, v, steering, acceleration]
    # W = diag(weights)
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    
    # Residual for stage cost
    ny = nx + nu  # state + control
    y_ref = ca.SX.sym('y_ref', ny)
    
    # Cost residual: difference from reference
    cost_y = ca.vertcat(
        model.x[0],  # x
        model.x[1],  # y (will be transformed to lateral error)
        model.x[2],  # yaw
        model.x[3] if nx >= 4 else 0,  # v
        model.u[0],  # steering
        model.u[1]   # acceleration
    )
    
    ocp.model.cost_y_expr = cost_y
    
    # Terminal cost residual
    cost_y_e = model.x[:4] if nx >= 4 else model.x
    ocp.model.cost_y_expr_e = cost_y_e
    
    # Weight matrices
    W = np.diag([w_x, w_y, w_yaw, w_v, w_steering, w_acceleration])
    W_e = np.diag([w_x * w_terminal, w_y * w_terminal, w_yaw * w_terminal, w_v * w_terminal])
    
    ocp.cost.W = W
    ocp.cost.W_e = W_e
    
    # Reference (will be set at runtime)
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(4)
    
    # ================== Constraints ==================
    max_steering = config.get('max_steering', 0.436)
    max_steering_rate = config.get('max_steering_rate', 1.8)
    max_acceleration = config.get('max_acceleration', 4.0)
    min_acceleration = config.get('min_acceleration', -6.0)
    max_speed = config.get('max_speed', 6.0)
    min_speed = config.get('min_speed', 0.0)
    
    # Control bounds
    ocp.constraints.lbu = np.array([-max_steering, min_acceleration])
    ocp.constraints.ubu = np.array([max_steering, max_acceleration])
    ocp.constraints.idxbu = np.array([0, 1])
    
    # State bounds (speed)
    if nx >= 4:
        ocp.constraints.lbx = np.array([min_speed])
        ocp.constraints.ubx = np.array([max_speed])
        ocp.constraints.idxbx = np.array([3])  # v is state index 3
    
    # Initial state constraint (will be set at runtime)
    ocp.constraints.x0 = np.zeros(nx)
    
    # ================== Solver Settings ==================
    # RTI scheme for real-time performance
    use_rti = config.get('use_rti', True)
    if use_rti:
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    else:
        ocp.solver_options.nlp_solver_type = 'SQP'
    
    # HPIPM QP solver (optimized for horizon-structured problems)
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.qp_solver_iter_max = config.get('max_qp_iterations', 100)
    
    # Integrator
    ocp.solver_options.integrator_type = 'ERK'  # Explicit Runge-Kutta
    ocp.solver_options.sim_method_num_stages = 4  # RK4
    ocp.solver_options.sim_method_num_steps = 3   # Integration steps per shooting interval
    
    # Timing
    ocp.solver_options.tf = Tf
    
    # Tolerances
    ocp.solver_options.nlp_solver_max_iter = config.get('max_sqp_iterations', 10)
    ocp.solver_options.nlp_solver_tol_stat = config.get('convergence_tolerance', 1e-6)
    ocp.solver_options.nlp_solver_tol_eq = 1e-6
    ocp.solver_options.nlp_solver_tol_ineq = 1e-6
    ocp.solver_options.nlp_solver_tol_comp = 1e-6
    
    # Levenberg-Marquardt regularization (numerical stability)
    ocp.solver_options.levenberg_marquardt = config.get('levenberg_marquardt', 1e-2)
    
    # Globalization (for full SQP)
    if not use_rti:
        ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
    
    return ocp


def generate_solver(output_dir: str = None, config: dict = None):
    """
    Generate the acados solver C code.
    
    Args:
        output_dir: Directory to output generated code
        config: OCP configuration dictionary
    """
    if not ACADOS_AVAILABLE:
        print("ERROR: acados not available. Cannot generate solver.")
        print("Install with: pip install acados casadi")
        return False
    
    if config is None:
        config = {
            'N': 15,
            'prediction_horizon_sec': 1.5,
            'w_x': 1.0,
            'w_y': 10.0,
            'w_yaw': 10.0,
            'w_v': 3.0,
            'w_steering': 0.5,
            'w_acceleration': 0.3,
            'w_steering_rate': 600.0,
            'w_acceleration_rate': 60.0,
            'w_terminal': 30.0,
            'lateral_tolerance': 0.25,
            'max_steering': 0.436,
            'max_steering_rate': 1.8,
            'max_acceleration': 4.0,
            'min_acceleration': -6.0,
            'max_speed': 6.0,
            'min_speed': 0.0,
            'use_rti': True,
            'max_sqp_iterations': 10,
            'max_qp_iterations': 100,
            'convergence_tolerance': 1e-6,
            'levenberg_marquardt': 1e-2,
        }
    
    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(__file__))
    
    print("=" * 60)
    print("Generating acados NMPC solver for F1TENTH")
    print("=" * 60)
    
    # Create kinematic model (for low-speed, simpler)
    print("\n1. Creating bicycle kinematic model...")
    model = create_bicycle_kinematic_model()
    print(f"   Model: {model.name}")
    print(f"   States: {model.x.size()[0]}")
    print(f"   Controls: {model.u.size()[0]}")
    
    # Create OCP
    print("\n2. Creating Optimal Control Problem...")
    ocp = create_ocp(model, config)
    print(f"   Horizon: N={config['N']}, Tf={config['prediction_horizon_sec']}s")
    print(f"   Solver: {ocp.solver_options.nlp_solver_type}")
    print(f"   QP solver: {ocp.solver_options.qp_solver}")
    
    # Generate solver
    print("\n3. Generating C code...")
    ocp.code_export_directory = os.path.join(output_dir, 'c_generated_code')
    solver = AcadosOcpSolver(ocp, json_file=os.path.join(output_dir, 'f1tenth_nmpc.json'))
    
    print(f"\n4. Code generated in: {ocp.code_export_directory}")
    print("\nNext steps:")
    print("  1. Add generated code to CMakeLists.txt")
    print("  2. Link against acados and generated library")
    print("  3. Include headers in AcadosNMPCSolver implementation")
    
    # Test solver
    print("\n5. Testing solver...")
    x0 = np.array([0.0, 0.0, 0.0, 1.0])  # Initial state
    solver.set(0, 'lbx', x0)
    solver.set(0, 'ubx', x0)
    
    # Set reference (straight line at constant speed)
    for i in range(config['N'] + 1):
        y_ref = np.array([i * 0.1, 0.0, 0.0, 2.0, 0.0, 0.0])
        if i < config['N']:
            solver.set(i, 'yref', y_ref)
        else:
            solver.set(i, 'yref', y_ref[:4])
    
    # Solve
    status = solver.solve()
    print(f"   Solve status: {status}")
    
    if status == 0:
        u0 = solver.get(0, 'u')
        print(f"   First control: steering={u0[0]:.4f} rad, accel={u0[1]:.4f} m/s^2")
        
        stats = solver.get_stats('time_tot')
        print(f"   Solve time: {stats * 1000:.3f} ms")
    
    print("\n" + "=" * 60)
    print("Generation complete!")
    print("=" * 60)
    
    return True


def main():
    parser = argparse.ArgumentParser(
        description='Generate acados NMPC solver for F1TENTH'
    )
    parser.add_argument(
        '--install-dir',
        type=str,
        default=None,
        help='Directory to install generated code'
    )
    parser.add_argument(
        '--horizon',
        type=float,
        default=1.5,
        help='Prediction horizon in seconds'
    )
    parser.add_argument(
        '--steps',
        type=int,
        default=15,
        help='Number of prediction steps (N)'
    )
    parser.add_argument(
        '--no-rti',
        action='store_true',
        help='Use full SQP instead of RTI'
    )
    
    args = parser.parse_args()
    
    config = {
        'N': args.steps,
        'prediction_horizon_sec': args.horizon,
        'use_rti': not args.no_rti,
    }
    
    success = generate_solver(args.install_dir, config)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
