/**
 * @file acados_nmpc_interface.hpp
 * @brief acados NMPC Solver Interface for Real-Time MPC
 * 
 * This file provides an abstraction layer for integrating acados with ROS 2.
 * acados provides:
 * - RTI (Real-Time Iteration) scheme for predictable solve times
 * - HPIPM QP solver optimized for horizon-structured problems
 * - Code generation for maximum performance
 * - Automatic differentiation via CasADi (no runtime re-taping like CppAD)
 * 
 * Key advantages over Ipopt-based NMPC:
 * 1. No barrier parameter scheduling issues (SQP/RTI vs Interior Point)
 * 2. Warm start is first-class citizen (designed for RTI)
 * 3. Block-structured QP solver (HPIPM) vs general sparse solver (MUMPS)
 * 4. Codegen eliminates AD overhead at runtime
 * 
 * Usage:
 * 1. Generate acados C code using the Python interface
 * 2. Include generated headers and link generated library
 * 3. Use this interface class for ROS 2 integration
 * 
 * @note This is a template/interface file. Actual acados code generation
 *       should be done using the acados Python interface with CasADi.
 */

#ifndef CONTROL_PKG__ACADOS_NMPC_INTERFACE_HPP_
#define CONTROL_PKG__ACADOS_NMPC_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <functional>
#include <chrono>
#include <atomic>
#include <mutex>

// Forward declarations for acados types
// These will be replaced with actual types when acados is installed
// For now, we define placeholder structures
namespace acados_interface
{

/**
 * @brief Vehicle state for NMPC
 */
struct VehicleState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
  double yaw_rate{0.0};  // For dynamic model
  double slip_angle{0.0}; // For dynamic model
};

/**
 * @brief Control input
 */
struct ControlInput
{
  double steering{0.0};
  double acceleration{0.0};
};

/**
 * @brief Reference trajectory point
 */
struct ReferencePoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
  double curvature{0.0};  // Path curvature for feedforward
};

/**
 * @brief Solver status codes (matching acados conventions)
 */
enum class SolverStatus : int
{
  SUCCESS = 0,           // Optimal solution found
  MAX_QP_ITER = 1,       // Maximum QP iterations reached
  MAX_SQP_ITER = 2,      // Maximum SQP iterations reached
  QP_FAILURE = 3,        // QP solver failure
  NAN_SOLUTION = 4,      // NaN detected in solution
  TIMEOUT = 5,           // Solver timeout
  NOT_INITIALIZED = 6    // Solver not initialized
};

/**
 * @brief Solver statistics
 */
struct SolverStats
{
  SolverStatus status{SolverStatus::NOT_INITIALIZED};
  int sqp_iterations{0};
  int qp_iterations{0};
  double total_time_ms{0.0};
  double qp_time_ms{0.0};
  double linearization_time_ms{0.0};
  double cost{0.0};
  double constraint_violation{0.0};
};

/**
 * @brief NMPC Configuration
 * 
 * These parameters define the OCP (Optimal Control Problem) structure.
 * When using acados code generation, these should match the Python definition.
 */
struct NMPCConfig
{
  // Horizon settings
  double prediction_horizon_sec{1.5};  // Total prediction horizon [s]
  int N{15};  // Number of shooting intervals
  double dt{0.1};  // Time step (computed: horizon/N)
  
  // Vehicle model parameters
  double wheelbase{0.33};        // [m]
  double mass{3.5};              // [kg] 
  double inertia{0.04};          // [kg*m^2] yaw moment of inertia
  double lf{0.17};               // [m] front axle to CG
  double lr{0.16};               // [m] rear axle to CG
  
  // Tire parameters (Pacejka magic formula)
  double tire_B{2.5};            // Stiffness factor
  double tire_C{1.3};            // Shape factor  
  double tire_D{4.5};            // Peak factor (max lateral force)
  
  // Cost function weights (Q matrix diagonal)
  double w_x{1.0};               // Position x tracking
  double w_y{10.0};              // Position y (lateral) tracking
  double w_yaw{10.0};            // Heading tracking
  double w_v{3.0};               // Velocity tracking
  
  // Control weights (R matrix diagonal)
  double w_steering{0.5};        // Steering effort
  double w_acceleration{0.3};    // Acceleration effort
  
  // Control rate weights (for smooth control)
  double w_steering_rate{600.0}; // CRITICAL: steering rate penalty
  double w_acceleration_rate{60.0};
  
  // Terminal cost weights
  double w_terminal{30.0};
  
  // Lateral tolerance (soft constraint tube)
  double lateral_tolerance{0.25};  // [m]
  
  // Constraints
  double max_steering{0.436};      // [rad] ~25 degrees
  double max_steering_rate{1.8};   // [rad/s]
  double max_acceleration{4.0};    // [m/s^2]
  double min_acceleration{-6.0};   // [m/s^2]
  double max_speed{6.0};           // [m/s]
  double min_speed{0.0};           // [m/s]
  
  // Solver settings
  int max_sqp_iterations{10};      // RTI typically uses 1
  int max_qp_iterations{100};
  double convergence_tolerance{1e-6};
  double timeout_ms{20.0};         // RTI target: < 20ms
  
  // RTI settings
  bool use_rti{true};              // Use Real-Time Iteration
  int rti_phase{0};                // 0=preparation+feedback, 1=preparation only, 2=feedback only
  
  // Latency compensation
  double latency_compensation_sec{0.05};  // [s] total system delay
};

/**
 * @brief Solution structure
 */
struct NMPCSolution
{
  ControlInput control;
  std::vector<VehicleState> predicted_trajectory;
  SolverStats stats;
  bool feasible{false};
};

/**
 * @brief Abstract interface for acados NMPC solver
 * 
 * This interface allows for:
 * 1. Mock implementation for testing without acados
 * 2. Easy swap between different solver implementations
 * 3. Thread-safe operations for multi-threaded ROS 2 nodes
 */
class IAcadosNMPCSolver
{
public:
  virtual ~IAcadosNMPCSolver() = default;
  
  /**
   * @brief Initialize the solver (call during on_configure)
   * @param config NMPC configuration
   * @return true if initialization successful
   */
  virtual bool initialize(const NMPCConfig& config) = 0;
  
  /**
   * @brief Clean up solver resources (call during on_cleanup)
   */
  virtual void cleanup() = 0;
  
  /**
   * @brief Set initial state for the OCP
   * @param state Current vehicle state (after latency compensation)
   */
  virtual void setInitialState(const VehicleState& state) = 0;
  
  /**
   * @brief Set reference trajectory
   * @param reference Vector of reference points for each shooting node
   */
  virtual void setReference(const std::vector<ReferencePoint>& reference) = 0;
  
  /**
   * @brief Set previous control for rate constraints
   * @param prev_control Previous control input
   */
  virtual void setPreviousControl(const ControlInput& prev_control) = 0;
  
  /**
   * @brief Solve the OCP
   * @return Solution containing optimal control, trajectory, and statistics
   */
  virtual NMPCSolution solve() = 0;
  
  /**
   * @brief Prepare phase for RTI (linearization only)
   * @note Call this after setReference but before feedback phase
   */
  virtual void preparationPhase() = 0;
  
  /**
   * @brief Feedback phase for RTI (QP solve only)
   * @param state Current state at feedback time
   * @return Solution from feedback phase
   * @note For minimum latency RTI workflow
   */
  virtual NMPCSolution feedbackPhase(const VehicleState& state) = 0;
  
  /**
   * @brief Warm start from previous solution
   * @note Called automatically if using RTI
   */
  virtual void warmStart() = 0;
  
  /**
   * @brief Reset solver state (e.g., after loss of localization)
   */
  virtual void reset() = 0;
  
  /**
   * @brief Check if solver is initialized
   */
  virtual bool isInitialized() const = 0;
  
  /**
   * @brief Get solver configuration
   */
  virtual const NMPCConfig& getConfig() const = 0;
};

/**
 * @brief Fallback NMPC solver using hand-written SQP
 * 
 * This is a fallback implementation that doesn't require acados installation.
 * It uses the same algorithm as the existing nmpc_engine_node.cpp but wrapped
 * in the interface for compatibility.
 * 
 * Use this when:
 * - acados is not installed
 * - For testing/simulation where real-time isn't critical
 * - As a baseline comparison
 */
class FallbackNMPCSolver : public IAcadosNMPCSolver
{
public:
  FallbackNMPCSolver();
  ~FallbackNMPCSolver() override;
  
  bool initialize(const NMPCConfig& config) override;
  void cleanup() override;
  void setInitialState(const VehicleState& state) override;
  void setReference(const std::vector<ReferencePoint>& reference) override;
  void setPreviousControl(const ControlInput& prev_control) override;
  NMPCSolution solve() override;
  void preparationPhase() override;
  NMPCSolution feedbackPhase(const VehicleState& state) override;
  void warmStart() override;
  void reset() override;
  bool isInitialized() const override;
  const NMPCConfig& getConfig() const override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

#ifdef ACADOS_AVAILABLE
/**
 * @brief Real acados NMPC solver implementation
 * 
 * This class wraps the generated acados C code and provides the
 * IAcadosNMPCSolver interface. 
 * 
 * Prerequisites:
 * 1. acados installed (including HPIPM, BLASFEO)
 * 2. Generated C code from Python OCP definition
 * 3. Linked against acados and generated libraries
 * 
 * The OCP is defined in Python using CasADi symbolic expressions,
 * then code is generated to C for maximum performance.
 */
class AcadosNMPCSolver : public IAcadosNMPCSolver
{
public:
  AcadosNMPCSolver();
  ~AcadosNMPCSolver() override;
  
  bool initialize(const NMPCConfig& config) override;
  void cleanup() override;
  void setInitialState(const VehicleState& state) override;
  void setReference(const std::vector<ReferencePoint>& reference) override;
  void setPreviousControl(const ControlInput& prev_control) override;
  NMPCSolution solve() override;
  void preparationPhase() override;
  NMPCSolution feedbackPhase(const VehicleState& state) override;
  void warmStart() override;
  void reset() override;
  bool isInitialized() const override;
  const NMPCConfig& getConfig() const override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
#endif  // ACADOS_AVAILABLE

/**
 * @brief Factory function to create appropriate solver
 * 
 * @param prefer_acados If true, try to use acados (if available)
 * @return Unique pointer to solver instance
 */
inline std::unique_ptr<IAcadosNMPCSolver> createNMPCSolver(bool prefer_acados = true)
{
#ifdef ACADOS_AVAILABLE
  if (prefer_acados) {
    return std::make_unique<AcadosNMPCSolver>();
  }
#else
  (void)prefer_acados;  // Suppress unused warning
#endif
  return std::make_unique<FallbackNMPCSolver>();
}

}  // namespace acados_interface

#endif  // CONTROL_PKG__ACADOS_NMPC_INTERFACE_HPP_
