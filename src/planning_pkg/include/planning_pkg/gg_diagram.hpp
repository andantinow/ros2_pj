#ifndef PLANNING_PKG__GG_DIAGRAM_HPP_
#define PLANNING_PKG__GG_DIAGRAM_HPP_

#include <cmath>
#include <limits>

namespace planning_pkg
{

/**
 * @brief G-G Diagram (Friction Circle) constraint
 * 
 * Physical constraint: a_x^2 + a_y^2 <= (μ*g)^2
 * This represents the maximum combined acceleration the tires can provide.
 */
class GGDiagram
{
public:
  GGDiagram(double mu, double g = 9.80665)
  : mu_(mu), g_(g), max_accel_(mu * g)
  {}

  /**
   * @brief Check if given accelerations are within friction circle
   * @param a_lon Longitudinal acceleration [m/s^2]
   * @param a_lat Lateral acceleration [m/s^2]
   * @return true if within limits
   */
  bool is_valid(double a_lon, double a_lat) const
  {
    double total_accel_sq = a_lon * a_lon + a_lat * a_lat;
    return total_accel_sq <= max_accel_ * max_accel_;
  }

  /**
   * @brief Get maximum longitudinal acceleration given lateral acceleration
   * @param a_lat Lateral acceleration [m/s^2]
   * @return Maximum allowed longitudinal acceleration [m/s^2]
   */
  double max_longitudinal_accel(double a_lat) const
  {
    double a_lat_sq = a_lat * a_lat;
    double max_accel_sq = max_accel_ * max_accel_;
    if (a_lat_sq >= max_accel_sq) {
      return 0.0;  // No longitudinal acceleration possible
    }
    return std::sqrt(max_accel_sq - a_lat_sq);
  }

  /**
   * @brief Get maximum lateral acceleration given longitudinal acceleration
   * @param a_lon Longitudinal acceleration [m/s^2]
   * @return Maximum allowed lateral acceleration [m/s^2]
   */
  double max_lateral_accel(double a_lon) const
  {
    double a_lon_sq = a_lon * a_lon;
    double max_accel_sq = max_accel_ * max_accel_;
    if (a_lon_sq >= max_accel_sq) {
      return 0.0;  // No lateral acceleration possible
    }
    return std::sqrt(max_accel_sq - a_lon_sq);
  }

  /**
   * @brief Calculate maximum speed for given curvature considering G-G diagram
   * @param kappa Path curvature [1/m]
   * @param a_lon_desired Desired longitudinal acceleration [m/s^2] (can be negative for braking)
   * @return Maximum allowed speed [m/s]
   */
  double max_speed_for_curvature(double kappa, double a_lon_desired = 0.0) const
  {
    if (std::abs(kappa) < 1e-6) {
      // Straight line: only longitudinal acceleration limit
      double max_a_lon = max_longitudinal_accel(0.0);
      if (a_lon_desired > max_a_lon) {
        return 0.0;  // Desired acceleration exceeds limit
      }
      return std::numeric_limits<double>::max();  // No curvature limit
    }

    // For curved path: v^2 * kappa = a_lat
    // Constraint: a_lon^2 + (v^2 * kappa)^2 <= (μ*g)^2
    // Solve for v: v^2 * kappa <= sqrt((μ*g)^2 - a_lon^2)
    
    double max_a_lat = max_lateral_accel(a_lon_desired);
    if (max_a_lat <= 0.0) {
      return 0.0;  // Cannot maintain desired longitudinal acceleration
    }

    // v^2 * |kappa| <= max_a_lat
    // v <= sqrt(max_a_lat / |kappa|)
    return std::sqrt(max_a_lat / std::abs(kappa));
  }

  /**
   * @brief Get maximum acceleration (magnitude)
   */
  double max_accel() const { return max_accel_; }

  /**
   * @brief Get friction coefficient
   */
  double mu() const { return mu_; }

private:
  double mu_;        // Friction coefficient
  double g_;         // Gravitational acceleration
  double max_accel_; // μ*g
};

}  // namespace planning_pkg

#endif  // PLANNING_PKG__GG_DIAGRAM_HPP_

