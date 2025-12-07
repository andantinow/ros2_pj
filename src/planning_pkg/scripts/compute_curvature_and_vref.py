#!/usr/bin/env python3
"""
Compute curvature (kappa) and reference velocity (v_ref) for raceline.

This script:
1. Reads an existing raceline CSV with x, y, psi coordinates
2. Computes curvature κ(s) from the path geometry
3. Computes v_ref(s) based on curvature using the formula:
   v_ref = min(v_max, sqrt(a_lat_max / (|κ| + ε)))
4. Writes updated raceline CSV with proper kappa and v_ref values

Reference:
- Curvature computation: κ ≈ dψ/ds (change in heading per unit arc length)
- Speed computation: v_max for straights, reduced in corners based on lateral acceleration limit
"""

import sys
import math
import csv
import argparse
from pathlib import Path


def normalize_angle(angle):
    """Normalize angle to [-π, π]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def compute_curvature_and_vref(input_csv, output_csv, v_max=5.5, a_lat_max=4.0, epsilon=0.01):
    """
    Compute curvature and reference velocity for raceline.
    
    Args:
        input_csv: Path to input raceline CSV
        output_csv: Path to output raceline CSV
        v_max: Maximum velocity on straights (m/s)
        a_lat_max: Maximum lateral acceleration (m/s²)
        epsilon: Small value to avoid division by zero
    
    Returns:
        Dictionary with statistics
    """
    # Read input CSV
    waypoints = []
    with open(input_csv, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            waypoints.append({
                's': float(row['s']),
                'x': float(row['x']),
                'y': float(row['y']),
                'psi': float(row['psi']),
                'kappa': 0.0,  # Will be computed
                'v_ref': 0.0   # Will be computed
            })
    
    n = len(waypoints)
    print(f"Loaded {n} waypoints from {input_csv}")
    
    # Compute curvature for each waypoint
    # Method: κ ≈ Δψ / Δs (change in heading over arc length)
    for i in range(n):
        # Get previous and next indices (with wraparound for closed track)
        i_prev = (i - 1) % n
        i_next = (i + 1) % n
        
        # Compute heading change
        psi_prev = waypoints[i_prev]['psi']
        psi_curr = waypoints[i]['psi']
        psi_next = waypoints[i_next]['psi']
        
        # Use central difference for better accuracy
        dpsi = normalize_angle(psi_next - psi_prev)
        
        # Compute arc length difference
        ds = waypoints[i_next]['s'] - waypoints[i_prev]['s']
        
        # Handle wraparound at end of track
        if ds < 0:
            # Wraparound case: add track length
            track_length = waypoints[-1]['s']
            ds += track_length
        
        # Compute curvature (avoid division by very small ds)
        if abs(ds) > 1e-6:
            kappa = dpsi / ds
        else:
            kappa = 0.0
        
        waypoints[i]['kappa'] = kappa
    
    # Compute v_ref based on curvature
    # Formula: v_ref = min(v_max, sqrt(a_lat_max / (|κ| + ε)))
    # This ensures the car can maintain the required lateral acceleration
    for i in range(n):
        kappa = waypoints[i]['kappa']
        abs_kappa = abs(kappa)
        
        # Compute speed limit based on curvature
        if abs_kappa < epsilon:
            # Nearly straight: use max speed
            v_ref = v_max
        else:
            # Corner: limit speed based on lateral acceleration
            # v²·κ ≤ a_lat_max  =>  v ≤ sqrt(a_lat_max / κ)
            v_curvature = math.sqrt(a_lat_max / (abs_kappa + epsilon))
            v_ref = min(v_max, v_curvature)
        
        waypoints[i]['v_ref'] = v_ref
    
    # Write output CSV
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['s', 'x', 'y', 'psi', 'kappa', 'v_ref'])
        
        for wp in waypoints:
            writer.writerow([
                f"{wp['s']:.6f}",
                f"{wp['x']:.6f}",
                f"{wp['y']:.6f}",
                f"{wp['psi']:.6f}",
                f"{wp['kappa']:.6f}",
                f"{wp['v_ref']:.6f}"
            ])
    
    # Compute statistics
    kappas = [abs(wp['kappa']) for wp in waypoints]
    vrefs = [wp['v_ref'] for wp in waypoints]
    
    stats = {
        'n_points': n,
        'kappa_min': min(kappas),
        'kappa_max': max(kappas),
        'kappa_mean': sum(kappas) / n,
        'vref_min': min(vrefs),
        'vref_max': max(vrefs),
        'vref_mean': sum(vrefs) / n,
        'straight_points': sum(1 for k in kappas if k < 0.1),
        'corner_points': sum(1 for k in kappas if k > 0.5)
    }
    
    return stats


def main():
    parser = argparse.ArgumentParser(description='Compute curvature and v_ref for raceline')
    parser.add_argument('--input', type=str,
                        default='data/raceline.csv',
                        help='Input raceline CSV')
    parser.add_argument('--output', type=str,
                        default='data/raceline.csv',
                        help='Output raceline CSV (default: overwrite input)')
    parser.add_argument('--v-max', type=float,
                        default=5.5,
                        help='Maximum velocity on straights (m/s, default: 5.5)')
    parser.add_argument('--a-lat-max', type=float,
                        default=4.0,
                        help='Maximum lateral acceleration (m/s², default: 4.0)')
    parser.add_argument('--epsilon', type=float,
                        default=0.01,
                        help='Small value to avoid division by zero (default: 0.01)')
    
    args = parser.parse_args()
    
    # Resolve paths relative to script directory
    script_dir = Path(__file__).parent.parent
    input_path = script_dir / args.input
    output_path = script_dir / args.output
    
    if not input_path.exists():
        print(f"Error: Input file not found: {input_path}")
        sys.exit(1)
    
    print("=" * 70)
    print("Computing curvature and v_ref for raceline")
    print("=" * 70)
    print(f"Input:  {input_path}")
    print(f"Output: {output_path}")
    print(f"v_max = {args.v_max:.2f} m/s (max speed on straights)")
    print(f"a_lat_max = {args.a_lat_max:.2f} m/s² (max lateral acceleration)")
    print(f"epsilon = {args.epsilon:.4f}")
    print()
    
    # Compute
    stats = compute_curvature_and_vref(
        input_path, output_path,
        v_max=args.v_max,
        a_lat_max=args.a_lat_max,
        epsilon=args.epsilon
    )
    
    print()
    print("=" * 70)
    print("Results:")
    print("=" * 70)
    print(f"Total waypoints: {stats['n_points']}")
    print()
    print("Curvature (κ) statistics:")
    print(f"  Min |κ|:  {stats['kappa_min']:.6f} (1/m)")
    print(f"  Max |κ|:  {stats['kappa_max']:.6f} (1/m)")
    print(f"  Mean |κ|: {stats['kappa_mean']:.6f} (1/m)")
    print(f"  Straight segments (|κ| < 0.1): {stats['straight_points']} points ({100*stats['straight_points']/stats['n_points']:.1f}%)")
    print(f"  Corner segments (|κ| > 0.5):   {stats['corner_points']} points ({100*stats['corner_points']/stats['n_points']:.1f}%)")
    print()
    print("Reference velocity (v_ref) statistics:")
    print(f"  Min v_ref:  {stats['vref_min']:.2f} m/s")
    print(f"  Max v_ref:  {stats['vref_max']:.2f} m/s")
    print(f"  Mean v_ref: {stats['vref_mean']:.2f} m/s")
    print()
    print("✓ Raceline updated successfully!")
    print(f"✓ Saved to: {output_path}")


if __name__ == '__main__':
    main()
