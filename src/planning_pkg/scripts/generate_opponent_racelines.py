#!/usr/bin/env python3
"""
Generate opponent outer and inner racelines from base raceline.

This script creates two opponent racelines:
- opponent_outer.csv: Offset to the outer side of the track
- opponent_inner.csv: Offset to the inner side of the track

The opponent runs at approximately half the ego's speed to enable overtaking.
"""

import sys
import math
import argparse
from pathlib import Path


def read_raceline_csv(csv_path):
    """Read raceline CSV file and return waypoints."""
    waypoints = []
    
    with open(csv_path, 'r') as f:
        lines = f.readlines()
    
    # Parse header
    header = lines[0].strip().split(',')
    
    # Expected columns: s, x, y, psi, kappa, v_ref (or v)
    try:
        s_idx = header.index('s')
        x_idx = header.index('x')
        y_idx = header.index('y')
        psi_idx = header.index('psi')
        kappa_idx = header.index('kappa')
        
        # Try v_ref first, then v
        try:
            v_idx = header.index('v_ref')
            v_col = 'v_ref'
        except ValueError:
            v_idx = header.index('v')
            v_col = 'v'
    except ValueError as e:
        print(f"Error: Missing required column in CSV. Expected columns: s, x, y, psi, kappa, v_ref (or v). Missing: {e}")
        sys.exit(1)
    
    # Parse data rows
    for line in lines[1:]:
        if not line.strip():
            continue
        
        parts = line.strip().split(',')
        waypoint = {
            's': float(parts[s_idx]),
            'x': float(parts[x_idx]),
            'y': float(parts[y_idx]),
            'psi': float(parts[psi_idx]),
            'kappa': float(parts[kappa_idx]),
            'v': float(parts[v_idx])
        }
        waypoints.append(waypoint)
    
    return waypoints, v_col


def compute_curvature_based_speed(kappa, v_max_straight=6.5, v_min_corner=2.5, k1=0.2, k2=0.8):
    """
    Compute speed based on curvature using the NMPC policy.
    
    Args:
        kappa: Curvature value (1/m)
        v_max_straight: Maximum speed on straights (m/s)
        v_min_corner: Minimum speed in tight corners (m/s)
        k1: Curvature threshold for straight (1/m)
        k2: Curvature threshold for tight corner (1/m)
    
    Returns:
        float: Reference speed in m/s (interpolated between v_min_corner and v_max_straight)
    """
    abs_kappa = abs(kappa)
    
    if abs_kappa < k1:
        # Straight section
        return v_max_straight
    elif abs_kappa > k2:
        # Tight corner
        return v_min_corner
    else:
        # Transition zone - linear interpolation
        t = (abs_kappa - k1) / (k2 - k1)
        return v_max_straight + t * (v_min_corner - v_max_straight)


def generate_opponent_raceline(waypoints, lateral_offset, speed_factor=0.5, use_curvature_speed=True):
    """
    Generate opponent raceline with lateral offset and reduced speed.
    
    Args:
        waypoints: List of waypoint dictionaries from base raceline
        lateral_offset: Lateral offset in meters (positive = left/outer, negative = right/inner)
        speed_factor: Speed multiplier (default 0.5 for half speed)
        use_curvature_speed: If True and base speeds are zero, compute from curvature
    
    Returns:
        List of opponent waypoints
    """
    opponent_waypoints = []
    n = len(waypoints)
    
    # Check if base raceline has meaningful speeds
    has_speed_data = any(wp['v'] > 0.1 for wp in waypoints)
    
    for i in range(n):
        wp = waypoints[i]
        
        # Get next waypoint for heading calculation (with wraparound)
        next_idx = (i + 1) % n
        wp_next = waypoints[next_idx]
        
        # Calculate heading from current to next waypoint
        # This gives us the direction of travel
        dx = wp_next['x'] - wp['x']
        dy = wp_next['y'] - wp['y']
        theta = math.atan2(dy, dx)
        
        # Calculate normal direction (perpendicular to heading)
        # Add π/2 to get left-pointing normal
        normal_angle = theta + math.pi / 2.0
        
        # Apply lateral offset in normal direction
        x_offset = wp['x'] + lateral_offset * math.cos(normal_angle)
        y_offset = wp['y'] + lateral_offset * math.sin(normal_angle)
        
        # Determine speed
        if has_speed_data:
            # Use provided speed data
            v_base = wp['v']
        elif use_curvature_speed:
            # Compute from curvature
            v_base = compute_curvature_based_speed(wp['kappa'])
        else:
            # Default constant speed
            v_base = 3.0  # m/s
        
        # Apply speed reduction factor for opponent
        v_opponent = v_base * speed_factor
        
        # Create opponent waypoint
        opp_wp = {
            's': wp['s'],
            'x': x_offset,
            'y': y_offset,
            'psi': wp['psi'],  # Keep original heading
            'kappa': wp['kappa'],  # Keep original curvature
            'v': v_opponent
        }
        
        opponent_waypoints.append(opp_wp)
    
    return opponent_waypoints


def write_opponent_csv(waypoints, output_path):
    """Write opponent raceline to CSV file."""
    with open(output_path, 'w') as f:
        # Write header
        f.write('s,x,y,psi,kappa,v\n')
        
        # Write data
        for wp in waypoints:
            f.write(f"{wp['s']},{wp['x']},{wp['y']},{wp['psi']},{wp['kappa']},{wp['v']}\n")
    
    print(f"✓ Generated: {output_path}")


def main():
    """
    Main entry point for opponent raceline generation.
    
    Workflow:
    1. Parse command-line arguments
    2. Read base raceline CSV
    3. Generate opponent_outer.csv and opponent_inner.csv with lateral offsets
    4. Apply speed reduction factor (default 50%)
    5. Save generated files to output directory
    
    Side effects:
    - Creates opponent_outer.csv and opponent_inner.csv in output directory
    - Prints progress and statistics to stdout
    """
    parser = argparse.ArgumentParser(description='Generate opponent racelines from base raceline')
    parser.add_argument('--input', type=str, 
                        default='data/raceline.csv',
                        help='Input raceline CSV file (default: data/raceline.csv)')
    parser.add_argument('--outer-offset', type=float, 
                        default=0.6,
                        help='Lateral offset for outer line in meters (default: 0.6)')
    parser.add_argument('--inner-offset', type=float, 
                        default=-0.6,
                        help='Lateral offset for inner line in meters (default: -0.6)')
    parser.add_argument('--speed-factor', type=float, 
                        default=0.31,
                        help='Speed reduction factor (default: 0.31 for ~1.0 m/s average speed)')
    parser.add_argument('--output-dir', type=str, 
                        default='data',
                        help='Output directory (default: data)')
    
    args = parser.parse_args()
    
    # Resolve paths
    script_dir = Path(__file__).parent.parent
    input_path = script_dir / args.input
    output_dir = script_dir / args.output_dir
    
    if not input_path.exists():
        print(f"Error: Input file not found: {input_path}")
        sys.exit(1)
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Reading base raceline from: {input_path}")
    waypoints, v_col = read_raceline_csv(input_path)
    print(f"  Found {len(waypoints)} waypoints")
    print(f"  Using velocity column: {v_col}")
    
    # Calculate average speed
    avg_speed = sum(wp['v'] for wp in waypoints) / len(waypoints)
    has_speed_data = any(wp['v'] > 0.1 for wp in waypoints)
    
    if has_speed_data:
        print(f"  Average base speed: {avg_speed:.2f} m/s")
        print(f"  Opponent speed will be: {avg_speed * args.speed_factor:.2f} m/s (×{args.speed_factor})")
    else:
        print(f"  Base raceline has no speed data (all zeros)")
        print(f"  Will compute speeds from curvature:")
        print(f"    - Straights (κ < 0.2): 6.5 m/s → {6.5 * args.speed_factor:.2f} m/s")
        print(f"    - Corners (κ > 0.8): 2.5 m/s → {2.5 * args.speed_factor:.2f} m/s")
        print(f"    - Transitions: interpolated")
    
    # Generate outer raceline
    print(f"\nGenerating outer raceline (offset: {args.outer_offset:.2f}m)...")
    outer_waypoints = generate_opponent_raceline(waypoints, args.outer_offset, args.speed_factor)
    outer_output = output_dir / 'opponent_outer.csv'
    write_opponent_csv(outer_waypoints, outer_output)
    
    # Show speed stats for outer
    outer_speeds = [wp['v'] for wp in outer_waypoints]
    print(f"  Speed range: {min(outer_speeds):.2f} - {max(outer_speeds):.2f} m/s")
    
    # Generate inner raceline
    print(f"\nGenerating inner raceline (offset: {args.inner_offset:.2f}m)...")
    inner_waypoints = generate_opponent_raceline(waypoints, args.inner_offset, args.speed_factor)
    inner_output = output_dir / 'opponent_inner.csv'
    write_opponent_csv(inner_waypoints, inner_output)
    
    # Show speed stats for inner
    inner_speeds = [wp['v'] for wp in inner_waypoints]
    print(f"  Speed range: {min(inner_speeds):.2f} - {max(inner_speeds):.2f} m/s")
    
    print("\n" + "="*60)
    print("SUCCESS: Opponent racelines generated!")
    print("="*60)
    print("\nNext steps:")
    print("1. Verify the paths are within track boundaries")
    print("2. Configure opponent_publisher to use one of these paths")
    print("3. Launch with line_mode='outer' or 'inner'")
    print("\nGenerated files:")
    print(f"  - {outer_output}")
    print(f"  - {inner_output}")


if __name__ == '__main__':
    main()
