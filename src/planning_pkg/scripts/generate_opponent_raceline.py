#!/usr/bin/env python3
"""
Generate opponent raceline with outside bias to leave room for inside overtakes.

This script generates a raceline positioned more toward the outside of the track
(outer wall) to create space for the ego car to perform inside overtakes.
The opponent follows this outside-biased raceline, while the ego can use the
inside lane for clean, textbook overtaking maneuvers.
"""

import sys
import csv
import math
import argparse
from pathlib import Path


def read_centerline_with_bounds(filepath):
    """Read centerline CSV with boundary information."""
    points = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append({
                'x': float(row['x']),
                'y': float(row['y']),
                'd_left': float(row['d_left']),
                'd_right': float(row['d_right']),
                'psi': float(row['psi'])
            })
    return points


def compute_gradient(values, ds):
    """Compute numerical gradient of a list of values."""
    n = len(values)
    grad = [0.0] * n
    
    if n < 2:
        return grad
    
    # Forward difference at start
    grad[0] = (values[1] - values[0]) / ds
    
    # Central difference in middle
    for i in range(1, n - 1):
        grad[i] = (values[i + 1] - values[i - 1]) / (2 * ds)
    
    # Backward difference at end
    grad[-1] = (values[-1] - values[-2]) / ds
    
    return grad


def resample_centerline(points, ds=0.5):
    """Resample centerline to uniform spacing."""
    if len(points) < 2:
        return points
    
    # Compute cumulative arc length
    s = [0.0]
    for i in range(1, len(points)):
        dx = points[i]['x'] - points[i-1]['x']
        dy = points[i]['y'] - points[i-1]['y']
        s.append(s[-1] + math.hypot(dx, dy))
    
    total_length = s[-1]
    
    # Generate new sample points
    resampled = []
    s_new = 0.0
    
    while s_new < total_length:
        # Find segment containing s_new
        for i in range(len(s) - 1):
            if s[i] <= s_new < s[i + 1]:
                # Linear interpolation
                t = (s_new - s[i]) / max(1e-12, s[i + 1] - s[i])
                
                resampled.append({
                    'x': (1 - t) * points[i]['x'] + t * points[i + 1]['x'],
                    'y': (1 - t) * points[i]['y'] + t * points[i + 1]['y'],
                    'd_left': (1 - t) * points[i]['d_left'] + t * points[i + 1]['d_left'],
                    'd_right': (1 - t) * points[i]['d_right'] + t * points[i + 1]['d_right'],
                    'psi': (1 - t) * points[i]['psi'] + t * points[i + 1]['psi']
                })
                break
        
        s_new += ds
    
    return resampled


def generate_opponent_raceline(centerline_file, output_file, lane_position=-0.3, 
                                wall_margin=0.3, ds=0.5):
    """
    Generate opponent raceline with outside bias.
    
    Args:
        centerline_file: Path to centerline CSV with boundaries
        output_file: Output path for opponent raceline
        lane_position: Position between walls (-1.0=outer, 0.0=center, 1.0=inner)
                      Default: -0.3 for moderate outside bias
        wall_margin: Minimum distance from walls (meters)
        ds: Sample spacing (meters)
    """
    print(f"Loading centerline from: {centerline_file}")
    points = read_centerline_with_bounds(centerline_file)
    print(f"Loaded {len(points)} points")
    
    # Resample to uniform spacing
    print(f"Resampling with spacing: {ds}m")
    points = resample_centerline(points, ds)
    print(f"Resampled to {len(points)} points")
    
    # Extract x, y coordinates
    px = [p['x'] for p in points]
    py = [p['y'] for p in points]
    
    # Compute tangent vectors
    d1x = compute_gradient(px, ds)
    d1y = compute_gradient(py, ds)
    
    # Apply lateral offset based on lane_position
    print(f"Applying lane_position={lane_position} with wall_margin={wall_margin}m")
    
    for i in range(len(points)):
        # Normal vector pointing left (toward outer wall for CCW track)
        norm = math.sqrt(d1x[i]**2 + d1y[i]**2)
        
        if norm > 1e-12:
            nx = -d1y[i] / norm
            ny = d1x[i] / norm
            
            # Calculate available space considering wall margin
            available_left = max(0.0, points[i]['d_left'] - wall_margin)
            available_right = max(0.0, points[i]['d_right'] - wall_margin)
            
            # Calculate offset
            # lane_position < 0 -> move toward left/outer wall
            # lane_position > 0 -> move toward right/inner wall
            offset = 0.0
            if lane_position < 0.0:
                # Move toward left/outer wall
                offset = -lane_position * available_left
            elif lane_position > 0.0:
                # Move toward right/inner wall
                offset = -lane_position * available_right
            
            # Apply offset
            points[i]['x'] += offset * nx
            points[i]['y'] += offset * ny
    
    # Recalculate psi and other parameters
    px = [p['x'] for p in points]
    py = [p['y'] for p in points]
    d1x = compute_gradient(px, ds)
    d1y = compute_gradient(py, ds)
    d2x = compute_gradient(d1x, ds)
    d2y = compute_gradient(d1y, ds)
    
    # Update psi and compute kappa
    s = [0.0]
    for i in range(len(points)):
        points[i]['psi'] = math.atan2(d1y[i], d1x[i] + 1e-12)
        
        # Compute curvature
        num = d1x[i] * d2y[i] - d1y[i] * d2x[i]
        den = (d1x[i]**2 + d1y[i]**2)**1.5
        points[i]['kappa'] = num / max(1e-12, den)
        
        # Compute arc length
        if i > 0:
            dx = points[i]['x'] - points[i-1]['x']
            dy = points[i]['y'] - points[i-1]['y']
            s.append(s[-1] + math.hypot(dx, dy))
        
        points[i]['s'] = s[i] if i < len(s) else s[-1]
    
    # Set constant velocity (opponent speed)
    # This will be overridden by opponent_publisher's speed parameter
    for p in points:
        p['v'] = 3.0  # Default opponent speed (m/s)
    
    # Write output
    print(f"Writing opponent raceline to: {output_file}")
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['s', 'x', 'y', 'psi', 'kappa', 'v'])
        for p in points:
            writer.writerow([
                p['s'],
                p['x'],
                p['y'],
                p['psi'],
                p['kappa'],
                p['v']
            ])
    
    print(f"✓ Opponent raceline generated successfully")
    print(f"  Points: {len(points)}")
    print(f"  Track length: {s[-1]:.2f}m")
    print(f"  Lane position: {lane_position} (outside bias)")
    print(f"  Wall margin: {wall_margin}m")


def main():
    parser = argparse.ArgumentParser(
        description='Generate opponent raceline with outside bias for inside overtakes'
    )
    parser.add_argument(
        '--centerline',
        default='tracks/centerline_with_bounds.csv',
        help='Input centerline CSV with boundaries'
    )
    parser.add_argument(
        '--output',
        default='data/opponent_raceline.csv',
        help='Output raceline CSV'
    )
    parser.add_argument(
        '--lane-position',
        type=float,
        default=-0.3,
        help='Position between walls (-1.0=outer, 0.0=center, 1.0=inner). '
             'Negative values create outside bias for opponent. Default: -0.3'
    )
    parser.add_argument(
        '--wall-margin',
        type=float,
        default=0.3,
        help='Minimum distance from walls in meters. Default: 0.3'
    )
    parser.add_argument(
        '--ds',
        type=float,
        default=0.5,
        help='Sample spacing in meters. Default: 0.5'
    )
    
    args = parser.parse_args()
    
    # Get planning_pkg directory
    script_dir = Path(__file__).parent
    pkg_dir = script_dir.parent
    
    # Resolve paths relative to package
    centerline_path = pkg_dir / args.centerline
    output_path = pkg_dir / args.output
    
    if not centerline_path.exists():
        print(f"Error: Centerline file not found: {centerline_path}", file=sys.stderr)
        return 1
    
    # Create output directory if needed
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    try:
        generate_opponent_raceline(
            str(centerline_path),
            str(output_path),
            lane_position=args.lane_position,
            wall_margin=args.wall_margin,
            ds=args.ds
        )
        return 0
    except Exception as e:
        print(f"Error generating opponent raceline: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
