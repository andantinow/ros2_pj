#!/usr/bin/env python3
"""
Generate opponent raceline with OUT-IN-OUT racing style.

This script generates a racing line for the opponent that follows the classic
OUT-IN-OUT racing line through corners:
- OUTSIDE approach before corner entry (wide line for better vision)
- INSIDE at apex (cutting close to the inner wall/apex)
- OUTSIDE on corner exit (wide exit for better acceleration)

The opponent follows this OUT-IN-OUT raceline, while the ego car uses the
global raceline + overtaking lanes for strategic overtaking maneuvers.

Key features:
- Curvature-based corner detection
- Dynamic lateral offset calculation based on corner phase
- Small margin from walls for safety
- Creates clear overtaking opportunities for ego car on alternative lines
"""

import sys
import csv
import math
import argparse
from pathlib import Path


# Constants for numerical stability and defaults
EPSILON = 1e-12  # Small epsilon to prevent division by zero
DEFAULT_OPPONENT_SPEED = 2.5  # Default opponent speed (m/s) - v6.0: reduced from 3.0 to match opponent_publisher
CURVATURE_THRESHOLD_CORNER = 0.08  # Threshold to detect corners
MAX_CURVATURE_NORMALIZATION = 0.3  # Maximum curvature for normalization
STRAIGHT_OUTSIDE_BIAS = 0.3  # Outside bias factor on straight sections


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
                t = (s_new - s[i]) / max(EPSILON, s[i + 1] - s[i])
                
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
                                wall_margin=0.25, ds=0.5, out_in_out_strength=0.75):
    """
    Generate opponent raceline with OUT-IN-OUT racing style through corners.
    
    The opponent follows a classic racing line that:
    - Stays OUTSIDE before corners (wider entry)
    - Cuts INSIDE at apex (tight to inner wall/apex)
    - Exits OUTSIDE after corners (wide exit for speed)
    - Maintains small margin from walls for safety
    - Creates clear space for ego car to overtake on alternative lines
    
    Args:
        centerline_file: Path to centerline CSV with boundaries
        output_file: Output path for opponent raceline
        lane_position: Base position between walls (-1.0=outer, 0.0=center, 1.0=inner)
                      Default: -0.3 for stronger outside bias on straights (v6.0: increased from -0.2)
        wall_margin: Minimum distance from walls (meters) (v6.0: reduced to 0.25 for closer racing line)
        ds: Sample spacing (meters)
        out_in_out_strength: Strength of OUT-IN-OUT bias (0.0-1.0), default 0.75 (v6.0: increased from 0.5)
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
    
    # Compute curvature to detect corners
    d2x = compute_gradient(d1x, ds)
    d2y = compute_gradient(d1y, ds)
    curvatures = []
    for i in range(len(points)):
        num = d1x[i] * d2y[i] - d1y[i] * d2x[i]
        den = (d1x[i]**2 + d1y[i]**2)**1.5
        kappa = num / max(EPSILON, den)
        curvatures.append(kappa)
    
    # Smooth curvature to reduce noise
    window_size = 5
    smoothed_curvatures = []
    for i in range(len(curvatures)):
        start = max(0, i - window_size)
        end = min(len(curvatures), i + window_size + 1)
        smoothed_curvatures.append(sum(curvatures[start:end]) / (end - start))
    
    # Apply OUT-IN-OUT racing line logic
    print(f"Applying OUT-IN-OUT racing line with strength={out_in_out_strength}, wall_margin={wall_margin}m")
    
    for i in range(len(points)):
        # Normal vector pointing left (toward outer wall for CCW track)
        norm = math.sqrt(d1x[i]**2 + d1y[i]**2)
        
        if norm > EPSILON:
            nx = -d1y[i] / norm
            ny = d1x[i] / norm
            
            # Calculate available space considering wall margin
            available_left = max(0.0, points[i]['d_left'] - wall_margin)
            available_right = max(0.0, points[i]['d_right'] - wall_margin)
            
            # Determine OUT-IN-OUT offset based on curvature
            kappa = smoothed_curvatures[i]
            
            # Base offset from lane_position (for straights)
            base_offset = 0.0
            if lane_position < 0.0:
                base_offset = -lane_position * available_left
            elif lane_position > 0.0:
                base_offset = -lane_position * available_right
            
            # OUT-IN-OUT adjustment based on corner phase
            out_in_out_offset = 0.0
            
            if abs(kappa) > CURVATURE_THRESHOLD_CORNER:
                # In a corner - apply OUT-IN-OUT logic
                # Positive kappa = left turn, negative kappa = right turn
                corner_direction = 1.0 if kappa > 0 else -1.0
                
                # For OUT-IN-OUT:
                # - OUTSIDE approach: opposite direction of corner
                # - INSIDE apex: same direction as corner (cut to apex)
                # - OUTSIDE exit: opposite direction of corner
                
                # Simplified: bias TOWARD inside (apex) in high curvature
                # This creates the classic racing line
                # Negative offset = move right (toward apex for left turn)
                # Positive offset = move left (toward apex for right turn)
                
                # Inside bias proportional to curvature magnitude
                curvature_factor = min(1.0, abs(kappa) / MAX_CURVATURE_NORMALIZATION)
                inside_bias = corner_direction * curvature_factor * out_in_out_strength
                
                # Apply bias to create IN at apex
                if inside_bias > 0:
                    out_in_out_offset = inside_bias * available_left
                else:
                    out_in_out_offset = inside_bias * available_right
            else:
                # Straight section - stay slightly outside for corner entry
                # Bias toward outside (negative for left wall, positive for right wall)
                out_in_out_offset = -base_offset * STRAIGHT_OUTSIDE_BIAS
            
            # Combine base offset and OUT-IN-OUT adjustment
            total_offset = base_offset + out_in_out_offset
            
            # Apply offset
            points[i]['x'] += total_offset * nx
            points[i]['y'] += total_offset * ny
    
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
        points[i]['psi'] = math.atan2(d1y[i], d1x[i] + EPSILON)
        
        # Compute curvature
        num = d1x[i] * d2y[i] - d1y[i] * d2x[i]
        den = (d1x[i]**2 + d1y[i]**2)**1.5
        points[i]['kappa'] = num / max(EPSILON, den)
        
        # Compute arc length
        if i > 0:
            dx = points[i]['x'] - points[i-1]['x']
            dy = points[i]['y'] - points[i-1]['y']
            s.append(s[-1] + math.hypot(dx, dy))
        
        points[i]['s'] = s[i] if i < len(s) else s[-1]
    
    # Set constant velocity (opponent speed)
    # This will be overridden by opponent_publisher's speed parameter
    for p in points:
        p['v'] = DEFAULT_OPPONENT_SPEED
    
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
    print(f"  Style: OUT-IN-OUT racing line (strength: {out_in_out_strength})")
    print(f"  Base lane position: {lane_position}")
    print(f"  Wall margin: {wall_margin}m")


def main():
    parser = argparse.ArgumentParser(
        description='Generate opponent raceline with OUT-IN-OUT racing style'
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
        help='Base position between walls for straights (-1.0=outer, 0.0=center, 1.0=inner). '
             'Default: -0.3 for stronger outside bias on straights (v6.0: increased from -0.2)'
    )
    parser.add_argument(
        '--wall-margin',
        type=float,
        default=0.25,
        help='Minimum distance from walls in meters. Default: 0.25 (v6.0: reduced from 0.3 for closer racing)'
    )
    parser.add_argument(
        '--ds',
        type=float,
        default=0.5,
        help='Sample spacing in meters. Default: 0.5'
    )
    parser.add_argument(
        '--out-in-out-strength',
        type=float,
        default=0.75,
        help='Strength of OUT-IN-OUT racing line (0.0-1.0). Default: 0.75 (v6.0: increased from 0.5 for more aggressive line)'
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
            ds=args.ds,
            out_in_out_strength=args.out_in_out_strength
        )
        return 0
    except Exception as e:
        print(f"Error generating opponent raceline: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
