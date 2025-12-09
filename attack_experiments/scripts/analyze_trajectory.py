#!/usr/bin/env python3
# Analyzes the trajectory CSV and figures out if the attack worked
# Looks at the data and generates a report with some basic stats

import csv
import argparse
import sys
from datetime import datetime


def load_trajectory(csv_file):
    """Read the CSV file and parse all the data points"""
    trajectory = []
    try:
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                trajectory.append({
                    'time': float(row['relative_time']),
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'yaw': float(row['yaw']),
                    'linear_x': float(row['linear_x']),
                    'angular_z': float(row['angular_z'])
                })
    except FileNotFoundError:
        print(f"Error: Can't find {csv_file}")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading file: {e}")
        sys.exit(1)
    
    return trajectory


def analyze_period(trajectory, start_time, end_time, period_name):
    """Analyze a specific time period"""
    period_data = [p for p in trajectory if start_time <= p['time'] <= end_time]
    
    if not period_data:
        return None
    
    x_values = [p['x'] for p in period_data]
    y_values = [p['y'] for p in period_data]
    linear_x_values = [p['linear_x'] for p in period_data]
    angular_z_values = [p['angular_z'] for p in period_data]
    
    # Calculate distance traveled
    total_distance = 0
    for i in range(1, len(period_data)):
        dx = period_data[i]['x'] - period_data[i-1]['x']
        dy = period_data[i]['y'] - period_data[i-1]['y']
        total_distance += (dx**2 + dy**2)**0.5
    
    # Calculate displacement
    start_pos = period_data[0]
    end_pos = period_data[-1]
    displacement = ((end_pos['x'] - start_pos['x'])**2 + 
                    (end_pos['y'] - start_pos['y'])**2)**0.5
    
    # Calculate average velocities
    avg_linear_x = sum(linear_x_values) / len(linear_x_values) if linear_x_values else 0
    avg_angular_z = sum(angular_z_values) / len(angular_z_values) if angular_z_values else 0
    
    # Calculate direction change (for turn detection)
    start_yaw = period_data[0]['yaw']
    end_yaw = period_data[-1]['yaw']
    yaw_change = end_yaw - start_yaw
    
    return {
        'name': period_name,
        'start_time': start_time,
        'end_time': end_time,
        'duration': end_time - start_time,
        'data_points': len(period_data),
        'start_position': (start_pos['x'], start_pos['y']),
        'end_position': (end_pos['x'], end_pos['y']),
        'total_distance': total_distance,
        'displacement': displacement,
        'avg_linear_x': avg_linear_x,
        'avg_angular_z': avg_angular_z,
        'yaw_change': yaw_change,
        'x_range': (min(x_values), max(x_values)),
        'y_range': (min(y_values), max(y_values))
    }


def generate_report(trajectory, attack_start=5.0, attack_duration=15.0):
    """Generate experiment analysis report"""
    if not trajectory:
        print("Error: No trajectory data")
        return
    
    total_duration = trajectory[-1]['time']
    attack_end = attack_start + attack_duration
    
    # Analyze different periods
    before_attack = analyze_period(trajectory, 0, attack_start, "Before Attack")
    during_attack = analyze_period(trajectory, attack_start, attack_end, "During Attack")
    after_attack = analyze_period(trajectory, attack_end, total_duration, "After Attack")
    
    # Generate report
    print("=" * 70)
    print("INJECTION ATTACK EXPERIMENT - TRAJECTORY ANALYSIS REPORT")
    print("=" * 70)
    print(f"Analysis Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Total Experiment Duration: {total_duration:.2f} seconds")
    print(f"Attack Period: {attack_start:.1f}s - {attack_end:.1f}s ({attack_duration:.1f}s)")
    print()
    
    # Before Attack Analysis
    if before_attack:
        print("-" * 70)
        print("BEFORE ATTACK (Normal Operation)")
        print("-" * 70)
        print(f"  Duration: {before_attack['duration']:.2f} seconds")
        print(f"  Start Position: ({before_attack['start_position'][0]:.3f}, {before_attack['start_position'][1]:.3f})")
        print(f"  End Position: ({before_attack['end_position'][0]:.3f}, {before_attack['end_position'][1]:.3f})")
        print(f"  Total Distance Traveled: {before_attack['total_distance']:.3f} m")
        print(f"  Displacement: {before_attack['displacement']:.3f} m")
        print(f"  Average Linear Velocity: {before_attack['avg_linear_x']:.3f} m/s")
        print(f"  Average Angular Velocity: {before_attack['avg_angular_z']:.3f} rad/s")
        print(f"  Yaw Change: {before_attack['yaw_change']:.3f} rad ({before_attack['yaw_change']*180/3.14159:.1f}°)")
        print(f"  X Range: [{before_attack['x_range'][0]:.3f}, {before_attack['x_range'][1]:.3f}]")
        print(f"  Y Range: [{before_attack['y_range'][0]:.3f}, {before_attack['y_range'][1]:.3f}]")
        print()
    
    # During Attack Analysis
    if during_attack:
        print("-" * 70)
        print("DURING ATTACK")
        print("-" * 70)
        print(f"  Duration: {during_attack['duration']:.2f} seconds")
        print(f"  Start Position: ({during_attack['start_position'][0]:.3f}, {during_attack['start_position'][1]:.3f})")
        print(f"  End Position: ({during_attack['end_position'][0]:.3f}, {during_attack['end_position'][1]:.3f})")
        print(f"  Total Distance Traveled: {during_attack['total_distance']:.3f} m")
        print(f"  Displacement: {during_attack['displacement']:.3f} m")
        print(f"  Average Linear Velocity: {during_attack['avg_linear_x']:.3f} m/s")
        print(f"  Average Angular Velocity: {during_attack['avg_angular_z']:.3f} rad/s")
        print(f"  Yaw Change: {during_attack['yaw_change']:.3f} rad ({during_attack['yaw_change']*180/3.14159:.1f}°)")
        print(f"  X Range: [{during_attack['x_range'][0]:.3f}, {during_attack['x_range'][1]:.3f}]")
        print(f"  Y Range: [{during_attack['y_range'][0]:.3f}, {during_attack['y_range'][1]:.3f}]")
        print()
    
    # After Attack Analysis
    if after_attack:
        print("-" * 70)
        print("AFTER ATTACK")
        print("-" * 70)
        print(f"  Duration: {after_attack['duration']:.2f} seconds")
        print(f"  Start Position: ({after_attack['start_position'][0]:.3f}, {after_attack['start_position'][1]:.3f})")
        print(f"  End Position: ({after_attack['end_position'][0]:.3f}, {after_attack['end_position'][1]:.3f})")
        print(f"  Total Distance Traveled: {after_attack['total_distance']:.3f} m")
        print(f"  Displacement: {after_attack['displacement']:.3f} m")
        print(f"  Average Linear Velocity: {after_attack['avg_linear_x']:.3f} m/s")
        print(f"  Average Angular Velocity: {after_attack['avg_angular_z']:.3f} rad/s")
        print(f"  Yaw Change: {after_attack['yaw_change']:.3f} rad ({after_attack['yaw_change']*180/3.14159:.1f}°)")
        print()
    
    # Attack Success Analysis
    print("=" * 70)
    print("ATTACK SUCCESS ANALYSIS")
    print("=" * 70)
    
    if before_attack and during_attack:
        # Compare angular velocity (should increase during attack)
        angular_change = during_attack['avg_angular_z'] - before_attack['avg_angular_z']
        yaw_change_during = abs(during_attack['yaw_change'])
        yaw_change_before = abs(before_attack['yaw_change'])
        
        print(f"Angular Velocity Change: {angular_change:.3f} rad/s")
        print(f"  Before: {before_attack['avg_angular_z']:.3f} rad/s")
        print(f"  During: {during_attack['avg_angular_z']:.3f} rad/s")
        print()
        
        print(f"Yaw Change Comparison:")
        print(f"  Before Attack: {yaw_change_before:.3f} rad ({yaw_change_before*180/3.14159:.1f}°)")
        print(f"  During Attack: {yaw_change_during:.3f} rad ({yaw_change_during*180/3.14159:.1f}°)")
        print()
        
        # Y position change (should increase if turning left)
        y_change_before = abs(before_attack['end_position'][1] - before_attack['start_position'][1])
        y_change_during = abs(during_attack['end_position'][1] - during_attack['start_position'][1])
        
        print(f"Y Position Change (Lateral Movement):")
        print(f"  Before Attack: {y_change_before:.3f} m")
        print(f"  During Attack: {y_change_during:.3f} m")
        print()
        
        # Success criteria
        print("SUCCESS CRITERIA:")
        success_indicators = []
        
        if angular_change > 0.1:  # Significant increase in angular velocity
            success_indicators.append("✅ Angular velocity increased significantly")
        else:
            success_indicators.append("❌ Angular velocity did not increase")
        
        if yaw_change_during > yaw_change_before * 2:  # More turning during attack
            success_indicators.append("✅ Yaw change increased (robot turned more)")
        else:
            success_indicators.append("❌ Yaw change did not increase significantly")
        
        if y_change_during > y_change_before * 2:  # More lateral movement
            success_indicators.append("✅ Lateral movement increased (robot turned left)")
        else:
            success_indicators.append("❌ Lateral movement did not increase")
        
        for indicator in success_indicators:
            print(f"  {indicator}")
        
        print()
        
        # Overall assessment
        success_count = sum(1 for s in success_indicators if s.startswith("✅"))
        if success_count >= 2:
            print("🎯 ATTACK SUCCESSFUL: Robot behavior changed significantly during attack")
        elif success_count == 1:
            print("⚠️  PARTIAL SUCCESS: Some attack effects detected")
        else:
            print("❌ ATTACK FAILED: No significant behavior change detected")
    
    print()
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(
        description='Analyze robot trajectory from CSV file',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Analyze with default attack timing (5s start, 15s duration)
  python3 analyze_trajectory.py trajectory_20231208_120000.csv
  
  # Analyze with custom attack timing
  python3 analyze_trajectory.py trajectory.csv --attack-start 10 --attack-duration 20
        """
    )
    
    parser.add_argument('csv_file', type=str,
                       help='Trajectory CSV file to analyze')
    parser.add_argument('--attack-start', type=float, default=5.0,
                       help='Attack start time in seconds (default: 5.0)')
    parser.add_argument('--attack-duration', type=float, default=15.0,
                       help='Attack duration in seconds (default: 15.0)')
    
    args = parser.parse_args()
    
    # Load trajectory
    print(f"Loading trajectory from: {args.csv_file}")
    trajectory = load_trajectory(args.csv_file)
    print(f"Loaded {len(trajectory)} data points")
    print()
    
    # Generate report
    generate_report(trajectory, args.attack_start, args.attack_duration)


if __name__ == '__main__':
    main()

