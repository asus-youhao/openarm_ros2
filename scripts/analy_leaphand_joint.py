#!/usr/bin/env python3
"""
Analyze LEAP Hand position data from CSV files
Automatically cleans corrupted data and plots pos_cmd, pos_error for motors 0-15
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
from pathlib import Path
from datetime import datetime

def to_float(x):
    """Safely convert to float, return NaN on error"""
    try:
        return float(x)
    except:
        return float('nan')

def load_and_clean_csv(csv_path, pos_jump_threshold=1.0, value_threshold=10.0):
    """
    Load CSV and clean corrupted data
    
    Args:
        csv_path: Path to CSV file
        pos_jump_threshold: Maximum allowed position jump between consecutive samples
        value_threshold: Maximum absolute value threshold for all variables
    
    Returns:
        Cleaned DataFrame
    """
    print(f"Loading CSV: {csv_path}")
    
    # Read CSV with manual parsing to handle corrupted lines
    valid_rows = []
    corrupted_count = 0
    
    # Define expected header
    expected_header = [
        'timestamp', 'motor_id', 'pos_cmd_urdf', 'pos_cmd_leap',
        'pos_state_urdf', 'pos_state_leap', 'pos_error_urdf', 'motor_name'
    ]
    expected_cols = len(expected_header)
    
    with open(csv_path, 'r') as f:
        # Read and fix header if necessary
        first_line = f.readline().strip()
        
        # Check if header might be split across multiple lines
        header_parts = [first_line]
        while len(','.join(header_parts).split(',')) < expected_cols:
            next_line = f.readline().strip()
            if not next_line or next_line[0].isdigit():  # Hit data line
                # Reset to use expected header
                f.seek(0)
                f.readline()  # Skip corrupted header
                header = expected_header
                print(f"  Using default header (original header was corrupted)")
                break
            header_parts.append(next_line)
        else:
            header = ','.join(header_parts).split(',')
            header = [h.strip() for h in header]
        
        print(f"  Expected columns: {expected_cols}")
        
        for line_num, line in enumerate(f, start=2):
            fields = line.strip().split(',')
            
            # Check if row has correct number of columns
            if len(fields) != expected_cols:
                corrupted_count += 1
                continue
            
            # Try to parse timestamp (must be valid number)
            try:
                timestamp = float(fields[0])
                # Timestamp should be a positive number
                if timestamp <= 0 or timestamp > 9999999999999:
                    corrupted_count += 1
                    continue
            except (ValueError, IndexError):
                corrupted_count += 1
                continue
            
            valid_rows.append(fields)
    
    print(f"  Removed {corrupted_count} corrupted/malformed rows")
    
    # Create DataFrame from valid rows
    df = pd.DataFrame(valid_rows, columns=expected_header)
    print(f"  Valid rows loaded: {len(df)}")
    
    # Convert columns to float (except motor_name)
    columns_to_convert = [
        'timestamp', 'motor_id', 'pos_cmd_urdf', 'pos_cmd_leap',
        'pos_state_urdf', 'pos_state_leap', 'pos_error_urdf'
    ]
    
    for col in columns_to_convert:
        if col in df.columns:
            df[col] = df[col].apply(to_float)
    
    # Remove rows with NaN in critical columns
    critical_cols = ['timestamp', 'motor_id', 'pos_cmd_urdf', 'pos_error_urdf']
    initial_len = len(df)
    df = df.dropna(subset=critical_cols)
    print(f"  Removed {initial_len - len(df)} rows with NaN in critical columns")
    
    # Validate motor_id (should be 0-15)
    initial_len = len(df)
    df = df[(df['motor_id'] >= 0) & (df['motor_id'] <= 15)]
    print(f"  Removed {initial_len - len(df)} rows with invalid motor_id")
    
    # Remove rows where values exceed threshold
    initial_len = len(df)
    for col in ['pos_cmd_urdf', 'pos_state_urdf', 'pos_error_urdf']:
        if col in df.columns:
            df = df[df[col].abs() < value_threshold]
    print(f"  Removed {initial_len - len(df)} rows with excessive values (threshold={value_threshold})")
    
    # Sort by motor_id and timestamp to ensure continuity
    df = df.sort_values(['motor_id', 'timestamp'])
    
    # Detect and remove timestamp discontinuities
    initial_len = len(df)
    df['timestamp_diff'] = df.groupby('motor_id')['timestamp'].diff()
    
    # Remove rows where timestamp goes backward or has huge gap (>10 seconds)
    max_time_gap = 10000  # milliseconds
    df = df[
        (df['timestamp_diff'].isna()) |  # Keep first row of each motor
        ((df['timestamp_diff'] > 0) & (df['timestamp_diff'] < max_time_gap))
    ]
    print(f"  Removed {initial_len - len(df)} rows with timestamp discontinuities")
    
    # Calculate position jump (displacement) for each motor
    df['pos_cmd_diff'] = df.groupby('motor_id')['pos_cmd_urdf'].diff().abs()
    df['pos_state_diff'] = df.groupby('motor_id')['pos_state_urdf'].diff().abs()
    
    # Remove rows with excessive position jumps
    initial_len = len(df)
    df = df[
        (df['pos_cmd_diff'].isna()) | 
        ((df['pos_cmd_diff'] < pos_jump_threshold) & (df['pos_state_diff'] < pos_jump_threshold))
    ]
    removed = initial_len - len(df)
    if removed > 0:
        print(f"  Removed {removed} rows with excessive position jumps (threshold={pos_jump_threshold} rad)")
    
    # Remove temporary columns
    df = df.drop(columns=['pos_cmd_diff', 'pos_state_diff', 'timestamp_diff'])
    
    print(f"  Final cleaned rows: {len(df)}")
    return df

def plot_leap_hand_analysis(df, output_path=None):
    """
    Plot pos_cmd and pos_error for motors 0-15
    
    Args:
        df: Cleaned DataFrame
        output_path: Optional path to save plot
    """
    motor_ids = range(16)
    
    # Create figure with 2 subplots
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle('LEAP Hand Control Analysis (Motors 0-15)', fontsize=16, fontweight='bold')
    
    # Plot pos_cmd_urdf
    for mid in motor_ids:
        mask = df['motor_id'] == mid
        data = df[mask]
        if len(data) > 0:
            # Convert timestamp to relative time (seconds from start)
            time_rel = (data['timestamp'] - data['timestamp'].min()) / 1000.0
            motor_name = data['motor_name'].iloc[0] if 'motor_name' in data.columns else f'Motor {mid}'
            axes[0].plot(time_rel, data['pos_cmd_urdf'], label=motor_name, alpha=0.7)
    
    axes[0].set_ylabel('Position Command (rad)', fontweight='bold')
    axes[0].legend(loc='upper right', ncol=4, fontsize=20)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Position Command (URDF)', fontsize=12)
    
    # Plot pos_error_urdf
    for mid in motor_ids:
        mask = df['motor_id'] == mid
        data = df[mask]
        if len(data) > 0:
            time_rel = (data['timestamp'] - data['timestamp'].min()) / 1000.0
            motor_name = data['motor_name'].iloc[0] if 'motor_name' in data.columns else f'Motor {mid}'
            axes[1].plot(time_rel, data['pos_error_urdf'], label=motor_name, alpha=0.7)
    
    axes[1].set_ylabel('Position Error (rad)', fontweight='bold')
    axes[1].set_xlabel('Time (seconds)', fontweight='bold')
    axes[1].legend(loc='upper right', ncol=4, fontsize=20)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('Position Error (URDF)', fontsize=12)
    
    plt.tight_layout()
    
    # Save and show
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"\nPlot saved to: {output_path}")
    
    # Always show plot for interactive zoom/pan
    print("Displaying interactive plot (you can zoom/pan)...")
    plt.show()

def print_statistics(df):
    """Print statistics for each motor"""
    print("\n" + "="*80)
    print("STATISTICS BY MOTOR")
    print("="*80)
    
    for mid in range(16):
        mask = df['motor_id'] == mid
        data = df[mask]
        
        if len(data) == 0:
            print(f"\nMotor {mid}: No data")
            continue
        
        motor_name = data['motor_name'].iloc[0] if 'motor_name' in data.columns else f'Motor {mid}'
        print(f"\nMotor {mid} ({motor_name}, {len(data)} samples):")
        print(f"  pos_cmd_urdf:   mean={data['pos_cmd_urdf'].mean():.4f}, std={data['pos_cmd_urdf'].std():.4f}, range=[{data['pos_cmd_urdf'].min():.4f}, {data['pos_cmd_urdf'].max():.4f}]")
        print(f"  pos_error_urdf: mean={data['pos_error_urdf'].mean():.4f}, std={data['pos_error_urdf'].std():.4f}, range=[{data['pos_error_urdf'].min():.4f}, {data['pos_error_urdf'].max():.4f}]")
        print(f"  avg abs error:  {data['pos_error_urdf'].abs().mean():.4f} rad")

def select_csv_file(csv_dir):
    """Let user select a CSV file from the csv directory"""
    csv_files = list(Path(csv_dir).glob('*.csv'))
    
    if not csv_files:
        print(f"Error: No CSV files found in {csv_dir}")
        sys.exit(1)
    
    print("\nAvailable CSV files:")
    print("=" * 80)
    for idx, csv_file in enumerate(csv_files, 1):
        file_size = csv_file.stat().st_size / 1024  # KB
        print(f"  [{idx}] {csv_file.name} ({file_size:.1f} KB)")
    
    while True:
        try:
            choice = input(f"\nSelect a file (1-{len(csv_files)}) or 'q' to quit: ").strip()
            if choice.lower() == 'q':
                print("Cancelled.")
                sys.exit(0)
            
            idx = int(choice) - 1
            if 0 <= idx < len(csv_files):
                return str(csv_files[idx])
            else:
                print(f"Please enter a number between 1 and {len(csv_files)}")
        except ValueError:
            print("Invalid input. Please enter a number.")

def main():
    # Setup paths
    script_dir = Path(__file__).parent
    csv_dir = script_dir / 'analye_joint' / 'csv'
    img_dir = script_dir / 'analye_joint' / 'img'
    
    # Create directories if they don't exist
    csv_dir.mkdir(parents=True, exist_ok=True)
    img_dir.mkdir(parents=True, exist_ok=True)
    
    # Select CSV file
    if len(sys.argv) >= 2:
        # Use provided CSV path
        csv_path = sys.argv[1]
        if not os.path.exists(csv_path):
            print(f"Error: File not found: {csv_path}")
            sys.exit(1)
    else:
        # Interactive selection
        csv_path = select_csv_file(csv_dir)
    
    # Generate output filename
    csv_filename = Path(csv_path).stem
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_path = img_dir / f"{csv_filename}_analysis_{timestamp}.png"
    
    print("\n" + "="*80)
    print("LEAP HAND POSITION DATA ANALYSIS")
    print("="*80)
    print(f"CSV file: {csv_path}")
    print(f"Output will be saved to: {output_path}")
    
    # Load and clean data
    df = load_and_clean_csv(
        csv_path,
        pos_jump_threshold=1.0,    # Adjust based on your data
        value_threshold=10.0        # Adjust based on your data (LEAP hand range is smaller)
    )
    
    if len(df) == 0:
        print("\nError: No valid data after cleaning!")
        sys.exit(1)
    
    # Print statistics
    print_statistics(df)
    
    # Plot
    print("\n" + "="*80)
    print("GENERATING PLOT")
    print("="*80)
    plot_leap_hand_analysis(df, output_path)
    
    print("\n" + "="*80)
    print("ANALYSIS COMPLETE")
    print("="*80)

if __name__ == "__main__":
    main()