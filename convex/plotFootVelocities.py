#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
from pathlib import Path

def plot_leg_data(data, leg, output_dir):
    """
    Plot velocity norm for a specific leg with contact regions highlighted
    Shows GT, Estimator, and FK velocities for comparison
    
    Args:
        data: DataFrame with CSV data
        leg: 'left' or 'right'
        output_dir: Directory to save plots
    """
    # Select columns based on leg
    contact_col = f'{leg}_foot_contact'
    
    # Calculate velocity norms for GT, EST, and FK
    gt_vx_col = f'{leg}_foot_gt_vx'
    gt_vy_col = f'{leg}_foot_gt_vy'
    gt_vz_col = f'{leg}_foot_gt_vz'
    
    est_vx_col = f'{leg}_foot_est_vx'
    est_vy_col = f'{leg}_foot_est_vy'
    est_vz_col = f'{leg}_foot_est_vz'
    
    fk_vx_col = f'{leg}_foot_fk_vx'
    fk_vy_col = f'{leg}_foot_fk_vy'
    fk_vz_col = f'{leg}_foot_fk_vz'
    
    # Calculate norms locally (don't modify input dataframe)
    gt_v_norm = np.sqrt(data[gt_vx_col]**2 + data[gt_vy_col]**2 + data[gt_vz_col]**2)
    est_v_norm = np.sqrt(data[est_vx_col]**2 + data[est_vy_col]**2 + data[est_vz_col]**2)
    fk_v_norm = np.sqrt(data[fk_vx_col]**2 + data[fk_vy_col]**2 + data[fk_vz_col]**2)
    
    # Filter data for contact regions
    contact_mask_bool = data[contact_col] == 1
    contact_data = data[contact_mask_bool].copy()
    contact_gt_norm = gt_v_norm[contact_mask_bool]
    contact_est_norm = est_v_norm[contact_mask_bool]
    contact_fk_norm = fk_v_norm[contact_mask_bool]
    
    leg_name = leg.capitalize()
    
    # Create single plot with all three velocity sources
    fig, ax = plt.subplots(figsize=(12, 4))
    
    # Highlight contact regions in red
    contact_mask = data[contact_col] == 1
    contact_changes = contact_mask.astype(int).diff().fillna(0)
    
    # Start of contact regions
    contact_starts = data.index[contact_changes == 1].tolist()
    # End of contact regions
    contact_ends = data.index[contact_changes == -1].tolist()
    
    # Handle edge cases
    if contact_mask.iloc[0]:
        contact_starts = [0] + contact_starts
    if contact_mask.iloc[-1]:
        contact_ends = contact_ends + [len(data) - 1]
    
    # Plot red regions for contacts
    for start_idx, end_idx in zip(contact_starts, contact_ends):
        ax.axvspan(data['timestamp_s'].iloc[start_idx], 
                  data['timestamp_s'].iloc[end_idx], 
                  alpha=0.2, color='red', label='Contact' if start_idx == contact_starts[0] else '')
    
    # Plot velocity norms for contact regions only
    if not contact_data.empty:
        ax.scatter(contact_data['timestamp_s'], contact_gt_norm, 
                  s=20, color='green', marker='o', label='Ground Truth', zorder=5, alpha=0.7)
        ax.scatter(contact_data['timestamp_s'], contact_est_norm, 
                  s=20, color='blue', marker='s', label='Estimator', zorder=5, alpha=0.7)
        ax.scatter(contact_data['timestamp_s'], contact_fk_norm, 
                  s=20, color='orange', marker='^', label='FK', zorder=5, alpha=0.7)
    
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Velocity Norm (m/s)', fontsize=12)
    ax.set_title(f'{leg_name} Leg - Velocity Norm (Contact Regions)', fontsize=14)
    ax.set_xlim(data['timestamp_s'].min(), data['timestamp_s'].max())
    ax.grid(True, alpha=0.3)
    
    # Add legend (avoid duplicates)
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    if by_label:
        ax.legend(by_label.values(), by_label.keys(), loc='upper right')
    
    # Save plot
    filename = os.path.join(output_dir, f"{leg_name.lower()}_leg_dv.png")
    plt.tight_layout()
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {filename}")

def main():
    # Define paths
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        csv_file = "results/fixed_lag_single_bias_foot_velocities.csv"
    
    output_dir = "results/plots"
    
    # Create output directory if it doesn't exist
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    print(f"Output directory: {output_dir}")
    
    # Read CSV data
    print(f"Reading CSV file: {csv_file}")
    try:
        data = pd.read_csv(csv_file)
        print(f"Loaded {len(data)} data points.")
    except FileNotFoundError:
        print(f"Error: File not found: {csv_file}")
        print(f"\nUsage: python3 {sys.argv[0]} [csv_file]")
        print(f"Example: python3 {sys.argv[0]} results/2026-05-04_19-51-14.csv")
        return
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return
    
    # Verify required columns exist
    required_cols = ['timestamp_s', 'left_foot_contact', 'right_foot_contact',
                     'left_foot_gt_vx', 'left_foot_gt_vy', 'left_foot_gt_vz',
                     'right_foot_gt_vx', 'right_foot_gt_vy', 'right_foot_gt_vz',
                     'left_foot_est_vx', 'left_foot_est_vy', 'left_foot_est_vz',
                     'right_foot_est_vx', 'right_foot_est_vy', 'right_foot_est_vz',
                     'left_foot_fk_vx', 'left_foot_fk_vy', 'left_foot_fk_vz',
                     'right_foot_fk_vx', 'right_foot_fk_vy', 'right_foot_fk_vz']
    
    missing_cols = [col for col in required_cols if col not in data.columns]
    if missing_cols:
        print(f"Error: Missing required columns: {missing_cols}")
        return
    
    # Generate plots for left leg
    print("\nGenerating plots for left leg...")
    plot_leg_data(data, 'left', output_dir)
    
    # Generate plots for right leg
    print("\nGenerating plots for right leg...")
    plot_leg_data(data, 'right', output_dir)
    
    print("\nAll plots saved successfully!")

if __name__ == "__main__":
    main()
