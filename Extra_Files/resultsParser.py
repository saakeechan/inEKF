"""
InEKF Results Parser
====================
Reads and parses CSV results from InEKF estimator runs.

Usage:
    from resultsParser import ResultsData
    
    # Load data
    data = ResultsData("../results/2026-05-02_04-25-19.csv")
    
    # Access data by attribute (C++ style - recommended)
    time = data.time
    est_px = data.EST_Px
    est_contact_leg1 = data.EST_Contact_leg1
    gt_velocity_x = data.GT_Vx
    
    # Or access by bracket notation
    time = data['time']
    est_px = data['EST_Px']
    
    # Or use convenience methods for grouped data
    est_position = data.get_estimated_position()  # Returns (N, 3) array
    gt_velocity = data.get_ground_truth_velocity()  # Returns (N, 3) array
"""

import pandas as pd
import numpy as np
from pathlib import Path


class ResultsData:
    """Class to load and access InEKF estimator results from CSV files."""
    
    def __init__(self, csv_path):
        """
        Load CSV data from file.
        
        Parameters:
        -----------
        csv_path : str or Path
            Path to the CSV results file
        """
        self.csv_path = Path(csv_path)
        if not self.csv_path.exists():
            raise FileNotFoundError(f"CSV file not found: {csv_path}")
        
        # Load CSV data
        self.df = pd.read_csv(csv_path)
        self.num_timesteps = len(self.df)
        
        # Detect number of legs from column headers
        self.num_legs = self._detect_num_legs()
        
        # Check if ground truth data exists
        self.has_ground_truth = 'GT_Px' in self.df.columns
        
        print(f"Loaded {self.num_timesteps} timesteps from {self.csv_path.name}")
        print(f"Number of legs: {self.num_legs}")
        print(f"Ground truth data: {'Available' if self.has_ground_truth else 'Not available'}")
    
    def _detect_num_legs(self):
        """Detect number of legs from column headers."""
        leg_cols = [col for col in self.df.columns if 'leg' in col.lower()]
        if not leg_cols:
            return 2  # Default to 2 legs (bipedal)
        
        # Extract leg numbers
        leg_nums = []
        for col in leg_cols:
            if 'leg' in col.lower():
                # Extract number after 'leg'
                parts = col.split('leg')
                if len(parts) > 1:
                    num_str = ''.join(filter(str.isdigit, parts[1]))
                    if num_str:
                        leg_nums.append(int(num_str))
        
        return max(leg_nums) if leg_nums else 2
    
    def __getitem__(self, key):
        """Access column data by name (e.g., data['EST_Px'])."""
        if key not in self.df.columns:
            raise KeyError(f"Column '{key}' not found in CSV. Available columns: {list(self.df.columns)}")
        return self.df[key].values
    
    def __getattr__(self, name):
        """
        Access column data as attributes (e.g., data.EST_Px).
        This enables direct attribute access like C++ style: data.EST_Px, data.GT_Contact_leg1, etc.
        """
        # Avoid infinite recursion for internal attributes
        if name.startswith('_') or name in ['df', 'csv_path', 'num_timesteps', 'num_legs', 'has_ground_truth']:
            raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")
        
        # Check if this is a column name
        if hasattr(self, 'df') and name in self.df.columns:
            return self.df[name].values
        
        # If not found, raise AttributeError
        raise AttributeError(f"Column '{name}' not found in CSV. Available columns: {list(self.df.columns) if hasattr(self, 'df') else 'N/A'}")
    
    def get_columns(self):
        """Get list of all available column names."""
        return list(self.df.columns)
    
    # ==================== Time ====================
    def get_time(self):
        """Get time vector."""
        return self.df['time'].values
    
    # ==================== Estimated State ====================
    def get_estimated_position(self):
        """Get estimated position as (N, 3) array [x, y, z]."""
        return self.df[['EST_Px', 'EST_Py', 'EST_Pz']].values
    
    def get_estimated_quaternion(self):
        """Get estimated quaternion as (N, 4) array [w, x, y, z]."""
        return self.df[['EST_Qw', 'EST_Qx', 'EST_Qy', 'EST_Qz']].values
    
    def get_estimated_rpy(self):
        """Get estimated roll-pitch-yaw as (N, 3) array [roll, pitch, yaw]."""
        return self.df[['EST_roll', 'EST_pitch', 'EST_yaw']].values
    
    def get_estimated_velocity(self):
        """Get estimated velocity as (N, 3) array [vx, vy, vz]."""
        return self.df[['EST_Vx', 'EST_Vy', 'EST_Vz']].values
    
    def get_estimated_angular_velocity(self):
        """Get estimated angular velocity (omega) as (N, 3) array [wx, wy, wz]."""
        return self.df[['EST_Omega_x', 'EST_Omega_y', 'EST_Omega_z']].values
    
    def get_estimated_gyro_bias(self):
        """Get estimated gyroscope bias as (N, 3) array."""
        return self.df[['EST_Bg_x', 'EST_Bg_y', 'EST_Bg_z']].values
    
    def get_estimated_accel_bias(self):
        """Get estimated accelerometer bias as (N, 3) array."""
        return self.df[['EST_Ba_x', 'EST_Ba_y', 'EST_Ba_z']].values
    
    def get_estimated_contact(self, leg=None):
        """
        Get estimated contact states.
        
        Parameters:
        -----------
        leg : int or None
            If specified, return contact for specific leg (1-indexed).
            If None, return array of shape (N, num_legs).
        """
        if leg is not None:
            return self.df[f'EST_Contact_leg{leg}'].values
        else:
            cols = [f'EST_Contact_leg{i+1}' for i in range(self.num_legs)]
            return self.df[cols].values
    
    def get_estimated_dv(self, leg=None):
        """
        Get estimated foot velocities (dv).
        
        Parameters:
        -----------
        leg : int or None
            If specified, return dv for specific leg as (N, 3) array.
            If None, return array of shape (N, num_legs*3).
        """
        if leg is not None:
            cols = [f'EST_dv_leg{leg}_x', f'EST_dv_leg{leg}_y', f'EST_dv_leg{leg}_z']
            return self.df[cols].values
        else:
            cols = []
            for i in range(self.num_legs):
                cols.extend([f'EST_dv_leg{i+1}_x', f'EST_dv_leg{i+1}_y', f'EST_dv_leg{i+1}_z'])
            return self.df[cols].values
    
    def get_estimated_dv_norm(self, leg=None):
        """
        Get estimated foot velocity norms.
        
        Parameters:
        -----------
        leg : int or None
            If specified, return dv_norm for specific leg as (N,) array.
            If None, return array of shape (N, num_legs).
        """
        if leg is not None:
            return self.df[f'EST_dv_norm_leg{leg}'].values
        else:
            cols = [f'EST_dv_norm_leg{i+1}' for i in range(self.num_legs)]
            return self.df[cols].values
    
    # ==================== Ground Truth State ====================
    def get_ground_truth_position(self):
        """Get ground truth position as (N, 3) array [x, y, z]."""
        if not self.has_ground_truth:
            raise ValueError("Ground truth data not available in this CSV")
        return self.df[['GT_Px', 'GT_Py', 'GT_Pz']].values
    
    def get_ground_truth_quaternion(self):
        """Get ground truth quaternion as (N, 4) array [w, x, y, z]."""
        if not self.has_ground_truth:
            raise ValueError("Ground truth data not available in this CSV")
        return self.df[['GT_Qw', 'GT_Qx', 'GT_Qy', 'GT_Qz']].values
    
    def get_ground_truth_rpy(self):
        """Get ground truth roll-pitch-yaw as (N, 3) array [roll, pitch, yaw]."""
        if not self.has_ground_truth:
            raise ValueError("Ground truth data not available in this CSV")
        return self.df[['GT_roll', 'GT_pitch', 'GT_yaw']].values
    
    def get_ground_truth_velocity(self):
        """Get ground truth velocity as (N, 3) array [vx, vy, vz]."""
        if not self.has_ground_truth:
            raise ValueError("Ground truth data not available in this CSV")
        return self.df[['GT_Vx', 'GT_Vy', 'GT_Vz']].values
    
    def get_ground_truth_angular_velocity(self):
        """Get ground truth angular velocity (omega) as (N, 3) array [wx, wy, wz]."""
        if not self.has_ground_truth:
            raise ValueError("Ground truth data not available in this CSV")
        return self.df[['GT_Omega_x', 'GT_Omega_y', 'GT_Omega_z']].values
    
    def get_ground_truth_contact(self, leg=None):
        """
        Get ground truth contact states.
        
        Parameters:
        -----------
        leg : int or None
            If specified, return contact for specific leg (1-indexed).
            If None, return array of shape (N, num_legs).
        """
        if not self.has_ground_truth:
            raise ValueError("Ground truth data not available in this CSV")
        
        if leg is not None:
            return self.df[f'GT_Contact_leg{leg}'].values
        else:
            cols = [f'GT_Contact_leg{i+1}' for i in range(self.num_legs)]
            return self.df[cols].values
    
    def get_ground_truth_dv(self, leg=None):
        """
        Get ground truth foot velocities (dv).
        
        Parameters:
        -----------
        leg : int or None
            If specified, return dv for specific leg as (N, 3) array.
            If None, return array of shape (N, num_legs*3).
        """
        if not self.has_ground_truth:
            raise ValueError("Ground truth data not available in this CSV")
        
        if leg is not None:
            cols = [f'GT_dv_leg{leg}_x', f'GT_dv_leg{leg}_y', f'GT_dv_leg{leg}_z']
            return self.df[cols].values
        else:
            cols = []
            for i in range(self.num_legs):
                cols.extend([f'GT_dv_leg{i+1}_x', f'GT_dv_leg{i+1}_y', f'GT_dv_leg{i+1}_z'])
            return self.df[cols].values
    
    def get_ground_truth_dv_norm(self, leg=None):
        """
        Get ground truth foot velocity norms.
        
        Parameters:
        -----------
        leg : int or None
            If specified, return dv_norm for specific leg as (N,) array.
            If None, return array of shape (N, num_legs).
        """
        if not self.has_ground_truth:
            raise ValueError("Ground truth data not available in this CSV")
        
        if leg is not None:
            return self.df[f'GT_dv_norm_leg{leg}'].values
        else:
            cols = [f'GT_dv_norm_leg{i+1}' for i in range(self.num_legs)]
            return self.df[cols].values
    



# Example usage
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        # Default to most recent result
        results_dir = Path("../results")
        csv_files = sorted(results_dir.glob("*.csv"))
        if not csv_files:
            print("No CSV files found in ../results/")
            sys.exit(1)
        csv_path = csv_files[-1]
        print(f"Using most recent file: {csv_path}")
    
    # Load and display basic info
    data = ResultsData(csv_path)
    print(f"\nAvailable columns: {data.get_columns()}")
    
    # Show some statistics
    if data.has_ground_truth:
        print("\n--- Error Statistics ---")
        pos_error = data.compute_position_error()
        print(f"Position RMSE: {pos_error['rmse']:.6f} m")
        
        vel_error = data.compute_velocity_error()
        print(f"Velocity RMSE: {vel_error['rmse']:.6f} m/s")
        
        print("\n--- Contact Accuracy ---")
        contact_acc = data.compute_contact_accuracy()
        for leg in range(data.num_legs):
            leg_num = leg + 1
            acc = contact_acc[f'leg{leg_num}']['accuracy']
            print(f"Leg {leg_num}: {acc:.2f}%")
        print(f"Overall: {contact_acc['overall']['accuracy']:.2f}%")
