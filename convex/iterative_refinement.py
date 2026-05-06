#!/usr/bin/env python3
"""
Iterative Contact Refinement Controller
========================================
Orchestrates an iterative loop between:
  1. C++ mainSmoother (estimates foot velocities from contacts)
  2. Python contactSmoother (refines contacts based on velocities)

Continues until convergence or max iterations reached.
"""

import subprocess
import pandas as pd
import numpy as np
from pathlib import Path
import time

# Configuration
MAX_ITERATIONS         = 8
CONVERGENCE_THRESHOLD  = 1.0    # Stop if <1% change from previous iteration (for convex method)
VARIANT                = "fixed_lag_single_bias"

# Contact smoother method selection
METHOD = "convex"  # Options: "convex" or "threshold"

# Convex smoother weights (used when METHOD="convex")
EPS_XY     = 3.0    # XY velocity threshold (m/s)
EPS_Z      = 0.3    # Z  velocity threshold (m/s)
BETA       = 3.0    # Weight for prior (deviation from original contacts)
RHO_XY     = 1.0    # Weight for XY velocity slack
RHO_Z      = 1.0    # Weight for Z  velocity slack
LAMBDA_TV  = 3.0    # Weight for TV epigraph (smoothness)

# Threshold smoother parameters (used when METHOD="threshold")
VEL_THRESHOLD = 0.3  # Velocity threshold (m/s) - remove contacts above this

DATA_DIR      = Path("Data")
RESULTS_DIR   = Path("results")
BUILD_DIR     = Path("../build")
CONTACTS_PATH = DATA_DIR / "contacts.csv"
FK_PATH = DATA_DIR / "robotstate_0_800.csv"


def run_cpp_estimator(iteration, variant="fixed_lag_single_bias", skip_plots=False):
    """
    Run the C++ mainSmoother.
    
    Args:
        iteration: Current iteration number
        variant: Estimator variant to run
        skip_plots: If True, don't generate plots (saves time during iterations)
    
    Returns:
        Path to the output foot velocities CSV
    """
    
    cmd = [
        str(BUILD_DIR / "mainSmoother"),
        "--dataset", str(DATA_DIR),
        "--output", str(RESULTS_DIR),
        "--variants", variant
    ]
    
    if skip_plots:
        cmd.append("--skip-plots")
    
    print()
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"\n❌ C++ estimator failed!")
        print(f"STDERR:\n{result.stderr}")
        print(f"STDOUT:\n{result.stdout}")
        raise RuntimeError(f"C++ estimator failed with code {result.returncode}")
    
    velocities_csv = RESULTS_DIR / f"{variant}_foot_velocities.csv"
    
    if not velocities_csv.exists():
        raise RuntimeError(f"Expected output not found: {velocities_csv}")
    
    return velocities_csv


def run_contact_smoother(iteration, velocities_csv, history_lfoot=None, history_rfoot=None):
    """
    Run Python contact smoother.

    Args:
        velocities_csv: Path to foot velocities from estimator (also has contact columns)
        iteration:      Current iteration number
        history_lfoot:  History of left foot contact sequences (for convergence tracking)
        history_rfoot:  History of right foot contact sequences (for convergence tracking)

    Returns:
        (change_pct, converged, history_lfoot, history_rfoot, left_contacts, right_contacts)
    """
    print(f"\n{'='*60}")
    print(f"[Iteration {iteration}] Running Contact Smoother")
    print(f"{'='*60}")

    # Import the smoother module
    try:
        from contactSmoother import smooth_contacts
    except ImportError as e:
        print(f"❌ Failed to import contactSmoother: {e}")
        print("   Make sure contactSmoother.py is in the same directory")
        raise

    # Run the smoother (updates contacts.csv in-place)
    change_pct, converged, history_lfoot, history_rfoot = smooth_contacts(
        velocities_path=velocities_csv,
        contacts_path=CONTACTS_PATH,
        fk_path=FK_PATH,
        method=METHOD,
        history_lfoot=history_lfoot,
        history_rfoot=history_rfoot,
        convergence_threshold=CONVERGENCE_THRESHOLD,
        eps_xy=EPS_XY,
        eps_z=EPS_Z,
        vel_threshold=VEL_THRESHOLD,
        beta=BETA,
        rho_xy=RHO_XY,
        rho_z=RHO_Z,
        lambda_tv=LAMBDA_TV,
    )

    # Read updated contacts to count left/right foot contacts
    contacts_df = pd.read_csv(CONTACTS_PATH)
    left_contacts = len(contacts_df[contacts_df['foot_name'] == 'left_foot'])
    right_contacts = len(contacts_df[contacts_df['foot_name'] == 'right_foot'])
    total_contacts = len(contacts_df)

    return change_pct, converged, history_lfoot, history_rfoot, left_contacts, right_contacts


def check_convergence(converged_flag, change_pct, threshold_pct, method):
    """
    Check if iteration has converged.
    
    For convex method: uses internal convergence flag from contactSmoother.
    For threshold method: uses simple percentage threshold.
    """
    if method == "convex":
        return converged_flag
    else:
        return change_pct < threshold_pct


def main():
    # Start wall clock timer for entire process
    total_start_time = time.time()
    
    if not BUILD_DIR.exists():
        print(f"❌ Build directory not found: {BUILD_DIR}")
        print("   Run ./docker.sh build first")
        raise SystemExit(1)

    if not RESULTS_DIR.exists():
        print(f"❌ Results directory not found: {RESULTS_DIR}")
        print("   Create the directory first")
        raise SystemExit(1)

    # Print configuration
    print("\n" + "="*60)
    print("Iterative Contact Refinement")
    print("="*60)
    print(f"Max iterations:         {MAX_ITERATIONS}")
    print(f"Convergence threshold:  {CONVERGENCE_THRESHOLD:.1f}%")
    print(f"Smoother method:        {METHOD}")
    if METHOD == "convex":
        print(f"  eps_xy={EPS_XY}, eps_z={EPS_Z}, beta={BETA}, rho_xy={RHO_XY}, rho_z={RHO_Z}, lambda_tv={LAMBDA_TV}")
    elif METHOD == "threshold":
        print(f"  vel_threshold={VEL_THRESHOLD} m/s")
    print("="*60)

    history = []
    history_lfoot = None  # Contact sequence history for convergence tracking
    history_rfoot = None

    for iteration in range(1, MAX_ITERATIONS + 1):
        try:
            # Step 1: Run C++ estimator (skip plots during iterations)
            velocities_csv = run_cpp_estimator(
                iteration,
                variant=VARIANT,
                skip_plots=True
            )

            # Step 2: Run Python smoother to refine contacts in-place
            change_pct, converged, history_lfoot, history_rfoot, left_contacts, right_contacts = run_contact_smoother(
                iteration,
                velocities_csv,
                history_lfoot=history_lfoot,
                history_rfoot=history_rfoot,
            )

            # Record history
            history.append({
                'iteration': iteration,
                'change_pct': change_pct,
                'converged': converged,
                'left_contacts': left_contacts,
                'right_contacts': right_contacts,
                'total_contacts': left_contacts + right_contacts,
            })

            # Check convergence
            if check_convergence(converged, change_pct, CONVERGENCE_THRESHOLD, METHOD):
                if METHOD == "convex":
                    # print(f"  Total iterations in history: {len(history_lfoot)}")
                    break

        except Exception as e:
            print(f"\n❌ Error in iteration {iteration}: {e}")
            import traceback
            traceback.print_exc()
            raise SystemExit(1)

    else:
        # Reached max iterations without convergence
        print(f"\n⚠ Max iterations ({MAX_ITERATIONS}) reached without convergence")
        print(f"  Final contacts: {CONTACTS_PATH}")

    # Final pass: Run estimator one more time with refined contacts (with plots)
    print("\n" + "="*60)
    print("Final Pass: Running Estimator with Refined Contacts")
    print("="*60)
    try:
        final_velocities_csv = run_cpp_estimator(
            iteration="FINAL",
            variant=VARIANT,
            skip_plots=False  # Generate plots on final pass
        )
    except Exception as e:
        print(f"\n❌ Error in final estimator pass: {e}")
        import traceback
        traceback.print_exc()
        raise SystemExit(1)

    # Print summary
    print("\n" + "="*60)
    print("Refinement Summary")
    print("="*60)
    if METHOD == "convex":
        print(f"{'Iteration':<12} {'Change %':<12} {'Converged':<12} {'Left':<8} {'Right':<8} {'Total':<8}")
        print("-" * 72)
        for record in history:
            print(f"{record['iteration']:<12} {record['change_pct']:<12.2f} "
                  f"{'Yes' if record['converged'] else 'No':<12} "
                  f"{record['left_contacts']:<8} {record['right_contacts']:<8} "
                  f"{record['total_contacts']:<8}")
        print("-" * 72)
    else:
        print(f"{'Iteration':<12} {'Change %':<12} {'Left':<8} {'Right':<8} {'Total':<8}")
        print("-" * 60)
        for record in history:
            print(f"{record['iteration']:<12} {record['change_pct']:<12.2f} "
                  f"{record['left_contacts']:<8} {record['right_contacts']:<8} "
                  f"{record['total_contacts']:<8}")
        print("-" * 60)
    print("="*60)
    
    # Print total wall clock time
    total_elapsed_time = time.time() - total_start_time
    print(f"\n⏱️  Total Wall Clock Time: {total_elapsed_time:.2f} seconds ({total_elapsed_time/60:.2f} minutes)")


if __name__ == "__main__":
    main()
