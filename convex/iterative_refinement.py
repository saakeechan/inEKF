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

# Configuration
MAX_ITERATIONS         = 5
CONVERGENCE_THRESHOLD  = 0.02   # Stop if <2% contacts change
MIN_CHANGE             = 0.10   # Minimum 10% change to consider significant
VARIANT                = "fixed_lag_single_bias"

DATA_DIR      = Path("Data")
RESULTS_DIR   = Path("results")
BUILD_DIR     = Path("../build")
CONTACTS_PATH = DATA_DIR / "contacts.csv"
FK_PATH = DATA_DIR / "robotstate_0_env2.csv"


def run_cpp_estimator(iteration, variant="fixed_lag_single_bias"):
    """
    Run the C++ mainSmoother.
    
    Args:
        iteration: Current iteration number
        variant: Estimator variant to run
    
    Returns:
        Path to the output foot velocities CSV
    """
    print(f"\n{'='*60}")
    print(f"[Iteration {iteration}] Running C++ Estimator")
    print(f"{'='*60}")
    print(f"  Variant:  {variant}")
    
    cmd = [
        str(BUILD_DIR / "mainSmoother"),
        "--dataset", str(DATA_DIR),
        "--output", str(RESULTS_DIR),
        "--variants", variant
    ]
    
    print(f"  Command: {' '.join(cmd)}")
    print()
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"\n❌ C++ estimator failed!")
        print(f"STDERR:\n{result.stderr}")
        print(f"STDOUT:\n{result.stdout}")
        raise RuntimeError(f"C++ estimator failed with code {result.returncode}")
    
    # Print last few lines of output for status
    output_lines = result.stdout.strip().split('\n')
    print("  C++ Output (last 5 lines):")
    for line in output_lines[-5:]:
        print(f"    {line}")
    
    velocities_csv = RESULTS_DIR / f"{variant}_foot_velocities.csv"
    
    if not velocities_csv.exists():
        raise RuntimeError(f"Expected output not found: {velocities_csv}")
    
    print(f"\n✓ Estimator completed successfully")
    print(f"  Output: {velocities_csv}")
    
    return velocities_csv


def run_contact_smoother(iteration, velocities_csv):
    """
    Run Python contact smoother.

    Args:
        velocities_csv: Path to foot velocities from estimator (also has contact columns)
        iteration:      Current iteration number

    Returns:
        change_percentage (float): Percentage of contact states that changed
    """
    print(f"\n{'='*60}")
    print(f"[Iteration {iteration}] Running Contact Smoother")
    print(f"{'='*60}")
    print(f"  Velocities: {velocities_csv}")
    print(f"  Contacts:   {CONTACTS_PATH} (updated in-place)")
    print()

    # Import the smoother module
    try:
        from contactSmoother import smooth_contacts
    except ImportError as e:
        print(f"❌ Failed to import contactSmoother: {e}")
        print("   Make sure contactSmoother.py is in the same directory")
        raise

    # Run the smoother (updates contacts.csv in-place)
    change_pct = smooth_contacts(
        velocities_path=velocities_csv,
        contacts_path=CONTACTS_PATH,
        fk_path=FK_PATH,
    )

    # Read updated contacts to count left/right foot contacts
    contacts_df = pd.read_csv(CONTACTS_PATH)
    left_contacts = len(contacts_df[contacts_df['foot_name'] == 'left_foot'])
    right_contacts = len(contacts_df[contacts_df['foot_name'] == 'right_foot'])
    total_contacts = len(contacts_df)

    print(f"\n✓ Smoother completed")
    print(f"  Changed: {change_pct:.2f}% of contact states")
    print(f"  Updated: {CONTACTS_PATH}")
    print(f"  Left foot contacts:  {left_contacts}")
    print(f"  Right foot contacts: {right_contacts}")
    print(f"  Total contacts:      {total_contacts}")

    return change_pct, left_contacts, right_contacts


def check_convergence(change_pct, threshold_pct):
    """Check if iteration has converged."""
    return change_pct < threshold_pct


def main():
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
    print(f"Convergence threshold:  {CONVERGENCE_THRESHOLD*100:.1f}%")
    print(f"Minimum change:         {MIN_CHANGE*100:.1f}%")
    print(f"Estimator variant:      {VARIANT}")
    print("="*60)

    history = []

    for iteration in range(1, MAX_ITERATIONS + 1):
        try:
            # Step 1: Run C++ estimator
            velocities_csv = run_cpp_estimator(
                iteration,
                variant=VARIANT
            )

            # Step 2: Run Python smoother to refine contacts in-place
            change_pct, left_contacts, right_contacts = run_contact_smoother(
                iteration,
                velocities_csv,
            )

            # Record history
            history.append({
                'iteration': iteration,
                'change_pct': change_pct,
                'left_contacts': left_contacts,
                'right_contacts': right_contacts,
                'total_contacts': left_contacts + right_contacts,
            })

            # Step 3: Check if changes are significant
            if change_pct < MIN_CHANGE * 100:
                print(f"\n⚠ Smoother made insignificant changes ({change_pct:.2f}% < {MIN_CHANGE*100:.1f}%)")
                print(f"  Stopping iteration - keeping contacts from previous step")
                break

            # Step 4: Check convergence
            convergence_threshold_pct = CONVERGENCE_THRESHOLD * 100
            if check_convergence(change_pct, convergence_threshold_pct):
                print(f"\n✓ Converged at iteration {iteration}")
                print(f"  Change: {change_pct:.2f}% < {convergence_threshold_pct:.2f}% threshold")
                print(f"  Final contacts: {CONTACTS_PATH}")
                break

            print(f"\n→ Proceeding to iteration {iteration + 1}")

        except Exception as e:
            print(f"\n❌ Error in iteration {iteration}: {e}")
            import traceback
            traceback.print_exc()
            raise SystemExit(1)

    else:
        # Reached max iterations without convergence
        print(f"\n⚠ Max iterations ({MAX_ITERATIONS}) reached without convergence")
        print(f"  Final contacts: {CONTACTS_PATH}")

    # Print summary
    print("\n" + "="*60)
    print("Refinement Summary")
    print("="*60)
    print(f"{'Iteration':<12} {'Change %':<12} {'Left':<8} {'Right':<8} {'Total':<8}")
    print("-" * 60)
    for record in history:
        print(f"{record['iteration']:<12} {record['change_pct']:<12.2f} "
              f"{record['left_contacts']:<8} {record['right_contacts']:<8} "
              f"{record['total_contacts']:<8}")
    print("-" * 60)
    print(f"Final contacts: {CONTACTS_PATH}")
    print("="*60)
    print("\n✓ Refinement Complete\n")


if __name__ == "__main__":
    main()
