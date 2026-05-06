#!/usr/bin/env python3
"""
Contact Smoother Module
=======================
Smooths contact sequences based on estimated foot velocities from the estimator.

High foot velocity → likely not in contact → adjust contact state
Low foot velocity → likely in contact → adjust contact state
"""

import pandas as pd
import numpy as np
from pathlib import Path
from typing import Tuple
import cvxpy as cp
import time


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------

def load_fk(fk_path: Path) -> pd.DataFrame:
    """
    Load FK CSV and extract timestamp + FK foot positions.
    
    Returns DataFrame with columns:
        timestamp_s, 
        left_foot_fk_x, left_foot_fk_y, left_foot_fk_z,
        right_foot_fk_x, right_foot_fk_y, right_foot_fk_z
    """
    df = pd.read_csv(fk_path)
    
    # Extract relevant columns and rename for clarity
    fk_df = pd.DataFrame({
        'timestamp_s': df['timestamp'],
        'left_foot_fk_x': df['fk_left_foot_pos_x'],
        'left_foot_fk_y': df['fk_left_foot_pos_y'],
        'left_foot_fk_z': df['fk_left_foot_pos_z'],
        'right_foot_fk_x': df['fk_right_foot_pos_x'],
        'right_foot_fk_y': df['fk_right_foot_pos_y'],
        'right_foot_fk_z': df['fk_right_foot_pos_z']
    })
    
    return fk_df.sort_values('timestamp_s').reset_index(drop=True)


def load_velocities(velocities_path: Path) -> pd.DataFrame:
    """
    Load foot velocities CSV and compute per-foot estimated velocity norms.

    Expected columns (output of mainSmoother):
        timestamp_s,
        left_foot_fk_vx/vy/vz,  right_foot_fk_vx/vy/vz,
        left_foot_contact,       right_foot_contact

    Returns DataFrame with added columns: left_est_vnorm, right_est_vnorm.
    """
    df = pd.read_csv(velocities_path)

    df['left_est_vnorm'] = np.sqrt(
        df['left_foot_fk_vx'] ** 2 +
        df['left_foot_fk_vy'] ** 2 +
        df['left_foot_fk_vz'] ** 2
    )
    df['right_est_vnorm'] = np.sqrt(
        df['right_foot_fk_vx'] ** 2 +
        df['right_foot_fk_vy'] ** 2 +
        df['right_foot_fk_vz'] ** 2
    )
    return df

# Storing the dv norm

def processContactSequence(
    contacts_df: pd.DataFrame,
    vel_df: pd.DataFrame,
    fk_df: pd.DataFrame,
    lfoot_contact_seq: np.ndarray,
    rfoot_contact_seq: np.ndarray,
) -> pd.DataFrame:
    """
    Update contacts.csv with smoothed contact sequences.
    
    - Add new contact rows when smoothed sequence adds contacts
    - Remove contact rows when smoothed sequence removes contacts
    - Update is_new_contact field based on smoothed transitions
    - Extract FK values (body_x/y/z) from robotstate CSV for ALL contacts
    
    Args:
        contacts_df:       Original contacts.csv (not used for FK anymore)
        vel_df:            Dense per-timestep DataFrame (timestamp_s, lfoot_contact, rfoot_contact)
        fk_df:             FK positions from FK CSV (timestamp_s, *_foot_fk_*)
        lfoot_contact_seq: Smoothed boolean array for left foot
        rfoot_contact_seq: Smoothed boolean array for right foot
    
    Returns:
        Updated contacts DataFrame with FK values from FK CSV
    """
    assert len(lfoot_contact_seq) == len(vel_df), "lfoot sequence length mismatch"
    assert len(rfoot_contact_seq) == len(vel_df), "rfoot sequence length mismatch"
    
    # Build contact sequence from smoothed arrays
    # Follows csvOrganizer.cpp logic: write event for EVERY timestep where any foot is in contact
    new_rows = []
    prev_lfoot = False
    prev_rfoot = False
    event_index = 0
    
    for i in range(len(vel_df)):
        ts = float(vel_df.iloc[i]['timestamp_s'])
        lfoot_contact = bool(lfoot_contact_seq[i])
        rfoot_contact = bool(rfoot_contact_seq[i])
        
        # Check if we need to write contact events for this timestep
        write_event = lfoot_contact or rfoot_contact
        
        if write_event:
            # Get FK values for this timestep
            fk_row = fk_df.iloc[i]
            
            # Detect new contacts
            lfoot_new_contact = lfoot_contact and not prev_lfoot
            rfoot_new_contact = rfoot_contact and not prev_rfoot
            
            # Write left foot contact if active
            if lfoot_contact:
                new_rows.append({
                    'event_index': event_index,
                    'timestamp_s': ts,
                    'foot_index': 0,
                    'foot_name': 'left_foot',
                    'is_new_contact': int(lfoot_new_contact),
                    'body_x': fk_row['left_foot_fk_x'],
                    'body_y': fk_row['left_foot_fk_y'],
                    'body_z': fk_row['left_foot_fk_z']
                })
            
            # Write right foot contact if active
            if rfoot_contact:
                new_rows.append({
                    'event_index': event_index,
                    'timestamp_s': ts,
                    'foot_index': 1,
                    'foot_name': 'right_foot',
                    'is_new_contact': int(rfoot_new_contact),
                    'body_x': fk_row['right_foot_fk_x'],
                    'body_y': fk_row['right_foot_fk_y'],
                    'body_z': fk_row['right_foot_fk_z']
                })
            
            event_index += 1
        
        # Update previous contact states
        prev_lfoot = lfoot_contact
        prev_rfoot = rfoot_contact
    
    # Create result DataFrame
    column_order = ['event_index', 'timestamp_s', 'foot_index', 'foot_name', 
                    'is_new_contact', 'body_x', 'body_y', 'body_z']
    
    if not new_rows:
        # Return empty DataFrame with correct columns
        return pd.DataFrame(columns=column_order)
    
    result = pd.DataFrame(new_rows)
    
    # Reorder columns to match original (event_index already assigned per-row)
    result = result[column_order]
    
    return result


def readContactsFromCsv(velocities_path: Path | str) -> pd.DataFrame:
    """
    Extract a dense per-timestep contact sequence from the foot velocities CSV.

    The velocities CSV already has one row per timestep with:
        timestamp_s        – time in seconds
        left_foot_contact  – 0/1, left foot in contact
        right_foot_contact – 0/1, right foot in contact

    Defaults for any missing timestep are False (no contact).

    Returns a DataFrame with columns:
        timestamp_s, lfoot_contact (bool), rfoot_contact (bool)
    one row per timestep, sorted by timestamp.
    """
    velocities_path = Path(velocities_path)
    df = pd.read_csv(velocities_path)

    required = ['timestamp_s', 'left_foot_contact', 'right_foot_contact']
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(
            f"readContactsFromCsv: velocities CSV missing columns: {missing}\n"
            f"  File: {velocities_path}"
        )

    out = df[required].copy().rename(columns={
        'left_foot_contact':  'lfoot_contact',
        'right_foot_contact': 'rfoot_contact',
    })
    out['lfoot_contact'] = out['lfoot_contact'].fillna(0).astype(bool)
    out['rfoot_contact'] = out['rfoot_contact'].fillna(0).astype(bool)
    return out.sort_values('timestamp_s').reset_index(drop=True)


# ---------------------------------------------------------------------------
# Core processing
# ---------------------------------------------------------------------------


def thresholdSmoother(
    contact_df: pd.DataFrame,
    velocities_df: pd.DataFrame,
    vel_threshold: float = 0.10,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Simple baseline contact smoother using velocity thresholding.
    
    Rule: Remove contact if estimated foot velocity exceeds threshold.
    This provides a baseline for comparison against the convex optimization approach.
    
    Args:
        contact_df:     DataFrame from readContactsFromCsv (timestamp_s,
                       lfoot_contact, rfoot_contact) — dense per-timestep.
        velocities_df:  DataFrame from load_velocities with velocity norms.
        vel_threshold:  Velocity threshold (m/s). Contacts with velocity above
                       this are removed.
    
    Returns:
        (lfoot_smoothed, rfoot_smoothed) – boolean numpy arrays, same length
        as contact_df.
    """
    lfoot_seq = contact_df['lfoot_contact'].values.astype(bool).copy()
    rfoot_seq = contact_df['rfoot_contact'].values.astype(bool).copy()
    timestamps = contact_df['timestamp_s'].values
    
    assert len(contact_df) == len(velocities_df), (
        "contact_df and velocities_df row counts differ"
    )
    
    # Get velocity norms (already computed in load_velocities)
    vel_left_norm = velocities_df['left_est_vnorm'].values
    vel_right_norm = velocities_df['right_est_vnorm'].values
    
    # Apply threshold: remove contact if velocity exceeds threshold
    lfoot_smooth = lfoot_seq & (vel_left_norm <= vel_threshold)
    rfoot_smooth = rfoot_seq & (vel_right_norm <= vel_threshold)
    
    # Log changes
    changes = []
    for i in range(len(timestamps)):
        if lfoot_seq[i] and not lfoot_smooth[i]:
            changes.append((timestamps[i], "left", "removed", vel_left_norm[i]))
        if rfoot_seq[i] and not rfoot_smooth[i]:
            changes.append((timestamps[i], "right", "removed", vel_right_norm[i]))
    
    if changes:
        print(f"  Contact adjustments: {len(changes)}")
        for t, foot, action, v in changes:
            print(f"    t={t:.3f}s {foot} {action} (v_norm={v:.4f} m/s)")
    else:
        print("  No contact adjustments made")
    
    print(f"    contacts: {lfoot_seq.sum() + rfoot_seq.sum()} → "
          f"{lfoot_smooth.sum() + rfoot_smooth.sum()}")
    
    return lfoot_smooth, rfoot_smooth


def convexSmoother(
    contact_df: pd.DataFrame,
    velocities_df: pd.DataFrame,
    history_lfoot: list = None,
    history_rfoot: list = None,
    convergence_threshold: float = 1.0,
    beta: float = 2.0,
    lambda_tv: float = 1.0,
    lambda_v: float = 1.0,
    r0: float = 0.1,
) -> Tuple[np.ndarray, np.ndarray, bool, float]:
    """
    Contact smoother using convex optimization (Linear Program) with convergence tracking.

    Formulation (per foot):
        min  sum_t [beta*r_t + lambda_v*(||v||_t - r0)*c_t] + lambda_tv * sum_{t>=2} d_t
        where ||v||_t = sqrt(vx^2 + vy^2 + 3*vz^2)  (weighted velocity norm, vz penalized 3x more)
        s.t. r_t >= c_t - bar_c_t                         (prior epigraph, upper)
             r_t >= bar_c_t - c_t                         (prior epigraph, lower)
             d_t >= c_t - c_{t-1}                         (TV epigraph, upper)
             d_t >= c_{t-1} - c_t                         (TV epigraph, lower)
             0 <= c_t <= 1
             d_t, r_t >= 0
    Threshold: c_hat_t = 1 if c_t >= 0.5 else 0.
    
    Velocity term: When ||v||_t < r0, cost coefficient is negative → encourages contact
                   When ||v||_t > r0, cost coefficient is positive → discourages contact

    Convergence tracking:
        - history[0] is always used as the prior (bar_c_t) in the cost function
        - history[-1] is compared with current result for convergence check
        - If not converged, caller should append current result to history

    Args:
        contact_df:            DataFrame from readContactsFromCsv (timestamp_s,
                               lfoot_contact, rfoot_contact) — dense per-timestep.
        velocities_df:         DataFrame from load_velocities.
        history_lfoot:         List of previous left foot contact sequences (boolean arrays).
                               If None, initializes with current contact_df state.
                               First entry is always used as prior.
        history_rfoot:         List of previous right foot contact sequences (boolean arrays).
                               If None, initializes with current contact_df state.
                               First entry is always used as prior.
        convergence_threshold: Percentage threshold for convergence (0-100).
                               If change from history[-1] is below this, converged=True.
        beta:                  Weight for prior (L1 deviation from original contacts).
        lambda_tv:             Weight for TV epigraph (smoothness) term.
        lambda_v:              Weight for velocity-based contact term.
        r0:                    Velocity threshold (m/s) - contact encouraged below, discouraged above.

    Returns:
        (lfoot_smoothed, rfoot_smoothed, converged, change_pct, history_lfoot, history_rfoot) – 
        smoothed boolean arrays, convergence flag, percentage change from history[-1], and updated histories.
    """
    lfoot_seq  = contact_df['lfoot_contact'].values.astype(bool).copy()
    rfoot_seq  = contact_df['rfoot_contact'].values.astype(bool).copy()
    timestamps = contact_df['timestamp_s'].values

    assert len(contact_df) == len(velocities_df), (
        "contact_df and velocities_df row counts differ — they must come from "
        "the same foot velocities CSV"
    )

    # Initialize history if not provided (first iteration)
    if history_lfoot is None:
        history_lfoot = [lfoot_seq.copy()]
        print("    Initializing left foot history with current contacts")
    if history_rfoot is None:
        history_rfoot = [rfoot_seq.copy()]
        print("    Initializing right foot history with current contacts")

    # Always use the FIRST sequence as prior for cost function
    prior_lfoot = history_lfoot[0]
    prior_rfoot = history_rfoot[0]
    
    # Use LATEST sequence for convergence check
    latest_lfoot = history_lfoot[-1]
    latest_rfoot = history_rfoot[-1]
    

    # Extract separate XY and Z velocity components for each foot
    vel_left_xy  = velocities_df[['left_foot_fk_vx',  'left_foot_fk_vy']].values   # (n, 2)
    vel_left_z   = velocities_df['left_foot_fk_vz'].values                            # (n,)
    vel_right_xy = velocities_df[['right_foot_fk_vx', 'right_foot_fk_vy']].values   # (n, 2)
    vel_right_z  = velocities_df['right_foot_fk_vz'].values                           # (n,)

    def solve_contact_cvx(
        vel_xy: np.ndarray,
        vel_z: np.ndarray,
        prior_contacts: np.ndarray,
    ) -> np.ndarray:
        """
        Solve the LP-structured contact smoothing problem for one leg.

        Decision variables:
            c     – continuous contact in [0, 1]
            d     – TV epigraph variable (>= 0), length n-1
            r     – prior epigraph variable (>= 0), length n
        
        Cost includes velocity term: lambda_v * (||v||_t - r0) * c_t
        where ||v||_t = sqrt(vx^2 + vy^2 + beta_z*vz^2)
        """
        n = len(vel_z)

        c    = cp.Variable(n, name="c")
        d    = cp.Variable(n - 1, name="d")
        r    = cp.Variable(n, name="r")

        c0 = prior_contacts.astype(float)  # Always use first sequence as prior
        
        # Compute weighted velocity norm: ||v||_t = sqrt(vx^2 + vy^2 + beta_z*vz^2)
        # where beta_z = 1 (penalize vertical velocity moderately)
        beta_z = 1.0
        vnorm = np.sqrt(
            vel_xy[:, 0]**2 + vel_xy[:, 1]**2 + beta_z * vel_z**2
        )  # (n,) — fixed data
        
        # Velocity-based cost coefficient: a_t = lambda_v * (||v||_t - r0)
        # When ||v||_t < r0: a_t < 0 → encourages contact (reduces cost)
        # When ||v||_t > r0: a_t > 0 → discourages contact (increases cost)
        a_vel = lambda_v * (vnorm - r0)  # (n,) — fixed data

        constraints = [
            # Box constraint
            c >= 0, c <= 1,
            # Epigraph non-negativity
            d >= 0, r >= 0,
            # Prior epigraph: r_t >= |c_t - bar_c_t|
            r >= c - c0,
            r >= c0 - c,
            # TV epigraph: d_t >= |c_t - c_{t-1}|
            d >= c[1:] - c[:-1],
            d >= c[:-1] - c[1:],
        ]

        cost = (beta      * cp.sum(r)
                + lambda_tv * cp.sum(d)
                + cp.sum(cp.multiply(a_vel, c)))  # Velocity-based encouragement

        problem = cp.Problem(cp.Minimize(cost), constraints)
        # Solve LP using HiGHS (efficient LP solver)
        problem.solve(
            solver=cp.SCIPY,
            scipy_options={"method": "highs"}
        )

        if problem.status not in ["optimal", "optimal_inaccurate"]:
            print(f"    ⚠ Warning: solver status = {problem.status}, falling back to prior")
            return c0 >= 0.5

        print(f"    Solver status: {problem.status}, cost: {problem.value:.4f}")
        vel_cost = float(cp.sum(cp.multiply(a_vel, c)).value)
        print(f"      prior={beta * float(cp.sum(r).value):.4f}  "
              f"TV={lambda_tv * float(cp.sum(d).value):.4f}  "
              f"vel_enc={vel_cost:.4f}")
        print(f"      contacts: {c0.sum():.0f}/{n} → {(c.value >= 0.5).sum()}/{n}")

        return c.value >= 0.5

    # Solve per foot (using prior from history[0])
    print("    Solving convex optimization for left foot...")
    lfoot_smooth = solve_contact_cvx(vel_left_xy, vel_left_z, prior_lfoot)

    print("    Solving convex optimization for right foot...")
    rfoot_smooth = solve_contact_cvx(vel_right_xy, vel_right_z, prior_rfoot)

    # Check convergence against LATEST sequence in history
    changes_lfoot = np.sum(lfoot_smooth != latest_lfoot)
    changes_rfoot = np.sum(rfoot_smooth != latest_rfoot)
    total_changes = changes_lfoot + changes_rfoot
    total_timesteps = len(lfoot_smooth) + len(rfoot_smooth)
    change_pct = (total_changes / total_timesteps) * 100.0
    
    converged = change_pct < convergence_threshold
    
    # Update history: append current result if not converged
    if not converged:
        history_lfoot.append(lfoot_smooth.copy())
        history_rfoot.append(rfoot_smooth.copy())
    
    # print(f"  Convergence check:")
    # print(f"    Changes from history[-1]: {total_changes}/{total_timesteps} = {change_pct:.2f}%")
    # print(f"    Converged: {converged}")
    
    # # Log changes from PRIOR (for debugging)
    # vel_left_norm  = np.linalg.norm(vel_left_xy,  axis=1)
    # vel_right_norm = np.linalg.norm(vel_right_xy, axis=1)
    # changes_from_prior = []
    # for i in range(len(timestamps)):
    #     if prior_lfoot[i] != lfoot_smooth[i]:
    #         action = "added" if lfoot_smooth[i] else "removed"
    #         changes_from_prior.append((timestamps[i], "left",  action, vel_left_norm[i]))
    #     if prior_rfoot[i] != rfoot_smooth[i]:
    #         action = "added" if rfoot_smooth[i] else "removed"
    #         changes_from_prior.append((timestamps[i], "right", action, vel_right_norm[i]))

    # if changes_from_prior:
    #     print(f"  Contact adjustments from prior: {len(changes_from_prior)}")
    #     # Only print first few to avoid clutter
    #     for t, foot, action, v in changes_from_prior[:5]:
    #         print(f"    t={t:.3f}s {foot} {action} (v_xy_norm={v:.4f} m/s)")
    #     if len(changes_from_prior) > 5:
    #         print(f"    ... and {len(changes_from_prior) - 5} more")
    # else:
    #     print("  No contact adjustments from prior")

    return lfoot_smooth, rfoot_smooth, converged, change_pct, history_lfoot, history_rfoot


def compute_change_percentage(
    orig_lfoot: np.ndarray,
    orig_rfoot: np.ndarray,
    smooth_lfoot: np.ndarray,
    smooth_rfoot: np.ndarray,
) -> float:
    """
    Percentage of per-timestep contact decisions that changed.

    Concatenates left and right arrays and counts differences.
    Returns a value in [0, 100].
    """
    orig   = np.concatenate([orig_lfoot,   orig_rfoot]).astype(bool)
    smooth = np.concatenate([smooth_lfoot, smooth_rfoot]).astype(bool)
    if len(orig) == 0:
        return 0.0
    return float(np.sum(orig != smooth)) / len(orig) * 100.0


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def smooth_contacts(
    velocities_path: Path | str,
    contacts_path: Path | str,
    fk_path: Path | str,
    method: str = "convex",
    history_lfoot: list = None,
    history_rfoot: list = None,
    convergence_threshold: float = 1.0,
    vel_threshold: float = 0.10,
    beta: float = 2.0,
    lambda_tv: float = 1.0,
    lambda_v: float = 1.0,
    r0: float = 0.1,
) -> Tuple[float, bool, list, list]:
    """
    Full contact-smoothing pipeline.

    1. Load foot velocities CSV (contains velocities and contact booleans).
    2. Load FK CSV (contains FK foot positions).
    3. Extract dense per-timestep contact sequences.
    4. Smooth the boolean sequences via convex optimization or simple threshold.
    5. Build new contacts.csv with FK values from FK CSV.
    6. Write updated contacts.csv back to contacts_path (in-place update).

    Args:
        velocities_path:       foot_velocities CSV from the C++ estimator.
                               Contains timestamp_s, *_est_v*, left_foot_contact,
                               right_foot_contact.
        contacts_path:         contacts.csv to be updated in-place.
        fk_path:               FK CSV with FK foot positions.
        method:                Smoothing method: "convex" or "threshold".
        history_lfoot:         List of previous left foot sequences for convergence tracking.
                               Pass from previous iteration; updated in-place if not converged.
        history_rfoot:         List of previous right foot sequences for convergence tracking.
                               Pass from previous iteration; updated in-place if not converged.
        convergence_threshold: Convergence threshold (%) for convex method.
        vel_threshold:         Velocity threshold (m/s) — threshold method only.
        beta:                  Weight for prior (L1 deviation from original contacts) — convex only.
        lambda_tv:             Weight for TV epigraph term — convex only.
        lambda_v:              Weight for velocity-based contact term — convex only.
        r0:                    Velocity threshold (m/s) — convex only.

    Returns:
        (change_pct, converged, history_lfoot, history_rfoot) –
        Percentage change from last iteration, convergence flag, and updated histories.
    """
    # Start wall clock timer for contactSmoother
    smoother_start_time = time.time()
    
    velocities_path = Path(velocities_path)
    contacts_path   = Path(contacts_path)
    fk_path         = Path(fk_path)

    # --- Load FK positions ---

    fk_df = load_fk(fk_path)

    # --- Load velocities ---
    velocities_df = load_velocities(velocities_path)

    # --- Extract dense contact booleans ---
    contact_df = readContactsFromCsv(velocities_path)

    orig_lfoot = contact_df['lfoot_contact'].values.astype(bool)
    orig_rfoot = contact_df['rfoot_contact'].values.astype(bool)

    # --- Smooth ---
    converged = False
    if method == "convex":
        smooth_lfoot, smooth_rfoot, converged, change_pct_conv, history_lfoot, history_rfoot = convexSmoother(
            contact_df, velocities_df,
            history_lfoot=history_lfoot,
            history_rfoot=history_rfoot,
            convergence_threshold=convergence_threshold,
            beta=beta,
            lambda_tv=lambda_tv,
            lambda_v=lambda_v,
            r0=r0,
        )
            
    elif method == "threshold":
        print(f"  Running threshold contact smoother (vel_threshold={vel_threshold} m/s)")
        smooth_lfoot, smooth_rfoot = thresholdSmoother(
            contact_df, velocities_df,
            vel_threshold=vel_threshold,
        )
        # Threshold method doesn't track convergence
        converged = True
        change_pct_conv = 0.0
    else:
        raise ValueError(f"Unknown smoothing method: {method}. Use 'convex' or 'threshold'.")

    # --- Change % (relative to original, for logging) ---
    change_pct_from_orig = compute_change_percentage(
        orig_lfoot, orig_rfoot, smooth_lfoot, smooth_rfoot
    )
    # Return the convergence-based change percentage for iterative methods
    change_pct = change_pct_conv if method == "convex" else change_pct_from_orig

    # --- Build new contacts with FK from robotstate ---
    # print(f"  Loading original contacts from {contacts_path}")
    original_contacts = pd.read_csv(contacts_path)
    # print(f"    {len(original_contacts)} contact rows")

    smoothed_contacts = processContactSequence(
        original_contacts, contact_df, fk_df, smooth_lfoot, smooth_rfoot
    )
    print(f"    {len(smoothed_contacts)} contact rows after smoothing")

    if not contacts_path.parent.exists():
        raise RuntimeError(f"Directory does not exist: {contacts_path.parent}")
    
    smoothed_contacts.to_csv(contacts_path, index=False)
    
    # Print wall clock time for this contactSmoother call
    smoother_elapsed_time = time.time() - smoother_start_time
    print(f"  ⏱️  Contact Smoother Wall Clock Time: {smoother_elapsed_time:.2f} seconds")

    return change_pct, converged, history_lfoot, history_rfoot


# ---------------------------------------------------------------------------
# Entry point (for standalone testing only)
# ---------------------------------------------------------------------------

def main():
    """
    Standalone entry point for testing.
    For production use, call smooth_contacts() from iterative_refinement.py
    which has the authoritative configuration.
    """
    # Default test configuration
    VELOCITIES_PATH = Path("convex/Data/foot_velocities.csv")
    CONTACTS_PATH   = Path("convex/Data/contacts.csv")
    FK_PATH         = Path("convex/Data/robotstate_0_800.csv")

    # Example: using threshold method (simple baseline)
    print("\n=== Threshold Method (Baseline) ===")
    change_pct, converged, _, _ = smooth_contacts(
        VELOCITIES_PATH,
        CONTACTS_PATH,
        FK_PATH,
        method="threshold",
        vel_threshold=0.10,  # 10 cm/s threshold
    )
    print(f"\nThreshold method: {change_pct:.2f}% of contact states changed")

    # Example: using convex method with iterative convergence (optimization-based)
    print("\n=== Convex Method (Optimization with Convergence Tracking) ===")
    history_lfoot = None
    history_rfoot = None
    max_iterations = 5
    
    for iteration in range(max_iterations):
        print(f"\n--- Iteration {iteration + 1} ---")
        change_pct, converged, history_lfoot, history_rfoot = smooth_contacts(
            VELOCITIES_PATH,
            CONTACTS_PATH,
            FK_PATH,
            method="convex",
            history_lfoot=history_lfoot,
            history_rfoot=history_rfoot,
            convergence_threshold=1.0,  # Converge when <1% change between iterations
            beta=2.0,
            lambda_tv=1.0,
            lambda_v=1.0,
            r0=0.1,
        )
        print(f"Iteration {iteration + 1}: {change_pct:.2f}% change, converged={converged}")
        
        if converged:
            # print(f"\n✓ Converged after {iteration + 1} iterations!")
            # print(f"  Total history length: {len(history_lfoot)}")
            break
    else:
        print(f"\n⚠ Did not converge after {max_iterations} iterations")
        print(f"  Final change: {change_pct:.2f}%")


if __name__ == "__main__":
    main()