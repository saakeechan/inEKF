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
        left_foot_est_vx/vy/vz,  right_foot_est_vx/vy/vz,
        left_foot_contact,       right_foot_contact

    Returns DataFrame with added columns: left_est_vnorm, right_est_vnorm.
    """
    df = pd.read_csv(velocities_path)

    df['left_est_vnorm'] = np.sqrt(
        df['left_foot_est_vx'] ** 2 +
        df['left_foot_est_vy'] ** 2 +
        df['left_foot_est_vz'] ** 2
    )
    df['right_est_vnorm'] = np.sqrt(
        df['right_foot_est_vx'] ** 2 +
        df['right_foot_est_vy'] ** 2 +
        df['right_foot_est_vz'] ** 2
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
    
    # Build new contact sequence from smoothed arrays
    new_rows = []
    prev_lfoot = False
    prev_rfoot = False
    
    for i in range(len(vel_df)):
        ts = float(vel_df.iloc[i]['timestamp_s'])
        lfoot_contact = bool(lfoot_contact_seq[i])
        rfoot_contact = bool(rfoot_contact_seq[i])
        
        # Get FK values from robotstate for this timestep
        fk_row = fk_df.iloc[i]
        
        # Process left foot
        if lfoot_contact:
            is_new = lfoot_contact and not prev_lfoot
            new_rows.append({
                'timestamp_s': ts,
                'foot_index': 0,
                'foot_name': 'left_foot',
                'is_new_contact': int(is_new),
                'body_x': fk_row['left_foot_fk_x'],
                'body_y': fk_row['left_foot_fk_y'],
                'body_z': fk_row['left_foot_fk_z']
            })
        prev_lfoot = lfoot_contact
        
        # Process right foot
        if rfoot_contact:
            is_new = rfoot_contact and not prev_rfoot
            new_rows.append({
                'timestamp_s': ts,
                'foot_index': 1,
                'foot_name': 'right_foot',
                'is_new_contact': int(is_new),
                'body_x': fk_row['right_foot_fk_x'],
                'body_y': fk_row['right_foot_fk_y'],
                'body_z': fk_row['right_foot_fk_z']
            })
        prev_rfoot = rfoot_contact
    
    # Create result DataFrame
    result = pd.DataFrame(new_rows)
    result['event_index'] = range(len(result))
    
    # Reorder columns to match original
    column_order = ['event_index', 'timestamp_s', 'foot_index', 'foot_name', 
                    'is_new_contact', 'body_x', 'body_y', 'body_z']
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


def convexSmoother(
    contact_df: pd.DataFrame,
    velocities_df: pd.DataFrame,
    velocity_threshold: float = 0.05,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Placeholder contact smoother (least-squares style velocity thresholding).

    Will be replaced with proper convex optimisation later.
    Current heuristic:
      - foot in contact   + high velocity → remove contact
      - foot not in contact + very low velocity → add contact

    Args:
        contact_df:         DataFrame from readContactsFromCsv (timestamp_s,
                            lfoot_contact, rfoot_contact) — dense per-timestep.
        velocities_df:      DataFrame from load_velocities.
        velocity_threshold: Speed threshold in m/s.

    Returns:
        (lfoot_smoothed, rfoot_smoothed) – boolean numpy arrays, same length
        as contact_df.
    """
    lfoot_seq = contact_df['lfoot_contact'].values.astype(bool).copy()
    rfoot_seq = contact_df['rfoot_contact'].values.astype(bool).copy()
    timestamps = contact_df['timestamp_s'].values

    lfoot_smooth = lfoot_seq.copy()
    rfoot_smooth = rfoot_seq.copy()

    # vel_times are already the same rows (both come from the same velocities CSV)
    vel_left  = velocities_df['left_est_vnorm'].values
    vel_right = velocities_df['right_est_vnorm'].values

    # contact_df and velocities_df are both derived from the same file so they
    # are already aligned row-by-row.
    assert len(contact_df) == len(velocities_df), (
        "contact_df and velocities_df row counts differ — they must come from "
        "the same foot velocities CSV"
    )

    changes = []

    # Convex Programming placeholder: generate random contact sequences for testing
    lfoot_smooth = np.random.randint(0, 2, size=len(lfoot_smooth)).astype(bool)
    rfoot_smooth = np.random.randint(0, 2, size=len(rfoot_smooth)).astype(bool)

    if changes:
        print(f"  Contact adjustments: {len(changes)}")
        for t, foot, action, v in changes[:5]:
            print(f"    t={t:.3f}s {foot} {action} (v={v:.4f} m/s)")
        if len(changes) > 5:
            print(f"    ... and {len(changes) - 5} more")
    else:
        print("  No contact adjustments made")

    return lfoot_smooth, rfoot_smooth


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
    velocity_threshold: float = 0.05,
    min_change_pct: float = 10.0,
) -> float:
    """
    Full contact-smoothing pipeline.

    1. Load foot velocities CSV (contains velocities and contact booleans).
    2. Load FK CSV (contains FK foot positions).
    3. Extract dense per-timestep contact sequences.
    4. Smooth the boolean sequences.
    5. Build new contacts.csv with FK values from FK CSV.
    6. Write updated contacts.csv back to contacts_path (in-place update).

    Args:
        velocities_path:    foot_velocities CSV from the C++ estimator.
                            Contains timestamp_s, *_est_v*, left_foot_contact,
                            right_foot_contact.
        contacts_path:      contacts.csv to be updated.
                            This file is updated in-place with smoothed contacts.
        fk_path:            FK CSV with FK foot positions.
        velocity_threshold: Speed threshold for smoothing heuristic (m/s).
        min_change_pct:     Log a warning when fewer than this % of contacts change.

    Returns:
        Percentage of per-timestep contact states that changed (0–100).
    """
    velocities_path = Path(velocities_path)
    contacts_path   = Path(contacts_path)
    fk_path = Path(fk_path)

    # --- Load FK positions ---
    print(f"  Loading FK positions from {fk_path}")
    fk_df = load_fk(fk_path)
    print(f"    {len(fk_df)} timesteps")

    # --- Load velocities ---
    print(f"  Loading velocities from {velocities_path}")
    velocities_df = load_velocities(velocities_path)
    print(f"    {len(velocities_df)} timesteps")

    # --- Extract dense contact booleans ---
    contact_df = readContactsFromCsv(velocities_path)

    orig_lfoot = contact_df['lfoot_contact'].values.astype(bool)
    orig_rfoot = contact_df['rfoot_contact'].values.astype(bool)

    # --- Smooth ---
    print(f"  Running contact smoother (threshold={velocity_threshold} m/s)")
    smooth_lfoot, smooth_rfoot = convexSmoother(
        contact_df, velocities_df, velocity_threshold=velocity_threshold
    )

    # --- Change % ---
    change_pct = compute_change_percentage(
        orig_lfoot, orig_rfoot, smooth_lfoot, smooth_rfoot
    )
    print(f"  Contact states changed: {change_pct:.2f}%")
    if change_pct < min_change_pct:
        print(f"  ⚠ Below min-change threshold ({min_change_pct:.1f}%)")

    # --- Build new contacts with FK from robotstate ---
    print(f"  Loading original contacts from {contacts_path}")
    original_contacts = pd.read_csv(contacts_path)
    print(f"    {len(original_contacts)} contact rows")

    smoothed_contacts = processContactSequence(
        original_contacts, contact_df, fk_df, smooth_lfoot, smooth_rfoot
    )
    print(f"    {len(smoothed_contacts)} contact rows after smoothing")

    if not contacts_path.parent.exists():
        raise RuntimeError(f"Directory does not exist: {contacts_path.parent}")
    
    smoothed_contacts.to_csv(contacts_path, index=False)
    print(f"  Written to {contacts_path}")

    return change_pct


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

VELOCITIES_PATH    = Path("convex/Data/foot_velocities.csv")
CONTACTS_PATH      = Path("convex/Data/contacts.csv")  # updated in-place
FK_PATH            = Path("convex/Data/robotstate_0_env2.csv")
VELOCITY_THRESHOLD = 0.05   # m/s
MIN_CHANGE_PCT     = 10.0   # %


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    change_pct = smooth_contacts(
        VELOCITIES_PATH,
        CONTACTS_PATH,
        FK_PATH,
        velocity_threshold=VELOCITY_THRESHOLD,
        min_change_pct=MIN_CHANGE_PCT,
    )
    print(f"\nFinal: {change_pct:.2f}% of contact states changed")


if __name__ == "__main__":
    main()