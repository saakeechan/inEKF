#!/usr/bin/env python3
"""Generate left-leg slip profile plots for all result CSV files."""

import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


REQUIRED_COLUMNS = (
    "time",
    "EST_Contact_leg1",
    "GT_Contact_leg1",
    "EST_dv_norm_leg1",
    "GT_dv_norm_leg1",
    "EST_dv_leg1_x",
    "EST_dv_leg1_y",
    "EST_dv_leg1_z",
)


def read_column(data, column_name):
    return np.atleast_1d(data[column_name]).astype(float)


def validate_columns(csv_path, data):
    missing_columns = [column for column in REQUIRED_COLUMNS if column not in data.dtype.names]
    if missing_columns:
        missing_text = ", ".join(missing_columns)
        raise ValueError(f"{csv_path.name} is missing required columns: {missing_text}")


def get_plot_y_limit(*series):
    finite_values = np.concatenate([values[np.isfinite(values)] for values in series])
    if finite_values.size == 0:
        return 1.0

    return max(float(np.max(finite_values)), 1.0) * 1.15


def calculate_contact_errors(est_dv_norm, gt_dv_norm, gt_contact):
    valid = gt_contact & np.isfinite(est_dv_norm) & np.isfinite(gt_dv_norm)
    if not np.any(valid):
        return None, None

    error = est_dv_norm[valid] - gt_dv_norm[valid]
    mae = float(np.mean(np.abs(error)))
    mse = float(np.mean(error**2))
    return mae, mse


def calculate_vector_norm(x_values, y_values, z_values):
    return np.linalg.norm(np.column_stack((x_values, y_values, z_values)), axis=1)


def format_error_label(name, mae, mse):
    if mae is None:
        return f"{name} MAE: n/a\n{name} MSE: n/a"
    return f"{name} MAE: {mae:.4f}\n{name} MSE: {mse:.4f}"


def plot_dv_profile(ax, time, est_contact, gt_contact, nn_dv_norm, model_dv_norm, gt_dv_norm, title):
    mismatch = est_contact != gt_contact
    nn_mae, nn_mse = calculate_contact_errors(nn_dv_norm, gt_dv_norm, gt_contact)
    model_mae, model_mse = calculate_contact_errors(model_dv_norm, gt_dv_norm, gt_contact)
    y_limit = get_plot_y_limit(nn_dv_norm, model_dv_norm, gt_dv_norm)

    ax.fill_between(
        time,
        0,
        y_limit,
        where=est_contact,
        step="post",
        color="lightcoral",
        alpha=0.25,
        label="Estimated left contact",
    )

    if np.any(mismatch):
        ax.vlines(
            time[mismatch],
            0,
            y_limit,
            color="darkred",
            linewidth=0.8,
            alpha=0.8,
            label="Contact mismatch",
            zorder=4,
        )
    else:
        ax.plot([], [], color="darkred", linewidth=0.8, label="Contact mismatch")

    ax.plot(
        time,
        gt_dv_norm,
        color="black",
        linewidth=1.8,
        label="Ground truth left dv norm",
    )
    ax.plot(
        time,
        nn_dv_norm,
        color="tab:blue",
        linestyle="--",
        linewidth=1.6,
        label="NN left dv norm",
    )
    ax.plot(
        time,
        model_dv_norm,
        color="purple",
        linestyle="--",
        linewidth=1.6,
        label="Model-based left dv norm",
    )

    ax.set_title(title)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("dv norm")
    ax.set_ylim(0, y_limit)
    ax.grid(True, alpha=0.3)

    ax.plot([], [], linestyle="", label=format_error_label("NN", nn_mae, nn_mse))
    ax.plot([], [], linestyle="", label=format_error_label("Model-based", model_mae, model_mse))
    ax.legend(loc="upper right")


def plot_slip_profile(csv_path, output_path):
    data = np.genfromtxt(csv_path, delimiter=",", names=True)
    validate_columns(csv_path, data)

    time = read_column(data, "time")
    est_contact = read_column(data, "EST_Contact_leg1").astype(bool)
    gt_contact = read_column(data, "GT_Contact_leg1").astype(bool)
    est_dv_norm = read_column(data, "EST_dv_norm_leg1")
    gt_dv_norm = read_column(data, "GT_dv_norm_leg1")
    est_dv_norm_from_components = calculate_vector_norm(
        read_column(data, "EST_dv_leg1_x"),
        read_column(data, "EST_dv_leg1_y"),
        read_column(data, "EST_dv_leg1_z"),
    )

    fig, ax = plt.subplots(figsize=(14, 6))
    plot_dv_profile(
        ax,
        time,
        est_contact,
        gt_contact,
        est_dv_norm,
        est_dv_norm_from_components,
        gt_dv_norm,
        f"Left Leg Slip Profile: {csv_path.stem}",
    )

    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def main():
    repo_root = Path(__file__).resolve().parent.parent
    results_dir = repo_root / "results"
    output_dir = results_dir / "slip_profiles"
    output_dir.mkdir(parents=True, exist_ok=True)

    csv_paths = sorted(results_dir.glob("*.csv"))
    if not csv_paths:
        print(f"No CSV files found in {results_dir}")
        return

    for csv_path in csv_paths:
        output_path = output_dir / f"{csv_path.stem}.png"
        try:
            plot_slip_profile(csv_path, output_path)
            print(f"Saved {output_path}")
        except Exception as exc:
            print(f"Skipping {csv_path.name}: {exc}")


if __name__ == "__main__":
    main()
