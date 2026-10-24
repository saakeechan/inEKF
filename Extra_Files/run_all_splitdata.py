#!/usr/bin/env python3
import argparse
import random
import subprocess
import time
from pathlib import Path
from typing import Optional


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def update_yaml_file_name(config_path: Path, csv_path: Path) -> None:
    lines = config_path.read_text().splitlines(keepends=True)
    updated_lines = []
    in_textfile = False
    replaced = False

    for line in lines:
        stripped = line.strip()

        if stripped == "textfile:":
            in_textfile = True
            updated_lines.append(line)
            continue

        if in_textfile and stripped and not line.startswith((" ", "\t")):
            in_textfile = False

        if in_textfile and stripped.startswith("file_name:"):
            indent = line[: len(line) - len(line.lstrip())]
            newline = "\n" if line.endswith("\n") else ""
            updated_lines.append(f'{indent}file_name: "{csv_path.as_posix()}"{newline}')
            replaced = True
        else:
            updated_lines.append(line)

    if not replaced:
        raise RuntimeError(f"Could not find textfile.file_name in {config_path}")

    config_path.write_text("".join(updated_lines))


def find_estimator(explicit_path: Optional[str], root: Path) -> Path:
    candidates = []
    if explicit_path:
        candidates.append(Path(explicit_path))

    candidates.extend(
        [
            root / "INEKF_Estimator",
            root / "build" / "INEKF_Estimator",
            root / "build" / "INEKF_Estimator_Test",
        ]
    )

    for candidate in candidates:
        if candidate.exists() and candidate.is_file():
            return candidate.resolve()

    raise FileNotFoundError(
        "Could not find estimator executable. Pass it with --estimator, "
        "for example: --estimator build/INEKF_Estimator_Test"
    )


def main() -> int:
    root = repo_root()

    parser = argparse.ArgumentParser(
        description="Run INEKF once per CSV by updating config/initialization_config.yaml."
    )
    parser.add_argument(
        "--config",
        default=str(root / "config" / "initialization_config.yaml"),
        help="Path to initialization_config.yaml.",
    )
    parser.add_argument(
        "--split-data",
        default=str(root / "splitData"),
        help="Folder containing CSV files.",
    )
    parser.add_argument(
        "--estimator",
        default=None,
        help="Path to estimator executable. Defaults to common repo/build locations.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the runs without editing the YAML or running the estimator.",
    )
    parser.add_argument(
        "--delay-seconds",
        type=float,
        default=1.1,
        help="Seconds to wait after each estimator run before starting the next one.",
    )
    parser.add_argument(
        "--sample-count",
        type=int,
        default=10,
        help="Number of CSV files to randomly choose from the split-data folder.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Optional random seed for repeatable CSV selection.",
    )
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    split_data_dir = Path(args.split_data).resolve()
    estimator_path = find_estimator(args.estimator, root)
    csv_files = sorted(split_data_dir.glob("*.csv"))

    if not csv_files:
        raise FileNotFoundError(f"No CSV files found in {split_data_dir}")
    if args.sample_count <= 0:
        raise ValueError("--sample-count must be greater than 0")

    rng = random.Random(args.seed)
    selected_csv_files = sorted(rng.sample(csv_files, min(args.sample_count, len(csv_files))))

    print(f"Found {len(csv_files)} CSV files in {split_data_dir}")
    print(f"Selected {len(selected_csv_files)} CSV files randomly")
    if args.seed is not None:
        print(f"Random seed: {args.seed}")
    print(f"Estimator: {estimator_path}")
    print(f"Config: {config_path}")
    print(f"Delay between runs: {args.delay_seconds} seconds")

    for index, csv_path in enumerate(selected_csv_files, start=1):
        yaml_csv_path = Path("..") / "splitData" / csv_path.name
        print(f"\n[{index}/{len(selected_csv_files)}] file_name: {yaml_csv_path.as_posix()}")

        if args.dry_run:
            continue

        update_yaml_file_name(config_path, yaml_csv_path)
        subprocess.run([str(estimator_path)], cwd=estimator_path.parent, check=True)
        if index < len(selected_csv_files) and args.delay_seconds > 0:
            print(f"Waiting {args.delay_seconds} seconds before next run...")
            time.sleep(args.delay_seconds)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
