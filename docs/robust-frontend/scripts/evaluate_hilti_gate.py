#!/usr/bin/env python3
"""Evaluate the fixed HILTI gate with the established sparse-ATE protocol."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from evo.core import metrics, sync
from evo.core.geometry import GeometryException
from evo.core.trajectory import PoseTrajectory3D
from evo.main_ape import ape as evo_ape
from scipy.spatial.transform import Rotation


TIP_FROM_IMU = np.array([0.059, -0.00855, 0.1964], dtype=float)
CASES = (
    {
        "id": "hilti22_exp03",
        "dataset": "hilti_2022",
        "sequence": "exp03_construction_stairs",
        "ground_truth": "exp03_construction_stairs.txt",
    },
    {
        "id": "hilti22_exp07",
        "dataset": "hilti_2022",
        "sequence": "exp07_long_corridor",
        "ground_truth": "exp07_long_corridor.txt",
    },
    {
        "id": "hilti22_exp09",
        "dataset": "hilti_2022",
        "sequence": "exp09_cupola",
        "ground_truth": "exp09_cupola.txt",
    },
    {
        "id": "hilti22_exp10",
        "dataset": "hilti_2022",
        "sequence": "exp10_cupola_2",
        "ground_truth": "exp10_cupola_2_imu_3dof.txt",
    },
    {
        "id": "hilti23_site2_handheld_5",
        "dataset": "hilti_2023",
        "sequence": "site2_handheld_5",
        "ground_truth": "site2_handheld_5.txt",
    },
)


def read_tum(path: Path) -> np.ndarray:
    data = np.loadtxt(path, comments="#", ndmin=2)
    if data.shape[1] != 8:
        raise ValueError(f"{path}: expected 8 TUM columns, got {data.shape}")
    if data.shape[0] < 2:
        raise ValueError(f"{path}: expected at least two poses")
    if not np.isfinite(data).all():
        raise ValueError(f"{path}: contains non-finite values")
    if np.any(np.diff(data[:, 0]) <= 0.0):
        raise ValueError(f"{path}: timestamps are not strictly increasing")
    return data


def path_length(positions: np.ndarray) -> float:
    if positions.shape[0] < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())


def evaluate_case(case: dict, run_root: Path, data_root: Path) -> dict:
    estimate_path = (
        run_root
        / case["dataset"]
        / case["sequence"]
        / "trajectory.tum"
    )
    ground_truth_path = (
        data_root
        / ("hilti2022/ground_truth" if case["dataset"] == "hilti_2022"
           else "hilti-2023/ground_truth")
        / case["ground_truth"]
    )
    record = {
        **case,
        "estimate_path": str(estimate_path),
        "ground_truth_path": str(ground_truth_path),
        "status": "missing_estimate",
    }
    if not estimate_path.is_file():
        return record
    if not ground_truth_path.is_file():
        record["status"] = "missing_ground_truth"
        return record

    estimate = read_tum(estimate_path)
    reference = read_tum(ground_truth_path)
    estimate_tip = estimate.copy()
    estimate_tip[:, 1:4] += Rotation.from_quat(
        estimate[:, 4:8]
    ).apply(TIP_FROM_IMU)

    gt = PoseTrajectory3D(
        positions_xyz=reference[:, 1:4],
        orientations_quat_wxyz=reference[:, [7, 4, 5, 6]],
        timestamps=reference[:, 0],
        name="ground_truth_tip",
    )
    estimate_evo = PoseTrajectory3D(
        positions_xyz=estimate_tip[:, 1:4],
        orientations_quat_wxyz=estimate_tip[:, [7, 4, 5, 6]],
        timestamps=estimate_tip[:, 0],
        name="sapphire_tip",
    )
    gt_sync, estimate_sync = sync.associate_trajectories(
        gt,
        estimate_evo,
        max_diff=2.0,
    )
    record.update(
        {
            "estimate_poses": int(estimate.shape[0]),
            "ground_truth_control_points": int(gt.num_poses),
            "matched_control_points": int(gt_sync.num_poses),
            "coverage_ratio": float(gt_sync.num_poses / gt.num_poses),
            "estimated_path_length_m": path_length(estimate_tip[:, 1:4]),
            "ground_truth_path_length_m": path_length(reference[:, 1:4]),
        }
    )
    if gt_sync.num_poses < 2:
        record["status"] = "insufficient_matches"
        return record

    try:
        result = evo_ape(
            gt_sync,
            estimate_sync,
            metrics.PoseRelation.translation_part,
            align=True,
            correct_scale=False,
        )
    except GeometryException as exc:
        record["status"] = "alignment_failed"
        record["evaluation_error"] = str(exc)
        return record
    errors = np.asarray(result.np_arrays["error_array"], dtype=float)
    complete = (
        gt_sync.num_poses == gt.num_poses
        and float(estimate_evo.timestamps[0])
            <= float(gt.timestamps[0]) + 2.0
        and float(estimate_evo.timestamps[-1])
            >= float(gt.timestamps[-1]) - 2.0
    )
    aligned = result.trajectories["sapphire_tip"].positions_xyz
    record.update(
        {
            "status": "ok" if complete else "incomplete_coverage",
            "full_coverage": bool(complete),
            "ate_rmse_m": float(result.stats["rmse"]),
            "ate_mean_m": float(result.stats["mean"]),
            "ate_median_m": float(result.stats["median"]),
            "ate_std_m": float(result.stats["std"]),
            "ate_p95_m": float(np.percentile(errors, 95)),
            "ate_max_m": float(result.stats["max"]),
            "max_timestamp_difference_sec": float(
                np.max(np.abs(gt_sync.timestamps - estimate_sync.timestamps))
            ),
            "per_control_point_errors_m": errors.tolist(),
        }
    )

    plot_dir = run_root / "evaluation" / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(9, 7))
    axis.plot(
        aligned[:, 0],
        aligned[:, 1],
        color="#2563eb",
        linewidth=1.8,
        label="Sapphire, full estimate",
    )
    axis.scatter(
        reference[:, 1],
        reference[:, 2],
        color="#ea580c",
        marker="x",
        s=65,
        linewidths=2,
        label="HILTI control points",
    )
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.25)
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title(
        f"{case['sequence']} | ATE {record['ate_rmse_m']:.4f} m"
        f" | coverage {record['matched_control_points']}/"
        f"{record['ground_truth_control_points']}"
    )
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(
        plot_dir / f"{case['id']}.png",
        dpi=160,
        bbox_inches="tight",
    )
    plt.close(figure)
    return record


def write_csv(records: list[dict], path: Path) -> None:
    fields = [
        "id",
        "dataset",
        "sequence",
        "status",
        "matched_control_points",
        "ground_truth_control_points",
        "coverage_ratio",
        "ate_rmse_m",
        "ate_median_m",
        "ate_p95_m",
        "ate_max_m",
        "estimated_path_length_m",
        "ground_truth_path_length_m",
        "estimate_poses",
    ]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in records:
            writer.writerow({key: record.get(key, "") for key in fields})


def write_markdown(records: list[dict], path: Path) -> None:
    lines = [
        "# HILTI robust-frontend gate",
        "",
        "Sparse control-point translation ATE, IMU-to-pole-tip transform, "
        "SE(3) Umeyama alignment, no scale correction, 2 s association.",
        "",
        "| Case | Status | Coverage | RMSE | Median | P95 | Max |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for record in records:
        if "ate_rmse_m" not in record:
            lines.append(
                f"| {record['id']} | {record['status']} | "
                f"{record.get('matched_control_points', 0)}/"
                f"{record.get('ground_truth_control_points', 0)} | "
                "- | - | - | - |"
            )
            continue
        lines.append(
            f"| {record['id']} | {record['status']} | "
            f"{record['matched_control_points']}/"
            f"{record['ground_truth_control_points']} | "
            f"{record['ate_rmse_m']:.6f} | "
            f"{record['ate_median_m']:.6f} | "
            f"{record['ate_p95_m']:.6f} | "
            f"{record['ate_max_m']:.6f} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path("/datasets"),
    )
    args = parser.parse_args()
    output = args.run_root / "evaluation"
    output.mkdir(parents=True, exist_ok=True)
    records = [
        evaluate_case(case, args.run_root, args.dataset_root)
        for case in CASES
    ]
    payload = {
        "metric": "sparse_ate_translation",
        "alignment": "SE(3) Umeyama",
        "scale_correction": False,
        "max_time_diff_sec": 2.0,
        "tip_from_imu_m": TIP_FROM_IMU.tolist(),
        "results": records,
    }
    (output / "summary.json").write_text(
        json.dumps(payload, indent=2) + "\n",
        encoding="utf-8",
    )
    write_csv(records, output / "summary.csv")
    write_markdown(records, output / "summary.md")
    failed = [
        record["id"]
        for record in records
        if record.get("status") not in {"ok", "incomplete_coverage"}
    ]
    print(json.dumps(
        {
            "summary": str(output / "summary.json"),
            "failed": failed,
        },
        indent=2,
    ))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
