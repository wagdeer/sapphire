#!/usr/bin/env python3
"""Summarize registration and ESKF health from one HILTI gate run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import statistics


QUALITY_RE = re.compile(
    r"\[registration-quality\].*?accepted=(?P<accepted>\w+) "
    r"(?:quality=(?P<quality>\d+) map=(?P<map_allowed>\w+) )?"
    r"converged=(?P<converged>\w+) source=(?P<source>\d+) "
    r"inliers=(?P<inliers>\d+) ratio=(?P<ratio>[0-9.eE+-]+) "
    r"iterations=(?P<iterations>\d+) error=(?P<error>[0-9.eE+-]+) "
    r"(?:error_per_inlier=(?P<error_per_inlier>[A-Za-z0-9.eE+-]+) )?"
    r"rot_min=(?P<rot_min>[0-9.eE+-]+) "
    r"trans_min=(?P<trans_min>[0-9.eE+-]+) "
    r"rot_cond=(?P<rot_cond>[0-9.eE+-]+) "
    r"trans_cond=(?P<trans_cond>[0-9.eE+-]+) "
    r"conditioning_valid=(?P<conditioning_valid>\w+)"
)
UPDATE_RE = re.compile(
    r"\[eskf\] update #(?P<update>\d+): "
    r"nu_rot=(?P<nu_rot>[0-9.eE+-]+)rad "
    r"nu_pos=(?P<nu_pos>[0-9.eE+-]+)m "
    r"dv=(?P<dv>[0-9.eE+-]+)m/s .*?"
    r"NIS/6=(?P<nis>[0-9.eE+-]+) mean=(?P<nis_mean>[0-9.eE+-]+)"
)
POSE_GAIN_RE = re.compile(r"Kpose=\((?P<values>[^)]*)\)")


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = int(index)
    upper = min(lower + 1, len(ordered) - 1)
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def longest_run(values: list[bool]) -> int:
    best = 0
    current = 0
    for value in values:
        current = current + 1 if value else 0
        best = max(best, current)
    return best


def analyze_log(path: Path) -> dict:
    quality = []
    updates = []
    min_pose_gain = []
    for line in path.open("r", encoding="utf-8", errors="replace"):
        match = QUALITY_RE.search(line)
        if match:
            record = match.groupdict()
            quality.append({
                "accepted": record["accepted"] == "true",
                "quality": (
                    int(record["quality"])
                    if record.get("quality") is not None else None
                ),
                "map_allowed": (
                    record["map_allowed"] == "true"
                    if record.get("map_allowed") is not None else None
                ),
                "converged": record["converged"] == "true",
                "source": int(record["source"]),
                "inliers": int(record["inliers"]),
                "ratio": float(record["ratio"]),
                "iterations": int(record["iterations"]),
                "error": float(record["error"]),
                "error_per_inlier": (
                    float(record["error_per_inlier"])
                    if record.get("error_per_inlier") is not None else None
                ),
                "rot_min": float(record["rot_min"]),
                "trans_min": float(record["trans_min"]),
                "rot_cond": float(record["rot_cond"]),
                "trans_cond": float(record["trans_cond"]),
                "conditioning_valid":
                    record["conditioning_valid"] == "true",
            })
        match = UPDATE_RE.search(line)
        if match:
            updates.append({
                "update": int(match.group("update")),
                "nu_rot": float(match.group("nu_rot")),
                "nu_pos": float(match.group("nu_pos")),
                "dv": float(match.group("dv")),
                "nis": float(match.group("nis")),
                "nis_mean": float(match.group("nis_mean")),
            })
        match = POSE_GAIN_RE.search(line)
        if match:
            values = [
                float(value)
                for value in re.split(r"[,;]", match.group("values"))
            ]
            if len(values) == 6:
                min_pose_gain.append(min(values))

    ratios = [record["ratio"] for record in quality]
    error_per_inlier = [
        record["error"] / record["inliers"]
        for record in quality
        if record["inliers"] > 0
    ]
    rotation_conditions = [
        record["rot_cond"]
        for record in quality
        if record["rot_cond"] >= 0.0
    ]
    translation_conditions = [
        record["trans_cond"]
        for record in quality
        if record["trans_cond"] >= 0.0
    ]
    nis_means = [record["nis_mean"] for record in updates]
    nu_pos = [record["nu_pos"] for record in updates]
    dv = [record["dv"] for record in updates]
    ratios_dv_nu = [
        update["dv"] / update["nu_pos"]
        for update in updates
        if update["nu_pos"] > 1e-12
    ]
    zero_inlier = [record["inliers"] == 0 for record in quality]
    rejected = [not record["accepted"] for record in quality]
    return {
        "log": str(path),
        "quality_samples": len(quality),
        "accepted": sum(record["accepted"] for record in quality),
        "healthy": sum(record["quality"] == 2 for record in quality),
        "degraded": sum(record["quality"] == 1 for record in quality),
        "map_updates_allowed": sum(
            record["map_allowed"] is True for record in quality
        ),
        "rejected": sum(rejected),
        "zero_inliers": sum(zero_inlier),
        "longest_zero_inlier_run": longest_run(zero_inlier),
        "longest_rejection_run": longest_run(rejected),
        "conditioning_invalid": sum(
            not record["conditioning_valid"] for record in quality
        ),
        "inlier_ratio": {
            "p05": percentile(ratios, 0.05),
            "median": percentile(ratios, 0.50),
            "p95": percentile(ratios, 0.95),
        },
        "error_per_inlier": {
            "p05": percentile(error_per_inlier, 0.05),
            "median": percentile(error_per_inlier, 0.50),
            "p95": percentile(error_per_inlier, 0.95),
        },
        "rotation_condition": {
            "median": percentile(rotation_conditions, 0.50),
            "p95": percentile(rotation_conditions, 0.95),
        },
        "translation_condition": {
            "median": percentile(translation_conditions, 0.50),
            "p95": percentile(translation_conditions, 0.95),
        },
        "eskf": {
            "update_samples": len(updates),
            "nu_pos_median": percentile(nu_pos, 0.50),
            "dv_median": percentile(dv, 0.50),
            "dv_over_nu_median": percentile(ratios_dv_nu, 0.50),
            "nis_mean_median": percentile(nis_means, 0.50),
            "nis_mean_max": max(nis_means) if nis_means else None,
            "min_pose_gain_median": percentile(min_pose_gain, 0.50),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, required=True)
    args = parser.parse_args()
    records = []
    for log in sorted(args.run_root.glob("hilti_*/*/run.log")):
        record = analyze_log(log)
        record["case"] = log.parent.name
        records.append(record)
    output = args.run_root / "evaluation" / "log_metrics.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps({"cases": records}, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"output": str(output)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
