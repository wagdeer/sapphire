"""Compare TUM trajectories at common GT timestamps using evo."""
import csv
import pathlib
import re
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def covered(times, poses):
    stamps = poses[:, 0]
    if not np.isfinite(poses).all() or np.any(np.diff(stamps) <= 0):
        raise ValueError("trajectory must be finite and ordered")
    i = np.searchsorted(stamps, times).clip(1, len(stamps) - 1)
    valid = ((stamps[i] - stamps[i - 1]) < .2) | (stamps[i] == times) | (stamps[i - 1] == times)
    return times[(times >= stamps[0]) & (times <= stamps[-1]) & valid]


def resample(poses, times):
    xyz = np.column_stack([np.interp(times, poses[:, 0], p) for p in poses[:, 1:4].T])
    rotation = Slerp(poses[:, 0], Rotation.from_quat(poses[:, 4:]))(times).as_quat()
    return np.column_stack((times, xyz, rotation))


def rmse(tool, truth, estimate, *options):
    output = subprocess.check_output([tool, "tum", str(truth), str(estimate), "-a", *options], text=True)
    return float(re.search(r"^\s*rmse\s+([\d.eE+-]+)", output, re.M)[1])


def run(root, gt):
    modes = ("main", "methodB")
    phases = ("online", "mapped")
    trajectories = {(mode, phase): np.loadtxt(root / mode / f"{phase}.tum") for mode in modes for phase in phases}
    poses = np.loadtxt(gt)
    stamps = poses[:, 0]
    for trajectory in trajectories.values():
        stamps = covered(stamps, trajectory)
    if len(stamps) < 2:
        raise ValueError("no shared timestamp coverage")

    shared = root / "aligned"
    shared.mkdir(exist_ok=True)
    truth = shared / "truth.tum"
    truth_poses = resample(poses, stamps)
    np.savetxt(truth, truth_poses, fmt="%.9f")
    distance = np.linalg.norm(np.diff(truth_poses[:, 1:4], axis=0), axis=1).sum()
    print("common poses", len(stamps))
    print("mode,phase,ape_m,rpe_10m_m")
    for (mode, phase), trajectory in trajectories.items():
        estimate = shared / f"{mode}-{phase}.tum"
        np.savetxt(estimate, resample(trajectory, stamps), fmt="%.9f")
        ape = rmse("evo_ape", truth, estimate)
        rpe = "n/a"
        if distance >= 9.8:
            error = rmse("evo_rpe", truth, estimate, "-d", "10", "-u", "m", "-t", ".2",
                         "--all_pairs", "--pairs_from_reference")
            rpe = f"{error:.4f}"
        print(f"{mode},{phase},{ape:.4f},{rpe}")

    print("mode,front_p99_ms,online_peak_mib,map_end_mib,map_peak_to_date_mib")
    for mode in modes:
        with (root / mode / "events.csv").open() as handle:
            front = [float(row["ms"]) for row in csv.DictReader(handle) if row["fresh"] == "1"]
        with (root / mode / "memory.csv").open() as handle:
            memory = {row["phase"]: row for row in csv.DictReader(handle)}
        online = int(memory["online_end"]["peak_kib"]) / 1024
        export = int(memory["map_end"]["rss_kib"]) / 1024
        peak = int(memory["map_end"]["peak_kib"]) / 1024
        print(f"{mode},{np.quantile(front, .99):.3f},{online:.1f},{export:.1f},{peak:.1f}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: compare_nclt.py RUN_DIRECTORY GROUND_TRUTH.tum")
    run(pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2]))
