#!/usr/bin/env python3
"""Run the fixed five-sequence HILTI gate inside the ROS 1 container."""

from __future__ import annotations

import argparse
import concurrent.futures
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import threading
import time


WORKSPACE = Path("/home/bindeer/git/sapphire_ws")
CORE = WORKSPACE / "src" / "sapphire"
CONTAINER = "sapphire_ros1_dev"
BAG_CACHE = Path("/tmp/sapphire_robust_frontend_bags")
ROS_TOPICS = "/hesai/pandar /alphasense/imu"

CASES = (
    {
        "id": "hilti22_exp03",
        "dataset": "hilti_2022",
        "sequence": "exp03_construction_stairs",
        "bag": Path("/mnt/e/dataset/hilti2022/rosbags/"
                    "exp03_construction_stairs.bag"),
        "config": CORE / "cfg" / "hilti_2022_handheld.toml",
        "launch": "hilti_2022_handheld.launch",
        "rate": "0.5",
    },
    {
        "id": "hilti22_exp07",
        "dataset": "hilti_2022",
        "sequence": "exp07_long_corridor",
        "bag": Path("/mnt/e/dataset/hilti2022/rosbags/"
                    "exp07_long_corridor.bag"),
        "config": CORE / "cfg" / "hilti_2022_handheld.toml",
        "launch": "hilti_2022_handheld.launch",
        "rate": "0.5",
    },
    {
        "id": "hilti22_exp09",
        "dataset": "hilti_2022",
        "sequence": "exp09_cupola",
        "bag": Path("/mnt/e/dataset/hilti2022/rosbags/exp09_cupola.bag"),
        "config": CORE / "cfg" / "hilti_2022_handheld.toml",
        "launch": "hilti_2022_handheld.launch",
        "rate": "0.5",
    },
    {
        "id": "hilti22_exp10",
        "dataset": "hilti_2022",
        "sequence": "exp10_cupola_2",
        "bag": Path("/mnt/e/dataset/hilti2022/rosbags/"
                    "exp10_cupola_2.bag"),
        "config": CORE / "cfg" / "hilti_2022_handheld.toml",
        "launch": "hilti_2022_handheld.launch",
        "rate": "0.5",
    },
    {
        "id": "hilti23_site2_handheld_5",
        "dataset": "hilti_2023",
        "sequence": "site2_handheld_5",
        "bag": Path("/mnt/e/dataset/hilti-2023/rosbags/"
                    "site2_handheld_5.bag"),
        "config": CORE / "cfg" / "hilti_2023_handheld.toml",
        "launch": "hilti_2023_handheld.launch",
        "rate": "1.0",
    },
)

_print_lock = threading.Lock()


def log(message: str) -> None:
    with _print_lock:
        print(f"[{dt.datetime.now().isoformat(timespec='seconds')}] {message}",
              flush=True)


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def git_output(repo: Path, *args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(repo), *args],
        text=True,
    )


def source_state() -> dict:
    status = git_output(CORE, "status", "--porcelain=v1")
    diff = subprocess.check_output(
        ["git", "-C", str(CORE), "diff", "--binary"],
        text=True,
        errors="replace",
    )
    return {
        "commit": git_output(CORE, "rev-parse", "HEAD").strip(),
        "branch": git_output(CORE, "branch", "--show-current").strip(),
        "dirty": bool(status.strip()),
        "changed_paths": [
            line[3:] for line in status.splitlines() if len(line) > 3
        ],
        "status_sha256": sha256_text(status),
        "diff_sha256": sha256_text(diff),
    }


def docker(*args: str, timeout: float | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["docker", *args],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
    )


def container_path(host_path: Path) -> str:
    try:
        relative = host_path.relative_to(WORKSPACE)
    except ValueError as exc:
        raise ValueError(
            f"{host_path} is not under workspace {WORKSPACE}"
        ) from exc
    return "/workspace/" + relative.as_posix()


def ensure_bag_cache(case: dict) -> Path:
    source = case["bag"]
    if not source.is_file():
        raise FileNotFoundError(source)
    destination = BAG_CACHE / source.name
    expected_size = source.stat().st_size
    check = docker(
        "exec",
        CONTAINER,
        "bash",
        "-lc",
        (
            f"test -f {shlex.quote(str(destination))} && "
            f"stat -c %s {shlex.quote(str(destination))}"
        ),
    )
    if check.returncode == 0 and check.stdout.strip() == str(expected_size):
        return destination

    log(f"copy {source.name} ({expected_size / 1024**3:.1f} GiB) to bag cache")
    copied = docker(
        "exec",
        CONTAINER,
        "bash",
        "-lc",
        (
            f"mkdir -p {shlex.quote(str(BAG_CACHE))} && "
            f"cp -- {shlex.quote(case['container_bag'])} "
            f"{shlex.quote(str(destination))}"
        ),
    )
    if copied.returncode != 0:
        raise RuntimeError(
            f"failed to cache {source.name}:\n{copied.stdout}"
        )
    return destination


def run_case(case: dict, run_root: Path, worker: int) -> dict:
    case_dir = run_root / case["dataset"] / case["sequence"]
    case_dir.mkdir(parents=True, exist_ok=True)
    trajectory = case_dir / "trajectory.tum"
    run_log = case_dir / "run.log"
    bag_log = case_dir / "bag.log"
    trajectory.unlink(missing_ok=True)

    local_bag = ensure_bag_cache(case)
    master_port = 12211 + 20 * worker
    script = r"""
set -uo pipefail
source /opt/ros/noetic/setup.bash
source /workspace/devel_ros1/setup.bash
export ROS_MASTER_URI="http://127.0.0.1:__PORT__"
export ROS_HOSTNAME=127.0.0.1
export ROS_HOME="__ROSHOME__"
mkdir -p "$ROS_HOME"
roslaunch sapphire_ros1 __LAUNCH__ \
  config_file:=__CONFIG__ \
  rviz:=false \
  bag_required:=false \
  trajectory_output:=__TRAJECTORY__ \
  >__RUNLOG__ 2>&1 </dev/null &
launch_pid=$!
sleep 5
rosbag play --clock --delay 2.0 --queue 1000 --rate __RATE__ \
  --topics __TOPICS__ --bags __BAG__ \
  >__BAGLOG__ 2>&1 </dev/null
play_status=$?
sleep 30
rosnode kill /sapphire_ros >/dev/null 2>&1 || true
for _ in $(seq 1 60); do
  kill -0 "$launch_pid" 2>/dev/null || break
  sleep 1
done
kill -INT "$launch_pid" 2>/dev/null || true
sleep 2
kill -TERM "$launch_pid" 2>/dev/null || true
wait "$launch_pid" 2>/dev/null || true
if [[ -s __TRAJECTORY__ ]]; then
  exit "$play_status"
fi
exit 20
"""
    replacements = {
        "__PORT__": str(master_port),
        "__ROSHOME__": f"/tmp/sapphire_robust_frontend_ros_w{worker}",
        "__LAUNCH__": case["launch"],
        "__CONFIG__": container_path(case["config"]),
        "__TRAJECTORY__": container_path(trajectory),
        "__RUNLOG__": container_path(run_log),
        "__RATE__": case["rate"],
        "__TOPICS__": ROS_TOPICS,
        "__BAG__": str(local_bag),
        "__BAGLOG__": container_path(bag_log),
    }
    for key, value in replacements.items():
        script = script.replace(key, value)

    started = dt.datetime.now(dt.timezone.utc)
    log(f"START {case['id']} worker={worker} port={master_port}")
    result = docker(
        "exec",
        CONTAINER,
        "bash",
        "-lc",
        script,
    )
    ended = dt.datetime.now(dt.timezone.utc)
    trajectory_rows = 0
    if trajectory.is_file():
        with trajectory.open("r", encoding="utf-8", errors="replace") as stream:
            trajectory_rows = sum(
                1 for line in stream
                if line.strip() and not line.lstrip().startswith("#")
            )
    status = "ok"
    if result.returncode != 0:
        status = "run_failed"
    if trajectory_rows < 2:
        status = "missing_trajectory"

    record = {
        "id": case["id"],
        "dataset": case["dataset"],
        "sequence": case["sequence"],
        "status": status,
        "returncode": result.returncode,
        "started_at_utc": started.isoformat(),
        "finished_at_utc": ended.isoformat(),
        "duration_sec": (ended - started).total_seconds(),
        "trajectory": str(trajectory),
        "trajectory_rows": trajectory_rows,
        "run_log": str(run_log),
        "bag_log": str(bag_log),
        "playback_rate": case["rate"],
    }
    (case_dir / "run_record.json").write_text(
        json.dumps(record, indent=2) + "\n",
        encoding="utf-8",
    )
    log(f"END {case['id']} status={status} rows={trajectory_rows}")
    return record


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--gate",
        required=True,
        help="short gate name, for example gate-0-observability",
    )
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument(
        "--case",
        action="append",
        dest="cases",
        help="run only the named case id; may be repeated",
    )
    args = parser.parse_args()

    now = dt.datetime.now(dt.timezone.utc)
    run_root = (
        WORKSPACE
        / "results"
        / "robust_frontend"
        / f"{now:%Y}"
        / f"{now:%m}"
        / f"{now:%d}"
        / f"{args.gate}_{now:%Y%m%dT%H%M%SZ}"
    )
    run_root.mkdir(parents=True, exist_ok=False)

    selected_cases = CASES
    if args.cases:
        known = {case["id"] for case in CASES}
        requested = set(args.cases)
        unknown = sorted(requested - known)
        if unknown:
            parser.error(f"unknown case id(s): {', '.join(unknown)}")
        selected_cases = tuple(
            case for case in CASES if case["id"] in requested
        )

    cases = []
    for case in selected_cases:
        item = dict(case)
        item["container_bag"] = "/datasets/" + str(
            case["bag"].relative_to("/mnt/e/dataset")
        )
        cases.append(item)

    manifest = {
        "schema_version": 1,
        "gate": args.gate,
        "created_at_utc": now.isoformat(),
        "source_state": source_state(),
        "protocol": {
            "container": CONTAINER,
            "playback_rates": {
                "hilti_2022": 0.5,
                "hilti_2023": 1.0,
            },
            "ros_clock": True,
            "rviz": False,
            "bag_cache": str(BAG_CACHE),
        },
        "cases": [
            {key: str(value) if isinstance(value, Path) else value
             for key, value in case.items()}
            for case in cases
        ],
    }
    (run_root / "run_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n",
        encoding="utf-8",
    )

    # Copy once before parallel replay. The five fixed bags exceed 100 GiB and
    # remain cached for subsequent gates in the same development session.
    for case in cases:
        ensure_bag_cache(case)

    lock = threading.Lock()
    records: list[dict] = []

    jobs = max(1, args.jobs)
    buckets: list[list[dict]] = [[] for _ in range(jobs)]
    for index, case in enumerate(cases):
        buckets[index % jobs].append(case)

    def worker_run(worker: int, assigned: list[dict]) -> list[dict]:
        return [
            run_case(case, run_root, worker)
            for case in assigned
        ]

    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as executor:
        futures = [
            executor.submit(worker_run, worker, assigned)
            for worker, assigned in enumerate(buckets)
            if assigned
        ]
        for future in concurrent.futures.as_completed(futures):
            completed_records = future.result()
            with lock:
                records.extend(completed_records)
                (run_root / "status.json").write_text(
                    json.dumps(
                        {
                            "gate": args.gate,
                            "results": sorted(
                                records, key=lambda item: item["id"]
                            ),
                        },
                        indent=2,
                    )
                    + "\n",
                    encoding="utf-8",
                )

    failed = [record for record in records if record["status"] != "ok"]
    print(json.dumps(
        {
            "run_root": str(run_root),
            "passed": len(records) - len(failed),
            "failed": [record["id"] for record in failed],
        },
        indent=2,
    ))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
