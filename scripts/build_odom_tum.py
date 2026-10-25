#!/usr/bin/env python3
"""从 Sapphire map.db 的已归档轨迹和优化位姿生成全局 TUM 轨迹。"""

from __future__ import annotations

import argparse
import os
import sqlite3
import sys
from pathlib import Path

import numpy as np

from build_global_map import decode_pose, load_optimized_poses


def resolve_database(input_path: Path) -> tuple[Path, Path]:
    path = input_path.expanduser().resolve()
    if path.is_file():
        return path.parent, path
    if path.is_dir() and (path / "map.db").is_file():
        return path, path / "map.db"
    raise RuntimeError(f"找不到地图数据库：{path}")


def rotation_to_quaternion(node_id: int, rotation: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion = np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
            quaternion = np.array(
                [
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                ]
            )
        elif axis == 1:
            scale = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
            quaternion = np.array(
                [
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                ]
            )
        else:
            scale = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
            quaternion = np.array(
                [
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                ]
            )
    norm = float(np.linalg.norm(quaternion))
    if not np.isfinite(norm) or norm <= np.finfo(np.float64).eps:
        raise RuntimeError(f"数据库节点 {node_id} 的全局姿态四元数无效")
    return quaternion / norm


def export(database_path: Path, output_path: Path) -> tuple[int, int]:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_name(output_path.name + ".tmp")
    anchors = set()
    pose_count = 0
    previous_timestamp = None
    connection = sqlite3.connect(f"{database_path.as_uri()}?mode=ro", uri=True)
    try:
        connection.execute("BEGIN")
        poses = load_optimized_poses(connection)
        rows = connection.execute("SELECT anchor_id,stamp,anchor_from_body FROM Trajectory ORDER BY sequence")
        try:
            with temporary_path.open("w", encoding="ascii", newline="\n") as output:
                for anchor, timestamp, relative_blob in rows:
                    if anchor not in poses:
                        raise RuntimeError(f"轨迹锚点 {anchor} 缺少优化位姿")
                    if not np.isfinite(timestamp) or (previous_timestamp is not None and timestamp <= previous_timestamp):
                        raise RuntimeError("轨迹时间戳非有限或未严格递增")
                    transform = poses[anchor] @ decode_pose(anchor, relative_blob)
                    quaternion = rotation_to_quaternion(anchor, transform[:3, :3])
                    values = [timestamp, *transform[:3, 3], *quaternion]
                    output.write(" ".join(f"{value:.9f}" for value in values) + "\n")
                    previous_timestamp = timestamp
                    anchors.add(anchor); pose_count += 1
                if not pose_count:
                    raise RuntimeError("map.db 中没有已归档的 Trajectory 记录")
            os.replace(temporary_path, output_path)
        except Exception:
            temporary_path.unlink(missing_ok=True)
            raise
    finally:
        connection.close()
    return len(anchors), pose_count


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="读取 map.db 中Trajectory 和优化位姿，生成全局 TUM 轨迹"
    )
    parser.add_argument("session", type=Path, help="地图会话目录或 map.db 路径")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="输出路径，默认写入地图会话目录下的 odom_pose.tum",
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        session_dir, database_path = resolve_database(arguments.session)
        output_path = (
            arguments.output.expanduser().resolve()
            if arguments.output
            else session_dir / "odom_pose.tum"
        )
        node_count, pose_count = export(database_path, output_path)
        print(
            f"完成：{output_path}，从 {node_count} 个优化锚点 "
            f"拼接 {pose_count} 条归档轨迹位姿"
        )
        return 0
    except (OSError, RuntimeError, sqlite3.Error, ValueError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
