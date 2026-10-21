#!/usr/bin/env python3
"""从 Sapphire map.db 的原始子图里程计记录生成全局 TUM 轨迹。"""

from __future__ import annotations

import argparse
import os
import sqlite3
import sys
from pathlib import Path

import numpy as np


def resolve_database(input_path: Path) -> tuple[Path, Path]:
    path = input_path.expanduser().resolve()
    if path.is_file():
        return path.parent, path
    if path.is_dir() and (path / "map.db").is_file():
        return path, path / "map.db"
    raise RuntimeError(f"找不到地图数据库：{path}")


def decode_submap_pose(node_id: int, blob: bytes) -> np.ndarray:
    if blob is None or len(blob) != 16 * 4:
        raise RuntimeError(f"数据库节点 {node_id} 的 submap_pose 长度无效")
    pose = np.frombuffer(blob, dtype="<f4").reshape((4, 4), order="F").astype(np.float64)
    validate_transform(node_id, "submap_pose", pose)
    return pose


def validate_transform(node_id: int, name: str, pose: np.ndarray) -> None:
    if not np.isfinite(pose).all():
        raise RuntimeError(f"数据库节点 {node_id} 的 {name} 包含非有限值")
    if not np.allclose(pose[3], (0.0, 0.0, 0.0, 1.0), atol=1e-5):
        raise RuntimeError(f"数据库节点 {node_id} 的 {name} 不是齐次变换")
    rotation = pose[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4) or not np.isclose(
        np.linalg.det(rotation), 1.0, atol=1e-4
    ):
        raise RuntimeError(f"数据库节点 {node_id} 的 {name} 旋转矩阵无效")


def quaternion_to_rotation(node_id: int, quaternion: np.ndarray) -> np.ndarray:
    if not np.isfinite(quaternion).all():
        raise RuntimeError(f"数据库节点 {node_id} 的里程计四元数包含非有限值")
    norm = float(np.linalg.norm(quaternion))
    if not np.isfinite(norm) or norm <= np.finfo(np.float64).eps:
        raise RuntimeError(f"数据库节点 {node_id} 的里程计四元数无效")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


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


def decode_odom_poses(
    node_id: int, count: int, timestamps_blob: bytes, poses_blob: bytes
) -> tuple[np.ndarray, np.ndarray]:
    if count <= 0:
        raise RuntimeError(f"数据库节点 {node_id} 的 odom_pose_count 无效")
    if timestamps_blob is None or len(timestamps_blob) != count * 8:
        raise RuntimeError(f"数据库节点 {node_id} 的 odom_timestamps 长度无效")
    if poses_blob is None or len(poses_blob) != count * 7 * 8:
        raise RuntimeError(f"数据库节点 {node_id} 的 odom_poses 长度无效")
    timestamps = np.frombuffer(timestamps_blob, dtype="<f8")
    poses = np.frombuffer(poses_blob, dtype="<f8").reshape((count, 7))
    if not np.isfinite(timestamps).all() or not np.isfinite(poses).all():
        raise RuntimeError(f"数据库节点 {node_id} 的原始里程计记录包含非有限值")
    return timestamps, poses


def odom_transform(node_id: int, values: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_to_rotation(node_id, values[3:7])
    transform[:3, 3] = values[:3]
    return transform


def export(database_path: Path, output_path: Path) -> tuple[int, int]:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_name(output_path.name + ".tmp")
    node_count = 0
    pose_count = 0
    previous_timestamp: float | None = None

    connection = sqlite3.connect(f"{database_path.as_uri()}?mode=ro", uri=True)
    try:
        rows = connection.execute(
            """
            SELECT id, odom_pose_count, odom_timestamps, odom_poses, submap_pose
            FROM Node
            ORDER BY id
            """
        )
        try:
            with temporary_path.open("w", encoding="ascii", newline="\n") as output:
                for node_id_value, count_value, timestamps_blob, poses_blob, submap_blob in rows:
                    node_id = int(node_id_value)
                    count = int(count_value)
                    timestamps, poses = decode_odom_poses(
                        node_id, count, timestamps_blob, poses_blob
                    )
                    T_map_submap = decode_submap_pose(node_id, submap_blob)
                    T_odom_submap = odom_transform(node_id, poses[0])
                    T_submap_odom = np.eye(4, dtype=np.float64)
                    T_submap_odom[:3, :3] = T_odom_submap[:3, :3].T
                    T_submap_odom[:3, 3] = (
                        -T_submap_odom[:3, :3] @ T_odom_submap[:3, 3]
                    )

                    for timestamp_value, odom_values in zip(timestamps, poses):
                        timestamp = float(timestamp_value)
                        if previous_timestamp is not None and timestamp <= previous_timestamp:
                            raise RuntimeError(
                                f"时间戳未全局严格递增：节点 {node_id} 的 {timestamp:.9f}"
                            )
                        T_odom_base = odom_transform(node_id, odom_values)
                        T_map_base = T_map_submap @ T_submap_odom @ T_odom_base
                        validate_transform(node_id, "T_map_base", T_map_base)
                        quaternion = rotation_to_quaternion(node_id, T_map_base[:3, :3])
                        translation = T_map_base[:3, 3]
                        output.write(
                            f"{timestamp:.9f} "
                            f"{translation[0]:.9f} {translation[1]:.9f} {translation[2]:.9f} "
                            f"{quaternion[0]:.9f} {quaternion[1]:.9f} "
                            f"{quaternion[2]:.9f} {quaternion[3]:.9f}\n"
                        )
                        previous_timestamp = timestamp
                        pose_count += 1
                    node_count += 1

                if node_count == 0:
                    raise RuntimeError("map.db 中没有地图节点")
            os.replace(temporary_path, output_path)
        except Exception:
            temporary_path.unlink(missing_ok=True)
            raise
    finally:
        connection.close()
    return node_count, pose_count


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="读取 map.db 中每个子图的原始里程计记录，生成全局 TUM 轨迹"
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
            f"完成：{output_path}，从 {node_count} 个当前 submap_pose "
            f"拼接 {pose_count} 条原始里程计位姿"
        )
        return 0
    except (OSError, RuntimeError, sqlite3.Error, ValueError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
