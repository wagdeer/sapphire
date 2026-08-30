#!/usr/bin/env python3
"""使用子图点云和位姿生成全局 PLY 点云地图。"""

from __future__ import annotations

import argparse
import os
import sqlite3
import sys
from pathlib import Path

import numpy as np


def resolve_map_paths(input_path: Path) -> tuple[Path, Path, Path]:
    path = input_path.expanduser().resolve()
    if (path / "submaps").is_dir():
        session_dir = path
        submap_dir = path / "submaps"
    elif path.is_dir() and (path / "poses.txt").is_file():
        submap_dir = path
        session_dir = path.parent
    else:
        raise RuntimeError(
            f"{path} 既不是包含 submaps/ 的地图目录，也不是包含 poses.txt 的子图目录"
        )
    return session_dir, submap_dir, session_dir / "map.db"


def validate_pose(submap_id: int, pose: np.ndarray) -> np.ndarray:
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise RuntimeError(f"子图 {submap_id} 的位姿无效")
    if not np.allclose(pose[3], (0.0, 0.0, 0.0, 1.0), atol=1e-4):
        raise RuntimeError(f"子图 {submap_id} 的位姿不是齐次变换矩阵")
    return pose


def load_database_poses(database_path: Path) -> tuple[dict[int, np.ndarray], int]:
    if not database_path.is_file():
        raise RuntimeError(f"找不到位姿数据库：{database_path}")

    poses: dict[int, np.ndarray] = {}
    optimized_count = 0
    connection = sqlite3.connect(f"file:{database_path}?mode=ro", uri=True)
    try:
        rows = connection.execute(
            """
            SELECT id, COALESCE(optimized_pose, odom_pose),
                   optimized_pose IS NOT NULL
            FROM Node ORDER BY id
            """
        )
        for node_id, blob, optimized in rows:
            submap_id = int(node_id) - 1
            if submap_id < 0 or blob is None or len(blob) != 16 * 4:
                raise RuntimeError(f"数据库节点 {node_id} 的位姿数据无效")
            # Eigen::Matrix4f 默认按列存储。
            pose = np.frombuffer(blob, dtype="<f4").reshape((4, 4), order="F")
            poses[submap_id] = validate_pose(submap_id, pose.astype(np.float64))
            optimized_count += int(optimized)
    finally:
        connection.close()

    if not poses:
        raise RuntimeError(f"数据库中没有子图位姿：{database_path}")
    return poses, optimized_count


def quaternion_pose(values: list[float], submap_id: int) -> np.ndarray:
    tx, ty, tz, qx, qy, qz, qw = values
    quaternion = np.asarray((qx, qy, qz, qw), dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-12:
        raise RuntimeError(f"子图 {submap_id} 的四元数无效")
    qx, qy, qz, qw = quaternion / norm

    rotation = np.asarray(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qz * qw),
                2.0 * (qx * qz + qy * qw),
            ],
            [
                2.0 * (qx * qy + qz * qw),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qx * qw),
            ],
            [
                2.0 * (qx * qz - qy * qw),
                2.0 * (qy * qz + qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=np.float64,
    )
    pose = np.eye(4, dtype=np.float64)
    pose[:3, :3] = rotation
    pose[:3, 3] = (tx, ty, tz)
    return validate_pose(submap_id, pose)


def load_text_poses(pose_path: Path) -> tuple[dict[int, np.ndarray], int]:
    if not pose_path.is_file():
        raise RuntimeError(f"找不到子图位姿文件：{pose_path}")

    poses: dict[int, np.ndarray] = {}
    with pose_path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) < 9:
                raise RuntimeError(f"{pose_path}:{line_number} 字段数不足")
            submap_id = int(fields[0])
            if submap_id in poses:
                raise RuntimeError(f"{pose_path}:{line_number} 子图 ID 重复")
            # 格式：id timestamp tx ty tz qx qy qz qw
            poses[submap_id] = quaternion_pose(
                [float(value) for value in fields[2:9]], submap_id
            )

    if not poses:
        raise RuntimeError(f"位姿文件中没有有效记录：{pose_path}")
    return poses, 0


def load_ascii_ply(path: Path) -> np.ndarray:
    with path.open("rb") as stream:
        vertex_count: int | None = None
        file_format: str | None = None
        vertex_properties: list[str] = []
        reading_vertex_properties = False

        while True:
            raw_line = stream.readline()
            if not raw_line:
                raise RuntimeError(f"PLY 文件头不完整：{path}")
            line = raw_line.decode("ascii").strip()
            fields = line.split()
            if fields[:1] == ["format"] and len(fields) >= 2:
                file_format = fields[1]
            elif fields[:2] == ["element", "vertex"] and len(fields) == 3:
                vertex_count = int(fields[2])
                reading_vertex_properties = True
            elif fields[:1] == ["element"]:
                reading_vertex_properties = False
            elif fields[:1] == ["property"] and reading_vertex_properties:
                if len(fields) != 3 or fields[1] == "list":
                    raise RuntimeError(f"不支持的顶点属性：{line}")
                vertex_properties.append(fields[2])
            elif line == "end_header":
                break

        if file_format != "ascii":
            raise RuntimeError(f"仅支持 Sapphire 生成的 ASCII PLY：{path}")
        if vertex_count is None or vertex_count < 0:
            raise RuntimeError(f"PLY 顶点数量无效：{path}")
        try:
            xyz_columns = tuple(vertex_properties.index(axis) for axis in ("x", "y", "z"))
        except ValueError as error:
            raise RuntimeError(f"PLY 缺少 x/y/z 属性：{path}") from error
        if vertex_count == 0:
            return np.empty((0, 3), dtype=np.float64)

        points = np.loadtxt(
            stream,
            dtype=np.float64,
            max_rows=vertex_count,
            usecols=xyz_columns,
            ndmin=2,
        )
        if points.shape != (vertex_count, 3):
            raise RuntimeError(
                f"PLY 点数不匹配：{path}，声明 {vertex_count}，读取 {points.shape[0]}"
            )
        return points[np.isfinite(points).all(axis=1)]


def ply_header(vertex_count: int, ascii_output: bool) -> str:
    output_format = "ascii 1.0" if ascii_output else "binary_little_endian 1.0"
    return (
        "ply\n"
        f"format {output_format}\n"
        "comment generated by Sapphire assemble_global_map.py\n"
        f"element vertex {vertex_count:20d}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
    )


def assemble(
    submap_dir: Path,
    poses: dict[int, np.ndarray],
    output_path: Path,
    ascii_output: bool,
) -> int:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_name(output_path.name + ".tmp")
    total_points = 0

    try:
        mode = "w+" if ascii_output else "w+b"
        open_kwargs = {"encoding": "ascii", "newline": "\n"} if ascii_output else {}
        with temporary_path.open(mode, **open_kwargs) as output:
            initial_header = ply_header(0, ascii_output)
            output.write(initial_header if ascii_output else initial_header.encode("ascii"))

            for index, submap_id in enumerate(sorted(poses), start=1):
                cloud_path = submap_dir / f"{submap_id}.ply"
                if not cloud_path.is_file():
                    raise RuntimeError(f"子图位姿存在，但点云文件不存在：{cloud_path}")

                points = load_ascii_ply(cloud_path)
                pose = poses[submap_id]
                transformed = points @ pose[:3, :3].T + pose[:3, 3]
                transformed = transformed.astype("<f4", copy=False)

                if ascii_output:
                    np.savetxt(output, transformed, fmt="%.9g")
                else:
                    transformed.tofile(output)
                total_points += transformed.shape[0]
                print(
                    f"\r正在拼接子图 {index}/{len(poses)}，累计 {total_points:,} 点",
                    end="",
                    flush=True,
                )

            final_header = ply_header(total_points, ascii_output)
            if len(final_header) != len(initial_header):
                raise RuntimeError("内部错误：PLY 文件头长度发生变化")
            output.seek(0)
            output.write(final_header if ascii_output else final_header.encode("ascii"))

        os.replace(temporary_path, output_path)
    except Exception:
        temporary_path.unlink(missing_ok=True)
        raise

    print()
    return total_points


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="读取 Sapphire 子图点云和位姿，拼接为全局 PLY 点云地图"
    )
    parser.add_argument(
        "map_directory",
        type=Path,
        help="地图会话目录（含 map.db 和 submaps/）或 submaps 目录",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="输出路径，默认写入地图会话目录下的 global_map.ply",
    )
    parser.add_argument(
        "--pose-source",
        choices=("auto", "database", "text"),
        default="auto",
        help="位姿来源：auto 优先使用 map.db 中的优化位姿，默认：auto",
    )
    parser.add_argument(
        "--ascii",
        action="store_true",
        help="输出 ASCII PLY；默认输出体积更小、写入更快的二进制 PLY",
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        session_dir, submap_dir, database_path = resolve_map_paths(
            arguments.map_directory
        )
        pose_source = arguments.pose_source
        if pose_source == "auto":
            pose_source = "database" if database_path.is_file() else "text"

        if pose_source == "database":
            poses, optimized_count = load_database_poses(database_path)
            print(
                f"从 {database_path} 读取 {len(poses)} 个位姿"
                f"（优化位姿 {optimized_count} 个，其余使用里程计位姿）"
            )
        else:
            poses, _ = load_text_poses(submap_dir / "poses.txt")
            print(f"从 {submap_dir / 'poses.txt'} 读取 {len(poses)} 个里程计位姿")

        output_path = (
            arguments.output.expanduser().resolve()
            if arguments.output
            else session_dir / "global_map.ply"
        )
        point_count = assemble(submap_dir, poses, output_path, arguments.ascii)
        print(f"完成：{output_path}，共 {point_count:,} 点")
        return 0
    except (OSError, RuntimeError, sqlite3.Error, ValueError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
