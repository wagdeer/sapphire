#!/usr/bin/env python3
"""从 Sapphire map.db 的固定字段 Gaussian payload 流式生成全局 PLY 地图。"""

from __future__ import annotations

import argparse
import math
import os
import sqlite3
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


GAUSSIAN_MAGIC = b"SAPHGAUS"
GAUSSIAN_HEADER = struct.Struct("<8sQ")
GAUSSIAN_DTYPE = np.dtype(
    [
        ("voxel", "<i8", (3,)),
        ("level", "<i4"),
        ("sample_count", "<i4"),
        ("mean", "<f4", (3,)),
        ("covariance", "<f4", (6,)),
        ("radius", "<f4"),
        ("is_plane", "u1"),
    ]
)
GAUSSIAN_RECORD_BYTES = 73
PLY_VERTEX_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("submap_id", "<u4"),
        ("is_plane", "u1"),
    ]
)
FLOAT32_MAX = np.finfo(np.float32).max
UINT32_MAX = np.iinfo(np.uint32).max
MAX_HEADER_VERTEX_COUNT = 10**20 - 1

if GAUSSIAN_DTYPE.itemsize != GAUSSIAN_RECORD_BYTES:
    raise RuntimeError("内部错误：Gaussian 固定记录长度不匹配")
if PLY_VERTEX_DTYPE.itemsize != 17:
    raise RuntimeError("内部错误：PLY vertex 固定记录长度不匹配")


@dataclass(frozen=True)
class SamplingConfig:
    sigma: float = 3.0
    sparse_plane_spacing: float = 0.20
    dense_plane_spacing: float = 0.05
    dense_nonplane_spacing: float = 0.20
    max_plane_samples: int = 256
    max_nonplane_samples: int = 16


def resolve_database(input_path: Path) -> tuple[Path, Path]:
    path = input_path.expanduser().resolve()
    if path.is_file():
        database_path = path
        session_dir = path.parent
    elif path.is_dir() and (path / "map.db").is_file():
        database_path = path / "map.db"
        session_dir = path
    elif path.is_dir() and path.name == "submaps" and (path.parent / "map.db").is_file():
        database_path = path.parent / "map.db"
        session_dir = path.parent
    else:
        raise RuntimeError(f"找不到地图数据库：{path}/map.db")
    return session_dir, database_path


def decode_pose(node_id: int, blob: bytes) -> np.ndarray:
    if blob is None or len(blob) != 16 * 4:
        raise RuntimeError(f"数据库节点 {node_id} 的位姿长度无效")
    pose = np.frombuffer(blob, dtype="<f4").reshape((4, 4), order="F").astype(np.float64)
    if not np.isfinite(pose).all():
        raise RuntimeError(f"数据库节点 {node_id} 的位姿包含非有限值")
    if not np.allclose(pose[3], (0.0, 0.0, 0.0, 1.0), atol=1e-4):
        raise RuntimeError(f"数据库节点 {node_id} 的位姿不是齐次变换矩阵")
    return pose


def decode_gaussians(node_id: int, database_count: int, payload: bytes) -> np.ndarray:
    if database_count < 0 or payload is None or len(payload) < GAUSSIAN_HEADER.size:
        raise RuntimeError(f"数据库节点 {node_id} 的 Gaussian payload 元数据无效")
    magic, payload_count = GAUSSIAN_HEADER.unpack_from(payload)
    if magic != GAUSSIAN_MAGIC:
        raise RuntimeError(f"数据库节点 {node_id} 的 Gaussian magic 无效")
    if payload_count != database_count:
        raise RuntimeError(f"数据库节点 {node_id} 的 Gaussian 数量不一致")
    expected_bytes = GAUSSIAN_HEADER.size + payload_count * GAUSSIAN_RECORD_BYTES
    if len(payload) != expected_bytes:
        raise RuntimeError(f"数据库节点 {node_id} 的 Gaussian payload 长度无效")

    records = np.frombuffer(
        payload,
        dtype=GAUSSIAN_DTYPE,
        count=payload_count,
        offset=GAUSSIAN_HEADER.size,
    )
    if (
        np.any(records["level"] < 0)
        or np.any(records["level"] > 30)
        or np.any(records["sample_count"] <= 0)
        or not np.isfinite(records["mean"]).all()
        or not np.isfinite(records["covariance"]).all()
        or not np.isfinite(records["radius"]).all()
        or np.any(records["radius"] <= 0.0)
        or np.any(records["is_plane"] > 1)
    ):
        raise RuntimeError(f"数据库节点 {node_id} 包含无效 Gaussian 固定字段")
    return records


def _centered_indices(extent: int):
    yield 0
    for index in range(1, extent + 1):
        yield index
        yield -index


def _initial_spacing(radii: np.ndarray, requested: float, cap: int) -> float:
    dimension = radii.size
    shape_factor = math.pi if dimension == 2 else 4.0 * math.pi / 3.0
    log_estimated_count = (
        math.log(shape_factor)
        + sum(math.log(float(radius)) for radius in radii)
        - dimension * math.log(requested)
    )
    if log_estimated_count <= math.log(cap):
        return requested
    return math.exp(
        math.log(requested)
        + (log_estimated_count - math.log(cap)) / dimension
    )


def _plane_grid(radii: np.ndarray, requested: float, cap: int) -> np.ndarray:
    spacing = _initial_spacing(radii, requested, cap)
    for _ in range(64):
        extents = tuple(
            int(math.floor(float(radius) / spacing)) for radius in radii
        )
        samples = [(0.0, 0.0)]
        exceeded = False
        for first in _centered_indices(int(extents[0])):
            normalized_first = (first * spacing / radii[0]) ** 2
            remaining = max(0.0, 1.0 - normalized_first)
            second_extent = min(
                extents[1],
                int(math.floor(radii[1] * math.sqrt(remaining) / spacing + 1e-12)),
            )
            for second in _centered_indices(second_extent):
                if first == 0 and second == 0:
                    continue
                samples.append((first * spacing, second * spacing))
                if len(samples) > cap:
                    exceeded = True
                    break
            if exceeded:
                break
        if not exceeded:
            return np.asarray(samples, dtype=np.float64)

        log_bbox = sum(math.log(2 * extent + 1) for extent in extents)
        scale = max(1.05, math.exp((log_bbox - math.log(cap)) / 2.0))
        spacing *= scale
        if not math.isfinite(spacing):
            break
    raise ValueError("无法在上限内构造平面采样网格")


def _ellipsoid_grid(radii: np.ndarray, requested: float, cap: int) -> np.ndarray:
    spacing = _initial_spacing(radii, requested, cap)
    for _ in range(64):
        extents = tuple(
            int(math.floor(float(radius) / spacing)) for radius in radii
        )
        samples = [(0.0, 0.0, 0.0)]
        exceeded = False
        for first in _centered_indices(int(extents[0])):
            normalized_first = (first * spacing / radii[0]) ** 2
            second_remaining = max(0.0, 1.0 - normalized_first)
            second_extent = min(
                extents[1],
                int(math.floor(radii[1] * math.sqrt(second_remaining) / spacing + 1e-12)),
            )
            for second in _centered_indices(second_extent):
                normalized_second = (second * spacing / radii[1]) ** 2
                remaining = max(0.0, second_remaining - normalized_second)
                third_extent = min(
                    extents[2],
                    int(math.floor(radii[2] * math.sqrt(remaining) / spacing + 1e-12)),
                )
                for third in _centered_indices(third_extent):
                    if first == 0 and second == 0 and third == 0:
                        continue
                    samples.append(
                        (first * spacing, second * spacing, third * spacing)
                    )
                    if len(samples) > cap:
                        exceeded = True
                        break
                if exceeded:
                    break
            if exceeded:
                break
        if not exceeded:
            return np.asarray(samples, dtype=np.float64)

        log_bbox = sum(math.log(2 * extent + 1) for extent in extents)
        scale = max(1.05, math.exp((log_bbox - math.log(cap)) / 3.0))
        spacing *= scale
        if not math.isfinite(spacing):
            break
    raise ValueError("无法在上限内构造三维采样网格")


def _covariance(record: np.void) -> np.ndarray:
    xx, xy, xz, yy, yz, zz = record["covariance"].astype(np.float64)
    return np.asarray(
        ((xx, xy, xz), (xy, yy, yz), (xz, yz, zz)), dtype=np.float64
    )


def sample_gaussian(
    node_id: int,
    gaussian_index: int,
    record: np.void,
    mode: str,
    config: SamplingConfig,
) -> np.ndarray | None:
    is_plane = bool(record["is_plane"])
    if mode == "sparse" and not is_plane:
        return None

    covariance = _covariance(record)
    try:
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    except np.linalg.LinAlgError as error:
        raise RuntimeError(
            f"数据库节点 {node_id} 的 Gaussian {gaussian_index} 协方差特征分解失败"
        ) from error
    if (
        not np.isfinite(eigenvalues).all()
        or not np.isfinite(eigenvectors).all()
        or np.any(eigenvalues <= 0.0)
    ):
        raise RuntimeError(
            f"数据库节点 {node_id} 的 Gaussian {gaussian_index} 协方差不是有限正定矩阵"
        )

    radii = config.sigma * np.sqrt(eigenvalues)
    if not np.isfinite(radii).all() or np.any(radii <= 0.0):
        raise RuntimeError(
            f"数据库节点 {node_id} 的 Gaussian {gaussian_index} 采样范围无效"
        )

    try:
        if is_plane:
            spacing = (
                config.sparse_plane_spacing
                if mode == "sparse"
                else config.dense_plane_spacing
            )
            coordinates = _plane_grid(radii[1:], spacing, config.max_plane_samples)
            offsets = coordinates @ eigenvectors[:, 1:].T
        else:
            coordinates = _ellipsoid_grid(
                radii, config.dense_nonplane_spacing, config.max_nonplane_samples
            )
            offsets = coordinates @ eigenvectors.T
    except (OverflowError, ValueError) as error:
        raise RuntimeError(
            f"数据库节点 {node_id} 的 Gaussian {gaussian_index} 采样网格范围无效"
        ) from error

    local_points = record["mean"].astype(np.float64) + offsets
    if not np.isfinite(local_points).all() or np.any(np.abs(local_points) > FLOAT32_MAX):
        raise RuntimeError(
            f"数据库节点 {node_id} 的 Gaussian {gaussian_index} 局部采样点超出 float32 范围"
        )
    return local_points


def transform_points(node_id: int, local_points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    with np.errstate(over="ignore", invalid="ignore"):
        transformed = local_points @ pose[:3, :3].T + pose[:3, 3]
    if not np.isfinite(transformed).all() or np.any(np.abs(transformed) > FLOAT32_MAX):
        raise RuntimeError(
            f"数据库节点 {node_id} 的采样点经 submap_pose 变换后超出 float32 PLY 数值范围"
        )
    transformed_float = transformed.astype("<f4", copy=False)
    if not np.isfinite(transformed_float).all():
        raise RuntimeError(f"数据库节点 {node_id} 的 PLY 坐标包含非有限值")
    return transformed_float


def make_vertices(
    node_id: int, points: np.ndarray, is_plane: np.ndarray
) -> np.ndarray:
    submap_id = node_id - 1
    if submap_id < 0 or submap_id > UINT32_MAX:
        raise RuntimeError(f"数据库节点 {node_id} 无法表示为 uint submap_id")
    vertices = np.empty(points.shape[0], dtype=PLY_VERTEX_DTYPE)
    vertices["x"] = points[:, 0]
    vertices["y"] = points[:, 1]
    vertices["z"] = points[:, 2]
    vertices["submap_id"] = submap_id
    vertices["is_plane"] = is_plane
    return vertices


def ply_header(vertex_count: int, ascii_output: bool) -> str:
    if vertex_count < 0 or vertex_count > MAX_HEADER_VERTEX_COUNT:
        raise RuntimeError("PLY vertex 数量超出固定文件头范围")
    output_format = "ascii 1.0" if ascii_output else "binary_little_endian 1.0"
    return (
        "ply\n"
        f"format {output_format}\n"
        "comment generated by Sapphire build_global_map.py\n"
        f"element vertex {vertex_count:20d}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uint submap_id\n"
        "property uchar is_plane\n"
        "end_header\n"
    )


def write_vertices(output, vertices: np.ndarray, ascii_output: bool) -> None:
    if ascii_output:
        columns = np.column_stack(
            (
                vertices["x"],
                vertices["y"],
                vertices["z"],
                vertices["submap_id"],
                vertices["is_plane"],
            )
        )
        np.savetxt(output, columns, fmt=("%.9g", "%.9g", "%.9g", "%u", "%u"))
    else:
        vertices.tofile(output)


def assemble(
    database_path: Path,
    output_path: Path,
    ascii_output: bool,
    mode_name: str = "trivial",
    config: SamplingConfig = SamplingConfig(),
) -> tuple[int, int]:
    if mode_name not in ("trivial", "sparse", "dense"):
        raise ValueError(f"无效采样模式：{mode_name}")
    validate_sampling_config(config)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_name(output_path.name + ".tmp")
    total_points = 0
    node_count = 0

    connection = sqlite3.connect(f"{database_path.as_uri()}?mode=ro", uri=True)
    try:
        expected_nodes = int(connection.execute("SELECT COUNT(*) FROM Node").fetchone()[0])
        scene_join = ""
        if connection.execute("PRAGMA user_version").fetchone()[0] == 3:
            missing = connection.execute(
                "SELECT 1 FROM Node n LEFT JOIN SceneState s ON s.node_id=n.id WHERE s.node_id IS NULL LIMIT 1"
            ).fetchone()
            orphan = connection.execute(
                "SELECT 1 FROM SceneState s LEFT JOIN Node n ON n.id=s.node_id WHERE n.id IS NULL LIMIT 1"
            ).fetchone()
            retained_payload = connection.execute(
                "SELECT 1 FROM SceneState s JOIN LaserRecord l ON l.node_id=s.node_id WHERE s.retired_by IS NOT NULL LIMIT 1"
            ).fetchone()
            if missing or orphan or retained_payload:
                raise RuntimeError("场景保留状态与 Node/LaserRecord 不一致")
            expected_nodes = int(connection.execute(
                "SELECT COUNT(*) FROM SceneState WHERE retired_by IS NULL"
            ).fetchone()[0])
            scene_join = "JOIN SceneState s ON s.node_id=n.id AND s.retired_by IS NULL"
        rows = connection.execute(
            f"""
            SELECT n.id, n.submap_pose, l.gaussian_count, l.payload
            FROM Node AS n
            JOIN LaserRecord AS l ON l.node_id = n.id
            {scene_join}
            ORDER BY n.id
            """
        )
        try:
            file_mode = "w+" if ascii_output else "w+b"
            open_kwargs = {"encoding": "ascii", "newline": "\n"} if ascii_output else {}
            with temporary_path.open(file_mode, **open_kwargs) as output:
                initial_header = ply_header(0, ascii_output)
                output.write(initial_header if ascii_output else initial_header.encode("ascii"))

                for raw_node_id, pose_blob, gaussian_count, payload in rows:
                    node_id = int(raw_node_id)
                    pose = decode_pose(node_id, pose_blob)
                    records = decode_gaussians(node_id, int(gaussian_count), payload)
                    if mode_name == "trivial":
                        local_points = records["mean"].astype(np.float64)
                        labels = records["is_plane"]
                    else:
                        point_chunks = []
                        label_chunks = []
                        for gaussian_index, record in enumerate(records):
                            sampled = sample_gaussian(
                                node_id, gaussian_index, record, mode_name, config
                            )
                            if sampled is None:
                                continue
                            point_chunks.append(sampled)
                            label_chunks.append(
                                np.full(sampled.shape[0], record["is_plane"], dtype=np.uint8)
                            )
                        local_points = (
                            np.concatenate(point_chunks)
                            if point_chunks
                            else np.empty((0, 3), dtype=np.float64)
                        )
                        labels = (
                            np.concatenate(label_chunks)
                            if label_chunks
                            else np.empty(0, dtype=np.uint8)
                        )

                    transformed = transform_points(node_id, local_points, pose)
                    vertices = make_vertices(node_id, transformed, labels)
                    write_vertices(output, vertices, ascii_output)
                    node_count += 1
                    total_points += vertices.shape[0]
                    if total_points > MAX_HEADER_VERTEX_COUNT:
                        raise RuntimeError(
                            f"数据库节点 {node_id} 使 PLY vertex 总数超出文件头范围"
                        )
                    print(
                        f"\r[{mode_name}] 正在转换数据库节点 {node_count}/{expected_nodes}，"
                        f"累计 {total_points:,} 点",
                        end="",
                        flush=True,
                    )

                if node_count != expected_nodes:
                    raise RuntimeError(
                        "活跃场景与 LaserRecord 不是一一对应，拒绝生成不完整地图"
                    )
                if node_count == 0:
                    raise RuntimeError("map.db 中没有地图节点")

                final_header = ply_header(total_points, ascii_output)
                if len(final_header) != len(initial_header):
                    raise RuntimeError("内部错误：PLY 文件头长度发生变化")
                output.seek(0)
                output.write(final_header if ascii_output else final_header.encode("ascii"))

            os.replace(temporary_path, output_path)
        except Exception:
            temporary_path.unlink(missing_ok=True)
            raise
    finally:
        connection.close()

    print()
    return total_points, node_count


def finite_positive(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("必须是有限且大于 0 的数")
    return parsed


def positive_integer(value: str) -> int:
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("必须是大于等于 1 的整数")
    return parsed


def validate_sampling_config(config: SamplingConfig) -> None:
    spacings = (
        config.sigma,
        config.sparse_plane_spacing,
        config.dense_plane_spacing,
        config.dense_nonplane_spacing,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in spacings):
        raise ValueError("所有 sigma/spacing 参数必须有限且大于 0")
    if config.max_plane_samples < 1 or config.max_nonplane_samples < 1:
        raise ValueError("采样数量上限必须大于等于 1")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="直接读取 Sapphire map.db，流式生成带子图和平面属性的全局 PLY 地图"
    )
    parser.add_argument(
        "map_directory",
        type=Path,
        help="地图会话目录、会话下的 submaps 目录，或 map.db 路径",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="输出路径，默认写入地图会话目录下的 global_map.ply",
    )
    parser.add_argument(
        "--ascii",
        action="store_true",
        help="输出 ASCII PLY；默认输出二进制 little-endian PLY",
    )
    parser.add_argument(
        "--mode",
        choices=("trivial", "sparse", "dense"),
        default="trivial",
        help="trivial 仅均值；sparse 稀疏平面；dense 稠密平面和稀疏非平面",
    )
    parser.add_argument("--sigma", type=finite_positive, default=3.0)
    parser.add_argument("--sparse-plane-spacing", type=finite_positive, default=0.20)
    parser.add_argument("--dense-plane-spacing", type=finite_positive, default=0.05)
    parser.add_argument("--dense-nonplane-spacing", type=finite_positive, default=0.20)
    parser.add_argument("--max-plane-samples", type=positive_integer, default=256)
    parser.add_argument("--max-nonplane-samples", type=positive_integer, default=16)
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        session_dir, database_path = resolve_database(arguments.map_directory)
        output_path = (
            arguments.output.expanduser().resolve()
            if arguments.output
            else session_dir / "global_map.ply"
        )
        config = SamplingConfig(
            sigma=arguments.sigma,
            sparse_plane_spacing=arguments.sparse_plane_spacing,
            dense_plane_spacing=arguments.dense_plane_spacing,
            dense_nonplane_spacing=arguments.dense_nonplane_spacing,
            max_plane_samples=arguments.max_plane_samples,
            max_nonplane_samples=arguments.max_nonplane_samples,
        )
        point_count, node_count = assemble(
            database_path, output_path, arguments.ascii, arguments.mode, config
        )
        print(
            f"完成：mode={arguments.mode}，{output_path}，{node_count} 个节点，"
            f"共 {point_count:,} 点；使用 map.db 中的当前 submap_pose"
        )
        return 0
    except (OSError, RuntimeError, sqlite3.Error, ValueError, struct.error) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
