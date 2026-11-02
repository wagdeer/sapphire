"""Separate processes are essential: in-memory state cannot mask persistence loss."""
import os
import math
import struct
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys
import tempfile

lidar, pipeline, config = sys.argv[1:]
env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")

def run(*args, failure=False):
    result = subprocess.run(args, env=env, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, timeout=90)
    if (result.returncode != 0) != failure:
        print(result.stdout)
        raise RuntimeError(f"Unexpected exit {result.returncode}: {args}")
    return result.stdout

with tempfile.TemporaryDirectory(prefix="sapphire-persistence-") as directory:
    root = Path(directory)
    database = root / "lidar.db"
    run(lidar, "write", str(database))
    with sqlite3.connect(database) as connection:
        assert connection.execute("PRAGMA user_version").fetchone()[0] == 14
        tables = {row[0] for row in connection.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        assert not {"Feature", "GlobalDescriptor", "LocalMap"} & tables
        assert "Data" not in tables
    for name, sql in (
        ("previous", "PRAGMA user_version=13"),
        ("old", "PRAGMA user_version=10"),
        ("corrupt", "UPDATE Node SET lidar_geometry=X'010203' WHERE id=1"),
    ):
        damaged = root / f"{name}.db"
        shutil.copyfile(database, damaged)
        with sqlite3.connect(damaged) as connection:
            connection.execute(sql)
        run(lidar, "reopen", str(damaged), failure=True)
        with sqlite3.connect(damaged) as connection:
            assert connection.execute("SELECT COUNT(*) FROM Node").fetchone()[0] == 14
    run(lidar, "reopen", str(database))
    with sqlite3.connect(database) as connection:
        assert connection.execute("SELECT MAX(id) FROM Node").fetchone()[0] > 14
    pipeline_db = root / "pipeline.db"
    run(pipeline, config, str(pipeline_db), "write")
    with sqlite3.connect(pipeline_db) as connection:
        original_trajectory = connection.execute("SELECT * FROM Trajectory ORDER BY sequence").fetchall()
        assert len(original_trajectory) == 2
        assert [row[2] for row in original_trajectory] == [1, 2]
        assert [row[1] for row in original_trajectory] == [1, 1]
        relative = struct.unpack('<12f', original_trajectory[1][3])
        assert abs(relative[3] - .2 * math.cos(.12)) < 1e-5
        assert abs(relative[7] + .2 * math.sin(.12)) < 1e-5
        assert abs(relative[11]) < 1e-5
    run(pipeline, config, str(pipeline_db), "cancel")
    with sqlite3.connect(pipeline_db) as connection:
        assert connection.execute("SELECT COUNT(*) FROM Node").fetchone()[0] == 1
        assert connection.execute("SELECT * FROM Trajectory ORDER BY sequence").fetchall() == original_trajectory
        assert connection.execute("SELECT high_water FROM NodeSequence").fetchone()[0] == 1
    run(pipeline, config, str(pipeline_db), "reopen")
    with sqlite3.connect(pipeline_db) as connection:
        trajectory = connection.execute("SELECT * FROM Trajectory ORDER BY sequence").fetchall()
        assert trajectory[:2] == original_trajectory and len(trajectory) == 4
        assert [row[1] for row in trajectory] == [1, 1, 2, 2]
        assert [row[2] for row in trajectory] == [1, 2, 1, 2]
        assert [row[0] for row in trajectory] == [1, 2, 3, 4]
        assert connection.execute("SELECT high_water FROM NodeSequence").fetchone()[0] == 2
print("PASS separate-process write/reopen/continue, schema and corrupt archive rejection")
