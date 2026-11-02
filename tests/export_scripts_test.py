"""Verify Python exporters against a database written by the native C++ codec."""
import os
from pathlib import Path
import shutil
import sqlite3
import struct
import subprocess
import sys
import tempfile
import zlib

import numpy as np

scripts = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(scripts))
from build_global_map import decode_gaussians, sample_gaussian, SamplingConfig

env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', PYTHONDONTWRITEBYTECODE='1')
def run(*args, fail=False):
    result = subprocess.run(args, env=env, text=True, capture_output=True, timeout=90)
    assert (result.returncode != 0) == fail, result.stdout + result.stderr

def archive(values, dtype, cv_type):
    values = np.asarray(values, dtype=dtype).reshape(-1)
    return zlib.compress(values.tobytes()) + struct.pack('=iii', 1, values.size, cv_type)

with tempfile.TemporaryDirectory(prefix='sapphire-exports-') as directory:
    root = Path(directory)
    database = root / 'map.db'
    run(sys.argv[1], 'write', str(database))
    with sqlite3.connect(database) as db:
        ids_blob, poses_blob = db.execute('SELECT opt_ids,opt_poses FROM Admin').fetchone()
        ids = np.frombuffer(zlib.decompress(ids_blob[:-12]), dtype='<i4')
        poses = np.frombuffer(zlib.decompress(poses_blob[:-12]), dtype='<f4').reshape(-1, 3, 4).copy()
        # Deliberately differs from Node.pose: exporters must use the saved optimized poses.
        poses[np.where(ids == 1)[0][0]] = [[0,-1,0,10],[1,0,0,20],[0,0,1,30]]
        db.execute('UPDATE Admin SET opt_poses=?', (archive(poses, '<f4', 5),))
        relative = np.array([[0,0,1,1],[0,1,0,2],[-1,0,0,3]], dtype='<f4').tobytes()
        db.execute('DELETE FROM Trajectory')
        db.execute('INSERT INTO Trajectory VALUES(1,1,10.5,?,?)', (relative, relative))
        geometry = db.execute('SELECT lidar_geometry FROM Node WHERE id=1').fetchone()[0]
    records = decode_gaussians(1, geometry)
    assert len(records) == 600 and records['covariance'].shape == (600,3,3)
    large_records = np.tile(records, 50)
    large_raw = b'SAPLID01' + struct.pack('<Q', len(large_records)) + large_records.tobytes()
    large_archive = archive(np.frombuffer(large_raw, dtype='u1'), 'u1', 0)
    assert len(decode_gaussians(1, large_archive)) == 30000
    forged = large_archive[:-8] + struct.pack('=ii', 16 + ((2**31 - 1 - 16) // 85) * 85, 0)
    try:
        decode_gaussians(1, forged)
    except RuntimeError:
        pass
    else:
        raise AssertionError('forged matrix length accepted')

    assert sample_gaussian(1,0,records[0],'dense',SamplingConfig()).shape[1] == 3
    assert sample_gaussian(1,1,records[1],'sparse',SamplingConfig()).shape[1] == 3
    ply = root / 'map.ply'
    tum = root / 'trajectory.tum'
    run(sys.executable, str(scripts/'build_global_map.py'), str(database), '-o', str(ply))
    run(sys.executable, str(scripts/'build_odom_tum.py'), str(database), '-o', str(tum))
    data = ply.read_bytes()
    header, vertices = data.split(b'end_header\n', 1)
    assert b'element vertex 00000000000000008400' in header or b'8400' in header
    assert len(vertices) == 8400 * 17
    assert np.allclose(struct.unpack_from('<fff', vertices), [8,20,30.2], atol=1e-5)
    trajectory = np.loadtxt(tum).reshape(-1,8)
    assert np.allclose(trajectory[0,:4], [10.5,8,21,33], atol=1e-6)
    q = trajectory[0,4:]
    assert abs(np.dot(q, [-.5,.5,.5,.5])) > 1 - 1e-6
    baseline = ply.read_bytes()
    for name, sql in [('old', 'PRAGMA user_version=13'),
                      ('corrupt', "UPDATE Node SET lidar_geometry=X'010203' WHERE id=1"),
                      ('no_solution', 'UPDATE Admin SET opt_poses=NULL'),
                      ('disconnected', 'DELETE FROM Link WHERE from_id=14 OR to_id=14')]:
        damaged = root / f'{name}.db'
        shutil.copyfile(database, damaged)
        with sqlite3.connect(damaged) as db: db.execute(sql)
        run(sys.executable, str(scripts/'build_global_map.py'), str(damaged), '-o', str(ply), fail=True)
        assert ply.read_bytes() == baseline and not ply.with_suffix('.ply.tmp').exists()
    with sqlite3.connect(database) as db: db.execute('UPDATE Trajectory SET anchor_id=9999')
    baseline_tum = tum.read_bytes()
    run(sys.executable, str(scripts/'build_odom_tum.py'), str(database), '-o', str(tum), fail=True)
    assert tum.read_bytes() == baseline_tum
print('PASS native Gaussian archive export, optimized poses, trajectory composition and corrupt input rejection')
