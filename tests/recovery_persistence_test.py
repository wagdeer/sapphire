"""Recovery archives independent segments; another process reopens and continues."""
from pathlib import Path
import sqlite3, subprocess, sys, tempfile
binary, config = sys.argv[1:]
with tempfile.TemporaryDirectory(prefix='sapphire-recovery-') as directory:
    root=Path(directory)
    subprocess.run([binary,config,directory],check=True,timeout=30)
    paths=list(root.glob('*.db'))
    assert paths==[root/'original.db']
    path=paths[0]
    def check(count):
        with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as c:
            assert c.execute('PRAGMA quick_check').fetchone()==('ok',)
            nodes=c.execute('SELECT id,map_id,stamp FROM Node ORDER BY id').fetchall()
            assert nodes[:4]==[(1,1,10.),(2,2,1.),(3,3,20.),(4,4,3.)],nodes
            assert len(nodes)==count
            trajectory=c.execute('SELECT stamp FROM Trajectory ORDER BY sequence').fetchall()
            assert trajectory==[(s,) for s in ([10.,11.,1.,2.,20.,21.,3.,4.]+([30.] if count==5 else []))],trajectory
            assert c.execute('SELECT count(*) FROM Link WHERE from_id>0 AND to_id>0').fetchone()==(0,)
    check(4)
    subprocess.run([binary,config,str(path),'continue'],check=True,timeout=20)
    check(5)
print('PASS shared archive, four isolated origins, sealed unmatched resume fragment, cross-process continue')
