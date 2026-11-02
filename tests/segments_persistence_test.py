from pathlib import Path
import hashlib, sqlite3, struct, subprocess, sys, tempfile, shutil
fixture, cli, config=sys.argv[1:]
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from build_global_map import assemble
from build_odom_tum import export
def run(*args,code=0):
    p=subprocess.run(list(args),capture_output=True,text=True,timeout=30)
    assert p.returncode==code,(args,p.returncode,p.stdout,p.stderr)
    return p.stdout
with tempfile.TemporaryDirectory(prefix='sapphire-segments-') as name:
    root=Path(name);source=root/'source.db'
    run(fixture,'write',str(source))
    run(fixture,'reject',str(source))
    def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()
    before=digest(source)
    for segment in (1,2,3):
        assert assemble(source,root/f'{segment}.ply',False,segment=segment)==(1200,2)
        assert export(source,root/f'{segment}.tum',segment=segment)==(2,2)
    try: assemble(source,root/'invalid.ply',False)
    except RuntimeError: pass
    else: raise AssertionError('Disconnected origins overlaid')
    def rows(p,table):
        with sqlite3.connect(p) as c:return c.execute('SELECT * FROM '+table).fetchall()
    listing=run(cli,'list',str(source));assert 'segment,nodes' in listing
    for ref,query in [('1','3'),('3','1')]:
        output=root/(ref+'_'+query+'.db')
        # Small finite seed. Registration must recover full asymmetric SE(3),
        # regardless of which side contains the older persistent datum.
        run(cli,'join',str(source),str(output),config,ref,query,'0','0','0','0','0','0','1')
        assert digest(source)==before
        run(fixture,'verify',str(output))
        assert assemble(output,root/'component.ply',False,component_node=1)==(2400,4)
        assert assemble(output,root/'third.ply',False,component_node=5)==(1200,2)
        assert export(output,root/'component.tum',component_node=1)==(4,4)
        assert rows(source,'Node')==rows(output,'Node')
        assert rows(source,'Trajectory')==rows(output,'Trajectory')
        saved=digest(output)
        run(cli,'join',str(source),str(output),config,ref,query,'0','0','0','0','0','0','1',code=1)
        assert digest(output)==saved
        run(fixture,'continue',str(output))
        with sqlite3.connect(output) as c:
            assert c.execute('SELECT MAX(id),COUNT(*) FROM Node').fetchone()==(7,7)
        assert digest(source)==before
    live=root/'live_join.db';shutil.copyfile(source,live)
    run(fixture,'join',str(live))
    run(fixture,'verify',str(live))
    rejected=root/'rejected.db'
    run(cli,'join',str(source),str(rejected),config,'1','3','100','0','0','0','0','0','1',code=3)
    assert not rejected.exists() and digest(source)==before
    assert not list(root.glob('*.partial_*'))
print('PASS geometric rejection, asymmetric bidirectional joins, original bytes preserved, fresh-process continue')
