from pathlib import Path
import hashlib, sqlite3, subprocess, sys, tempfile
binary,cli,config=sys.argv[1:]
def run(*args):
 p=subprocess.run(args,capture_output=True,text=True,timeout=60)
 assert p.returncode==0,(args,p.returncode,p.stdout,p.stderr)
def rows(path,table):
 with sqlite3.connect(path) as c:return c.execute('SELECT * FROM '+table).fetchall()
with tempfile.TemporaryDirectory(prefix='sapphire-automatic-') as d:
 root=Path(d);source=root/'source.db';output=root/'output.db'
 run(binary,'write',str(source));digest=hashlib.sha256(source.read_bytes()).hexdigest()
 run(cli,'relocalize',str(source),str(output),config)
 assert hashlib.sha256(source.read_bytes()).hexdigest()==digest
 for table in ('Node','Trajectory','MapFrame'):
  assert rows(source,table)==rows(output,table)
 run(binary,'verify',str(output));run(binary,'continue',str(output));run(binary,'verify4',str(output))
 run(binary,'auto',str(source));run(binary,'verify',str(source))
 run(binary,'live',str(root/'live.db'));run(binary,'verify',str(root/'live.db'))
 run(binary,'same',str(root/'same.db'))
 run(binary,'async',str(root/'async.db'))
 run(binary,'evidence',str(root/'evidence.db'))
print('PASS automatic cross-session/same-session SE3, rejection, immutable evidence, separate-process index rebuild and continuation')
