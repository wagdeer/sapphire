"""Check module dependency direction and source-root-qualified own includes."""
from pathlib import Path
import re
import sys

root = Path(sys.argv[1]).resolve()
modules = {'frontend', 'backend', 'common', 'tools'}
allowed = {'tools': {'tools'}, 'common': {'common', 'tools'},
           'frontend': {'frontend', 'common', 'tools'}, 'backend': {'backend', 'common', 'tools'}}
assert {p.name for p in root.iterdir() if p.is_dir()} == modules
assert {p.name for p in (root / 'frontend').iterdir()} == {'initialize', 'eskf', 'ricp', 'common'}
assert {p.name for p in (root / 'backend/visual').iterdir() if p.is_dir()} == {'feature', 'faiss', 'utils', 'solver'}
assert not (root / 'common/camera.hpp').exists()
assert not (root / 'common/camera.cpp').exists()
headers = {p.name for p in root.rglob('*') if p.suffix in ('.hpp', '.h')}
errors = []
for module in sorted(modules):
    for path in sorted((root / module).rglob('*')):
        if path.suffix not in ('.cpp', '.hpp', '.h'):
            continue
        for include in re.findall(r'^\s*#\s*include\s*["<]([^">]+)', path.read_text(), re.M):
            owner = include.split('/')[0]
            if path.is_relative_to(root / 'frontend/common') and include.startswith(('frontend/eskf/', 'frontend/ricp/', 'frontend/initialize/')):
                errors.append(f'{path}: shared frontend code must not depend on an estimator: {include}')
            if owner in modules:
                if owner not in allowed[module]:
                    errors.append(f'{path}: forbidden {module} -> {owner}: {include}')
                if not (root / include).is_file():
                    errors.append(f'{path}: missing header {include}')
            elif include == 'pipeline.hpp' or include.startswith(('mapping/', 'odometry/')):
                errors.append(f'{path}: forbidden composition/old-path include {include}')
            elif module == 'tools' and include == 'parameters.h':
                errors.append(f'{path}: tools must not depend on system configuration')
            elif include in headers and include != 'parameters.h':
                errors.append(f'{path}: own header must be qualified: {include}')
if errors:
    raise SystemExit('\n'.join(errors))
print('PASS: tools is independent; common uses tools; frontend/backend independently use common/tools.')
