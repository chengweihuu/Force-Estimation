import argparse,json,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output-dir',required=True);a=p.parse_args();out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True);start=time.monotonic()
def run(argv):subprocess.run(argv,check=True,timeout=max(1,3500-(time.monotonic()-start)))
try:
 run(['/opt/piakf-venv/bin/python','-m','venv',str(out/'venv')])
 py=str(out/'venv/bin/python')
 wheelhouse=out/'wheelhouse';wheelhouse.mkdir();cache=out/'cache';cache.mkdir()
 run([py,'-m','pip','download','--disable-pip-version-check','--only-binary=:all:','--cache-dir',str(cache),'--dest',str(wheelhouse),'--retries','2','--timeout','60','pin==3.8.0','cmeel-urdfdom==4.0.1','mujoco==3.3.7','scipy','matplotlib'])
 base=[py,'-m','pip','install','--disable-pip-version-check','--no-index','--find-links',str(wheelhouse)]
 run(base+['--dry-run','--report',str(out/'resolution.json'),'pin==3.8.0','cmeel-urdfdom==4.0.1','mujoco==3.3.7','scipy','matplotlib'])
 resolution=json.loads((out/'resolution.json').read_text())
 lock='\n'.join(sorted(x['metadata']['name']+'=='+x['metadata']['version'] for x in resolution['install']))+'\n'
 (out/'requirements.lock').write_text(lock)
 run(base+['-r',str(out/'requirements.lock')]);run([py,'-m','pip','check'])
 with (out/'packages.txt').open('w') as f:subprocess.run([py,'-m','pip','freeze'],check=True,stdout=f)
 run([py,'-c','import pinocchio as p, mujoco, scipy, numpy, matplotlib; assert p.__version__=="3.8.0"; assert mujoco.__version__=="3.3.7"; print("COMPATIBILITY",p.__version__,mujoco.__version__,numpy.__version__,scipy.__version__)'])
 (out/'metrics.json').write_text(json.dumps(dict(passed=True,wall_seconds=time.monotonic()-start)))
except Exception as e:
 (out/'metrics.json').write_text(json.dumps(dict(passed=False,error=repr(e),wall_seconds=time.monotonic()-start)));raise
