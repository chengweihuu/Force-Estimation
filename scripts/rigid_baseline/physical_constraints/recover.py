"""Read the saved solution, never invoke an optimizer; finish interrupted checks."""
import argparse,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from constrained import read,write,require,physical,design,csvwrite
p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--source-run',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args()
c=read(a.config);source=Path(a.source_run);out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True);tick=time.monotonic()
try:
    require(read(source/'precheck/metrics.json')['passed'],'Original precheck required')
    status=read(source/'solve/solver_status.json');require(status['status']=='optimal','Saved solve not optimal')
    t=np.load(source/'solve/theta.npy');require(t.shape==(66,) and np.isfinite(t).all(),'Invalid saved solution')
    np.save(out/'theta.npy',t)
    A,y=design(c);r=A@t-y
    with np.load(source/'precheck/compressed.npz') as f:
        norms=f['norms'];compressed=float(np.linalg.norm(f['R'][:,:66]@(t*f['scales'])-f['R'][:,66])**2)
    status.update(original_train_loss=float(r@r),compressed_loss_recomputed=compressed,optimizer_reexecuted=False,source_run=str(source),dual_values_available=False)
    write(out/'solver_status.json',status)
    require(abs(status['original_train_loss']-compressed)<=1e-10*max(1,status['original_train_loss']),'Saved solution objective mismatch')
    ph=physical(SimpleNamespace(njoints=7),t)
    mins=np.array([min(x['pseudo_inertia_eigenvalues']) for x in ph['bodies']])
    ph.update(lmi_margin=c['margin'],lmi_violation=np.maximum(c['margin']-mins,0),friction_violation=np.maximum(-t[60:],0))
    ph['independent_passed']=bool(ph['all_bodies_feasible'] and np.max(ph['lmi_violation'])<=1e-8 and np.min(t[60:])>=0)
    write(out/'physical_feasibility.json',ph)
    csvwrite(out/'identified_parameters.csv',[dict(index=i,name=c['labels'][i],value=float(t[i]),unit=('N m s / rad' if i>=60 else ('kg' if i%10==0 else 'kg m' if i%10<4 else 'kg m^2')),train_column_norm=float(norms[i])) for i in range(66)])
    require(ph['independent_passed'],'Independent physical feasibility failed')
    write(out/'metrics.json',dict(passed=True,optimizer_reexecuted=False,physical_passed=True,validation_used=False,test_read=False,wall_seconds=time.monotonic()-tick))
except Exception as e:
    write(out/'metrics.json',dict(passed=False,error=repr(e),wall_seconds=time.monotonic()-tick));raise
