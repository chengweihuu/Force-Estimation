"""Inspect failed SVD inputs; never decompose or fit the research matrix."""
import argparse, contextlib, csv, io, json, subprocess, sys, time
from pathlib import Path
import numpy as np
import scipy
from scipy import linalg

def write(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
def scalar(x):
    x=float(x)
    return x if np.isfinite(x) else None

def stats(a):
    finite=np.isfinite(a); nz=np.abs(a[finite & (a!=0)])
    return dict(shape=list(a.shape),dtype=str(a.dtype),strides=list(a.strides),c_contiguous=a.flags.c_contiguous,f_contiguous=a.flags.f_contiguous,
        nan=int(np.isnan(a).sum()),inf=int(np.isinf(a).sum()),zero=int(np.count_nonzero(a==0)),
        minimum=scalar(a[finite].min()) if finite.any() else None,maximum=scalar(a[finite].max()) if finite.any() else None,
        minimum_nonzero_abs=scalar(nz.min()) if len(nz) else None,maximum_abs=scalar(nz.max()) if len(nz) else None)

def probe():
    rng=np.random.default_rng(1729)
    dense=rng.normal(size=(96,12)); deficient=dense.copy();deficient[:,11]=deficient[:,0]
    examples={'diagonal':np.diag([4.,2.,1.,0.]),'dense':dense,'rank_deficient':deficient}
    rows=[]
    for name,a in examples.items():
        for order in ['C','F']:
            a=np.array(a,order=order)
            for driver in ['gesdd','gesvd','numpy']:
                try:
                    if driver=='numpy':u,s,vh=np.linalg.svd(a,full_matrices=False)
                    else:u,s,vh=linalg.svd(a,full_matrices=False,lapack_driver=driver)
                    err=float(np.linalg.norm((u*s)@vh-a)/np.linalg.norm(a))
                    rows.append(dict(matrix=name,shape=list(a.shape),order=order,driver=driver,returned=True,relative_reconstruction=err,singular_values=s.tolist()))
                except Exception as e:rows.append(dict(matrix=name,shape=list(a.shape),order=order,driver=driver,returned=False,error=repr(e)))
    buf=io.StringIO()
    with contextlib.redirect_stdout(buf):
        np.show_config();scipy.show_config()
    return dict(executable=sys.executable,numpy=np.__version__,scipy=scipy.__version__,svd_source=linalg.svd.__code__.co_filename,configuration=buf.getvalue(),probes=rows)

def main(a):
    tick=time.monotonic();out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True)
    if a.probe_only:
        write(out/a.probe_name,probe());return
    scale=np.asarray(json.loads(Path(a.normalization).read_text())['tau_scale'])
    labels=json.loads(Path(a.schema).read_text())['labels'];xs=[];ys=[];source=[]
    for path in a.design:
        with np.load(path) as f:x=f['X'];y=f['offset']
        source.append(dict(path=path,X=stats(x),offset=stats(y)));xs.append(x);ys.append(y)
    X=np.concatenate(xs);y=np.concatenate(ys)
    with np.errstate(all='ignore'):
        weight=np.tile(1/scale,len(y)//6)/np.sqrt(len(y));A=X*weight[:,None];target=-y*weight
        norms=np.linalg.norm(A,axis=0);nz=norms>0;As=A[:,nz]/norms[nz]
        rownorm=np.linalg.norm(As,axis=1);columnnorm=np.linalg.norm(As,axis=0)
    report=dict(sources=source,tau_scale=scale.tolist(),original=stats(X),weighted=stats(A),normalized=stats(As),target=stats(target),row_weights=stats(weight),column_norms=stats(norms),normalized_row_norms=stats(rownorm),normalized_column_norms=stats(columnnorm),normalized_column_norm_max_deviation_from_one=scalar(np.max(np.abs(columnnorm-1))),exact_zero_columns=[labels[j] for j in np.flatnonzero(~nz)],research_svd_executed=False,fit_executed=False,validation_read=False,test_read=False)
    write(out/'input_diagnostics.json',report)
    rows=[]
    for j,name in enumerate(labels):
        nonzero=np.abs(X[:,j][X[:,j]!=0])
        rows.append(dict(index=j,parameter=name,unweighted_column_norm=scalar(np.linalg.norm(X[:,j])),weighted_column_norm=scalar(norms[j]),unweighted_max_abs=scalar(np.max(np.abs(X[:,j]))),unweighted_min_nonzero_abs=scalar(nonzero.min()) if len(nonzero) else None,excluded_exact_zero=bool(not nz[j])))
    with (out/'column_scales.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    write(out/'rigid_library_probes.json',probe())
    # The second interpreter is already installed in the same approved image.
    subprocess.run(['/opt/piakf-venv/bin/python','-B',__file__,'--probe-only','--probe-name','main_library_probes.json','--output-dir',str(out)],check=True,timeout=max(1,105-(time.monotonic()-tick)))
    write(out/'metrics.json',dict(passed=True,semantics='diagnostic completed; individual toy SVD failures are findings, not fit approval',wall_seconds=time.monotonic()-tick,research_svd_executed=False,fit_executed=False,validation_read=False,test_read=False))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output-dir',required=True);p.add_argument('--probe-only',action='store_true');p.add_argument('--probe-name',default='library_probes.json');p.add_argument('--design',action='append');p.add_argument('--normalization');p.add_argument('--schema');a=p.parse_args()
    try:main(a)
    except Exception as e:
        target=Path(a.output_dir)/'metrics.json'
        if not target.exists():write(target,dict(passed=False,error=repr(e)))
        raise
