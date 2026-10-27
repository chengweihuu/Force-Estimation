"""Same full-core evaluator as the baseline; new binding and comparison only."""
import argparse,sys,csv,time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'identification'))
from experiment import read,write,require,physical,csvwrite,Operator,make_model,modelpath,packet,refpath,regressor
def evaluate(cfg,out,pre,builddir,solvedir):
 require(read(solvedir/'metrics.json')['passed'],'Fit required');theta=np.load(solvedir/'theta.npy');op=Operator(cfg['old']);h=cfg['old']['physics_dt_s']
 scale=np.array(read(Path(cfg['root'])/'learn/normalization.json')['tau_scale'])
 baseline=read(cfg['pinn_checkpoint']);pinnphysical=read(cfg['pinn_physical']);comparisons=[];six=[];physicalrows=[]
 for spec in cfg['records']:
  model,_,_,_=make_model(modelpath(cfg,spec),False);data=model.createData()
  if spec==cfg['records'][0]:write(out/'physical_feasibility.json',physical(model,theta))
  q,v,dv,tau=packet(cfg['root'],spec)
  with np.load(builddir/f"design_{spec['seed']}.npz") as f:r=(f['X']@theta+f['offset']).reshape(-1,6)
  with np.load(refpath(cfg,spec)) as f:parts=f['parts']
  c=np.empty_like(q)
  for i in range(len(q)):c[i]=regressor(model,data,q[i],v[i],np.zeros(6))@theta[:60]
  ce=op(h*c-parts[:,2]);del parts,c
  indices=np.arange(48000,208000,16);Ms=np.stack([np.column_stack([regressor(model,data,q[i],np.zeros(6),np.eye(6)[j])@theta[:60] for j in range(6)]) for i in indices])
  with np.load(Path(cfg['root'])/spec['folder']/'diagnostics_1kHz.npz') as f:Mt=f['M'][3000:13000]
  vv=v[indices];p=np.einsum('bij,bj->bi',Ms,vv);pt=np.einsum('bij,bj->bi',Mt,vv);eig=np.linalg.eigvalsh(Ms);err=Ms-Mt
  ph=dict(seed=spec['seed'],matrix_relative_frobenius_percent=float(100*np.linalg.norm(err)/np.linalg.norm(Mt)),
    matrix_absolute_frobenius=float(np.linalg.norm(err)),matrix_frobenius_rms_kg_m2=float(np.sqrt(np.mean(np.sum(err**2,axis=(1,2))))),matrix_entry_rmse_kg_m2=np.sqrt(np.mean(err**2,0)),
    mass_diagonal_rmse_kg_m2=np.sqrt(np.mean((np.diagonal(Ms,axis1=1,axis2=2)-np.diagonal(Mt,axis1=1,axis2=2))**2,0)),
    momentum_relative_frobenius_percent=float(100*np.linalg.norm(p-pt)/np.linalg.norm(pt)),momentum_rmse_Nm_s=np.sqrt(np.mean((p-pt)**2,0)),
    minimum_eigenvalue=float(eig.min()),nonpositive_eigenvalue_states=int(np.sum(eig[:,0]<=0)),symmetry_max_abs=float(np.abs(Ms-Ms.transpose(0,2,1)).max()))
  physicalrows.append(ph)
  pp=next(x for x in pinnphysical if x['updates']==200 and x['seed']==spec['seed']);ps=next(x for x in baseline['scores'] if x['seed']==spec['seed'])
  with np.load(Path(cfg['pinn_components'])/f"components_{spec['seed']}.npz") as f:pce=f['EC']
  for name,residual,vel,phy in [('rigid_constrained',r,ce,ph),('PINN_update200',None,pce,pp)]:
   loss=float(np.mean((residual/scale)**2)) if residual is not None else ps['loss'];rmse=np.sqrt(np.mean(residual**2,0)) if residual is not None else np.array(ps['rmse_Nm'])
   comparisons.append(dict(seed=spec['seed'],split=spec['split'],model=name,loss=loss,matrix_relative_frobenius_percent=phy['matrix_relative_frobenius_percent'],momentum_relative_frobenius_percent=phy['momentum_relative_frobenius_percent'],minimum_mass_eigenvalue=phy['minimum_eigenvalue'],matrix_error_mean_square=float(np.sum(np.array(phy['matrix_entry_rmse_kg_m2'])**2)),matrix_reference_mean_square=float(np.mean(np.sum(Mt**2,axis=(1,2)))),momentum_error_mean_square=float(np.sum(np.array(phy['momentum_rmse_Nm_s'])**2)),momentum_reference_mean_square=float(np.mean(np.sum(pt**2,axis=1)))))
   for j in range(6):six.append(dict(seed=spec['seed'],split=spec['split'],model=name,joint=j+1,residual_rmse_Nm=rmse[j],velocity_interval_rmse_Nm=np.sqrt(np.mean(vel[:,j]**2)),mass_diagonal_rmse_kg_m2=phy['mass_diagonal_rmse_kg_m2'][j],momentum_rmse_Nm_s=phy['momentum_rmse_Nm_s'][j]))
  print('EVALUATED',spec['seed'],flush=True)
 for name in ['rigid_constrained','PINN_update200']:
  rows=[x for x in comparisons if x['split']=='train' and x['model']==name]
  comparisons.append(dict(seed='train_joint',split='train',model=name,loss=float(np.mean([x['loss'] for x in rows])),matrix_relative_frobenius_percent=float(100*np.sqrt(sum(x['matrix_error_mean_square'] for x in rows)/sum(x['matrix_reference_mean_square'] for x in rows))),momentum_relative_frobenius_percent=float(100*np.sqrt(sum(x['momentum_error_mean_square'] for x in rows)/sum(x['momentum_reference_mean_square'] for x in rows))),minimum_mass_eigenvalue=min(x['minimum_mass_eigenvalue'] for x in rows),**{key:float(np.mean([x[key] for x in rows])) for key in ['matrix_error_mean_square','matrix_reference_mean_square','momentum_error_mean_square','momentum_reference_mean_square']}))
 for name in ['rigid_constrained','PINN_update200']:
  for j in range(1,7):
   rows=[x for x in six if x['split']=='train' and x['model']==name and x['joint']==j]
   row=dict(seed='train_joint',split='train',model=name,joint=j)
   for key in ['residual_rmse_Nm','velocity_interval_rmse_Nm','mass_diagonal_rmse_kg_m2','momentum_rmse_Nm_s']:
    row[key]=float(np.sqrt(np.mean([x[key]**2 for x in rows])))
   six.append(row)
 csvwrite(out/'model_comparison.csv',comparisons);csvwrite(out/'six_axis_metrics.csv',six);write(out/'physical_metrics.json',physicalrows)
 return dict(passed=True,test_read=False,simulation_steps=0)

def compare(out,baseline):
    for name in ['model_comparison.csv','six_axis_metrics.csv']:
        with (out/name).open() as f: rows=list(csv.DictReader(f))
        with (baseline/'evaluate'/name).open() as f: rows += [r for r in csv.DictReader(f) if r['model']=='rigid_unconstrained']
        with (out/name).open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    with (out/'six_axis_metrics.csv').open() as f: rows=list(csv.DictReader(f))
    fig,axs=plt.subplots(1,3,figsize=(15,4))
    for name in ['rigid_constrained','rigid_unconstrained','PINN_update200']:
        rs=[r for r in rows if r['seed']=='20261020' and r['model']==name]
        for ax,key,title in zip(axs,['residual_rmse_Nm','velocity_interval_rmse_Nm','momentum_rmse_Nm_s'],['Residual (N m)','Velocity term (N m)','Momentum (N m s)']):
            ax.semilogy(range(1,7),[float(r[key]) for r in rs],'o-',label=name);ax.set(xlabel='Joint',ylabel='RMSE',title=title)
    axs[0].legend(fontsize=7);fig.tight_layout();fig.savefig(out/'comparison.png',dpi=150);plt.close(fig)
    ph=read(out/'physical_feasibility.json')
    fig,axs=plt.subplots(1,2,figsize=(10,4))
    axs[0].semilogy(range(1,7),[min(r['pseudo_inertia_eigenvalues']) for r in ph['bodies']],'o-');axs[0].axhline(1e-6,color='r',label='Fixed margin');axs[0].set(xlabel='Body',ylabel='Min eigenvalue (SI normalized)',title='Pseudo inertia');axs[0].legend()
    axs[1].plot(range(1,7),ph['b'],'o-');axs[1].set(xlabel='Joint',ylabel='N m s / rad',title='Identified viscous friction');fig.tight_layout();fig.savefig(out/'physical_margins.png',dpi=150);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--output-dir',required=True);p.add_argument('--run-root',required=True);a=p.parse_args()
    c=read(a.config);cfg=read(c['evaluation_config']);out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True);tick=time.monotonic()
    try:
        source=Path(c['source']);r=evaluate(cfg,out,source/'small',source/'build',Path(a.run_root)/'solve');compare(out,Path(c['baseline']));r['wall_seconds']=time.monotonic()-tick;write(out/'metrics.json',r)
    except Exception as e:
        write(out/'metrics.json',dict(passed=False,error=repr(e),wall_seconds=time.monotonic()-tick));raise
