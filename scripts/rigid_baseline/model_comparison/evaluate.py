import argparse,csv,time
from pathlib import Path
import numpy as np
from models import read,write,feedback
from replay import terms

def csvwrite(path,rows):
 keys=list(dict.fromkeys(k for r in rows for k in r))
 with path.open('x',newline='') as f:w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
def run(c,root,out):
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 cfg=read(c['kf_config']);six=[];responses=[];diag=[];allresult={}
 for case in cfg['cases']:
  truth=terms(Path(c['logs'])/case/'references.npz');f=feedback(c,case);t=f['t'];steady=t>=(.2 if case in ['static','motion'] else 2.);ref=terms(root/'rigid_terms'/case/'reference.npz')
  fig,axs=plt.subplots(2,3,figsize=(13,7),sharex=True)
  for j,ax in enumerate(axs.flat):ax.plot(t,truth['wrench'][:,j],color='black',ls='--',label='Applied');ax.set(xlabel='Time (s)',ylabel=['Fx (N)','Fy (N)','Fz (N)','Mx (N m)','My (N m)','Mz (N m)'][j]);ax.grid(alpha=.2)
  allresult[case]={}
  for name in ['oracle','rigid','pinn']:
   est=terms(root/'replay'/case/(name+'.npz'));error=est['wrench']-truth['wrench'];joint=est['joint_external_torque']-truth['tau_ext']
   r=dict(rmse_full=np.sqrt(np.mean(error**2,0)),rmse_steady=np.sqrt(np.mean(error[steady]**2,0)),bias_steady=error[steady].mean(0),peak_error_full=np.abs(error).max(0),peak_error_steady=np.abs(error[steady]).max(0),joint_torque_rmse=np.sqrt(np.mean(joint**2,0)))
   allresult[case][name]=r
   for j in range(6):
    six.append(dict(case=case,model=name,axis=['Fx','Fy','Fz','Mx','My','Mz'][j],unit='N' if j<3 else 'N m',**{k:float(val[j]) for k,val in r.items()},joint_index=j+1));axs.flat[j].plot(t,est['wrench'][:,j],label=name,alpha=.8)
   if case=='sine':
    w=2*np.pi*cfg['force']['frequency_Hz'];X=np.column_stack([np.sin(w*(t[steady]-1)),np.cos(w*(t[steady]-1)),np.ones(steady.sum())]);a,b,offset=np.linalg.lstsq(X,est['wrench'][steady,2],rcond=None)[0];phase=np.arctan2(b,a)
    responses.append(dict(case=case,model=name,amplitude_N=np.hypot(a,b),phase_deg=np.degrees(phase),equivalent_delay_ms=-phase/w*1000,offset_N=offset))
   if case=='step':
    rr=dict(case=case,model=name,steady_mean_N=est['wrench'][steady,2].mean(),peak_after_onset_N=est['wrench'][t>=1,2].max(),pre_onset_Fz_N=est['wrench'][np.flatnonzero(t<1)[-1],2])
    for frac in [.1,.9]:
     hits=np.flatnonzero((t>=1)&(est['wrench'][:,2]>=3*frac));rr[f't{int(frac*100)}_ms']=float((t[hits[0]]-1)*1000) if len(hits) else None;rr[f'pre_onset_above_{int(frac*100)}']=bool(rr['pre_onset_Fz_N']>=3*frac)
    responses.append(rr)
   m=ref if name=='oracle' else terms(root/('rigid_terms' if name=='rigid' else 'pinn_terms')/case/'model_terms.npz')
   mv=np.einsum('bij,bj->bi',m['M'],f['v']);pv=np.einsum('bij,bj->bi',ref['M'],f['v']);mdv=np.einsum('bij,bj->bi',m['Mdot'],f['v']);mdref=np.einsum('bij,bj->bi',ref['Mdot'],f['v'])
   uref=mdref+f['tau']-ref['b']*f['v']-ref['c']-ref['G']
   dm=m['M']-ref['M'];mr=100*np.linalg.norm(dm)/np.linalg.norm(ref['M']);den=np.linalg.norm(pv);pr=100*np.linalg.norm(mv-pv)/den if den else None
   for j in range(6):diag.append(dict(case=case,model=name,joint=j+1,M_relative_percent=mr,momentum_relative_percent=pr,M_diagonal_rmse_kg_m2=np.sqrt(np.mean(dm[:,j,j]**2)),momentum_rmse_Nm_s=np.sqrt(np.mean((mv[:,j]-pv[:,j])**2)),Mdot_v_rmse_Nm=np.sqrt(np.mean((mdv[:,j]-mdref[:,j])**2)),c_rmse_Nm=np.sqrt(np.mean((m['c'][:,j]-ref['c'][:,j])**2)),u_rmse_Nm=np.sqrt(np.mean((est['nominal_input'][:,j]-uref[:,j])**2))))
  axs.flat[0].legend();fig.suptitle(case+' — common covariance, reference gravity');fig.tight_layout();fig.savefig(out/(case+'.png'),dpi=150);plt.close(fig)
 csvwrite(out/'six_axis_errors.csv',six);csvwrite(out/'response_metrics.csv',responses);csvwrite(out/'model_term_diagnostics.csv',diag);write(out/'evaluation.json',allresult)
 return dict(passed=True,test_read=False,simulation_steps=0)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--run-root',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args();out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True);tick=time.monotonic()
 try:r=run(read(a.config),Path(a.run_root),out);r['wall_seconds']=time.monotonic()-tick;write(out/'metrics.json',r)
 except Exception as e:write(out/'metrics.json',dict(passed=False,error=repr(e),wall_seconds=time.monotonic()-tick));raise
