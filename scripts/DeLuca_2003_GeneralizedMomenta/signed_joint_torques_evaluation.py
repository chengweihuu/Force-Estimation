import json,csv
import numpy as np
import matplotlib.pyplot as plt
def packet(p):
 with np.load(p) as f:return {k:f[k] for k in f.files}
def csvwrite(p,rs):
 with p.open('w') as f:w=csv.DictWriter(f,fieldnames=list(rs[0]));w.writeheader();w.writerows(rs)
def evaluate(c,root,out):
 assert json.loads((root/'simulate/metrics.json').read_text())['passed'];rows=[];accept=[];times=[];cross=[];symmetry=[];series={};baseline=packet(root/'simulate/motion/estimates.npz')['residual']
 for case,cc in c['cases'].items():
  d=packet(root/'simulate'/case/'measurements.npz');truth=packet(root/'simulate'/case/'references.npz')['tau_ext'];e=packet(root/'simulate'/case/'estimates.npz');t=d['t'];est=e['residual'];delta=est-baseline;series[case]=delta;ref=np.zeros_like(truth);continuous=np.zeros_like(truth);gain=np.array(c['gain_s_inv']);injected=cc.get('joint',-1)
  for k in range(1,len(t)):
   h=t[k]-t[k-1];ref[k]=ref[k-1]+h*gain*(truth[k-1]-ref[k-1]);continuous[k]=truth[k-1]+(continuous[k-1]-truth[k-1])*np.exp(-gain*h)
  for label,target in [('raw',truth),('euler_reference',ref),('continuous_reference',continuous)]:
   err=est-target
   for j in range(6):
    k=np.argmax(abs(err[:,j]));rows.append(dict(case=case,joint=j+1,target=label,rmse_Nm=float(np.sqrt(np.mean(err[:,j]**2))),bias_Nm=float(err[:,j].mean()),peak_Nm=float(abs(err[k,j])),peak_time_s=float(t[k])))
  for label,lo,hi in [('initial',.1,2.),('loaded',2.1,3.),('removed',3.1,6.001)]:
   mask=(t>=lo)&(t<hi)
   for j in range(6):
    er=est[mask,j]-truth[mask,j];active=bool(np.any(abs(truth[mask,j])>0));unforced=float(abs(delta[mask,j]).max()) if not active else None;bias=float(er.mean());accept.append(dict(case=case,window=label,joint=j+1,active=active,bias_Nm=bias,peak_error_Nm=float(abs(er).max()),unforced_increment_peak_Nm=unforced,role='recovery' if j==injected and label=='removed' else 'injected' if j==injected else 'uninj',passed=bool(abs(bias)<=.05 and (unforced is None or unforced<=.05))))
  if injected>=0:
   for j in range(6):
    mask=t>=2;val=delta[mask,j];k=np.argmax(abs(val));cross.append(dict(case=case,output_joint=j+1,is_main=j==injected,rmse_Nm=float(np.sqrt(np.mean(val**2))),signed_peak_Nm=float(val[k]),peak_Nm=float(abs(val[k])),peak_time_s=float(t[mask][k])))
   amp=float(cc['sign']);z=est[:,injected]/amp;row=dict(case=case,joint=injected+1,amplitude_Nm=amp)
   for frac in [.1,.9,.95]:
    ids=np.flatnonzero((t>=2)&(t<3)&(z>=frac));k=int(ids[0]) if len(ids) else None;row[f't{int(frac*100)}_ms']=float((t[k]-2)*1000) if k is not None else None;row[f'previous{int(frac*100)}_ms']=float((t[k-1]-2)*1000) if k is not None else None
   row['rise10_90_ms']=row['t90_ms']-row['t10_ms'] if row['t90_ms'] is not None and row['t10_ms'] is not None else None
   ids=np.flatnonzero((t>=2)&(t<3));k=ids[np.argmax(z[ids])];row['overshoot_pct']=float((z[k]-1)*100);row['peak_time_s']=float(t[k]);ids=np.flatnonzero((t>=3)&(abs(est[:,injected])<=.05));k=int(ids[0]) if len(ids) else None;row['removal_first5pct_ms']=float((t[k]-3)*1000) if k is not None else None;times.append(row)
  np.savez_compressed(out/(case+'_references.npz'),t=t,raw=truth,euler=ref,continuous=continuous,residual=est,delta=delta)
  fig,axs=plt.subplots(3,2,figsize=(11,8),sharex=True)
  for j,ax in enumerate(axs.flat):
   ax.plot(t,truth[:,j],'k:',label='raw');ax.plot(t,ref[:,j],'--',label='Euler');ax.plot(t,est[:,j],label='residual');ax.set(xlabel='Time s',ylabel=f'J{j+1} N m');ax.grid(alpha=.2)
  axs[0,0].legend(fontsize=7);fig.tight_layout();fig.savefig(out/(case+'.png'),dpi=100);plt.close(fig)
 for joint in range(1,7):
  vals=series[f'J{joint}+']+series[f'J{joint}-']
  for j in range(6):symmetry.append(dict(input_joint=joint,output_joint=j+1,sum_rmse_Nm=float(np.sqrt(np.mean(vals[:,j]**2))),sum_peak_Nm=float(abs(vals[:,j]).max())))
 for file,rr in [('six_joint_errors.csv',rows),('engineering_checks.csv',accept),('response_times.csv',times),('channel_matrix.csv',cross),('sign_symmetry.csv',symmetry)]:csvwrite(out/file,rr)
 return dict(passed=all(r['passed'] for r in accept),failed_checks=sum(not r['passed'] for r in accept),checked_items=len(accept),observer_calls=0,simulation_steps=0)
