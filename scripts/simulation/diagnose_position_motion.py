"""Offline sampled-control diagnostic; no MuJoCo imports or trajectory integration."""
import argparse,json,csv,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--feedback',required=True);p.add_argument('--trajectory',required=True);p.add_argument('--config-dir',required=True);p.add_argument('--model',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args();out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True)
rows=json.loads(Path(a.feedback).read_text());tr=json.loads(Path(a.trajectory).read_text());cfg=Path(a.config_dir);design=json.loads((cfg/'design.json').read_text());names=design['joint_names'];g=yaml.safe_load((cfg/'pids.yaml').read_text())['/**']['ros__parameters']['pid_gains']['position'];kp=np.array([g[n]['p'] for n in names]);kd=np.array([g[n]['d'] for n in names]);assert all(g[n]['i']==0 for n in names)
root=ET.parse(a.model);B=np.array([float(root.find(f'.//joint[@name="{n}"]').get('damping')) for n in names]);t=np.array([r['time_s']-tr['start_s'] for r in rows]);q=np.array([r['q'] for r in rows]);v=np.array([r['v'] for r in rows]);tau=np.array([r['tau'] for r in rows]);qr=np.tile(tr['knots_s_rad'][0][1],(len(t),1));vr=np.zeros_like(qr)
for k,x in enumerate(t):
 for (ta,qa),(tb,qb) in zip(tr['knots_s_rad'],tr['knots_s_rad'][1:]):
  if ta<=x<=tb:
   u=(x-ta)/(tb-ta);delta=np.array(qb)-qa;qr[k]=np.array(qa)+(10*u**3-15*u**4+6*u**5)*delta;vr[k]=(30*u**2-60*u**3+30*u**4)/(tb-ta)*delta;break
P=(qr-q)*kp;D=(vr-v)*kd;drag=-v*B;approx=P+D;summary=[]
for i,n in enumerate(names):
 sel=(t>=2+i)&(t<=3+i);ind=np.flatnonzero(sel);ip=ind[np.argmax(qr[sel,i])];it=ind[np.argmax(abs(tau[sel,i]))]
 summary.append({'joint':n,'Kp_Nm_rad':kp[i],'Kd_Nms_rad':kd[i],'B_Nms_rad':B[i],'B_over_Kd':B[i]/kd[i],'B_over_Kp_s':B[i]/kp[i],'reference_speed_peak_rad_s':float(max(abs(vr[sel,i]))),'actual_speed_peak_rad_s':float(max(abs(v[sel,i]))),'tau_peak_Nm':float(max(abs(tau[sel,i]))),'drag_peak_Nm':float(max(abs(drag[sel,i]))),'approx_PID_rmse_Nm':float(np.sqrt(np.mean((approx[sel,i]-tau[sel,i])**2))),'approx_PID_max_error_Nm':float(max(abs(approx[sel,i]-tau[sel,i]))),'tau_minus_Bv_rmse_Nm':float(np.sqrt(np.mean((tau[sel,i]-B[i]*v[sel,i])**2))),'reference_peak_sample':{'relative_time_s':float(t[ip]),'q_ref_rad':float(qr[ip,i]),'q_rad':float(q[ip,i]),'v_ref_rad_s':float(vr[ip,i]),'v_rad_s':float(v[ip,i]),'P_Nm':float(P[ip,i]),'D_approx_Nm':float(D[ip,i]),'tau_Nm':float(tau[ip,i]),'drag_Nm':float(drag[ip,i])},'torque_peak_sample':{'relative_time_s':float(t[it]),'P_Nm':float(P[it,i]),'D_approx_Nm':float(D[it,i]),'tau_Nm':float(tau[it,i]),'drag_Nm':float(drag[it,i])}})
with (out/'components.csv').open('w') as f:
 w=csv.writer(f);w.writerow(['relative_time_s','joint','qref_rad','q_rad','vref_rad_s','v_rad_s','P_Nm','D_continuous_approx_Nm','tau_measured_Nm','passive_damping_Nm'])
 for k,x in enumerate(t):
  for i in range(6):w.writerow([x,i+1,qr[k,i],q[k,i],vr[k,i],v[k,i],P[k,i],D[k,i],tau[k,i],drag[k,i]])
fig,axes=plt.subplots(3,1,figsize=(9,8),sharex=True);sel=(t>=6.8)&(t<=9)
axes[0].plot(t[sel],qr[sel,5],label='sent reference');axes[0].plot(t[sel],q[sel,5],label='measured');axes[0].set_ylabel('J6 rad')
axes[1].plot(t[sel],vr[sel,5],label='reference velocity');axes[1].plot(t[sel],v[sel,5],label='measured velocity');axes[1].set_ylabel('rad/s')
for values,label in [(tau,'measured drive'),(P,'P reconstructed'),(D,'D continuous approximation'),(drag,'passive damping')]:axes[2].plot(t[sel],1000*values[sel,5],label=label)
axes[2].set_ylabel('mN m');axes[2].set_xlabel('s after trajectory start')
for ax in axes:ax.grid(True);ax.legend()
fig.tight_layout();fig.savefig(out/'j6_components.png',dpi=130);plt.close(fig)
result={'passed':bool(np.isfinite(approx).all()),'physics_steps':0,'joints':summary,'limitations':'sent reference reconstructed, not logged controller reference; derivative is continuous approximation, not 2kHz internal finite difference; no identification or independent causal isolation'};(out/'metrics.json').write_text(json.dumps(result,indent=2));print(json.dumps(summary[5],indent=2))
