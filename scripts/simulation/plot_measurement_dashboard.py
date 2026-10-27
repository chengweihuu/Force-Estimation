"""Four six-axis dashboards from existing ROS logs, no simulation."""
import argparse,json,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--input-dir',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args();src=Path(a.input_dir);out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True)
rows=json.loads((src/'feedback.json').read_text());packets=json.loads((src/'measurement.json').read_text());tr=json.loads((src/'trajectory.json').read_text())
t=np.array([r['time_s']-tr['start_s'] for r in rows]);q=np.array([r['q'] for r in rows]);v=np.array([r['v'] for r in rows]);tau=np.array([r['tau'] for r in rows]);qr=np.tile(tr['knots_s_rad'][0][1],(len(t),1));vr=np.zeros_like(qr)
for (lo,qa),(hi,qb) in zip(tr['knots_s_rad'],tr['knots_s_rad'][1:]):
 mask=(t>=lo)&(t<=hi);u=(t[mask]-lo)/(hi-lo);delta=np.array(qb)-qa
 qr[mask]=qa+(10*u**3-15*u**4+6*u**5)[:,None]*delta;vr[mask]=((30*u**2-60*u**3+30*u**4)/(hi-lo))[:,None]*delta
mapping={round(r['time_s']*1e9):i for i,r in enumerate(rows)};matched=[(mapping[p['stamp_sec']*1000000000+p['stamp_nanosec']],p) for p in packets if p['stamp_sec']*1000000000+p['stamp_nanosec'] in mapping]
idx=np.array([i for i,p in matched]);current=np.array([p['current_A'] for i,p in matched]);estimate=np.array([p['drive_torque_estimate_Nm'] for i,p in matched]);mt=t[idx]

def paired_plot(tt,reference,actual,title,unit,labels,filename):
 fig,axes=plt.subplots(3,4,figsize=(16,10));sel=(tt>=0)&(tt<=9)
 for j in range(6):
  row=j//2;col=(j%2)*2;ax=axes[row,col];er=axes[row,col+1]
  ax.plot(tt[sel],reference[sel,j],color='black',ls='--',lw=1.5,label=labels[0]);ax.plot(tt[sel],actual[sel,j],color='#087eaa',lw=1.1,label=labels[1]);ax.set_title(f'J{j+1}: {unit}');ax.ticklabel_format(useOffset=False,axis='y');ax.legend(fontsize=7)
  er.plot(tt[sel],actual[sel,j]-reference[sel,j],color='#bd5325',lw=1.1);er.axhline(0,color='grey',lw=.5);er.set_title(f'J{j+1}: error ({unit})');er.ticklabel_format(useOffset=False,axis='y')
  for axis in [ax,er]:axis.grid(alpha=.25);axis.set_xlabel('Time from trajectory start (s)')
 fig.suptitle(title,fontsize=14);fig.tight_layout(rect=(0,0,1,.96));fig.savefig(out/filename,dpi=135);plt.close(fig)
paired_plot(t,qr,q,'Candidate: position tracking and error (measured - sent reference)','rad',('Sent reference','Measured'),'position.png')
paired_plot(t,vr,v,'Candidate: velocity tracking and error (measured - sent reference)','rad/s',('Sent reference','Measured'),'velocity.png')
paired_plot(mt,tau[idx],estimate,'Candidate: drive torque from equivalent current (NOT estimated external torque)','N m',('Actuator torque feedback','Current-derived drive torque'),'drive_torque.png')
fig,axes=plt.subplots(3,2,figsize=(12,9));sel=(mt>=0)&(mt<=9)
for j,ax in enumerate(axes.flat):
 ax.plot(mt[sel],current[sel,j],color='#087eaa',lw=1.1);ax.set_title(f'J{j+1} equivalent current');ax.set_ylabel('A');ax.set_xlabel('Time from trajectory start (s)');ax.grid(alpha=.25);ax.ticklabel_format(useOffset=False,axis='y')
fig.suptitle('Candidate: read-only equivalent current, K_eq = 1 N m/A; no noise');fig.tight_layout(rect=(0,0,1,.96));fig.savefig(out/'current.png',dpi=135);plt.close(fig)
core=(t>=2)&(t<=9);mc=(mt>=2)&(mt<=9);stats=[]
for j in range(6):
 e=q[core,j]-qr[core,j];ev=v[core,j]-vr[core,j];et=estimate[mc,j]-tau[idx[mc],j]
 stats.append({'joint':f'J{j+1}','position_RMSE_rad':float(np.sqrt(np.mean(e**2))),'position_bias_rad':float(np.mean(e)),'position_peak_rad':float(max(abs(e))),'velocity_RMSE_rad_s':float(np.sqrt(np.mean(ev**2))),'velocity_bias_rad_s':float(np.mean(ev)),'velocity_peak_rad_s':float(max(abs(ev))),'current_peak_A':float(max(abs(current[mc,j]))),'drive_torque_conversion_RMSE_Nm':float(np.sqrt(np.mean(et**2))),'drive_torque_conversion_peak_Nm':float(max(abs(et)))})
with (out/'six_axis_metrics.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(stats[0]));w.writeheader();w.writerows(stats)
meta={'source':str(src),'plots_interval_s':[0,9],'metrics_interval_s':[2,9],'reference':'sent quintic trajectory reconstructed, not controller internal reference','alignment':'source timestamps; each run relative to its recorded trajectory start','interpolation_of_measured_data':False,'smoothing':False,'matched_packets':len(matched),'unmatched_measurement_packets':len(packets)-len(matched),'feedback_without_measurement':len(rows)-len(set(idx)),'current':'algebraic joint-side equivalent A, K=1 Nm/A, not native electrical state','noise':False,'KF':False,'metrics':stats};(out/'plot_metadata.json').write_text(json.dumps(meta,indent=2));print(json.dumps({k:v for k,v in meta.items() if k!='metrics'},indent=2))
