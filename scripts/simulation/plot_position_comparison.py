"""Plot existing motion logs only; no simulation or parameter changes."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--baseline',required=True);p.add_argument('--candidate',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args();out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True)
def load(path):
 path=Path(path);rows=json.loads((path/'feedback.json').read_text());tr=json.loads((path/'trajectory.json').read_text());return np.array([r['time_s']-tr['start_s'] for r in rows]),np.array([r['q'] for r in rows]),np.array([r['v'] for r in rows]),np.array([r['tau'] for r in rows]),tr
bt,bq,bv,bu,tr=load(a.baseline);ct,cq,cv,cu,_=load(a.candidate)
t=np.linspace(6.8,9,1101);q=np.zeros(len(t));v=q.copy()
for lo,hi,sign,start in [(7,7.5,1,0),(7.5,8,-1,.02)]:
 sel=(t>=lo)&(t<=hi);s=(t[sel]-lo)/(hi-lo);q[sel]=start+sign*.02*(10*s**3-15*s**4+6*s**5);v[sel]=sign*.02*(30*s**2-60*s**3+30*s**4)/(hi-lo)
fig,ax=plt.subplots(3,1,figsize=(10,9),sharex=True)
ax[0].plot(t,q,'k--',lw=2,label='Command');ax[1].plot(t,v,'k--',lw=2,label='Command')
for tt,qq,vv,uu,color,label in [(bt,bq,bv,bu,'#cf6b25','Original'),(ct,cq,cv,cu,'#087eaa','Damping-aware Kp candidate')]:
 sel=(tt>=6.8)&(tt<=9)
 for axis,y in zip(ax,[qq[:,5],vv[:,5],1000*uu[:,5]]):axis.plot(tt[sel],y[sel],color=color,lw=1.7,label=label)
for axis,label in zip(ax,['J6 position (rad)','J6 velocity (rad/s)','J6 drive torque (mN m)']):axis.set_ylabel(label);axis.grid(alpha=.3);axis.legend(loc='best')
ax[2].set_xlabel('Time relative to each trajectory start (s)');ax[0].set_title('Measured J6 response: original vs candidate (existing logs)')
fig.tight_layout();fig.savefig(out/'j6_actual_comparison.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True)
def ref(tt):
 result=np.tile(tr['knots_s_rad'][0][1],(len(tt),1))
 for (lo,qa),(hi,qb) in zip(tr['knots_s_rad'],tr['knots_s_rad'][1:]):
  sel=(tt>=lo)&(tt<=hi);u=(tt[sel]-lo)/(hi-lo);result[sel]=np.array(qa)+(10*u**3-15*u**4+6*u**5)[:,None]*(np.array(qb)-qa)
 return result
for tt,qq,color,label in [(bt,bq,'#cf6b25','Original'),(ct,cq,'#087eaa','Candidate')]:
 sel=(tt>=2)&(tt<=9);err=qq-ref(tt)
 for i,axis in enumerate(axes.flat):axis.plot(tt[sel],1000*err[sel,i],color=color,label=label);axis.set_title('J'+str(i+1));axis.grid(alpha=.3);axis.set_ylabel('Position error (mrad)')
for axis in axes[-1]:axis.set_xlabel('Time relative to trajectory start (s)')
axes[0,0].legend();fig.suptitle('All six axes: measured position minus sent reference');fig.tight_layout();fig.savefig(out/'six_axis_error_comparison.png',dpi=150);plt.close(fig)
print('Saved two plots from existing logs; no integration.')
