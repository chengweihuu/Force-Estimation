from pathlib import Path
import json,csv
import numpy as np

def packet(p):
 with np.load(p) as f:return dict(f)
def write(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def csvwrite(p,rows):
 with p.open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def evaluate(c,root,out):
 import matplotlib.pyplot as plt
 assert json.loads((root/'simulate/metrics.json').read_text())['passed']
 base=packet(root/'simulate/motion/estimates.npz')['fault_residual'];rows=[];checks=[];summary=[];closed=[]
 windows=[('all',0,6.001),('healthy',.1,2),('J2_only',2.1,2.5),('overlap',2.6,3),('J5_only_or_recovery',3.1,3.5),('recovered',3.6,6.001)]
 for case in c['cases']:
  d=packet(root/'simulate'/case/'measurements.npz');tr=packet(root/'simulate'/case/'references.npz');e=packet(root/'simulate'/case/'estimates.npz');t=d['t'];truth=tr['tau_missing'];est=e['fault_residual'];diag=e['actual_input_residual'];ref=np.zeros_like(truth);cont=ref.copy()
  for k in range(1,len(t)):
   dt=t[k]-t[k-1];ref[k]=ref[k-1]+dt*50*(truth[k-1]-ref[k-1]);cont[k]=truth[k-1]+(cont[k-1]-truth[k-1])*np.exp(-50*dt)
  # Difference between nominal and actual-input observers is exactly the filtered missing input.
  closure=est+diag-ref;ce=float(abs(closure).max());closed.append(dict(case=case,max_Nm=ce,passed=ce<1e-9));assert ce<1e-9,'nominal/actual algebra closure'
  for window,lo,hi in windows:
   sel=(t>=lo)&(t<hi)
   for label,err in [('raw_missing',est-truth),('euler_reference',est-ref),('continuous_reference',est-cont),('actual_input_zero',diag),('induced_from_healthy',est-base)]:
    for j in range(6):
     ids=np.flatnonzero(sel);pk=ids[np.argmax(abs(err[sel,j]))];x=err[sel,j]
     rows.append(dict(case=case,window=window,joint=j+1,target=label,rmse_Nm=float(np.sqrt(np.mean(x*x))),bias_Nm=float(x.mean()),peak_Nm=float(abs(err[pk,j])),peak_time_s=float(t[pk])))
   if window!='all':
    for j in range(6):
     active=bool(np.any(tr['rho'][sel,j]!=0));bias=float(np.mean((est-truth)[sel,j]));peak=float(abs((est-base)[sel,j]).max());ok=abs(bias)<=.05 and (active or peak<=.05)
     checks.append(dict(case=case,window=window,joint=j+1,fault_active=active,raw_bias_Nm=bias,nonfault_induced_peak_Nm=peak if not active else None,passed=bool(ok)))
  ever=np.any(tr['rho']!=0,axis=0);post=t>=2
  for j in np.flatnonzero(ever):
   mask=tr['rho'][:,j]!=0
   summary.append(dict(case=case,joint=int(j+1),missing_min_Nm=float(truth[mask,j].min()),missing_max_Nm=float(truth[mask,j].max()),reference_rmse_Nm=float(np.sqrt(np.mean((est[:,j]-ref[:,j])**2))),raw_missing_rmse_Nm=float(np.sqrt(np.mean((est[:,j]-truth[:,j])**2))),actual_input_rmse_Nm=float(np.sqrt(np.mean(diag[:,j]**2))),actual_input_peak_Nm=float(abs(diag[:,j]).max()),unfaulted_transient_peak_Nm=float(abs((est-base)[post][:,~ever]).max()),tracking_max_rad=json.loads((root/'simulate'/case/'checks.json').read_text())['max_tracking_rad']))
  np.savez_compressed(out/(case+'_evaluation.npz'),t=t,missing=truth,reference=ref,continuous=cont,fault_residual=est,actual_input_residual=diag,induced=est-base,closure=closure)
  fig,axs=plt.subplots(3,2,figsize=(12,8),sharex=True)
  for j,ax in enumerate(axs.flat):
   ax.plot(t,truth[:,j],'k:',label='Missing torque');ax.plot(t,ref[:,j],'--',label='Euler reference');ax.plot(t,est[:,j],label='Fault residual');ax.plot(t,-diag[:,j],label='Actual-input diagnostic (negated)');ax.set(ylabel=f'J{j+1} N m',xlabel='Time s');ax.grid(alpha=.2)
  axs[0,0].legend(fontsize=7);fig.tight_layout();fig.savefig(out/(case+'.png'),dpi=120);plt.close(fig)
 csvwrite(out/'six_axis_metrics.csv',rows);csvwrite(out/'engineering_checks.csv',checks);csvwrite(out/'summary.csv',summary);write(out/'closure_checks.json',closed)
 failed=[x for x in checks if not x['passed']];write(out/'failed_checks.json',failed)
 lines=['# 执行器部分失效：UR5 探索结果','','使用命令力矩输入，论文缺失力矩输出为原外力残差的负号。所有外力为零；实际力矩输入仅作同日志独立诊断。未设置报警阈值。','','| 场景 | 关节 | 缺失力矩范围 N·m | 离散参考 RMSE N·m | 原始缺失力矩 RMSE N·m | 实际输入诊断 RMSE N·m | 非故障轴瞬态峰值 N·m |','|---|---|---|---|---|---|---|']
 for x in summary:lines.append(f"| {x['case']} | J{x['joint']} | {x['missing_min_Nm']:.6g} 至 {x['missing_max_Nm']:.6g} | {x['reference_rmse_Nm']:.6g} | {x['raw_missing_rmse_Nm']:.6g} | {x['actual_input_rmse_Nm']:.6g} | {x['unfaulted_transient_peak_Nm']:.6g} |")
 lines+=['',f'工程检查：{len(checks)-len(failed)}/{len(checks)}；失败完整保留于 failed_checks.json。',f'两种输入残差与独立参考的代数闭合最大误差：{max(x["max_Nm"] for x in closed):.6g} N·m。','', '统计口径：表中 RMSE 为完整6秒原生1ms样本；非故障轴峰值为2秒后相对健康场景的增量，六轴原始误差另存。原始缺失力矩误差包含正常一阶滤波滞后；不将其等同于实现误差。实际输入残差与参考误差大小相同是代数关系，并非两份独立证据。','', '范围：确定性、准确模型、无外力、部分损失，J2与J5。不能推断全故障类别、报警误报/漏报、含接触或真机能力。控制器仍按故障后反馈计算命令，因此缺失力矩是时变信号。','', '既有 J6 离散偏差不重开排查；本轮不调整增益或步长。下一步只建议在独立正常工况上预先定义报警阈值，再评价当前故障的检测与隔离，不自动执行。']
 if failed:lines+=['','本轮有原定工程界限未通过；不能宣布整体验收通过。',*['- '+str(x) for x in failed]]
 (out/'decision.md').write_text('\n'.join(lines)+'\n')
 return dict(passed=not failed,failed_checks=len(failed),checks=len(checks),closure_max_Nm=max(x['max_Nm'] for x in closed),simulation_steps=0,observer_calls=0,model_evaluations=0)
