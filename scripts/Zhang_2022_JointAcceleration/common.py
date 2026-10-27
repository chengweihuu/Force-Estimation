from pathlib import Path
import importlib.util
import sys,json,csv
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts/rigid_baseline/reference_kf'))
sys.path.insert(0,str(ROOT/'scripts/DeLuca_2003_GeneralizedMomenta'))
from oracle_kf import Model
from observer import MomentumResidual

def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
helper=load(ROOT/'scripts/simulation/ur5_helpers.py','ur5_helpers')

def write(path,data):
    def convert(x):
        if isinstance(x,np.ndarray):return x.tolist()
        if isinstance(x,np.generic):return x.item()
        if isinstance(x,Path):return str(x)
        raise TypeError(type(x).__name__)
    path.write_text(json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False,default=convert)+'\n')

def packet(path):
    with np.load(path) as f:return dict(f)

def csvwrite(path,rows):
    if not rows:return
    with path.open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def config(path):
    c=json.loads(path.read_text());c['model']=str(ROOT/c['model'])
    modes=c['replay_modes']
    assert modes and modes[0]['name']=='clean', 'Clean replay must run first'
    assert len({m['name'] for m in modes})==len(modes), 'Duplicate replay modes'
    for mode in modes:
        period=1./mode['rate_Hz']
        assert np.isclose(period/c['record_dt_s'],round(period/c['record_dt_s'])), 'Noninteger replay stride'
        assert mode['noise_std_rad_s']>=0 and (mode['seed']>=0 or mode['noise_std_rad_s']==0)
        assert set(mode.get('cases',c['cases'])).issubset(c['cases']), 'Unknown replay cases'
    return c

def core_mask(t,case):
    return (t>=.2)&(t<=2) if case=='static' else (t>=2)&(t<=8)
