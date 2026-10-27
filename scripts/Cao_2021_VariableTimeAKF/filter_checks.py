"""Independent finite-sum and state-machine checks; no physical simulation/KF calls."""
import math
import numpy as np
from msma import CurrentFilter, process, wma_weights, hma_weights, linear_series


def run_checks(cfg):
    checks = {}
    def match(name, actual, expected):
        if not np.allclose(actual, expected, rtol=cfg['rtol'], atol=cfg['atol']):
            raise RuntimeError(f"Failed: {name}")
        checks[name] = float(np.max(np.abs(np.asarray(actual)-np.asarray(expected))))
    def condition(name, value):
        if not bool(value):
            raise RuntimeError(f"Failed: {name}")
        checks[name] = True
    def direct_wma(x,k,n):
        return sum((n-j)*x[k-j] for j in range(n)) / (n*(n+1)/2)
    def direct_hma(x,k,n):
        outer=math.isqrt(n)
        return sum((outer-j)*(2*direct_wma(x,k-j,n//2)-direct_wma(x,k-j,n))
                   for j in range(outer)) / (outer*(outer+1)/2)
    n, h = cfg['window'], cfg['history_count']
    rng=np.random.default_rng(8510)
    x=rng.standard_normal((h+160,6))
    for name,signal in [('random',x),('constant',np.ones_like(x)*3),
                        ('ramp',np.arange(len(x))[:,None]*np.arange(1,7)[None,:]/100),
                        ('impulse',np.eye(len(x),1,k=-h)*np.ones((1,6)))]:
        for method in ['wma','hma']:
            actual=process(signal,method,n,np.ones(6),h)['current']
            fn=direct_wma if method=='wma' else direct_hma
            expected=np.asarray([fn(signal,k,n) for k in range(h,len(signal))])
            match(name+'_'+method+'_nested_sum',actual,expected)
            kernel=wma_weights(n) if method=='wma' else hma_weights(n)
            match(name+'_'+method+'_equivalent_kernel',actual,linear_series(signal,kernel)[h:])
    match('WMA1_is_identity',wma_weights(1),[1.0])
    match('WMA3_hand_example',wma_weights(3)@np.array([3,2,1]),14/6)
    match('HMA_constant_gain',hma_weights(n).sum(),1)
    condition('HMA_has_negative_coefficients',np.min(hma_weights(n))<0)

    filt=CurrentFilter('msma',4,np.ones(6))
    filt.prime(np.zeros((8,6)))
    states=[]
    for k in range(5):
        states.append(filt.update(np.zeros(6)))
    condition('growth_four_then_HMA',all(np.all(s['mode']==1) for s in states[:4]) and np.all(states[4]['mode']==2))
    condition('growth_has_no_threshold_check',all(not s['threshold_checked'].any() for s in states[:4]))
    state=filt.update(np.ones(6))
    condition('threshold_equality_uses_HMA',np.all(state['mode']==2) and not state['reset'].any())
    value=state['current'].copy(); value[0]+=2
    state=filt.update(value)
    condition('mature_reset_current_frame',state['reset'][0] and state['length'][0]==1 and state['current'][0]==value[0])
    condition('reset_independent_axes',not state['reset'][1:].any() and np.all(state['mode'][1:]==2))
    value[0]+=20
    state=filt.update(value)
    condition('growth_spike_does_not_reset',state['length'][0]==2 and not state['reset'][0] and not state['threshold_checked'][0])
    condition('reset_keeps_HMA_cache',len(filt.intermediate)==filt.outer and len(filt.raw)==filt.N)

    # Re-enter HMA after a reset with thresholds large enough to avoid a second reset.
    filt=CurrentFilter('msma',4,np.full(6,100.0)); history=np.zeros((8,6)); filt.prime(history)
    seen=list(history)
    for k in range(11):
        value=np.full(6,200.0 if k>=5 else 0.0); seen.append(value)
        state=filt.update(value)
        if k==5: condition('reset_w1',np.all(state['reset']) and np.all(state['length']==1))
        if k==9:
            condition('post_reset_HMA_reentry',np.all(state['mode']==2))
            match('post_reset_full_HMA_history',state['current'],direct_hma(np.asarray(seen),len(seen)-1,4))
    threshold=np.full(6,.25)
    full=process(x,'msma',n,threshold,h)
    reference=np.asarray([direct_hma(x,k,n) for k in range(h,len(x))])
    selected=full['mode']==2
    if selected.any():match('MSMA_HMA_uses_full_history',full['current'][selected],reference[selected])
    prefix=process(x[:h+83],'msma',n,threshold,h)
    changed=x.copy(); changed[h+83:]+=100
    alternate=process(changed,'msma',n,threshold,h)
    for key in full:
        condition('exact_causal_prefix_'+key,np.array_equal(prefix[key],full[key][:83]) and np.array_equal(alternate[key][:83],full[key][:83]))
    rejected=0
    for action in [lambda: CurrentFilter('msma',n,np.zeros(6)),
                   lambda: CurrentFilter('msma',n,np.ones(5)),
                   lambda: CurrentFilter('msma',1,np.ones(6)),
                   lambda: CurrentFilter('msma',n,np.ones(6)).update(np.ones(6)),
                   lambda: CurrentFilter('msma',n,np.ones(6)).prime(np.zeros((3,6)))]:
        try: action()
        except ValueError: rejected+=1
        else: raise RuntimeError('Invalid filter input accepted')
    checks['invalid_inputs_rejected']=rejected
    return dict(passed=True,checks=checks,KF_calls=0,physics_steps=0)
