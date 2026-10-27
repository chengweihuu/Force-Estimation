"""Create a fresh workflow request; never launches a simulation or replay."""
import argparse
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent.relative_to(ROOT)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task-id', required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--preset', choices=['baseline', 'frequencies'], required=True)
    parser.add_argument('--request', type=Path, required=True)
    parser.add_argument('--image', default='piakf:h022-physical')
    args = parser.parse_args()
    if not re.fullmatch(r'H-\d+', args.task_id):
        parser.error('Expected H-<number> task ID')
    if not re.fullmatch(r'R-\d{8}T\d{6}Z-[a-z0-9-]+', args.run_id):
        parser.error('Expected UTC R-YYYYMMDDTHHMMSSZ-<short-id>')
    request = args.request.resolve()
    request.relative_to(ROOT)
    output = Path('runs') / args.task_id / args.run_id
    if request.exists() or (ROOT / output).exists():
        parser.error('Request or run already exists; choose fresh paths')
    cfg = HERE / 'configs' / (args.preset + '.json')
    inputs = [str(HERE / name) for name in [
        'acceleration.py', 'common.py', 'collect.py', 'replay.py', 'evaluate.py',
        'preflight.py', 'run.py', 'docker_step.py', 'make_request.py']]
    inputs += [str(cfg), 'scripts/DeLuca_2003_GeneralizedMomenta/observer.py',
               'scripts/DeLuca_2003_GeneralizedMomenta/external_torque.py',
               'scripts/rigid_baseline/reference_kf/oracle_kf.py',
               'scripts/simulation/ur5_helpers.py', 'scripts/models/ur5/nominal_16khz.mjb']
    steps = []
    for phase, cost, timeout, purpose in [
        ('preflight', 1, 60, 'Check model, GM/HODO equations, initialization and truth isolation'),
        ('collect', 2, 300, 'Collect configured scenes and a fresh no-force motion baseline'),
        ('replay', 2, 700, 'Replay the fixed methods and explicitly configured conditions'),
        ('evaluate', 2, 120, 'Six-axis errors, frequency response and configured controls')]:
        argv = ['python3', str(HERE / 'docker_step.py'), '--phase', phase,
                '--config', str(cfg), '--image', args.image, '--run-root', str(output),
                '--timeout', str(timeout), '--output-dir', '{output_dir}']
        steps.append(dict(id=phase, relation='prerequisite' if phase=='preflight' else 'direct',
                          purpose=purpose, costUnits=cost, timeoutMs=timeout*1000,
                          argv=argv, resultFile='metrics.json',
                          **{'pass': [dict(field='passed', op='eq', value=True)]}))
    payload = dict(version=1, taskId=args.task_id, stage='EXPLORE', hardware='local-compute',
                   environment=f'{args.image}; Python3.12.3 MuJoCo3.3.7 NumPy1.26.4; 2CPU/4GiB single-thread BLAS',
                   outputDir=str(output), budget=dict(maxCostUnits=7, maxWallTimeMs=1200000),
                   inputs=inputs, steps=steps)
    request.parent.mkdir(parents=True, exist_ok=True)
    request.write_text(json.dumps(payload, indent=2) + '\n')
    print(f'Request created: {request.relative_to(ROOT)}; no computation launched.')


if __name__ == '__main__':
    main()
