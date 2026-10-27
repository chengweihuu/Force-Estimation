"""Docker transport only; no scientific defaults or dependency installation."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase', required=True)
    parser.add_argument('--run-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--timeout', type=int, required=True)
    parser.add_argument('--image', required=True)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    out = args.output_dir.resolve()
    inside = Path('/workspace') / out.relative_to(root)
    config = Path('/workspace') / args.config.resolve().relative_to(root)
    run_root = Path('/workspace') / args.run_root.resolve().relative_to(root)
    if args.timeout <= 5:
        parser.error('Timeout must exceed five seconds')
    out.mkdir(parents=True, exist_ok=True)
    name = 'zhang-acceleration-' + str(os.getpid())
    start = time.monotonic()
    command = ['docker', 'run', '--rm', '--name', name, '--network', 'none',
               '--cpus', '2', '--memory', '4g', '--user', f'{os.getuid()}:{os.getgid()}',
               '-e', 'OPENBLAS_NUM_THREADS=1', '-e', 'OMP_NUM_THREADS=1',
               '-e', 'MKL_NUM_THREADS=1', '-e', 'MPLBACKEND=Agg',
               '-e', 'MPLCONFIGDIR=/tmp/mpl', '-v', f'{root}:/workspace:ro',
               '-v', f'{out}:{inside}:rw', '-w', '/workspace',
               '--entrypoint', '/opt/piakf-venv/bin/python', args.image, '-B',
               'scripts/Zhang_2022_JointAcceleration/run.py', '--phase', args.phase,
               '--config', str(config), '--run-root', str(run_root),
               '--output-dir', str(inside)]
    try:
        subprocess.run(command, check=True, timeout=args.timeout - 5)
    finally:
        subprocess.run(['docker', 'rm', '-f', name], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, timeout=3)
        (out / 'carrier.json').write_text(json.dumps(dict(
            wall_seconds=time.monotonic() - start, argv=command, robot_runs=0), indent=2) + '\n')


if __name__ == '__main__':
    main()
