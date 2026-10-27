"""One phase of the UR5 reproduction; output and configuration are explicit."""
import argparse
from pathlib import Path
import time

from common import config, write
import collect
import evaluate
import preflight
import replay

PHASES = ('preflight', 'collect', 'replay', 'evaluate')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase', choices=PHASES, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--run-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    root = args.run_root.resolve()
    out = args.output_dir.resolve()
    if out != root / args.phase:
        parser.error('--output-dir must be <run-root>/<phase>')
    if (out / 'metrics.json').exists():
        parser.error('Phase already has results; use a new run directory')
    out.mkdir(parents=True, exist_ok=True)
    start = time.monotonic()
    try:
        c = config(args.config)
        phase_index = PHASES.index(args.phase)
        for prerequisite in PHASES[:phase_index]:
            import json
            previous = json.loads((root / prerequisite / 'metrics.json').read_text())
            if previous.get('passed') is not True:
                raise RuntimeError('Failed prerequisite: ' + prerequisite)
        if args.phase == 'preflight':
            result = preflight.run(c, out)
        elif args.phase == 'collect':
            result = collect.run(c, out)
        elif args.phase == 'replay':
            result = replay.run(c, root, out)
        else:
            result = evaluate.run(c, root, out)
        result['wall_seconds'] = time.monotonic() - start
        write(out / 'metrics.json', result)
    except Exception as exc:
        write(out / 'metrics.json', dict(passed=False, error=repr(exc),
                                       wall_seconds=time.monotonic() - start))
        raise


if __name__ == '__main__':
    main()
