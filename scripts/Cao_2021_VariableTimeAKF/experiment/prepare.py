"""Bind the fixed comparison to a fresh task/run; does not execute science."""
import argparse
import datetime
import json
import re
import secrets
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPOSITORY = HERE.parents[2]


def prepare(task_id, run_id=None):
    if not re.fullmatch(r'H-\d{3,}', task_id):
        raise ValueError('task-id must be a project H-task ID, for example H-101')
    if run_id is None:
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        run_id = f'R-{stamp}-{secrets.token_hex(2)}'
    if not re.fullmatch(r'R-\d{8}T\d{6}Z-[a-z0-9]+', run_id):
        raise ValueError('run-id must use R-YYYYMMDDTHHMMSSZ-shortid')
    run_root = Path('runs') / task_id / run_id
    binding = Path('scratch') / task_id / f'cao-{run_id}'
    # Exclusive paths: preserve completed runs, requests, and partial preparations.
    if (REPOSITORY / run_root).exists() or (REPOSITORY / binding).exists():
        raise FileExistsError('Run or binding already exists; supply a fresh run-id')
    cfg = json.loads((HERE.parent / 'configs/core_comparison.json').read_text())
    container_root = '/workspace/' + run_root.as_posix()
    cfg.update(run_root=container_root, source_run=container_root,
               table_dir=container_root + '/calibrate')
    config_path = binding / 'config.json'
    template = (HERE / 'request.template.json').read_text()
    for token, value in {
        '__TASK_ID__': task_id,
        '__RUN_ROOT__': run_root.as_posix(),
        '__CONFIG_PATH__': config_path.as_posix(),
        '__CONTAINER_CONFIG_PATH__': '/workspace/' + config_path.as_posix(),
    }.items():
        template = template.replace(token, value)
    request = json.loads(template)
    if '__' in template:
        raise ValueError('Unresolved request template field')
    for relative in request['inputs']:
        if relative != config_path.as_posix() and not (REPOSITORY / relative).is_file():
            raise FileNotFoundError(relative)
    destination = REPOSITORY / binding
    destination.mkdir(parents=True, exist_ok=False)
    for name, value in [('config.json', cfg), ('request.json', request)]:
        with (destination / name).open('x') as handle:
            json.dump(value, handle, indent=2, ensure_ascii=False)
            handle.write('\n')
    return {'config': config_path.as_posix(),
            'request': (binding / 'request.json').as_posix(),
            'output': run_root.as_posix(),
            'execution_started': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task-id', required=True)
    parser.add_argument('--run-id', help='Optional fresh UTC Run ID; generated if omitted')
    args = parser.parse_args()
    print(json.dumps(prepare(args.task_id, args.run_id), indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
