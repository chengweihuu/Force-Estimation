"""Replace only the covariance guard; reuse fixed replay and reporting code."""
import argparse
import inspect
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import run as core


class CovarianceGuard:
    def __init__(self):
        self.rows = []
        self.segments = []

    def __call__(self, P):
        core.require(np.isfinite(P).all(), "Nonfinite covariance")
        core.require(np.allclose(P, P.T, rtol=1e-8, atol=1e-9), "Asymmetric covariance")
        # No symmetrization, jitter, clipping or replacement of P.
        L = np.linalg.cholesky(P)
        self.rows.append((float(np.linalg.eigvalsh(P).min()),
                          float(np.diag(L).min()), float(np.max(np.abs(P-P.T)))))

    def save(self, out):
        a = np.asarray(self.rows, dtype=float).reshape(-1, 3)
        np.savez_compressed(out/"guard_trace.npz", minimum_eigenvalue=a[:, 0],
                            minimum_cholesky_diagonal=a[:, 1], symmetry_max_abs=a[:, 2])
        core.write(out/"guard_trace.json", dict(
            segments=self.segments, checked_matrices=len(a),
            nonpositive_raw_eigenvalue_count=int(np.sum(a[:, 0] <= 0)),
            minimum_raw_eigenvalue=float(a[:, 0].min()) if len(a) else None,
            minimum_cholesky_diagonal=float(a[:, 1].min()) if len(a) else None,
            rule="finite, symmetry rtol=1e-8 atol=1e-9, Cholesky succeeds on original P",
            eigenvalues="diagnostic only; no independent sign gate",
            filter_matrix_modified=False))


def guarded_setup(config):
    api, original, binding, cfg = core.setup(config)
    source = inspect.getsource(original.filter_run)
    old = "require(np.linalg.eigvalsh(P).min()>0,'Nonpositive covariance')"
    new = "covariance_guard(P)"
    core.require(source.count(old) == 1, "Covariance guard source changed")
    modified = source.replace(old, new)
    core.require(modified.replace(new, old) == source, "Unexpected source difference")
    guard = CovarianceGuard()
    namespace = dict(original.__dict__, covariance_guard=guard)
    exec(compile(modified, str(Path(__file__))+"::filter_guard_only", "exec"), namespace)
    function = namespace["filter_run"]

    def wrapped(*args, **kwargs):
        start = len(guard.rows)
        result = function(*args, **kwargs)
        guard.segments.append(dict(invocation=len(guard.segments), start=start,
                                   stop=len(guard.rows), samples=len(result["wrench"])))
        return result

    replay = SimpleNamespace(filter_run=wrapped)
    return api, original, replay, binding, cfg, guard, dict(
        replaced_expression=old, replacement=new, source_changes=1,
        arithmetic_unchanged=True, kernel_source=cfg["kf_source"])


def preflight(config, source, failure_path, out):
    api, original, replay, binding, cfg, guard, diff = guarded_setup(config)
    core.require(cfg == core.read(source/"preflight/kf_config.json"), "KF configuration changed")
    failed = core.npz(failure_path)["P_failure"]
    tiny_negative = np.eye(12)
    tiny_negative[-1, -1] = -1e-20
    singular = np.eye(12)
    singular[-1, -1] = 0
    asymmetric = np.eye(12)
    asymmetric[0, 1] = .01
    nonfinite = np.eye(12)
    nonfinite[0, 0] = np.nan
    examples = [("saved_positive_matrix_with_eigenvalue_sign_error", failed, True),
                ("identity_spd", np.eye(12), True),
                ("negative_diagonal_1e_20", tiny_negative, False),
                ("singular", singular, False), ("asymmetric", asymmetric, False),
                ("nonfinite", nonfinite, False)]
    tests = []
    for name, matrix, expected in examples:
        before = matrix.copy()
        try:
            guard(matrix)
            accepted, error = True, None
        except (ValueError, np.linalg.LinAlgError) as exc:
            accepted, error = False, repr(exc)
        unchanged = np.array_equal(before, matrix, equal_nan=True)
        tests.append(dict(name=name, expected_accept=expected, accepted=accepted,
                          input_unchanged=unchanged, error=error))
        core.require(accepted == expected and unchanged, "Guard regression failed: "+name)
    core.require(np.linalg.eigvalsh(failed).min() <= 0, "Original guard failure not reproduced")
    f = {k: v[:101] for k, v in core.npz(config["historical_feedback"]).items()}
    ref = {k: v[:101] for k, v in core.npz(config["historical_reference"]).items()}
    comparisons = []
    for kind in ["oracle", "rigid", "pinn"]:
        m = ref if kind == "oracle" else {
            k: v[:101] for k, v in core.npz(config["historical_terms"][kind]).items()}
        old = original.filter_run(cfg, f, m, ref)
        new = replay.filter_run(cfg, f, m, ref)
        archived = core.npz(config["historical_replays"][kind])
        for key in old:
            core.require(np.array_equal(old[key], new[key]), "Arithmetic changed: "+kind+"."+key)
            comparison = core.checked(new[key], archived[key][:101], kind+"."+key)
            comparisons.append(dict(**comparison, old_new_bitwise_equal=True))
    # The known failure frame and all preceding frames must remain identical too.
    first = config["records"][0]
    core.require(first["split"] == "train", "Failure prefix must use first training record")
    f = core.npz(source/"preflight/feedback"/(first["id"]+".npz"))
    ref = core.npz(source/"rigid_terms"/first["id"]/"reference.npz")
    saved = core.npz(failure_path)
    count = int(saved["index"])+1
    result = replay.filter_run(cfg, f, ref, ref, n=count)
    prior_prefix = core.npz(Path(failure_path).with_name("successful_prefix.npz"))
    for key in prior_prefix:
        if key != "t":
            core.require(np.array_equal(result[key][:-1], prior_prefix[key]),
                         "Recorded prefix changed: "+key)
    core.require(np.array_equal(result["covariance"][-1], saved["P_failure"]),
                 "Recorded failed covariance changed")
    core.require(np.array_equal(result["wrench"][-1], saved["x_failure"][6:]),
                 "Recorded failed wrench changed")
    core.write(out/"checks.json", dict(passed=True, source_diff=diff, guard_cases=tests,
        historical_comparisons=comparisons, failure_prefix_bitwise_equal=True,
        failure_state_bitwise_equal=True, failure_prefix_samples=count))
    guard.save(out)
    return dict(passed=True, historical_models=3, guard_cases=len(tests),
                failure_prefix_bitwise_equal=True)


def replay_records(config, source, root, out):
    core.require(core.read(root/"preflight/metrics.json")["passed"], "Preflight incomplete")
    api, original, replay, binding, cfg, guard, diff = guarded_setup(config)
    previous_setup = core.setup
    try:
        core.setup = lambda _: (api, replay, binding, cfg)
        result = core.replay_records(config, source, out)
    finally:
        core.setup = previous_setup
        guard.save(out)
    labels = []
    for spec in config["records"]:
        for model in ["oracle", "rigid", "pinn"]:
            for size in ["full", "prefix101"]:
                labels.append(dict(record=spec["id"], model=model, window=size))
    core.require(len(labels) == len(guard.segments), "Trace segment count differs")
    core.write(out/"guard_sequences.json", [dict(**label, **segment)
               for label, segment in zip(labels, guard.segments)])
    return result


class RunView:
    """Explicit read-only binding to old successful inputs and the new replay."""
    def __init__(self, source, root):
        self.source, self.root = source, root

    def __truediv__(self, key):
        first = Path(key).parts[0]
        if first == "replay":
            return self.root/key
        core.require(first in {"preflight", "rigid_terms", "pinn_terms", "geometry"},
                     "Unexpected report source: "+str(key))
        return self.source/key


def evaluate(config, source, root, out):
    import report
    core.write(out/"source_map.json", dict(source_root=str(source), replay_root=str(root),
        reused=["preflight/feedback", "rigid_terms", "pinn_terms", "geometry"],
        report_implementation=str(Path(report.__file__)),
        new_model_queries=0, no_sample_exclusion=True))
    return report.summarize(config, RunView(source, root), out)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    for name in ["phase", "config", "source-root", "run-root", "failure-snapshot", "output-dir"]:
        parser.add_argument("--"+name, required=True)
    args = parser.parse_args()
    config = core.read(args.config)
    source, root, out = Path(args.source_root), Path(args.run_root), Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    start = time.monotonic()
    try:
        if args.phase == "preflight":
            result = preflight(config, source, args.failure_snapshot, out)
        elif args.phase == "replay":
            result = replay_records(config, source, root, out)
        elif args.phase == "evaluate":
            result = evaluate(config, source, root, out)
        else:
            raise ValueError("Unknown phase "+args.phase)
        result.update(wall_seconds=time.monotonic()-start, test_read=False, training_updates=0,
                      simulation_steps=0, pass_semantics="engineering completion only")
        core.write(out/"metrics.json", result)
    except Exception as exc:
        core.write(out/"metrics.json", dict(passed=False, error=repr(exc),
                   wall_seconds=time.monotonic()-start))
        raise
