"""Capture the unchanged observer's first failure; diagnostic computations only."""
import argparse
import contextlib
import csv
import importlib.util
import io
import json
import sys
import time
import traceback
from decimal import Decimal, localcontext
from pathlib import Path

import numpy as np
import scipy
import scipy.linalg


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    def convert(x):
        if isinstance(x, np.ndarray):
            return x.tolist()
        if isinstance(x, np.generic):
            return x.item()
        raise TypeError(type(x).__name__)
    with Path(path).open("x") as stream:
        json.dump(value, stream, default=convert, indent=2, ensure_ascii=False, allow_nan=False)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def npz(path):
    with np.load(path, allow_pickle=False) as source:
        return {k: source[k].copy() for k in source.files}


def decimal_ldl(a):
    """Unpivoted LDL of represented symmetric entries, using 80 decimal digits."""
    with localcontext() as ctx:
        ctx.prec = 80
        n = len(a)
        A = [[Decimal.from_float(float(x)) for x in row] for row in a]
        L = [[Decimal(int(i == j)) for j in range(n)] for i in range(n)]
        d = []
        for j in range(n):
            pivot = A[j][j] - sum((L[j][k]*L[j][k]*d[k] for k in range(j)), Decimal(0))
            d.append(pivot)
            if pivot == 0:
                return dict(completed=False, zero_pivot_index=j, pivots=[str(x) for x in d])
            for i in range(j+1, n):
                L[i][j] = (A[i][j] - sum((L[i][k]*L[j][k]*d[k] for k in range(j)),
                                        Decimal(0)))/pivot
        residual = max(abs(A[i][j]-sum((L[i][k]*d[k]*L[j][k] for k in range(n)), Decimal(0)))
                       for i in range(n) for j in range(n))
        return dict(completed=True, all_pivots_positive=all(x > 0 for x in d),
                    pivots=[str(x) for x in d], max_reconstruction_abs=str(residual),
                    precision_decimal_digits=80, input="exact conversion of float64 entries")


def matrix_diagnostics(name, a):
    require(np.isfinite(a).all() and a.dtype == np.float64, name+": not finite float64")
    symmetric = (a+a.T)/2
    original_symmetry = float(np.max(np.abs(a-a.T)))
    eig_l = np.linalg.eigvalsh(a, UPLO="L")
    eig_u = np.linalg.eigvalsh(a, UPLO="U")
    s = np.linalg.svd(a, compute_uv=False)
    row = dict(name=name, shape=list(a.shape), dtype=str(a.dtype),
               max_abs=float(np.max(np.abs(a))), symmetry_max_abs=original_symmetry,
               diagonal_min=float(np.diag(a).min()), diagonal_max=float(np.diag(a).max()),
               eigenvalues_numpy_lower=eig_l, eigenvalues_numpy_upper=eig_u,
               min_eigenvalue_over_eps_norm=float(eig_l[0]/(np.finfo(float).eps*s[0])),
               singular_values=s, condition_svd=float(s[0]/s[-1]) if s[-1] > 0 else None,
               raw_units="mixed for full P; coordinate-scale-dependent eigenvalues/condition")
    try:
        chol = np.linalg.cholesky(a)
        row.update(cholesky_success=True, cholesky_min_diagonal=float(np.diag(chol).min()),
                   cholesky_reconstruction_relative=float(np.linalg.norm(chol@chol.T-a)/np.linalg.norm(a)))
    except np.linalg.LinAlgError as exc:
        row.update(cholesky_success=False, cholesky_error=str(exc))
    row["scipy_eigenvalues"] = {driver: scipy.linalg.eigh(a, eigvals_only=True, driver=driver)
                              for driver in ["ev", "evd", "evr", "evx"]}
    row["decimal_ldl_symmetrized_diagnostic"] = decimal_ldl(symmetric)
    # Congruence is used only to inspect scale; it does not replace any filter matrix.
    if np.all(np.diag(symmetric) > 0):
        scale = np.sqrt(np.diag(symmetric))
        normalized = symmetric/scale[:, None]/scale[None, :]
        row["unit_diagonal_diagnostic"] = dict(
            eigenvalues=np.linalg.eigvalsh(normalized),
            scale=scale, condition_svd=float(np.linalg.cond(normalized)),
            equation="D^-1 sym(A) D^-1, D=diag(sqrt(diag(sym(A)))); diagnostic only")
    return row


def diagnose(args, out):
    cfg = read(args.kf_config)
    f, ref = npz(args.feedback), npz(args.reference)
    require(set(f) == {"t", "q", "v", "tau"} and len(f["t"]) == 17001, "Feedback boundary")
    require(np.array_equal(f["t"], ref["t"]), "Reference times differ")
    model_api = load(args.model_api, "models")
    sys.modules["models"] = model_api
    replay = load(args.replay_api, "fixed_replay")
    kernel_path = str(Path(cfg["kf_source"]).resolve())
    record = {}
    calls = 0

    def on_kernel(frame, event, value):
        if event == "return":
            record["locals"] = {k: v.copy() if isinstance(v, np.ndarray) else v
                                for k, v in frame.f_locals.items()}
            record["returned"] = tuple(x.copy() for x in value)
        return on_kernel

    def trace_call(frame, event, value):
        nonlocal calls
        if (event == "call" and frame.f_code.co_name == "kf_step"
                and str(Path(frame.f_code.co_filename).resolve()) == kernel_path):
            calls += 1
            record.clear()
            record["inputs"] = {k: v.copy() if isinstance(v, np.ndarray) else v
                                for k, v in frame.f_locals.items()}
            return on_kernel
        return None

    failure = None
    sys.settrace(trace_call)
    try:
        replay.filter_run(cfg, f, ref, ref)
    except ValueError as exc:
        require(str(exc) == "Nonpositive covariance", "Unexpected failure: "+repr(exc))
        tb = exc.__traceback__
        while tb:
            if tb.tb_frame.f_code is replay.filter_run.__code__:
                failure = tb.tb_frame.f_locals.copy()
            tb = tb.tb_next
        failure_text = traceback.format_exc()
    finally:
        sys.settrace(None)
    require(failure is not None, "Original failure was not reproduced")
    i = int(failure["i"])
    P = failure["P"].copy()
    P0 = np.zeros((12, 12))
    P0[:6, :6] = ref["R"][0]
    P0[6:, 6:] = np.diag(cfg["observer"]["initial_wrench_variance"])
    arrays = dict(index=np.array(i), t=np.array(f["t"][i]), q=f["q"][i], v=f["v"][i],
                  tau=f["tau"][i], M=ref["M"][i], J=ref["J"][i], R=ref["R"][i],
                  P_initial=P0, P_failure=P, x_failure=failure["x"].copy())
    matrices = dict(P_initial=P0, R_initial=ref["R"][0], M_failure=ref["M"][i],
                    R_failure=ref["R"][i], P_failure=P)
    checks = {}
    if i > 0:
        require(calls == i and "returned" in record, "Trace/call index mismatch")
        last, before = record["locals"], record["inputs"]
        require(np.array_equal(record["returned"][1], P), "Captured covariance differs")
        require(np.array_equal(failure["out"]["covariance"][-1], before["P"]),
                "Previous covariance differs")
        require(np.array_equal(failure["out"]["covariance"][0], P0), "Initialization differs")
        for prefix, values in [("last_input", before), ("last_local", last)]:
            for key, value in values.items():
                if isinstance(value, np.ndarray) or isinstance(value, (int, float)):
                    arrays[prefix+"_"+key] = np.asarray(value)
        matrices.update(P_previous=before["P"], P_prediction=last["Pp"], Q_discrete=last["Q"],
                        S_innovation=last["S"], P_Joseph_unsymmetrized=last["P"])
        require(np.array_equal((last["P"]+last["P"].T)/2, P), "Symmetrization differs")
        rhs = last["Pp"][:, :6].T
        residual = last["S"]@last["K"].T-rhs
        checks.update(solve_absolute_residual=float(np.linalg.norm(residual)),
                      solve_backward_relative=float(np.linalg.norm(residual)/
                          (np.linalg.norm(last["S"])*np.linalg.norm(last["K"].T)+np.linalg.norm(rhs))),
                      captured_covariance_bitwise_equal=True,
                      previous_covariance_bitwise_equal=True, initial_covariance_bitwise_equal=True,
                      symmetrization_bitwise_equal=True)
    else:
        require(calls == 0 and np.array_equal(P, P0), "Initial failure mismatch")
        checks["initial_covariance_bitwise_equal"] = True
    prefix = {k: np.asarray(v) for k, v in failure["out"].items()}
    np.savez_compressed(out/"failure_snapshot.npz", **arrays)
    np.savez_compressed(out/"successful_prefix.npz", t=f["t"][:i], **prefix)
    (out/"original_exception.txt").write_text(failure_text)
    rows = [matrix_diagnostics(name, matrix) for name, matrix in matrices.items()]
    summary = dict(failure_index_zero_based=i, failure_time_s=float(f["t"][i]),
        successful_samples=i, kernel_updates_including_failure=calls,
        original_exception="Nonpositive covariance",
        original_rule="np.linalg.eigvalsh(P).min() > 0 (unchanged)",
        original_rule_passed=bool(np.linalg.eigvalsh(P).min() > 0),
        no_samples_processed_after_failure=True, checks=checks,
        versions=dict(python=sys.version, numpy=np.__version__, scipy=scipy.__version__),
        numeric_precision=dict(dtype="float64", eps=np.finfo(float).eps,
            decimal="80 digits for diagnostic LDL only; not used in estimator"),
        units=dict(state="p[6] in N m s, F[3] in N, moment[3] in N m",
            P="pairwise products of state units; whole-matrix raw eigenvalues depend on units",
            R_S="(N m s)^2", time="s"),
        diagnostic_boundary="Cholesky, Decimal LDL, alternate eigensolvers and congruence do not replace acceptance",
        matrices=rows)
    write(out/"diagnostic.json", summary)
    with (out/"matrix_comparison.csv").open("x", newline="") as stream:
        fields = ["matrix", "numpy_lower_min", "numpy_upper_min", "scipy_ev_min", "scipy_evd_min",
                  "scipy_evr_min", "scipy_evx_min", "cholesky_success", "cholesky_relative_error",
                  "decimal_ldl_all_positive", "unit_diagonal_min", "symmetry_max_abs",
                  "max_abs", "diagonal_min", "diagonal_max", "condition_svd",
                  "min_eigenvalue_over_eps_norm"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(dict(matrix=row["name"],
                numpy_lower_min=row["eigenvalues_numpy_lower"][0],
                numpy_upper_min=row["eigenvalues_numpy_upper"][0],
                **{"scipy_"+k+"_min": v[0] for k, v in row["scipy_eigenvalues"].items()},
                cholesky_success=row["cholesky_success"],
                cholesky_relative_error=row.get("cholesky_reconstruction_relative"),
                decimal_ldl_all_positive=row["decimal_ldl_symmetrized_diagnostic"].get("all_pivots_positive"),
                unit_diagonal_min=row.get("unit_diagonal_diagnostic", {}).get("eigenvalues", [None])[0],
                **{k: row[k] for k in ["symmetry_max_abs", "max_abs", "diagonal_min", "diagonal_max",
                                     "condition_svd", "min_eigenvalue_over_eps_norm"]}))
    info = io.StringIO()
    with contextlib.redirect_stdout(info):
        np.show_config()
    (out/"numpy_build.txt").write_text(info.getvalue())
    return dict(passed=True, failure_reproduced=True, failure_index=i, failure_time_s=float(f["t"][i]),
        original_filter_acceptance_passed=False, test_read=False, simulation_steps=0, training_updates=0,
        pass_semantics="failure capture completed; original observer remains failed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    for field in ["kf-config", "feedback", "reference", "model-api", "replay-api", "output-dir"]:
        parser.add_argument("--"+field, required=True)
    args = parser.parse_args()
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    start = time.monotonic()
    try:
        result = diagnose(args, out)
        result["wall_seconds"] = time.monotonic()-start
        write(out/"metrics.json", result)
    except Exception as exc:
        write(out/"metrics.json", dict(passed=False, error=repr(exc), wall_seconds=time.monotonic()-start))
        raise
