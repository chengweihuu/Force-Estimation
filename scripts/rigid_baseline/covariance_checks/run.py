"""Fixed-model development replay. All experiment bindings arrive via config."""
import argparse
import csv
import importlib.util
import json
import sys
import time
from pathlib import Path

import numpy as np


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
        json.dump(value, stream, default=convert, ensure_ascii=False, indent=2, allow_nan=False)


def require(value, message):
    if not value:
        raise ValueError(message)


def npz(path):
    with np.load(path, allow_pickle=False) as archive:
        return {k: archive[k].copy() for k in archive.files}


def csvwrite(path, rows):
    with Path(path).open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def checked(a, b, label, rtol=1e-8, atol=1e-9):
    require(a.shape == b.shape, label + ": shape")
    result = dict(label=label, passed=bool(np.allclose(a, b, rtol=rtol, atol=atol)),
                  max_abs=float(np.max(np.abs(a-b))), rtol=rtol, atol=atol)
    require(result["passed"], label + ": numerical mismatch " + str(result["max_abs"]))
    return result


def setup(config):
    api = load(config["model_api"], "models")
    # The existing replay module imports this precise, unmodified API.
    sys.modules["models"] = api
    replay = load(config["replay_api"], "common_replay")
    binding = read(config["model_binding"])
    cfg = read(binding["kf_config"])
    return api, replay, binding, cfg


def query(api, binding, cfg, feedback, kind):
    q, v = feedback["q"], feedback["v"]
    if kind == "pinn":
        model = api.Pinn(binding, cfg)
        norm = read(binding["normalization"])
        require(np.array_equal(model.net.network.input_center.numpy(), norm["q_center"]),
                "PINN normalization center differs")
        require(np.array_equal(model.net.network.input_scale.numpy(), norm["q_scale"]),
                "PINN normalization scale differs")
        chunks = [model.batch(q[i:i+256], v[i:i+256]) for i in range(0, len(q), 256)]
        result = {k: np.concatenate([x[k] for x in chunks]) for k in chunks[0]}
        return dict(t=feedback["t"], **result), None
    require(kind == "rigid", "Unknown query kind")
    model = api.Rigid(binding, cfg)
    oracle_api = load(binding["oracle_code"], "reference_model")
    oracle = oracle_api.Model(cfg["model"], cfg["derivative_epsilon_s"])
    Sv = np.diag(np.square(cfg["observer"]["assumed_velocity_std_rad_s"]))
    terms = {k: [] for k in ["M", "Mdot", "c", "b"]}
    reference = {k: [] for k in ["M", "Mdot", "c", "b", "G", "J", "R"]}
    for qi, vi in zip(q, v):
        row = model.one(qi, vi)
        for k in terms:
            terms[k].append(row[k])
        M, h, Md, J = oracle.evaluate(qi, vi)
        G = oracle.evaluate(qi, np.zeros(6))[1]
        ref = dict(M=M, Mdot=Md, c=h-G, b=oracle.B, G=G, J=J, R=M@Sv@M.T)
        for k in reference:
            reference[k].append(ref[k])
    return (dict(t=feedback["t"], **{k: np.asarray(x) for k, x in terms.items()}),
            dict(t=feedback["t"], **{k: np.asarray(x) for k, x in reference.items()}))


def model_check(terms):
    require(all(np.isfinite(x).all() for x in terms.values()), "Nonfinite model term")
    M = terms["M"]
    symmetry = checked(M, M.transpose(0, 2, 1), "Mass symmetry")
    minimum = float(np.linalg.eigvalsh(M).min())
    require(minimum > 0, "Mass not positive definite")
    require(np.min(terms["b"]) >= 0, "Negative friction")
    return dict(symmetry=symmetry, minimum_mass_eigenvalue_kg_m2=minimum)


def old_precheck(config, kind, out):
    api, replay, binding, cfg = setup(config)
    f = {k: x[:101] for k, x in npz(config["historical_feedback"]).items()}
    require(set(f) == {"t", "q", "v", "tau"}, "Historical feedback schema")
    terms, reference = query(api, binding, cfg, f, kind)
    archived = npz(config["historical_terms"][kind])
    rows = [checked(value, archived[k][:101], kind + "." + k) for k, value in terms.items()]
    checks = {kind: model_check(terms)}
    np.savez_compressed(out/"model_terms.npz", **terms)
    if reference is not None:
        archived_ref = npz(config["historical_reference"])
        rows += [checked(value, archived_ref[k][:101], "reference." + k)
                 for k, value in reference.items()]
        checks["reference"] = model_check(reference)
        np.savez_compressed(out/"reference.npz", **reference)
    versions = dict(numpy=np.__version__)
    if kind == "pinn":
        import torch
        versions["torch"] = torch.__version__
    else:
        import pinocchio
        import mujoco
        versions.update(pinocchio=pinocchio.__version__, mujoco=mujoco.__version__)
    write(out/"checks.json", dict(passed=True, archive=rows, models=checks, versions=versions))
    return dict(passed=True, kind=kind, states=101)


def feedback_csv(spec):
    expected = (["t_s"] + [f"q{i}_rad" for i in range(1, 7)] +
                [f"v{i}_rad_s" for i in range(1, 7)] + [f"tau{i}_Nm" for i in range(1, 7)])
    with Path(spec["feedback"]).open() as stream:
        require(stream.readline().strip().split(",") == expected, "Feedback CSV header")
    x = np.loadtxt(spec["feedback"], delimiter=",", skiprows=1, dtype=np.float64)
    require(x.shape == (17001, 19) and np.isfinite(x).all(), "Feedback size/finiteness")
    checked(x[:, 0], np.arange(17001)*.001, "Feedback timestamp", rtol=0, atol=1e-9)
    require(np.all(np.diff(x[:, 0]) > 0), "Nonmonotonic feedback")
    require(np.array_equal(x[0, 7:13], np.zeros(6)), "Nonzero initial velocity")
    c, checks = read(spec["collection_config"]), read(spec["collection_checks"])
    require(c["seed"] == spec["seed"] and c["split"] == spec["split"], "Seed/split mismatch")
    require(c["physics_dt_s"] == .0000625 and c["control_dt_s"] == .0005
            and c["feedback_dt_s"] == .001, "Collection rates differ")
    required = ["no_external_force", "no_moving_link_contact", "no_constraint_torque",
                "no_command_clipping", "synchronized_1kHz_feedback"]
    require(checks["passed"] and all(checks["checks"][k] for k in required),
            "Original collection acceptance failed")
    return dict(t=x[:, 0], q=x[:, 1:7], v=x[:, 7:13], tau=x[:, 13:19])


def preflight(config, root, out):
    api, replay, binding, cfg = setup(config)
    f = {k: x[:101] for k, x in npz(config["historical_feedback"]).items()}
    require(read(root/"preflight_rigid/metrics.json")["passed"], "Rigid precheck")
    require(read(root/"preflight_pinn/metrics.json")["passed"], "PINN precheck")
    ref = npz(root/"preflight_rigid/reference.npz")
    checks = []
    for kind in ["oracle", "rigid", "pinn"]:
        m = ref if kind == "oracle" else npz(root/("preflight_"+kind)/"model_terms.npz")
        result = replay.filter_run(cfg, f, m, ref)
        archive = npz(config["historical_replays"][kind])
        checks += [checked(value, archive[k][:101], kind + ".replay." + k)
                   for k, value in result.items()]
    require([s["split"] for s in config["records"]] == ["train", "train", "validation"],
            "Unexpected split schedule")
    require([s["seed"] for s in config["records"]] == config["allowed_seeds"], "Seed allowlist")
    binding_rows = []
    dest = out/"feedback"
    dest.mkdir()
    for spec in config["records"]:
        f = feedback_csv(spec)
        np.savez_compressed(dest/(spec["id"]+".npz"), **f)
        binding_rows.append(dict(**spec, feedback_fields=list(f), rows=len(f["t"]),
                                 control_Hz=2000, feedback_Hz=1000))
    write(out/"kf_config.json", cfg)
    write(out/"binding_manifest.json", dict(records=binding_rows, model_binding=binding,
          historical_feedback=config["historical_feedback"],
          historical_control_Hz=1000, observer_input="t,q,v,actual actuator torque",
          shared_priors=["reference G", "reference J", "reference M used to generate R"],
          normalization="unchanged training-only parameters", weights_updates=200,
          external_wrench_input=False, test_read=False))
    write(out/"checks.json", dict(passed=True, archive_replay=checks))
    return dict(passed=True, records=len(binding_rows), archive_models=3)


def query_records(config, root, out, kind):
    api, replay, binding, cfg = setup(config)
    checks = []
    for spec in config["records"]:
        f = npz(root/"preflight/feedback"/(spec["id"]+".npz"))
        terms, ref = query(api, binding, cfg, f, kind)
        dest = out/spec["id"]
        dest.mkdir()
        row = dict(record=spec["id"], model=model_check(terms))
        np.savez_compressed(dest/"model_terms.npz", **terms)
        if ref is not None:
            row["reference"] = model_check(ref)
            np.savez_compressed(dest/"reference.npz", **ref)
        checks.append(row)
        print("QUERIED", kind, spec["id"], len(f["t"]), flush=True)
    write(out/"checks.json", dict(passed=True, records=checks))
    return dict(passed=True, kind=kind, records=len(checks))


def core_indices(f):
    # Nominal ticks avoid accumulated simulator timestamp rounding at 3/13 s.
    require(len(f["t"]) == 17001, "Core requires complete collection")
    return np.arange(3000, 13000)


def geometry_summary(J, t, mask, scale):
    A = np.swapaxes(J, 1, 2)*scale[None, None, :]
    s = np.linalg.svd(A, compute_uv=False)
    tolerance = 6*np.finfo(np.float64).eps*s[:, :1]
    rank = np.sum(s > tolerance, axis=1)
    condition = np.full(len(s), np.inf)
    np.divide(s[:, 0], s[:, -1], out=condition, where=s[:, -1] > 0)
    ix = np.asarray(mask)
    vals = condition[ix]
    finite = vals[np.isfinite(vals)]
    quantiles = np.quantile(finite, [.0, .5, .9, .99, 1.]) if len(finite) else [None]*5
    worst = ix[int(np.argmax(vals))]
    row = dict(n=len(ix), numerical_rank_min=int(rank[ix].min()),
               fraction_numerically_rank_deficient=float(np.mean(rank[ix] < 6)),
               minimum_singular_value_Nm=float(s[ix, -1].min()),
               median_minimum_singular_value_Nm=float(np.median(s[ix, -1])),
               nonfinite_condition_count=int(np.sum(~np.isfinite(vals))),
               finite_condition_min=quantiles[0], finite_condition_median=quantiles[1],
               finite_condition_p90=quantiles[2], finite_condition_p99=quantiles[3],
               finite_condition_max=quantiles[4], worst_time_s=float(t[worst]),
               worst_condition=float(condition[worst]) if np.isfinite(condition[worst]) else None)
    return dict(t=t, J=J, singular_values_Nm=s, numerical_rank=rank,
                condition=condition, rank_tolerance=tolerance[:, 0]), row


def geometry(config, root, out):
    binding = read(config["model_binding"])
    norm = read(binding["normalization"])
    center, scale = np.array(norm["q_center"]), np.array(norm["q_scale"])
    require(np.all(scale > 0), "Nonpositive fixed input scale")
    records = []
    for spec in config["records"]:
        f = npz(root/"preflight/feedback"/(spec["id"]+".npz"))
        ref = npz(root/"rigid_terms"/spec["id"]/"reference.npz")
        records.append((spec, f, ref, core_indices(f), "core_[3,13)_nominal_ticks"))
    training = np.concatenate([f["q"][ix] for spec, f, ref, ix, window in records
                               if spec["split"] == "train"])
    qmin, qmax = training.min(0), training.max(0)
    historical = npz(config["historical_feedback"])
    hspec = dict(id="historical_motion", seed="not_a_split_seed", split="historical")
    records.append((hspec, historical, npz(config["historical_reference"]),
                    np.arange(len(historical["t"])), "full_old_motion"))
    coverage, summaries = [], []
    for spec, f, ref, ix, window in records:
        q, v = f["q"][ix], f["v"][ix]
        z = (q-center)/scale
        outside = (q < qmin) | (q > qmax)
        for j in range(6):
            coverage.append(dict(record=spec["id"], seed=spec["seed"], split=spec["split"],
                window=window, joint=j+1, q_min_rad=q[:, j].min(), q_max_rad=q[:, j].max(),
                q_rms_rad=np.sqrt(np.mean(q[:, j]**2)), v_min_rad_s=v[:, j].min(),
                v_max_rad_s=v[:, j].max(), v_rms_rad_s=np.sqrt(np.mean(v[:, j]**2)),
                normalized_q_min=z[:, j].min(), normalized_q_max=z[:, j].max(),
                train_q_min_rad=qmin[j], train_q_max_rad=qmax[j],
                outside_axis_fraction=outside[:, j].mean(),
                outside_any_axis_fraction=outside.any(axis=1).mean(), n=len(ix)))
        arrays, summary = geometry_summary(ref["J"], f["t"], ix, np.array(config["wrench_scale"]))
        np.savez_compressed(out/(spec["id"]+"_geometry.npz"), q=f["q"], **arrays)
        summaries.append(dict(record=spec["id"], window=window, **summary))
        if spec["split"] != "historical":
            _, full_summary = geometry_summary(ref["J"], f["t"], np.arange(len(f["t"])),
                                                np.array(config["wrench_scale"]))
            summaries.append(dict(record=spec["id"], window="full", **full_summary))
    csvwrite(out/"coverage.csv", coverage)
    csvwrite(out/"geometry.csv", summaries)
    write(out/"conventions.json", dict(position_normalization=norm, wrench_scale=config["wrench_scale"],
          geometry_matrix="J.T @ diag(wrench_scale); N m per unit normalized wrench",
          rank_rule="s > 6 * float64_eps * largest_singular_value",
          conditional_quantiles="finite condition values only; nonfinite count reported",
          core="nominal ticks 3000:13000; no state exclusion",
          coverage_boundary="componentwise q bounds are not joint-distribution support",
          geometry_boundary="instantaneous numerical rank is not time-varying observability"))
    return dict(passed=True, coverage_rows=len(coverage), geometry_rows=len(summaries))


def replay_records(config, root, out):
    api, replay, binding, cfg = setup(config)
    checks = []
    for spec in config["records"]:
        f = npz(root/"preflight/feedback"/(spec["id"]+".npz"))
        ref = npz(root/"rigid_terms"/spec["id"]/"reference.npz")
        dest = out/spec["id"]
        dest.mkdir()
        oracle_covariance = None
        for kind in ["oracle", "rigid", "pinn"]:
            m = ref if kind == "oracle" else npz(root/(kind+"_terms")/spec["id"]/"model_terms.npz")
            require(np.array_equal(f["t"], m["t"]) and np.array_equal(f["t"], ref["t"]),
                    "Model/feedback timestamps differ")
            result = replay.filter_run(cfg, f, m, ref)
            prefix = replay.filter_run(cfg, f, m, ref, n=101)
            prefix_error = max(float(np.max(np.abs(result[k][:101]-prefix[k]))) for k in prefix)
            require(prefix_error == 0, "Causal prefix mismatch")
            if kind == "oracle":
                oracle_covariance = result["covariance"].copy()
            covariance_error = float(np.max(np.abs(result["covariance"]-oracle_covariance)))
            require(covariance_error == 0, "Common covariance mismatch")
            symmetry = checked(result["covariance"], result["covariance"].transpose(0, 2, 1),
                               "Covariance symmetry")
            dt = np.diff(f["t"])
            balance = (np.diff(result["momentum"], axis=0) -
                       dt[:, None]*result["nominal_input"][:-1])
            np.savez_compressed(dest/(kind+".npz"), t=f["t"], **result,
                                balance_t=f["t"][1:], no_force_momentum_balance=balance)
            checks.append(dict(record=spec["id"], model=kind, prefix_max_abs=prefix_error,
                 covariance_max_abs=covariance_error, covariance_symmetry=symmetry,
                 minimum_covariance_eigenvalue=float(np.linalg.eigvalsh(result["covariance"]).min())))
            print("REPLAYED", spec["id"], kind, flush=True)
    write(out/"checks.json", dict(passed=True, checks=checks,
          shared_inputs=["feedback", "G", "J", "R", "Qp", "Qw", "P0"],
          no_external_wrench_input=True, no_sample_exclusion=True))
    return dict(passed=True, main_sequences=len(checks))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--phase", required=True)
    parser.add_argument("--run-root", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    config, root, out = read(args.config), Path(args.run_root), Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    start = time.monotonic()
    try:
        if args.phase in ["preflight_rigid", "preflight_pinn"]:
            result = old_precheck(config, args.phase.split("_")[1], out)
        elif args.phase == "preflight":
            result = preflight(config, root, out)
        elif args.phase in ["rigid_terms", "pinn_terms"]:
            result = query_records(config, root, out, args.phase.split("_")[0])
        elif args.phase == "geometry":
            result = geometry(config, root, out)
        elif args.phase == "replay":
            result = replay_records(config, root, out)
        else:
            raise ValueError("Unknown phase " + args.phase)
        result.update(wall_seconds=time.monotonic()-start, test_read=False,
                      simulation_steps=0, training_updates=0,
                      pass_semantics="engineering completion, not scientific acceptance")
        write(out/"metrics.json", result)
    except Exception as exc:
        write(out/"metrics.json", dict(passed=False, error=repr(exc),
              wall_seconds=time.monotonic()-start))
        raise


if __name__ == "__main__":
    main()
