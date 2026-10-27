"""Report fixed-model diagnostics without changing or selecting a model."""
import argparse
import time
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from run import read, write, npz, csvwrite, require, core_indices


def matvec(M, v):
    return np.einsum("nij,nj->ni", M, v)


def relative(x, reference):
    denominator = np.linalg.norm(reference)
    return float(100*np.linalg.norm(x-reference)/denominator) if denominator > 0 else None


def axis_rows(metadata, quantity, error, t, units):
    rows = []
    require(error.shape == (len(t), 6) and np.isfinite(error).all(), "Metric shape/finiteness")
    for j in range(6):
        x = error[:, j]
        peak = int(np.argmax(np.abs(x)))
        rows.append(dict(**metadata, quantity=quantity, axis=j+1, unit=units[j],
            rmse=np.sqrt(np.mean(x*x)), bias=x.mean(), peak_abs=np.abs(x[peak]),
            peak_signed=x[peak], peak_time_s=t[peak], n=len(t)))
    return rows


def summarize(config, root, out):
    require(read(root/"replay/metrics.json")["main_sequences"] == 9, "Incomplete replay")
    axis, matrices = [], []
    plot_data = {}
    specs = config["records"] + [dict(id="historical_motion", split="historical")]
    for spec in specs:
        key = spec["id"]
        old = spec["split"] == "historical"
        if old:
            f = npz(config["historical_feedback"])
            ref = npz(config["historical_reference"])
            models = {k: npz(v) for k, v in config["historical_terms"].items()}
            results = {k: npz(v) for k, v in config["historical_replays"].items()}
            windows = {"old_[0.2,end]": np.flatnonzero(f["t"] >= .2)}
        else:
            f = npz(root/"preflight/feedback"/(key+".npz"))
            ref = npz(root/"rigid_terms"/key/"reference.npz")
            models = {k: npz(root/(k+"_terms")/key/"model_terms.npz") for k in ["rigid", "pinn"]}
            results = {k: npz(root/"replay"/key/(k+".npz")) for k in ["oracle", "rigid", "pinn"]}
            windows = {"core_[3,13)": core_indices(f), "full": np.arange(len(f["t"]))}
            plot_data[key] = (f, results, spec["split"])
        models["oracle"] = ref
        v, t = f["v"], f["t"]
        p_ref = matvec(ref["M"], v)
        mdv_ref = matvec(ref["Mdot"], v)
        u_ref = mdv_ref + f["tau"] - ref["b"]*v - ref["c"] - ref["G"]
        for kind in ["oracle", "rigid", "pinn"]:
            m, r = models[kind], results[kind]
            require(np.array_equal(t, m["t"]) and np.array_equal(t, r["t"]), "Metric time mismatch")
            p = matvec(m["M"], v)
            mdv = matvec(m["Mdot"], v)
            u = mdv + f["tau"] - m["b"]*v - m["c"] - ref["G"]
            balance = np.diff(p, axis=0) - np.diff(t)[:, None]*u[:-1]
            for window, ix in windows.items():
                meta = dict(record=key, split=spec["split"], model=kind, window=window)
                errors = {
                    "mass_diagonal_error": (np.diagonal(m["M"]-ref["M"], axis1=1, axis2=2), ["kg m2"]*6),
                    "momentum_error": (p-p_ref, ["N m s"]*6),
                    "velocity_term_error": (m["c"]-ref["c"], ["N m"]*6),
                    "Mdot_v_error": (mdv-mdv_ref, ["N m"]*6),
                    "friction_torque_error": ((m["b"]-ref["b"])*v, ["N m"]*6),
                    "nominal_input_error": (u-u_ref, ["N m"]*6),
                    "false_wrench": (r["wrench"], ["N"]*3+["N m"]*3),
                    "false_joint_torque": (r["joint_external_torque"], ["N m"]*6),
                }
                for quantity, (error, units) in errors.items():
                    axis += axis_rows(meta, quantity, error[ix], t[ix], units)
                # A balance is assigned to its right endpoint. No core restart.
                right = ix[ix > 0]
                axis += axis_rows(meta, "one_step_momentum_balance", balance[right-1], t[right],
                                  ["N m s"]*6)
                axis += axis_rows(meta, "one_step_balance_over_dt",
                                  balance[right-1]/np.diff(t)[right-1, None], t[right], ["N m"]*6)
                dM = m["M"][ix]-ref["M"][ix]
                matrices.append(dict(**meta, n=len(ix),
                    mass_relative_Frobenius_percent=relative(m["M"][ix], ref["M"][ix]),
                    mass_Frobenius_RMS_kg_m2=np.sqrt(np.mean(np.sum(dM*dM, axis=(1, 2)))),
                    momentum_relative_percent=relative(p[ix], p_ref[ix]),
                    mass_min_eigenvalue_kg_m2=np.linalg.eigvalsh(m["M"][ix]).min()))
    csvwrite(out/"six_axis_metrics.csv", axis)
    csvwrite(out/"matrix_momentum_metrics.csv", matrices)
    write(out/"metric_definitions.json", dict(
        no_force_reference="zero external wrench and zero external joint torque",
        model_reference="MuJoCo at identical q/v; reference G and J, reference-M-derived R shared",
        primary_window="nominal feedback indices 3000:13000; KF starts at index 0",
        balance="p[k]-p[k-1] - (t[k]-t[k-1])*u[k-1]; assigned to t[k]",
        balance_over_dt="one-step momentum balance divided by dt; NOT 16 kHz interval training loss",
        relative="100 * norm(prediction-reference) / norm(reference), all selected samples and entries",
        old_window="archived samples with t>=0.2; different pose and controller rate; not causal intervention",
        snapshot_selection="fixed updates=200; no new fitting or selection",
        pass_semantics="engineering completion only"))
    colors = dict(oracle="#222222", rigid="#2274a5", pinn="#d1495b")
    labels = ["Fx (N)", "Fy (N)", "Fz (N)", "Mx (N m)", "My (N m)", "Mz (N m)"]
    for key, (f, results, split) in plot_data.items():
        # Each model has separate axes to preserve visibility at very different scales.
        fig, axs = plt.subplots(3, 6, figsize=(19, 9), sharex=True)
        for row, kind in enumerate(["oracle", "rigid", "pinn"]):
            for j in range(6):
                a = axs[row, j]
                a.plot(f["t"], results[kind]["wrench"][:, j], color=colors[kind], linewidth=.6)
                a.axvspan(3, 13, color="grey", alpha=.08)
                a.set_title(kind+" "+labels[j], fontsize=9)
                a.grid(alpha=.2)
                if row == 2:
                    a.set_xlabel("time (s)")
        fig.suptitle(key+" / "+split+" / zero external wrench; independent y scales")
        fig.tight_layout()
        fig.savefig(out/(key+"_false_wrench.png"), dpi=140)
        plt.close(fig)
    fig, axs = plt.subplots(2, 3, figsize=(13, 7))
    for spec in specs:
        key = spec["id"]
        g = npz(root/"geometry"/(key+"_geometry.npz"))
        for j, a in enumerate(axs.flat):
            a.semilogy(g["t"], g["singular_values_Nm"][:, j], label=key, linewidth=.7)
            a.set_title("Singular value "+str(j+1)+" (N m)")
            a.set_xlabel("time (s)")
            a.grid(alpha=.2)
    axs[0, 0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out/"geometry_singular_values.png", dpi=140)
    plt.close(fig)
    primary = [r for r in matrices if r["window"] == "core_[3,13)"]
    write(out/"summary.json", dict(primary_matrix_metrics=primary,
          validation_false_wrench=[r for r in axis if r["split"] == "validation"
              and r["window"] == "core_[3,13)" and r["quantity"] == "false_wrench"],
          validation_false_joint_torque=[r for r in axis if r["split"] == "validation"
              and r["window"] == "core_[3,13)" and r["quantity"] == "false_joint_torque"]))
    return dict(passed=True, axis_rows=len(axis), matrix_rows=len(matrices),
                training_updates=0, simulation_steps=0, test_read=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--run-root", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    start = time.monotonic()
    try:
        result = summarize(read(args.config), Path(args.run_root), out)
        result["wall_seconds"] = time.monotonic()-start
        write(out/"metrics.json", result)
    except Exception as exc:
        write(out/"metrics.json", dict(passed=False, error=repr(exc), wall_seconds=time.monotonic()-start))
        raise
