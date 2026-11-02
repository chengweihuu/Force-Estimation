#!/usr/bin/env python3
"""Container-only entries for known-model and structured-learning probes.

Use the research-workflow request for execution and lifecycle records. This
module does not start Docker, ROS, a robot driver or parameter search.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import traceback


def save_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)
                    + "\n", encoding="utf-8")


def validate_config(config):
    if config["scenarios"] != ["no_force", "constant", "sine"] or config["seed"] != 0:
        raise ValueError("The first probe contains exactly the three declared scenarios and seed 0")
    if not 0 < config["load_start_s"] < config["load_end_s"] < config["duration_s"] <= 5:
        raise ValueError("Invalid first-probe time range")
    if not 0 < config["evaluation"]["transient_exclusion_s"] < (
            config["load_end_s"] - config["load_start_s"]):
        raise ValueError("Invalid fixed evaluation window")
    if config["evaluation"]["crossing_levels"] != [0.1, 0.9]:
        raise ValueError("Step response uses predeclared 10/90 percent crossings")


def report_text(summaries):
    lines = ["# Known-dynamics momentum KF prerequisite", "",
             "EXPLORE only. UR5/MuJoCo substitute; no learned dynamics, TCN, Gaussian compensation or WMA.",
             "Reference and estimates retain their original timestamps. Values below are Fx errors in N.", "",
             "| Scenario | Full RMSE | Loaded RMSE | Loaded after transient RMSE |", "|---|---:|---:|---:|"]
    for name, metrics in summaries.items():
        values = [metrics[key]["rmse"][0] for key in ("full", "loaded", "loaded_after_transient")]
        lines.append(f"| {name} | " + " | ".join(f"{value:.6g}" for value in values) + " |")
    lines += ["", "Per-axis N/Nm metrics and response diagnostics: each scenario's metrics.json.",
              "Measurement, reference and estimate arrays are stored in separate NPZ files.",
              "This matching-model prerequisite does not measure the benefit of PIAKF learning or compensation."]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("checks", "simulate", "network-checks", "collect", "train", "evaluate"), required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--prerequisite-dir", type=Path,
                        help="Explicit output directory of the preceding checks step")
    parser.add_argument("--data-dir", type=Path)
    parser.add_argument("--training-dir", type=Path)
    args = parser.parse_args()
    if not Path("/.dockerenv").is_file():
        parser.error("Execute inside the declared Docker service; host execution is not authorized")
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ[name] = "1"
    root = Path(__file__).resolve().parents[2]
    config = json.loads(args.config.read_text(encoding="utf-8"))
    learning_stage = args.stage in ("network-checks", "collect", "train", "evaluate")
    if learning_stage:
        from learning import validate_learning_config
        validate_learning_config(config)
    else:
        validate_config(config)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    stage_output = output
    if (output / "metrics.json").exists() or (output / "config.snapshot.json").exists():
        raise FileExistsError("Do not overwrite or silently rerun a completed probe")
    try:
        if learning_stage:
            from learning import collect, replay, train
            save_json(output / "config.snapshot.json", config)
            if args.stage == "network-checks":
                from learning_checks import run_learning_checks
                metrics = run_learning_checks(config, output)
            else:
                if args.prerequisite_dir is None:
                    raise ValueError("Each learning step requires an explicit prerequisite directory")
                prerequisite = json.loads((args.prerequisite_dir / "metrics.json").read_text())
                if prerequisite.get("passed") is not True:
                    raise RuntimeError("Preceding step did not pass")
                if json.loads((args.prerequisite_dir / "config.snapshot.json").read_text()) != config:
                    raise RuntimeError("Learning config changed across steps")
                if args.stage == "collect":
                    metrics = collect(root, config, output)
                elif args.stage == "train":
                    if args.data_dir is None:
                        raise ValueError("Training requires --data-dir")
                    metrics = train(config, args.data_dir, output)
                else:
                    if args.data_dir is None or args.training_dir is None:
                        raise ValueError("Frozen evaluation requires data and training output paths")
                    metrics = replay(root, config, args.training_dir, args.data_dir, output)
        elif args.stage == "checks":
            from checks import run_checks
            save_json(output / "config.snapshot.json", config)
            provenance = json.loads((Path(__file__).parent / "provenance.json").read_text(encoding="utf-8"))
            save_json(output / "provenance.snapshot.json", provenance)
            metrics = run_checks(root, config)
        else:
            from checks import environment
            from simulation import run_scenario
            if args.prerequisite_dir is None:
                raise ValueError("Simulation requires an explicit --prerequisite-dir")
            prerequisite = json.loads((args.prerequisite_dir / "metrics.json").read_text())
            if prerequisite.get("passed") is not True:
                raise RuntimeError("Prerequisite checks must pass before any physical stepping")
            if json.loads((args.prerequisite_dir / "config.snapshot.json").read_text()) != config:
                raise RuntimeError("Config changed after the checks; use a new reviewed request/Run")
            runtime = environment()
            summaries = {}
            for scenario in config["scenarios"]:
                summaries[scenario] = run_scenario(root, config, scenario, output / scenario)
            metrics = {"safety_passed": True, "scenario_count": len(summaries),
                       "environment": runtime, "scenarios": summaries,
                       "scope": "known-dynamics prerequisite; not a full PIAKF reproduction"}
            metrics["environment"]["physics_steps"] = sum(
                round(config["duration_s"] / config["physics_dt_s"]) for _ in summaries)
            (output / "report.md").write_text(report_text(summaries), encoding="utf-8")
        metrics["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
        save_json(stage_output / "metrics.json", metrics)
        print(json.dumps({"stage": args.stage, "status": "passed", "output_dir": str(output)}))
        return 0
    except Exception as error:
        save_json(stage_output / "failure.json", {
            "stage": args.stage, "type": type(error).__name__, "message": str(error),
            "traceback": traceback.format_exc(), "at_utc": datetime.now(timezone.utc).isoformat(),
            "automatic_retry": False,
        })
        print(f"{args.stage} stopped: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
