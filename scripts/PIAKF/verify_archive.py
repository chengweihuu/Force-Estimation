#!/usr/bin/env python3
"""Check archive integrity and optional frozen 32-frame endpoint regression."""
import argparse
import ast
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
from urllib.parse import unquote


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def map_values(value, mapping):
    if isinstance(value, dict):
        return {key: map_values(item, mapping) for key, item in value.items()}
    if isinstance(value, list):
        return [map_values(item, mapping) for item in value]
    return mapping.get(value, value) if isinstance(value, str) else value


def strings(value):
    if isinstance(value, str):
        yield value
    elif isinstance(value, dict):
        for item in value.values():
            yield from strings(item)
    elif isinstance(value, list):
        for item in value:
            yield from strings(item)


def verify_files(root, base, manifest, compare_originals):
    assets = manifest["assets"]
    if len({item["path"] for item in assets}) != len(assets):
        raise ValueError("Duplicate archive destinations")
    for item in assets:
        path = base / item["path"]
        if path.stat().st_size != item["bytes"] or sha256(path) != item["sha256"]:
            raise ValueError(f"Archive content differs: {path}")
        if compare_originals:
            source = root / item["original_path"]
            expected = item["sha256"] if item["kind"] == "exact_copy" else item["source_file_sha256"]
            if sha256(source) != expected:
                raise ValueError(f"Original source differs: {source}")
    for item in manifest["tracked_dependencies"]:
        if sha256(root / item["path"]) != item["sha256"]:
            raise ValueError(f"Robot/environment dependency differs: {item['path']}")
    for item in manifest["source_code_checks"]:
        if sha256(root / item["path"]) != item["sha256"]:
            raise ValueError(f"Scientific source differs from archived Run: {item['path']}")
    mapping = {item["before"]: item["after"] for item in manifest["runtime_path_relocations"]}
    for item in manifest["config_snapshots"]:
        before = json.loads((base / item["original_config"]).read_text())
        after = json.loads((base / item["active_config"]).read_text())
        if map_values(before, mapping) != after:
            raise ValueError(f"Scientific configuration changed: {item['active_config']}")
        for value in strings(after):
            if value.startswith(("runs/", "scratch/")):
                raise ValueError(f"Runtime config depends on deleted workspace: {value}")
            if value.startswith(("scripts/", "reproductions/")) and not (root / value).exists():
                raise FileNotFoundError(value)
    sources = list(base.glob("*.py"))
    for path in sources:
        ast.parse(path.read_text(), filename=str(path))
    documents = [base / "README.md", base / "conclusion/summary.md",
                 root / "reproductions/PIAKF/HANDOFF.md", root / "reproductions/PIAKF/PINN机器人动力学估计方法论.md"]
    links = 0
    for path in documents:
        for target in re.findall(r"\]\(([^\n]+?)\)", path.read_text()):
            target = target.strip("<>").split("#", 1)[0]
            if not target or target.startswith(("http://", "https://")):
                continue
            if not (path.parent / unquote(target)).resolve().exists():
                raise FileNotFoundError(f"Broken link in {path}: {target}")
            links += 1
    return {"assets_verified": len(assets), "retained_bytes": sum(item["bytes"] for item in assets),
            "tracked_dependencies_verified": len(manifest["tracked_dependencies"]),
            "original_scientific_modules_unchanged": len(manifest["source_code_checks"]),
            "scientific_configurations_unchanged": len(manifest["config_snapshots"]),
            "python_syntax_checked": len(sources), "local_links_checked": links,
            "original_sources_compared": compare_originals}


def verify_runtime(root, base, manifest, compare_originals):
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ[name] = "1"
    import numpy as np
    import torch
    from checks import environment
    from paired_motion_diagnosis import run
    from static_gravity import training_bounds
    from structured import load_network

    torch.set_num_threads(1)
    runtime = environment()
    projections = 0
    for item in manifest["assets"]:
        if item["kind"] != "array_projection":
            continue
        with np.load(base / item["path"], allow_pickle=False) as f:
            if set(f.files) != set(item["arrays"]):
                raise ValueError("Projected array set differs")
            for key, expected in item["arrays"].items():
                value = f[key]
                if list(value.shape) != expected["shape"] or str(value.dtype) != expected["dtype"]:
                    raise ValueError("Projected shape/dtype differs")
                if not np.isfinite(value).all() or hashlib.sha256(value.tobytes()).hexdigest() != expected["sha256"]:
                    raise ValueError("Projected numerical content differs")
                if compare_originals:
                    with np.load(root / item["original_path"], allow_pickle=False) as source:
                        np.testing.assert_array_equal(value, source[key])
        projections += 1
    checkpoint_dir = base / "reference_data/checkpoints"
    checkpoints = {name: torch.load(checkpoint_dir / filename, map_location="cpu", weights_only=True)
                   for name, filename in (("initial", "initial.pt"), ("dynamic", "dynamic.pt"),
                                          ("baseline", "paired_baseline.pt"), ("augmented", "static_augmented.pt"))}
    for name, checkpoint in checkpoints.items():
        if checkpoint["updates"] != (0 if name == "initial" else 1000):
            raise ValueError("Checkpoint update count differs")
    for name, value in checkpoints["dynamic"]["state_dict"].items():
        torch.testing.assert_close(value, checkpoints["baseline"]["state_dict"][name], rtol=0, atol=0)
    model = load_network(checkpoint_dir / "dynamic.pt")
    training_bounds(root, {"training_config": "scripts/PIAKF/configs/learning.json",
                           "training_data_dir": "scripts/PIAKF/reference_data/learning"}, model)
    config = json.loads((base / "configs/paired_motion_diagnosis.json").read_text())
    result = run(root, config, base / "conclusion", check_only=True)
    return {"environment": runtime, "array_projections_verified": projections, "checkpoints_verified": len(checkpoints),
            "prefix_frames": result["processed_frames"], "diagnostic_combinations": result["variant_count"],
            "native_R_regression": result["native_R_regression"],
            "shared_covariance_max_abs_difference": result["shared_covariance_max_abs_difference"],
            "signal_reconstruction_max_abs_difference": result["signal_reconstruction_max_abs_difference"],
            "training_updates": 0, "physics_steps": 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", action="store_true", help="Requires the existing Docker numerical environment")
    parser.add_argument("--compare-originals", action="store_true", help="Before cleanup, compare retained assets with original Run files")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    base = Path(__file__).resolve().parent
    root = base.parents[1]
    manifest = json.loads((base / "archive_manifest.json").read_text())
    result = {"passed": True, "checked_at_utc": datetime.now(timezone.utc).isoformat(),
              "files": verify_files(root, base, manifest, args.compare_originals)}
    if args.runtime:
        if not Path("/.dockerenv").is_file():
            parser.error("Use --runtime inside the existing Docker service")
        result["runtime"] = verify_runtime(root, base, manifest, args.compare_originals)
    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / "archive_verification.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
