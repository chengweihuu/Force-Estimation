#!/usr/bin/env python3
"""Read-only structural checks for the research harness."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


REQUIRED_ACTIVE = (
    "## Identity",
    "## Question",
    "## Context",
    "## Allowed Changes",
    "## Non-goals",
    "## Budget",
    "## Done When",
    "## Stop and Ask",
    "## Outputs",
    "## Information Gain",
)
INACTIVE = "# Current Task\n\nStatus: Inactive\n\n下一步：根据 CURRENT 进入 THINK；用户确认计划后再替换本文件。"
CONTRACT_COLUMNS = {
    "Validation Contract": (
        "Hypothesis / Claim", "Treatment", "Baseline", "Endpoint",
        "Population / Inferential Unit", "Seed Schedule", "Code / Config Revision", "Failure / Stop Rule",
    ),
    "Experiment Contract": (
        "Claim", "Frozen Protocol", "Holdout / Data", "Metric / Decision Rule",
        "Immutable Request / Environment Binding", "Evidence Location",
    ),
    "Execution Contract": (
        "Requirement", "Owner / Interface", "Inputs / Outputs / Scientific Invariants", "Verification",
    ),
}
WIKI_COLUMNS = (
    "Memory ID", "Status", "Authoritative object", "Evidence pointer",
    "Applicable boundary", "Reusable lesson", "Restart condition",
)


def sections(text: str) -> dict[str, str]:
    headings = list(re.finditer(r"^## [^\n]+", text, re.MULTILINE))
    return {
        heading.group().strip(): text[heading.end():headings[i + 1].start() if i + 1 < len(headings) else len(text)].strip()
        for i, heading in enumerate(headings)
    }


def table_rows(text: str) -> list[list[str]]:
    return [
        [cell.strip() for cell in line.strip()[1:-1].split("|")]
        for line in text.splitlines() if line.strip().startswith("|") and line.strip().endswith("|")
    ]


def check_table(text: str, columns: tuple[str, ...], label: str) -> list[str]:
    rows = table_rows(text)
    if not rows or any(column not in rows[0] for column in columns):
        return [f"{label} missing required columns: {', '.join(columns)}"]
    if len(rows) < 3 or len(rows[1]) != len(rows[0]) or not all(re.fullmatch(r":?-{3,}:?", cell) for cell in rows[1]):
        return [f"{label} requires a table header, separator, and populated row"]
    indexes = [rows[0].index(column) for column in columns]
    if any(len(row) != len(rows[0]) or any(not row[i] for i in indexes) for row in rows[2:]):
        return [f"{label} has an incomplete row"]
    return []


def check_task(text: str) -> list[str]:
    errors: list[str] = []
    if text.strip() == INACTIVE:
        return errors
    statuses = re.findall(r"^Status:[\t ]*(\S+)[\t ]*$", text, re.MULTILINE)
    if statuses == ["Inactive"]:
        return ["Inactive TASK must use the exact minimal shape"]
    if statuses != ["Active"]:
        return ["TASK requires exactly one Status: Active or the minimal Inactive shape"]
    parts = sections(text)
    headings = re.findall(r"^## [^\n]+", text, re.MULTILINE)
    if len(headings) != len(set(headings)):
        errors.append("TASK has duplicate sections")
    for heading in REQUIRED_ACTIVE:
        if not parts.get(heading):
            errors.append(f"TASK missing or empty {heading}")
    present = [headings.index(heading) for heading in REQUIRED_ACTIVE if heading in headings]
    if present != sorted(present):
        errors.append("TASK sections are out of order")
    for heading, fields in (
        ("## Identity", ("Task ID", "Stage", "Progress", "Research objects")),
        ("## Budget", ("Files", "Time / Compute", "Robot runs")),
        ("## Information Gain", ("Scientific", "Engineering")),
    ):
        for field in fields:
            values = re.findall(rf"^- {re.escape(field)}:[\t ]*(.+)$", parts.get(heading, ""), re.MULTILINE)
            if len(values) != 1 or not values[0].strip():
                errors.append(f"{heading} requires one non-empty {field}")
    stages = re.findall(r"^- Stage:[\t ]*(\S+)[\t ]*$", parts.get("## Identity", ""), re.MULTILINE)
    stage = stages[0] if len(stages) == 1 else None
    if stage not in ("EXPLORE", "VALIDATE", "FORMAL"):
        errors.append("TASK Stage must be EXPLORE, VALIDATE, or FORMAL")
    required_contract = {"VALIDATE": "Validation Contract", "FORMAL": "Experiment Contract"}.get(stage)
    for name, columns in CONTRACT_COLUMNS.items():
        heading = f"## {name}"
        if name == required_contract or heading in parts:
            errors.extend(check_table(parts.get(heading, ""), columns, name))
            if heading in headings and "## Outputs" in headings and "## Information Gain" in headings:
                if not headings.index("## Outputs") < headings.index(heading) < headings.index("## Information Gain"):
                    errors.append(f"{name} must be between Outputs and Information Gain")
    if stage == "FORMAL":
        if not re.search(r"^- Frozen commit:[\t ]*`?[0-9a-f]{7,40}`?[\t ]*$", parts.get("## Context", ""), re.MULTILINE):
            errors.append("FORMAL requires Frozen commit in Context")
        rows = table_rows(parts.get("## Experiment Contract", ""))
        if rows and "Frozen Protocol" in rows[0]:
            index = rows[0].index("Frozen Protocol")
            if any(len(row) <= index or not re.search(r"\bP-\d+\b", row[index]) for row in rows[2:]):
                errors.append("FORMAL requires a Protocol ID in each Experiment Contract row")
    return errors


def check_repository(root: Path) -> list[str]:
    errors: list[str] = []
    task = root / "state/TASK.md"
    if not task.is_file():
        return ["state/TASK.md is missing"]
    errors.extend(check_task(task.read_text(encoding="utf-8")))

    required_files = (
        "AGENTS.md",
        "state/AGENTS.md",
        "state/CONTEXT.md",
        "docs/references/REFERENCE_TEMPLATE.md",
        "docker/ENVIRONMENT_CONTRACT.md",
        "docs/RESEARCH_WIKI.md",
        "docs/ideas/AGENTS.md",
        "docs/protocols/AGENTS.md",
        "docs/attachment/workflow_architecture.svg",
        "docs/pi_agent_capabilities.md",
    )
    for relative in required_files:
        if not (root / relative).is_file():
            errors.append(f"required template missing: {relative}")

    context = (root / "state/CONTEXT.md").read_text(encoding="utf-8") if (root / "state/CONTEXT.md").is_file() else ""
    context_lower = context.lower()
    for needle in ("delegation packet", "result packet", "sha256", "raw tool output"):
        if needle not in context_lower:
            errors.append(f"CONTEXT missing {needle}")

    env = (root / "docker/ENVIRONMENT_CONTRACT.md").read_text(encoding="utf-8") if (root / "docker/ENVIRONMENT_CONTRACT.md").is_file() else ""
    for needle in ("Mode:", "Dependency lockfile", "Data manifest", "Approved compute stage"):
        if needle not in env:
            errors.append(f"environment contract missing {needle}")

    ignored = (root / ".gitignore").read_text(encoding="utf-8")
    for needle in ("/runs/", "context-cache", "raw/"):
        if needle not in ignored:
            errors.append(f".gitignore missing artifact rule: {needle}")

    # Check maintained structure; prose wording is not an executable scientific rule.
    wiki = root / "docs/RESEARCH_WIKI.md"
    if wiki.is_file():
        errors.extend(check_table(wiki.read_text(encoding="utf-8"), WIKI_COLUMNS, "Research Wiki"))
    return errors


def self_test() -> list[str]:
    """Exercise stage-aware checks without writing files or running research."""
    bodies = {heading: "Declared." for heading in REQUIRED_ACTIVE}
    bodies["## Identity"] = "- Task ID: H-101\n- Stage: EXPLORE\n- Progress: Ready\n- Research objects: I-001:C-01"
    bodies["## Budget"] = "- Files: 1\n- Time / Compute: CPU 1 minute\n- Robot runs: 0"
    bodies["## Information Gain"] = "- Scientific: Pending\n- Engineering: Pending"
    valid = "# Current Task\n\nStatus: Active\n\n" + "\n\n".join(f"{heading}\n\n{body}" for heading, body in bodies.items())
    cases = {
        "status": valid.replace("Status: Active", "Status: Inactive"),
        "duplicate status": valid + "\nStatus: Inactive",
        "legacy type": valid.replace("Stage: EXPLORE", "Type: BUILD"),
        "unknown stage": valid.replace("Stage: EXPLORE", "Stage: MAINTENANCE"),
        "budget": valid.replace("- Time / Compute: CPU 1 minute", ""),
        "validation contract": valid.replace("Stage: EXPLORE", "Stage: VALIDATE"),
        "formal prerequisites": valid.replace("Stage: EXPLORE", "Stage: FORMAL"),
    }
    failures: list[str] = []
    for name, fixture in (("Inactive", INACTIVE), ("EXPLORE", valid)):
        if check_task(fixture):
            failures.append(f"valid {name} TASK fixture was rejected")
    for name, fixture in cases.items():
        if not check_task(fixture):
            failures.append(f"self-test fixture unexpectedly passed: {name}")

    wiki = "\n".join("| " + " | ".join(row) + " |" for row in (WIKI_COLUMNS, ["---"] * len(WIKI_COLUMNS), ["Declared"] * len(WIKI_COLUMNS)))
    if check_table(wiki, WIKI_COLUMNS, "Research Wiki"):
        failures.append("valid failure-memory table was rejected")
    if not check_table(wiki.replace("Restart condition", ""), WIKI_COLUMNS, "Research Wiki"):
        failures.append("failure-memory fixture unexpectedly passed without restart condition")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    errors = check_repository(args.repo)
    if args.self_test:
        errors.extend(self_test())
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    print("harness validation passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
