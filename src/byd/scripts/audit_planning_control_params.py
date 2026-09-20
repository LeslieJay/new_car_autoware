#!/usr/bin/env python3
"""Audit Autoware planning/control launch parameters without starting ROS.

The audit deliberately separates facts (inventory and effective values) from
policy checks (cross-module envelopes).  It is safe to run in CI: inputs are
read-only and no ROS daemon is required.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml


SEVERITY_ORDER = {"P0": 0, "P1": 1, "P2": 2, "P3": 3}
ACTIVE = "default-enabled"
CONDITIONAL = "conditional"
DISABLED = "default-disabled"
UNREFERENCED = "unreferenced"


class UniqueKeyLoader(yaml.SafeLoader):
    """YAML loader that rejects duplicate mapping keys."""


def _construct_unique_mapping(loader: UniqueKeyLoader, node: yaml.MappingNode, deep: bool = False):
    mapping: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise ValueError(f"duplicate key {key!r} at line {key_node.start_mark.line + 1}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


@dataclass(frozen=True)
class InventoryItem:
    file: str
    domain: str
    status: str
    launch_reference: bool
    source_contracts: tuple[str, ...]
    install_matches_source: bool | None


@dataclass(frozen=True)
class Finding:
    id: str
    severity: str
    title: str
    file: str
    parameter: str
    value: Any
    related: str
    evidence: str
    impact: str
    recommendation: str
    confidence: str = "high"


def load_yaml(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return yaml.load(stream, Loader=UniqueKeyLoader)


def flatten(value: Any, prefix: tuple[str, ...] = ()) -> Iterable[tuple[tuple[str, ...], Any]]:
    if isinstance(value, dict):
        for key, child in value.items():
            yield from flatten(child, prefix + (str(key),))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from flatten(child, prefix + (str(index),))
    else:
        yield prefix, value


def params(data: Any) -> dict[str, Any]:
    if not isinstance(data, dict):
        return {}
    wildcard = data.get("/**")
    if not isinstance(wildcard, dict):
        return {}
    result = wildcard.get("ros__parameters")
    return result if isinstance(result, dict) else {}


def nested(data: dict[str, Any], dotted: str) -> Any:
    current: Any = data
    for key in dotted.split("."):
        if not isinstance(current, dict) or key not in current:
            raise KeyError(dotted)
        current = current[key]
    return current


def preset_defaults(path: Path) -> dict[str, str]:
    data = load_yaml(path)
    result: dict[str, str] = {}
    if not isinstance(data, dict) or not isinstance(data.get("launch"), list):
        return result
    for entry in data["launch"]:
        if not isinstance(entry, dict) or not isinstance(entry.get("arg"), dict):
            continue
        arg = entry["arg"]
        if "name" in arg and "default" in arg:
            result[str(arg["name"])] = str(arg["default"]).lower()
    return result


PLANNING_MODULE_ARGS = {
    "autoware_behavior_path_static_obstacle_avoidance_module": "launch_static_obstacle_avoidance",
    "autoware_behavior_path_simple_avoidance_module": "launch_simple_avoidance",
    "avoidance_by_lane_change": "launch_avoidance_by_lane_change_module",
    "simple_lc_avoidance": "launch_simple_lc_avoidance",
    "autoware_behavior_path_dynamic_obstacle_avoidance_module": "launch_dynamic_obstacle_avoidance",
    "sampling_planner": "launch_sampling_planner_module",
    "side_shift": "launch_side_shift_module",
    "start_planner": "launch_start_planner_module",
    "goal_planner": "launch_goal_planner_module",
    "autoware_behavior_path_bidirectional_traffic_module": "launch_bidirectional_traffic_module",
    "crosswalk.param.yaml": "launch_crosswalk_module",
    "walkway.param.yaml": "launch_walkway_module",
    "traffic_light.param.yaml": "launch_traffic_light_module",
    "intersection.param.yaml": "launch_intersection_module",
    "roundabout.param.yaml": "launch_roundabout_module",
    "blind_spot.param.yaml": "launch_blind_spot_module",
    "detection_area.param.yaml": "launch_detection_area_module",
    "virtual_traffic_light.param.yaml": "launch_virtual_traffic_light_module",
    "no_stopping_area.param.yaml": "launch_no_stopping_area_module",
    "stop_line.param.yaml": "launch_stop_line_module",
    "occlusion_spot.param.yaml": "launch_occlusion_spot_module",
    "speed_bump.param.yaml": "launch_speed_bump_module",
    "no_drivable_lane.param.yaml": "launch_no_drivable_lane_module",
    "obstacle_stop.param.yaml": "launch_obstacle_stop_module",
    "obstacle_slow_down.param.yaml": "launch_obstacle_slow_down_module",
    "obstacle_cruise.param.yaml": "launch_obstacle_cruise_module",
    "dynamic_obstacle_stop.param.yaml": "launch_dynamic_obstacle_stop_module",
    "out_of_lane.param.yaml": "launch_out_of_lane_module",
    "obstacle_velocity_limiter.param.yaml": "launch_obstacle_velocity_limiter_module",
    "run_out.param.yaml": "launch_run_out_module",
    "boundary_departure_prevention.param.yaml": "launch_boundary_departure_prevention_module",
    "road_user_stop.param.yaml": "launch_road_user_stop_module",
    "surround_obstacle_checker": "launch_surround_obstacle_checker",
    "freespace_planner": "launch_parking_module",
    "latency_checker.param.yaml": "launch_latency_checker",
    "trajectory_checker.param.yaml": "launch_trajectory_checker",
    "intersection_collision_checker.param.yaml": "launch_intersection_collision_checker",
    "rear_collision_checker.param.yaml": "launch_rear_collision_checker",
}

CONTROL_MODULE_ARGS = {
    "lane_departure_checker": "launch_lane_departure_checker",
    "control_validator": "launch_control_validator",
    "autoware_autonomous_emergency_braking": "launch_autonomous_emergency_braking",
    "autoware_collision_detector": "launch_collision_detector",
    "obstacle_collision_checker": "launch_obstacle_collision_checker",
    "predicted_path_checker": "launch_predicted_path_checker",
    "external_cmd_selector": "launch_external_cmd_selector",
}


def _module_arg(relative: str, mapping: dict[str, str]) -> str | None:
    for marker, arg in mapping.items():
        if marker in relative:
            return arg
    return None


def classify_status(relative: str, planning_preset: dict[str, str], control_preset: dict[str, str], referenced: bool) -> str:
    if "/preset/default_preset.yaml" in f"/{relative}":
        return ACTIVE
    if "/preset/" in f"/{relative}":
        return CONDITIONAL
    if relative.startswith("planning/"):
        arg = _module_arg(relative, PLANNING_MODULE_ARGS)
        if arg:
            return ACTIVE if planning_preset.get(arg) == "true" else DISABLED
        if relative.endswith(("L2.param.yaml", "Linf.param.yaml")):
            return CONDITIONAL
        if relative.endswith(("Analytical.param.yaml", "JerkFiltered.param.yaml")):
            return ACTIVE
        if "path_optimizer.param.yaml" in relative or "/path_sampler/" in relative:
            return DISABLED
        if "/path_generator/" in relative:
            return DISABLED
        if "/rtc_replayer/" in relative:
            return UNREFERENCED
        return ACTIVE if referenced else UNREFERENCED
    arg = _module_arg(relative, CONTROL_MODULE_ARGS)
    if arg:
        return ACTIVE if control_preset.get(arg) == "true" else DISABLED
    if relative.endswith("lateral/mpc.param.yaml"):
        return CONDITIONAL
    if relative.endswith(("lateral/pure_pursuit.param.yaml", "longitudinal/pid.param.yaml")):
        return ACTIVE
    return ACTIVE if referenced else UNREFERENCED


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def make_inventory(workspace: Path, config_root: Path, files: list[Path]) -> list[InventoryItem]:
    planning_launch = workspace / "src/launcher/autoware_launch/autoware_launch/launch/components/tier4_planning_component.launch.xml"
    control_launch = workspace / "src/launcher/autoware_launch/autoware_launch/launch/components/tier4_control_component.launch.xml"
    launch_text = "\n".join(path.read_text(encoding="utf-8") for path in (planning_launch, control_launch))
    planning_preset = preset_defaults(config_root / "planning/preset/default_preset.yaml")
    control_preset = preset_defaults(config_root / "control/preset/default_preset.yaml")
    contract_index: dict[str, list[Path]] = {}
    for source_root in (workspace / "src/core", workspace / "src/universe"):
        if not source_root.exists():
            continue
        for candidate in source_root.rglob("*.yaml"):
            if "/build/" not in str(candidate):
                contract_index.setdefault(candidate.name, []).append(candidate)
    install_root = workspace / "install/autoware_launch/share/autoware_launch/config"
    inventory = []
    for path in files:
        relative = path.relative_to(config_root).as_posix()
        referenced = path.name in launch_text or any(part in launch_text for part in path.parts[-3:-1])
        installed = install_root / relative
        install_match = sha256(path) == sha256(installed) if installed.is_file() else None
        contracts = tuple(
            sorted(candidate.relative_to(workspace).as_posix() for candidate in contract_index.get(path.name, []))
        )
        inventory.append(
            InventoryItem(
                file=relative,
                domain=relative.split("/", 1)[0],
                status=classify_status(relative, planning_preset, control_preset, referenced),
                launch_reference=referenced,
                source_contracts=contracts,
                install_matches_source=install_match,
            )
        )
    return inventory


def finding(fid: str, severity: str, title: str, file: str, parameter: str, value: Any,
            related: str, evidence: str, impact: str, recommendation: str,
            confidence: str = "high") -> Finding:
    return Finding(fid, severity, title, file, parameter, value, related, evidence, impact, recommendation, confidence)


def range_order_violations(node: Any, path: tuple[str, ...] = ()) -> list[tuple[str, float, str, float]]:
    """Return unambiguous sibling min/max inversions from a parameter tree."""
    violations: list[tuple[str, float, str, float]] = []
    if not isinstance(node, dict):
        return violations
    numeric = {
        str(key): value
        for key, value in node.items()
        if isinstance(value, (int, float)) and not isinstance(value, bool)
    }
    for key, minimum in numeric.items():
        candidates = []
        if key.startswith("min_"):
            candidates.append("max_" + key[4:])
        if key.endswith("_min"):
            candidates.append(key[:-4] + "_max")
        for maximum_key in candidates:
            if maximum_key in numeric and minimum > numeric[maximum_key]:
                violations.append((
                    ".".join(path + (key,)), minimum,
                    ".".join(path + (maximum_key,)), numeric[maximum_key],
                ))
    for key, child in node.items():
        violations.extend(range_order_violations(child, path + (str(key),)))
    return violations


def severity_for(status: str, active: str = "P1", inactive: str = "P2") -> str:
    return active if status == ACTIVE else inactive


def audit(workspace: Path, config_root: Path) -> tuple[list[InventoryItem], list[Finding], dict[str, Any]]:
    files = sorted((*config_root.joinpath("planning").rglob("*.yaml"), *config_root.joinpath("control").rglob("*.yaml")))
    if not files:
        raise RuntimeError(f"no YAML files below {config_root}")
    documents: dict[str, Any] = {}
    parse_findings: list[Finding] = []
    for path in files:
        relative = path.relative_to(config_root).as_posix()
        try:
            documents[relative] = load_yaml(path)
        except Exception as exc:
            parse_findings.append(finding(
                "YAML_PARSE", "P1", "YAML cannot be parsed uniquely", relative, "-", None, "YAML loader",
                str(exc), "The node may reject the file or silently load an unintended value.",
                "Fix syntax or duplicate keys before launch.",
            ))
    inventory = make_inventory(workspace, config_root, files)
    status = {item.file: item.status for item in inventory}
    findings = list(parse_findings)

    for item in inventory:
        if item.file not in documents or "/preset/" in f"/{item.file}":
            continue
        if not params(documents[item.file]):
            findings.append(finding(
                "ROS_PARAM_SHAPE", severity_for(item.status), "Invalid ROS parameter file shape", item.file,
                "/**.ros__parameters", None, "ROS 2 parameter file convention",
                "The file does not contain a non-empty /**.ros__parameters mapping.",
                "Parameters may not reach the intended node.", "Wrap parameters below /**: ros__parameters:.",
            ))
        if item.install_matches_source is False:
            findings.append(finding(
                "INSTALL_STALE", severity_for(item.status), "Installed config differs from source", item.file,
                "-", None, "install/autoware_launch", "SHA-256 content differs.",
                "Runtime can use values different from the reviewed source.", "Rebuild autoware_launch and re-run the audit.",
            ))

    # Generic, high-confidence sibling range checks.
    for relative, document in documents.items():
        for minimum_key, minimum, maximum_key, maximum in range_order_violations(params(document)):
            findings.append(finding(
                "RANGE_ORDER", severity_for(status.get(relative, UNREFERENCED)),
                "Minimum exceeds maximum", relative, minimum_key, minimum, maximum_key,
                f"{minimum} > {maximum}", "Sampling or clamping can become empty or inverted.",
                "Restore min <= max using the module's documented units.",
            ))

    common_file = "planning/scenario_planning/common/common.param.yaml"
    validator_file = "planning/scenario_planning/common/planning_validator/trajectory_checker.param.yaml"
    common = params(documents[common_file])
    validator = params(documents[validator_file])
    limit_min_acc = float(nested(common, "limit.min_acc"))
    validator_min_acc = float(nested(validator, "trajectory_checker.min_lon_accel.threshold"))

    producers = [
        (common_file, "limit.min_acc"),
        ("planning/scenario_planning/common/autoware_velocity_smoother/Analytical.param.yaml", "backward.min_acc"),
        ("planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/behavior_velocity_planner_common.param.yaml", "max_accel"),
        ("planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/intersection.param.yaml", "intersection.common.max_accel"),
        ("planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_cruise.param.yaml", "obstacle_cruise.cruise_planning.pid_based_planner.min_accel_during_cruise"),
        ("planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml", "obstacle_stop.stop_planning.object_type_specified_params.unknown.limit_min_acc"),
        ("planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/road_user_stop.param.yaml", "road_user_stop.stop_planning.limit_min_acc"),
        ("planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/blind_spot.param.yaml", "blind_spot.brake.critical.deceleration"),
        ("planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/blind_spot.param.yaml", "blind_spot.brake.semi_critical.deceleration"),
    ]
    for relative, key in producers:
        try:
            value = float(nested(params(documents[relative]), key))
        except (KeyError, TypeError, ValueError):
            continue
        if value < validator_min_acc:
            findings.append(finding(
                "PLANNER_VALIDATOR_MIN_ACC", severity_for(status.get(relative, UNREFERENCED)),
                "Planner can exceed planning-validator deceleration envelope", relative, key, value,
                f"{validator_file}: trajectory_checker.min_lon_accel.threshold={validator_min_acc}",
                f"The producer permits {value} m/s^2, below the validator floor {validator_min_acc} m/s^2.",
                "A valid stop trajectory can be rejected and replaced by the validator soft-stop trajectory.",
                "Choose one system deceleration envelope, then keep every active producer at or above the validator floor; replay one change at a time.",
            ))

    if float(nested(params(documents[
        "planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/behavior_velocity_planner_common.param.yaml"
    ]), "max_accel")) < limit_min_acc:
        findings.append(finding(
            "PLANNER_HARD_LIMIT", "P1", "Behavior planner requests more deceleration than the common hard limit",
            "planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/behavior_velocity_planner_common.param.yaml",
            "max_accel", nested(params(documents[
                "planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/behavior_velocity_planner_common.param.yaml"
            ]), "max_accel"), f"{common_file}: limit.min_acc={limit_min_acc}",
            "The behavior-velocity deceleration is outside the common smoother limit.",
            "Different stages can clamp or reshape the intended stop inconsistently.",
            "Align the behavior-velocity deceleration magnitude with the common planning limit and validator envelope.",
        ))

    planning_preset = preset_defaults(config_root / "planning/preset/default_preset.yaml")
    scene_file = "planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/scene_module_manager.param.yaml"
    scene = params(documents[scene_file])
    slot2 = scene.get("slot2", [])
    if planning_preset.get("launch_simple_avoidance") == "true" and planning_preset.get("launch_simple_lc_avoidance") == "true":
        lc_index = slot2.index("simple_lane_change_avoidance") if "simple_lane_change_avoidance" in slot2 else -1
        avoid_index = slot2.index("simple_avoidance") if "simple_avoidance" in slot2 else -1
        exclusive = not bool(nested(scene, "simple_lane_change_avoidance.enable_simultaneous_execution_as_candidate_module"))
        if lc_index >= 0 and avoid_index >= 0 and lc_index < avoid_index and exclusive:
            findings.append(finding(
                "AVOIDANCE_ARBITRATION", "P1", "Higher-priority lane-change avoidance suppresses simple avoidance fallback",
                scene_file, "slot2", slot2, "planning/preset/default_preset.yaml",
                "Both modules are enabled; simple_lane_change_avoidance is earlier in slot2 and is exclusive.",
                "When lane change has no room or no adjacent lane it can own the scene and output a stop instead of allowing the lower-priority shifter to act.",
                "Define one owner for static-obstacle avoidance or make the failure-to-plan fallback explicit and regression-test the arbitration.",
            ))

    simple_lc_file = "planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/simple_lc_avoidance/simple_lc_avoidance.param.yaml"
    simple_lc = params(documents[simple_lc_file])["simple_lane_change_avoidance"]
    max_speed = float(common["max_vel"])
    normal_decel = abs(float(nested(common, "normal.min_acc")))
    control_delay = float(nested(params(documents[
        "control/trajectory_follower/longitudinal/pid.param.yaml"
    ]), "delay_compensation_time"))
    available = float(simple_lc["min_forward_distance"]) - float(simple_lc["stop_margin_before_object"])
    required = max_speed * control_delay + max_speed * max_speed / (2.0 * normal_decel)
    if available < required:
        findings.append(finding(
            "SIMPLE_LC_STOP_ENVELOPE", severity_for(status.get(simple_lc_file, UNREFERENCED)),
            "Minimum lane-change-avoidance stop distance is physically insufficient",
            simple_lc_file, "min_forward_distance - stop_margin_before_object", round(available, 3),
            "common.max_vel, common.normal.min_acc, control.pid.delay_compensation_time",
            f"available={available:.2f} m; conservative minimum={max_speed:.2f}*{control_delay:.2f} + {max_speed:.2f}^2/(2*{normal_decel:.2f})={required:.2f} m",
            "A target first accepted near the minimum range cannot be stopped before with the configured normal deceleration.",
            "Increase the minimum actionable distance or reduce speed/delay; include localization and actuation margins before selecting the final value.",
        ))

    control_validator_file = "control/control_validator/control_validator.param.yaml"
    validator_delay = float(nested(params(documents[control_validator_file]), "thresholds.assumed_delay_time"))
    if validator_delay < control_delay:
        findings.append(finding(
            "CONTROL_DELAY_UNDERESTIMATE", "P1", "Control validator assumes less delay than the controller compensates",
            control_validator_file, "thresholds.assumed_delay_time", validator_delay,
            "control/trajectory_follower/longitudinal/pid.param.yaml: delay_compensation_time",
            f"validator={validator_delay:.2f} s < controller={control_delay:.2f} s",
            "The control validator can under-predict stop-point overrun.",
            "Use a measured end-to-end delay bound consistently in controller and validator, then verify with bag timing.",
        ))

    counts = {
        "yaml_files": len(files),
        "planning_files": sum(item.domain == "planning" for item in inventory),
        "control_files": sum(item.domain == "control" for item in inventory),
        "status_counts": {name: sum(item.status == name for item in inventory) for name in (ACTIVE, CONDITIONAL, DISABLED, UNREFERENCED)},
        "finding_counts": {name: sum(item.severity == name for item in findings) for name in SEVERITY_ORDER},
        "source_contract_coverage": sum(bool(item.source_contracts) for item in inventory),
        "install_compared": sum(item.install_matches_source is not None for item in inventory),
        "install_mismatches": sum(item.install_matches_source is False for item in inventory),
    }
    findings.sort(key=lambda item: (SEVERITY_ORDER[item.severity], item.id, item.file, item.parameter))
    return inventory, findings, counts


def markdown_report(inventory: list[InventoryItem], findings: list[Finding], counts: dict[str, Any]) -> str:
    lines = [
        "# Planning and control parameter static audit",
        "",
        "## Summary",
        "",
        f"- YAML coverage: **{counts['yaml_files']}** ({counts['planning_files']} planning, {counts['control_files']} control)",
        f"- Findings: " + ", ".join(f"{key}={counts['finding_counts'][key]}" for key in SEVERITY_ORDER),
        f"- Source-contract filename coverage: {counts['source_contract_coverage']}/{counts['yaml_files']}",
        f"- Source/install comparisons: {counts['install_compared']}; mismatches: {counts['install_mismatches']}",
        "",
        "Exit status is 1 when P0/P1 findings exist, 2 on audit failure, otherwise 0.",
        "",
        "## Findings",
        "",
    ]
    if not findings:
        lines.append("No findings.")
    for item in findings:
        lines.extend([
            f"### {item.severity} {item.id}: {item.title}", "",
            f"- File: `{item.file}`", f"- Parameter: `{item.parameter}`", f"- Value: `{item.value}`",
            f"- Related: `{item.related}`", f"- Evidence: {item.evidence}", f"- Impact: {item.impact}",
            f"- Recommendation: {item.recommendation}", f"- Confidence: {item.confidence}", "",
        ])
    lines.extend(["## Inventory", "", "| File | Status | Launch ref | Contract candidates | Install match |", "|---|---|---:|---:|---:|"])
    for item in inventory:
        install = "n/a" if item.install_matches_source is None else str(item.install_matches_source).lower()
        lines.append(f"| `{item.file}` | {item.status} | {str(item.launch_reference).lower()} | {len(item.source_contracts)} | {install} |")
    lines.extend(["", "## Method limits", "", "- Static envelope findings identify incompatible permissions, not proof that every module exercised the extreme value.", "- Runtime-loaded values and vehicle response require the companion replay tool.", "- Filename-based source-contract coverage is evidence routing; it does not claim full generated-schema validation where no schema exists.", ""])
    return "\n".join(lines)


def main() -> int:
    script = Path(__file__).resolve()
    workspace_default = script.parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=workspace_default)
    parser.add_argument("--config-root", type=Path)
    parser.add_argument("--markdown", type=Path, help="write Markdown report")
    parser.add_argument("--json", type=Path, dest="json_path", help="write machine-readable report")
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    config_root = (args.config_root or workspace / "src/launcher/autoware_launch/autoware_launch/config").resolve()
    try:
        inventory, findings, counts = audit(workspace, config_root)
        markdown = markdown_report(inventory, findings, counts)
        if args.markdown:
            args.markdown.parent.mkdir(parents=True, exist_ok=True)
            args.markdown.write_text(markdown, encoding="utf-8")
        if args.json_path:
            args.json_path.parent.mkdir(parents=True, exist_ok=True)
            args.json_path.write_text(json.dumps({
                "summary": counts,
                "findings": [asdict(item) for item in findings],
                "inventory": [asdict(item) for item in inventory],
            }, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(json.dumps(counts, ensure_ascii=False))
        return 1 if any(item.severity in {"P0", "P1"} for item in findings) else 0
    except Exception as exc:
        print(f"audit failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
