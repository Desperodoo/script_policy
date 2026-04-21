"""Standalone runtime session helpers for SDK-first script execution."""

from __future__ import annotations

import json
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

from script_runtime.adapters import (
    FMFirstGraspStackAdapter,
    MockSDKBridge,
    NullLearnedModuleAdapter,
    NullPerceptionAdapter,
    OraclePerceptionAdapter,
    RoboTwinDepthPoseProvider,
    RoboTwinBridge,
    SDKBridge,
    build_default_fm_first_grasp_stack,
)
from script_runtime.core import SkillContext, TaskBlackboard, WorldState
from script_runtime.core.skill_base import clear_pending_refresh_reason, set_pending_refresh_reason
from script_runtime.executors import TraceRecorder, TreeExecutor
from script_runtime.factory import build_default_skill_registry
from script_runtime.tasks import ArticulatedProbeTask, HandoverProbeTask, PegInsertTask, PickPlaceTask, StagedPlaceProbeTask
from script_runtime.place import ClosedLoopPlaceModule, HeuristicPlaceModule

TASK_NAME_CONTRACT_HINTS = {
    "handover_block": "handover_probe",
    "handover_mic": "handover_probe",
    "open_laptop": "articulated_probe",
    "open_microwave": "articulated_probe",
    "place_can_basket": "staged_place_probe",
    "place_bread_basket": "staged_place_probe",
    "place_object_basket": "staged_place_probe",
}
TASK_CONTRACT_ALIASES = {
    "articulated_probe": "articulated_probe",
    "articulated_task": "articulated_probe",
    "drawer_open_pick": "articulated_probe",
    "handover": "handover_probe",
    "handover_probe": "handover_probe",
    "peg_insert": "peg_insert",
    "pick_place": "pick_place",
    "pickplace": "pick_place",
    "place_only": "pick_place",
    "staged_place": "staged_place_probe",
    "staged_place_probe": "staged_place_probe",
}


def load_runtime_config(config_path: str | Path) -> Dict[str, Any]:
    path = Path(config_path)
    with path.open("r", encoding="utf-8") as handle:
        if path.suffix == ".json":
            return json.load(handle)
        return yaml.safe_load(handle) or {}


def normalize_task_contract(value: Any) -> str:
    text = str(value or "").strip().lower().replace("-", "_")
    if not text:
        return "pick_place"
    return TASK_CONTRACT_ALIASES.get(text, text)


def resolve_task_contract(config: Dict[str, Any]) -> str:
    runtime = dict(config.get("runtime") or {})
    task_goal = dict(config.get("task_goal") or {})
    robotwin = dict(config.get("robotwin") or {})
    explicit = (
        config.get("task_contract")
        or runtime.get("task_contract")
        or task_goal.get("task_contract")
        or robotwin.get("task_contract")
    )
    if explicit:
        return normalize_task_contract(explicit)
    task_name = str(robotwin.get("task_name") or task_goal.get("task_name") or "").strip()
    if task_name:
        return normalize_task_contract(TASK_NAME_CONTRACT_HINTS.get(task_name, "pick_place"))
    return "pick_place"


def build_task_from_config(config: Dict[str, Any]) -> Any:
    task_goal = dict(config.get("task_goal") or {})
    contract = resolve_task_contract(config)
    if contract == "pick_place":
        return PickPlaceTask(goal=task_goal)
    if contract == "staged_place_probe":
        return StagedPlaceProbeTask(goal=task_goal)
    if contract == "handover_probe":
        return HandoverProbeTask(goal=task_goal)
    if contract == "articulated_probe":
        return ArticulatedProbeTask(goal=task_goal)
    if contract == "peg_insert":
        return PegInsertTask(goal=task_goal)
    raise ValueError(f"Unsupported task_contract: {contract}")


def seed_pick_place_blackboard(blackboard: TaskBlackboard, config: Dict[str, Any]) -> TaskBlackboard:
    scene = config.get("scene", {})
    execution = config.get("execution", {})
    task_goal = config.get("task_goal", {})
    poses = config.get("poses", {})
    gripper = config.get("gripper", {})
    grasp_semantics = config.get("grasp_semantics", {})
    runtime = config.get("runtime", {})
    robotwin = config.get("robotwin", {})
    task_contract = resolve_task_contract(config)
    suite_role = str(runtime.get("suite_role", config.get("suite_role", "")) or "")
    probe_type = str(runtime.get("probe_type", config.get("probe_type", "")) or "")
    gate = bool(runtime.get("gate", config.get("gate", False)))
    canary = bool(runtime.get("canary", config.get("canary", False)))

    blackboard.update_world(
        perception={
            "detection_confidence": float(scene.get("detection_confidence", 1.0 if scene.get("object_pose") else 0.0)),
            "tracking_lost": bool(scene.get("tracking_lost", False)),
            "depth_anomaly": bool(scene.get("depth_anomaly", False)),
            "calibration_version": str(scene.get("calibration_version", "static-sdk")),
        },
        scene={
            "object_pose": scene.get("object_pose"),
            "container_pose": scene.get("container_pose"),
            "place_pose": poses.get("place_pose"),
            "grasped": bool(scene.get("grasped", False)),
            "contact_state": str(scene.get("contact_state", "nominal")),
            "workspace_ready": bool(scene.get("workspace_ready", True)),
        },
        execution={
            "task_goal": dict(task_goal),
            "active_source": str(execution.get("active_source", "policy")),
            "control_owner": str(execution.get("control_owner", "script_runtime")),
            "task_contract": task_contract,
            "suite_role": suite_role,
            "probe_type": probe_type,
            "gate": gate,
            "canary": canary,
        },
    )
    blackboard.set("task_contract", task_contract)
    blackboard.set("suite_role", suite_role)
    blackboard.set("probe_type", probe_type)
    blackboard.set("is_gate_run", gate)
    blackboard.set("is_canary_run", canary)

    for key in ("home_joints", "reset_joints", "pregrasp_pose", "lift_pose", "place_pose", "retreat_pose"):
        if key in poses:
            blackboard.set(key, poses[key])
    if "place_release_pose" in poses:
        blackboard.set("place_release_pose", poses["place_release_pose"])
    elif "place_pose" in poses:
        blackboard.set("place_release_pose", poses["place_pose"])

    if "object_pose" in scene:
        blackboard.set("object_pose", scene["object_pose"])
    if "grasp_candidates" in scene:
        blackboard.set("grasp_candidates", scene["grasp_candidates"])
    if "servo_delta" in poses:
        blackboard.set("servo_delta", poses["servo_delta"])
    if "open_width" in gripper:
        blackboard.set("open_gripper_width", gripper["open_width"])
    if "close_width" in gripper:
        blackboard.set("close_gripper_width", gripper["close_width"])
    if runtime.get("artifact_dir"):
        blackboard.set("runtime_artifact_dir", runtime["artifact_dir"])
    if "pregrasp_distance" in robotwin:
        blackboard.set("pregrasp_distance", robotwin["pregrasp_distance"])
    if grasp_semantics:
        if "required" in grasp_semantics:
            blackboard.set("semantic_grasp_required", bool(grasp_semantics.get("required")))
        if "required_affordances" in grasp_semantics:
            blackboard.set("required_grasp_affordances", list(grasp_semantics.get("required_affordances") or []))
        if "incompatible_affordances" in grasp_semantics:
            blackboard.set(
                "incompatible_grasp_affordances",
                list(grasp_semantics.get("incompatible_affordances") or []),
            )
        if "visual_review_required" in grasp_semantics:
            blackboard.set("visual_review_required", bool(grasp_semantics.get("visual_review_required")))
        if "overrides" in grasp_semantics:
            blackboard.set("grasp_affordance_overrides", list(grasp_semantics.get("overrides") or []))
    return blackboard


@dataclass
class ScriptRuntimeSession:
    """Runtime session that assembles executor, registry, adapters, and trace."""

    task: Any
    registry: Any
    blackboard: TaskBlackboard
    adapters: Dict[str, Any]
    trace_recorder: TraceRecorder
    trace_path: Optional[Path] = None
    artifact_dir: Optional[Path] = None
    task_id: str = ""
    runtime_artifacts: Dict[str, Any] = None
    export_artifacts: bool = True
    write_trace: bool = True

    def run(self):
        task_id = self.task_id or f"{self.task.name}-{uuid.uuid4().hex[:8]}"
        self.blackboard.update_world(execution={"task_id": task_id})
        run_output_dir = self._resolve_run_output_dir(task_id)
        actual_trace_path = self._resolve_trace_output_path(task_id, run_output_dir)
        if actual_trace_path is not None:
            self.trace_path = actual_trace_path
        sdk = self.adapters.get("sdk")
        if sdk is not None:
            init_result = sdk.initialize()
            if not init_result.get("ok", False):
                raise RuntimeError(f"Failed to initialize SDK runtime: {init_result}")
            if hasattr(sdk, "attach_blackboard"):
                sdk.attach_blackboard(self.blackboard)
            set_pending_refresh_reason(self.blackboard, "session_initialize")
            sdk.refresh_world(self.blackboard)
            clear_pending_refresh_reason(self.blackboard)

        root = self.task.root or self.task.build(self.registry)
        executor = TreeExecutor(root=root, registry=self.registry, trace_recorder=self.trace_recorder)
        context = SkillContext(
            blackboard=self.blackboard,
            adapters=self.adapters,
            task_id=task_id,
            metadata={"session_type": "sdk_first"},
        )
        result = executor.run(context)
        if sdk is not None:
            set_pending_refresh_reason(self.blackboard, "session_finalize")
            sdk.refresh_world(self.blackboard)
            clear_pending_refresh_reason(self.blackboard)
        if self.write_trace and self.trace_path is not None:
            self.trace_path.parent.mkdir(parents=True, exist_ok=True)
            self.trace_path.write_text(self.trace_recorder.to_jsonl(), encoding="utf-8")
        self.runtime_artifacts = {}
        if run_output_dir is not None:
            self.runtime_artifacts["run_dir"] = str(run_output_dir)
        if self.write_trace and self.trace_path is not None:
            self.runtime_artifacts["trace_path"] = str(self.trace_path)
        if self.export_artifacts and sdk is not None and hasattr(sdk, "export_episode_artifacts"):
            artifact_outputs = dict(sdk.export_episode_artifacts(task_id=task_id, output_dir=run_output_dir) or {})
            self.runtime_artifacts.update(artifact_outputs)
        return result

    def shutdown(self) -> Dict[str, Any]:
        sdk = self.adapters.get("sdk")
        if sdk is None:
            return {"ok": True, "skipped": True}
        return sdk.shutdown()

    def _resolve_run_output_dir(self, task_id: str) -> Optional[Path]:
        base_dir = self.artifact_dir
        if base_dir is None and self.trace_path is not None:
            base_dir = self.trace_path.parent
        if base_dir is None:
            return None
        run_dir = Path(base_dir) / task_id
        run_dir.mkdir(parents=True, exist_ok=True)
        return run_dir

    def _resolve_trace_output_path(self, task_id: str, run_output_dir: Optional[Path]) -> Optional[Path]:
        if self.trace_path is None and run_output_dir is None:
            return None
        if run_output_dir is None:
            return self.trace_path
        if self.trace_path is None:
            return run_output_dir / f"{task_id}_trace.jsonl"
        return run_output_dir / self.trace_path.name


def build_pick_place_session(
    config: Dict[str, Any],
    sdk_bridge: Optional[SDKBridge] = None,
    registry: Optional[Any] = None,
    use_mock: bool = False,
) -> ScriptRuntimeSession:
    world = WorldState()
    blackboard = TaskBlackboard(world)
    seed_pick_place_blackboard(blackboard, config)

    runtime = config.get("runtime", {})
    task = build_task_from_config(config)
    registry = registry or build_default_skill_registry()
    trace_path = runtime.get("trace_path")

    if sdk_bridge is None:
        sdk_bridge = MockSDKBridge() if use_mock else _build_sdk_bridge_from_config(config)

    adapters = {
        "sdk": sdk_bridge,
        "learned": NullLearnedModuleAdapter(),
        "perception": _build_perception_adapter_from_config(config, sdk_bridge=sdk_bridge, robotwin=False),
        "place_module": _build_place_module_from_config(config),
    }
    return ScriptRuntimeSession(
        task=task,
        registry=registry,
        blackboard=blackboard,
        adapters=adapters,
        trace_recorder=TraceRecorder(),
        trace_path=Path(trace_path) if trace_path else None,
        artifact_dir=Path(runtime["artifact_dir"]) if runtime.get("artifact_dir") else None,
        task_id=str(runtime.get("task_id", "")),
        runtime_artifacts={},
        export_artifacts=bool(runtime.get("export_artifacts", True)),
        write_trace=bool(runtime.get("write_trace", True)),
    )


def build_robotwin_pick_place_session(
    config: Dict[str, Any],
    registry: Optional[Any] = None,
) -> ScriptRuntimeSession:
    world = WorldState()
    blackboard = TaskBlackboard(world)
    seed_pick_place_blackboard(blackboard, config)

    runtime = config.get("runtime", {})
    task = build_task_from_config(config)
    registry = registry or build_default_skill_registry()
    trace_path = runtime.get("trace_path")
    sdk_bridge = _build_robotwin_bridge_from_config(config)

    adapters = {
        "sdk": sdk_bridge,
        "learned": NullLearnedModuleAdapter(),
        "camera": sdk_bridge,
        "perception": _build_perception_adapter_from_config(config, sdk_bridge=sdk_bridge, robotwin=True),
        "place_module": _build_place_module_from_config(config),
    }
    return ScriptRuntimeSession(
        task=task,
        registry=registry,
        blackboard=blackboard,
        adapters=adapters,
        trace_recorder=TraceRecorder(),
        trace_path=Path(trace_path) if trace_path else None,
        artifact_dir=Path(runtime["artifact_dir"]) if runtime.get("artifact_dir") else None,
        task_id=str(runtime.get("task_id", "")),
        runtime_artifacts={},
        export_artifacts=bool(runtime.get("export_artifacts", True)),
        write_trace=bool(runtime.get("write_trace", True)),
    )


def _build_sdk_bridge_from_config(config: Dict[str, Any]) -> SDKBridge:
    from script_runtime.adapters.sdk_bridge import CArmSDKBridge

    robot = config.get("robot", {})
    gripper = config.get("gripper", {})
    runtime = config.get("runtime", {})
    return CArmSDKBridge(
        robot_ip=str(robot["robot_ip"]),
        port=int(robot.get("port", 8090)),
        timeout=float(robot.get("timeout", 1.0)),
        control_mode=int(robot.get("control_mode", 1)),
        speed_level=float(robot.get("speed_level", 3.0)),
        response_level=int(robot.get("response_level", 20)),
        tool_index=robot.get("tool_index"),
        collision_enable=robot.get("collision_enable"),
        collision_sensitivity=int(robot.get("collision_sensitivity", 0)),
        ready_on_initialize=bool(runtime.get("ready_on_initialize", True)),
        servo_enable_on_initialize=bool(runtime.get("servo_enable_on_initialize", True)),
        open_width=float(gripper.get("open_width", 0.08)),
        close_width=float(gripper.get("close_width", 0.0)),
        gripper_tau=float(gripper.get("tau", 10.0)),
        use_linear_for_move_l=bool(robot.get("use_linear_for_move_l", True)),
        sync=bool(robot.get("sync", True)),
    )


def _build_perception_adapter_from_config(
    config: Dict[str, Any],
    *,
    sdk_bridge: Any | None,
    robotwin: bool,
) -> Any:
    runtime = config.get("runtime", {})
    stack_cfg = dict(config.get("perception_stack") or runtime.get("perception_stack") or {})
    stack_type = str(stack_cfg.get("type", "") or "").strip().lower()

    if stack_type in {"fm", "fm_first", "fm-first", "foundation_model"}:
        return _build_fm_first_grasp_stack_from_config(config, sdk_bridge=sdk_bridge, robotwin=robotwin)

    if robotwin:
        return RoboTwinDepthPoseProvider(
            oracle_backend=sdk_bridge,
            use_oracle_fallback=bool(runtime.get("perception_oracle_fallback", True)),
        )
    if sdk_bridge is not None:
        return OraclePerceptionAdapter(sdk_bridge)
    return NullPerceptionAdapter()


def _build_fm_first_grasp_stack_from_config(
    config: Dict[str, Any],
    *,
    sdk_bridge: Any | None,
    robotwin: bool,
) -> FMFirstGraspStackAdapter:
    runtime = config.get("runtime", {})
    stack_cfg = dict(config.get("perception_stack") or runtime.get("perception_stack") or {})
    repos = dict(stack_cfg.get("repos") or {})
    enabled = dict(stack_cfg.get("enabled") or {})
    grounded_cfg = dict(stack_cfg.get("grounded_sam2") or {})
    foundationpose_cfg = dict(stack_cfg.get("foundationpose") or {})
    contact_graspnet_cfg = dict(stack_cfg.get("contact_graspnet") or {})

    depth_provider = None
    if robotwin:
        depth_provider = RoboTwinDepthPoseProvider(
            oracle_backend=sdk_bridge,
            use_oracle_fallback=bool(runtime.get("perception_oracle_fallback", True)),
            foreground_offset_mm=float(stack_cfg.get("foreground_offset_mm", 18.0)),
            min_component_area=int(stack_cfg.get("min_component_area", 120)),
            backend_candidate_reads=int(stack_cfg.get("backend_candidate_reads", 2)),
        )

    return build_default_fm_first_grasp_stack(
        oracle_backend=sdk_bridge,
        robotwin_depth_provider=depth_provider,
        grounded_sam2_repo=repos.get("grounded_sam2"),
        grounded_sam2_model_id=str(grounded_cfg.get("model_id", "IDEA-Research/grounding-dino-tiny")),
        grounded_sam2_box_threshold=float(grounded_cfg.get("box_threshold", 0.35)),
        grounded_sam2_text_threshold=float(grounded_cfg.get("text_threshold", 0.25)),
        grounded_sam2_max_detections=int(grounded_cfg.get("max_detections", 8)),
        grounded_sam2_device=grounded_cfg.get("device"),
        foundationpose_repo=repos.get("foundationpose"),
        foundationpose_python_bin=foundationpose_cfg.get("python_bin"),
        foundationpose_timeout_s=int(foundationpose_cfg.get("timeout_s", 180)),
        contact_graspnet_repo=repos.get("contact_graspnet"),
        contact_graspnet_python_bin=contact_graspnet_cfg.get("python_bin"),
        contact_graspnet_timeout_s=int(contact_graspnet_cfg.get("timeout_s", 180)),
        contact_graspnet_max_candidates=int(contact_graspnet_cfg.get("max_candidates", 12)),
        graspnet_repo=repos.get("graspnet_baseline"),
        graspgen_repo=repos.get("graspgen"),
        include_grounded_sam2=bool(enabled.get("grounded_sam2", True)),
        include_task_goal_grounder=bool(enabled.get("task_goal_grounder", True)),
        include_foundationpose=bool(enabled.get("foundationpose", True)),
        include_contact_graspnet=bool(enabled.get("contact_graspnet", True)),
        include_graspnet_baseline=bool(enabled.get("graspnet_baseline", False)),
        include_graspgen=bool(enabled.get("graspgen", False)),
        include_oracle_pose=bool(enabled.get("oracle_pose", True)),
        include_oracle_grasp=bool(enabled.get("oracle_grasp", True)),
        include_depth_pose=bool(enabled.get("robotwin_depth_pose", robotwin)),
        include_depth_grasp=bool(enabled.get("robotwin_depth_grasp", robotwin)),
    )


def _build_place_module_from_config(config: Dict[str, Any]) -> Any:
    runtime = config.get("runtime", {})
    module_cfg = dict(config.get("place_module") or runtime.get("place_module") or {})
    module_type = str(module_cfg.get("type", "heuristic")).strip().lower()
    if module_type in {"", "heuristic", "default"}:
        return HeuristicPlaceModule()
    if module_type == "closed_loop":
        return ClosedLoopPlaceModule(
            max_alignment_steps=int(module_cfg.get("max_alignment_steps", 3)),
            xy_gain=float(module_cfg.get("xy_gain", 0.45)),
            z_gain=float(module_cfg.get("z_gain", 0.2)),
            xy_step_limit=float(module_cfg.get("xy_step_limit", 0.03)),
            z_step_limit=float(module_cfg.get("z_step_limit", 0.01)),
            alignment_speed=float(module_cfg.get("alignment_speed", 0.12)),
            target_xy_tolerance=float(module_cfg.get("target_xy_tolerance", 0.03)),
            target_z_tolerance=float(module_cfg.get("target_z_tolerance", 0.015)),
            min_xy_improvement=float(module_cfg.get("min_xy_improvement", 0.0001)),
            max_xy_worsening=float(module_cfg.get("max_xy_worsening", 0.002)),
            response_prior=float(module_cfg.get("response_prior", 0.18)),
            response_ridge=float(module_cfg.get("response_ridge", 0.06)),
            response_ema=float(module_cfg.get("response_ema", 0.6)),
            response_diag_min=float(module_cfg.get("response_diag_min", 0.03)),
            response_diag_max=float(module_cfg.get("response_diag_max", 0.75)),
            response_cross_limit=float(module_cfg.get("response_cross_limit", 0.25)),
        )
    raise ValueError(f"Unsupported place_module.type: {module_type}")


def _build_robotwin_bridge_from_config(config: Dict[str, Any]) -> RoboTwinBridge:
    robotwin = config.get("robotwin", {})
    runtime = config.get("runtime", {})
    object_functional_point_id = robotwin.get("object_functional_point_id", 0)
    return RoboTwinBridge(
        task_name=str(robotwin.get("task_name", "place_empty_cup")),
        task_config=str(robotwin.get("task_config", "demo_clean")),
        robotwin_root=robotwin.get("root"),
        seed=int(robotwin.get("seed", 0)),
        episode_index=int(robotwin.get("episode_index", 0)),
        render_freq=int(robotwin.get("render_freq", 0)),
        use_seed=bool(robotwin.get("use_seed", True)),
        collect_data=bool(robotwin.get("collect_data", False)),
        save_data=bool(robotwin.get("save_data", False)),
        eval_mode=bool(robotwin.get("eval_mode", True)),
        save_path=runtime.get("artifact_dir"),
        object_attr=str(robotwin.get("object_attr", "cup")),
        target_attr=str(robotwin.get("target_attr", "coaster")),
        target_pose_attr=robotwin.get("target_pose_attr"),
        target_functional_point_id=int(robotwin.get("target_functional_point_id", 0)),
        object_functional_point_id=None if object_functional_point_id is None else int(object_functional_point_id),
        pregrasp_distance=float(robotwin.get("pregrasp_distance", 0.1)),
        place_approach_distance=float(robotwin.get("place_approach_distance", 0.1)),
        place_release_distance=float(robotwin.get("place_release_distance", 0.02)),
        retreat_distance=float(robotwin.get("retreat_distance", 0.08)),
        place_constrain=str(robotwin.get("place_constrain", "auto")),
        camera_shader_dir=str(robotwin.get("camera_shader_dir", "default")),
        ray_tracing_denoiser=robotwin.get("ray_tracing_denoiser"),
        ray_tracing_samples_per_pixel=int(robotwin.get("ray_tracing_samples_per_pixel", 32)),
        ray_tracing_path_depth=int(robotwin.get("ray_tracing_path_depth", 8)),
        capture_video=bool(robotwin.get("capture_video", True)),
        capture_every_n_steps=int(robotwin.get("capture_every_n_steps", 1)),
        capture_skills=list(robotwin.get("capture_skills", [])) or None,
        capture_command_types=list(robotwin.get("capture_command_types", [])) or None,
        episode_name=str(robotwin.get("episode_name", "robotwin_pick_place")),
        active_arm=robotwin.get("active_arm"),
    )
