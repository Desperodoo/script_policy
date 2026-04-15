"""Standalone runtime session helpers for SDK-first script execution."""

from __future__ import annotations

import json
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

from script_runtime.adapters import (
    MockSDKBridge,
    NullLearnedModuleAdapter,
    NullPerceptionAdapter,
    OraclePerceptionAdapter,
    RoboTwinDepthPoseProvider,
    RoboTwinBridge,
    SDKBridge,
)
from script_runtime.core import SkillContext, TaskBlackboard, WorldState
from script_runtime.executors import TraceRecorder, TreeExecutor
from script_runtime.factory import build_default_skill_registry
from script_runtime.tasks.pick_place import PickPlaceTask


def load_runtime_config(config_path: str | Path) -> Dict[str, Any]:
    path = Path(config_path)
    with path.open("r", encoding="utf-8") as handle:
        if path.suffix == ".json":
            return json.load(handle)
        return yaml.safe_load(handle) or {}


def seed_pick_place_blackboard(blackboard: TaskBlackboard, config: Dict[str, Any]) -> TaskBlackboard:
    scene = config.get("scene", {})
    execution = config.get("execution", {})
    task_goal = config.get("task_goal", {})
    poses = config.get("poses", {})
    gripper = config.get("gripper", {})

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
        },
    )

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
            sdk.refresh_world(self.blackboard)

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
            sdk.refresh_world(self.blackboard)
        if self.trace_path is not None:
            self.trace_path.parent.mkdir(parents=True, exist_ok=True)
            self.trace_path.write_text(self.trace_recorder.to_jsonl(), encoding="utf-8")
        self.runtime_artifacts = {}
        if run_output_dir is not None:
            self.runtime_artifacts["run_dir"] = str(run_output_dir)
        if self.trace_path is not None:
            self.runtime_artifacts["trace_path"] = str(self.trace_path)
        if sdk is not None and hasattr(sdk, "export_episode_artifacts"):
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
    task = PickPlaceTask(goal=config.get("task_goal", {}))
    registry = registry or build_default_skill_registry()
    trace_path = runtime.get("trace_path")

    if sdk_bridge is None:
        sdk_bridge = MockSDKBridge() if use_mock else _build_sdk_bridge_from_config(config)

    adapters = {
        "sdk": sdk_bridge,
        "learned": NullLearnedModuleAdapter(),
        "perception": OraclePerceptionAdapter(sdk_bridge) if sdk_bridge is not None else NullPerceptionAdapter(),
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
    )


def build_robotwin_pick_place_session(
    config: Dict[str, Any],
    registry: Optional[Any] = None,
) -> ScriptRuntimeSession:
    world = WorldState()
    blackboard = TaskBlackboard(world)
    seed_pick_place_blackboard(blackboard, config)

    runtime = config.get("runtime", {})
    task = PickPlaceTask(goal=config.get("task_goal", {}))
    registry = registry or build_default_skill_registry()
    trace_path = runtime.get("trace_path")
    sdk_bridge = _build_robotwin_bridge_from_config(config)

    adapters = {
        "sdk": sdk_bridge,
        "learned": NullLearnedModuleAdapter(),
        "camera": sdk_bridge,
        "perception": RoboTwinDepthPoseProvider(
            oracle_backend=sdk_bridge,
            use_oracle_fallback=bool(runtime.get("perception_oracle_fallback", True)),
        ),
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
