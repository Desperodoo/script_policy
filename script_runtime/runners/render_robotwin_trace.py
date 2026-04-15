"""Render lightweight visualization artifacts from a RoboTwin runtime trace."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Optional


def _safe_pose(value: Any) -> Optional[List[float]]:
    if isinstance(value, list) and len(value) >= 3:
        return [float(v) for v in value[:7]]
    return None


def _extract_timeline(trace_path: Path) -> List[Dict[str, Any]]:
    rows = [json.loads(line) for line in trace_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    timeline: List[Dict[str, Any]] = []
    object_pose: Optional[List[float]] = None
    goal_pose: Optional[List[float]] = None
    grasp_pose: Optional[List[float]] = None
    tcp_pose: Optional[List[float]] = None

    for row in rows:
        skill = row.get("skill_name", "")
        payload = row.get("payload", {}) or {}
        inputs = row.get("inputs_summary", {}) or {}
        payload_inputs = inputs.get("payload", {}) or {}

        if skill == "GetObjectPose":
            object_pose = _safe_pose(payload.get("object_pose"))
        if skill == "GetGraspCandidates":
            candidates = payload.get("grasp_candidates") or []
            if candidates:
                grasp_pose = _safe_pose((candidates[0] or {}).get("pose"))
        command = payload.get("command", {}) or {}
        if command.get("type") == "move_l":
            tcp_pose = _safe_pose(command.get("pose"))
        if skill in {"PlaceApproach", "PlaceRelease"}:
            goal_pose = _safe_pose(command.get("pose"))
        if skill == "ExecuteGraspPhase":
            tcp_pose = _safe_pose(inputs.get("eef_pose"))
        timeline.append(
            {
                "step": len(timeline),
                "skill": skill,
                "result": row.get("result"),
                "failure_code": row.get("failure_code"),
                "command_type": command.get("type", ""),
                "object_pose": object_pose,
                "goal_pose": goal_pose,
                "grasp_pose": grasp_pose,
                "tcp_pose": tcp_pose,
                "env_success": bool(((payload_inputs.get("env_result") or {}).get("success", False))),
            }
        )
    return timeline


def _render_frame(draw: Any, row: Dict[str, Any], width: int = 640, height: int = 480) -> None:
    x_min, x_max = -0.2, 0.7
    y_min, y_max = -0.35, 0.35
    panel_left, panel_top = 20, 80
    panel_w, panel_h = 600, 360

    def project(pose: Optional[List[float]]) -> Optional[tuple[int, int]]:
        if pose is None or len(pose) < 2:
            return None
        px = int(panel_left + ((float(pose[0]) - x_min) / (x_max - x_min)) * panel_w)
        py = int(panel_top + panel_h - ((float(pose[1]) - y_min) / (y_max - y_min)) * panel_h)
        return px, py

    draw.rectangle((panel_left, panel_top, panel_left + panel_w, panel_top + panel_h), outline=(120, 130, 140), width=2)
    draw.text((20, 16), "RoboTwin Trace Visualization", fill=(20, 20, 24))
    draw.text((20, 38), f"step={row['step']} skill={row['skill']}", fill=(35, 42, 54))
    draw.text((20, 58), f"result={row['result']} failure={row['failure_code']}", fill=(35, 42, 54))

    for pose, color, label in (
        (row.get("goal_pose"), (46, 125, 50), "goal"),
        (row.get("object_pose"), (198, 40, 40), "obj"),
        (row.get("tcp_pose"), (21, 101, 192), "tcp"),
        (row.get("grasp_pose"), (245, 124, 0), "grasp"),
    ):
        point = project(pose)
        if point is None:
            continue
        px, py = point
        draw.ellipse((px - 6, py - 6, px + 6, py + 6), fill=color, outline=(0, 0, 0))
        draw.text((px + 8, py - 8), label, fill=color)


def render_robotwin_trace(trace_path: Path, output_dir: Path) -> Dict[str, str]:
    from PIL import Image, ImageDraw

    output_dir.mkdir(parents=True, exist_ok=True)
    timeline = _extract_timeline(trace_path)
    stem = trace_path.stem.replace("_trace", "")

    timeline_path = output_dir / f"{stem}_timeline.json"
    timeline_path.write_text(json.dumps(timeline, ensure_ascii=False, indent=2), encoding="utf-8")

    frames = []
    for row in timeline:
        image = Image.new("RGB", (640, 480), (250, 250, 252))
        draw = ImageDraw.Draw(image)
        _render_frame(draw, row)
        frames.append(image)

    outputs = {"timeline_json": str(timeline_path)}
    if frames:
        gif_path = output_dir / f"{stem}_rollout.gif"
        frames[0].save(gif_path, save_all=True, append_images=frames[1:], duration=320, loop=0)
        outputs["rollout_gif"] = str(gif_path)

        summary_path = output_dir / f"{stem}_summary.png"
        frames[-1].save(summary_path)
        outputs["summary_png"] = str(summary_path)
    return outputs


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Render lightweight artifacts from RoboTwin trace JSONL.")
    parser.add_argument(
        "--trace",
        default="script_runtime/artifacts/robotwin_place_empty_cup_trace.jsonl",
        help="Path to trace JSONL.",
    )
    parser.add_argument(
        "--out-dir",
        default="script_runtime/artifacts/robotwin_trace_viz",
        help="Directory for rendered artifacts.",
    )
    return parser


def main() -> int:
    args = build_argparser().parse_args()
    outputs = render_robotwin_trace(trace_path=Path(args.trace), output_dir=Path(args.out_dir))
    for key, value in outputs.items():
        print(f"{key}={value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
