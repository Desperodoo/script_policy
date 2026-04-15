"""Compare RoboTwin depth-based pose estimate against oracle pose."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

from script_runtime.adapters import PerceptionObservation, RoboTwinDepthPoseProvider
from script_runtime.session import _build_robotwin_bridge_from_config, load_runtime_config


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Diagnose RoboTwin depth-based pose provider against oracle pose.")
    parser.add_argument(
        "--config",
        default="script_runtime/configs/tasks/place_empty_cup_robotwin.yaml",
        help="Path to runtime YAML config.",
    )
    parser.add_argument(
        "--out",
        default="script_runtime/artifacts/robotwin_pose_diagnostics.json",
        help="Output JSON path.",
    )
    return parser


def main() -> int:
    args = build_argparser().parse_args()
    config = load_runtime_config(args.config)
    sdk = _build_robotwin_bridge_from_config(config)
    provider = RoboTwinDepthPoseProvider(oracle_backend=sdk, use_oracle_fallback=False)
    try:
        init = sdk.initialize()
        if not init.get("ok", False):
            raise RuntimeError(f"Failed to initialize RoboTwin bridge: {init}")
        snapshot = sdk.get_snapshot()
        observation = PerceptionObservation(
            rgb=snapshot.rgb,
            depth=snapshot.depth,
            task_goal=dict(config.get("task_goal", {})),
            metadata=dict(snapshot.metadata),
        )
        component_diag = provider.export_component_diagnostics(observation)
        estimated_pose = provider.get_object_pose(observation)
        oracle_pose = sdk.get_object_pose()
        result = {
            "task_name": sdk.task_name,
            "task_config": sdk.task_config,
            "estimated_pose": estimated_pose,
            "oracle_pose": oracle_pose,
            "component_diagnostics": _sanitize_component_diagnostics(component_diag),
        }
        if estimated_pose is not None and oracle_pose is not None:
            result["translation_error_l2_m"] = sum(
                (float(estimated_pose[i]) - float(oracle_pose[i])) ** 2 for i in range(3)
            ) ** 0.5
        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        render_outputs = _render_component_visuals(
            snapshot_rgb=snapshot.rgb,
            snapshot_depth=snapshot.depth,
            component_diag=component_diag,
            output_root=out_path.with_suffix(""),
            estimated_pose=estimated_pose,
            oracle_pose=oracle_pose,
        )
        if render_outputs:
            result["visual_outputs"] = render_outputs
            out_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        sdk.shutdown()
    return 0


def _sanitize_component_diagnostics(component_diag: Dict[str, Any]) -> Dict[str, Any]:
    if not component_diag.get("ok", False):
        return dict(component_diag)
    sanitized = dict(component_diag)
    sanitized.pop("foreground_mask", None)
    return sanitized


def _render_component_visuals(
    snapshot_rgb: Any,
    snapshot_depth: Any,
    component_diag: Dict[str, Any],
    output_root: Path,
    estimated_pose: List[float] | None,
    oracle_pose: List[float] | None,
) -> Dict[str, str]:
    if not component_diag.get("ok", False):
        return {}
    try:
        from PIL import Image, ImageDraw
    except Exception:
        return {}

    rgb = np.asarray(snapshot_rgb) if snapshot_rgb is not None else None
    depth = np.asarray(snapshot_depth, dtype=np.float64) if snapshot_depth is not None else None
    foreground = np.asarray(component_diag.get("foreground_mask"))
    components = list(component_diag.get("components", []))
    if rgb is None or depth is None or foreground is None:
        return {}

    if rgb.dtype != np.uint8:
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
    if rgb.ndim == 3 and rgb.shape[2] > 3:
        rgb = rgb[:, :, :3]

    overlay = Image.fromarray(rgb)
    draw = ImageDraw.Draw(overlay)
    palette = [
        (229, 57, 53),
        (3, 155, 229),
        (67, 160, 71),
        (251, 140, 0),
        (142, 36, 170),
        (0, 137, 123),
        (124, 77, 255),
    ]
    for index, component in enumerate(components):
        x, y, w, h = [int(v) for v in component.get("bbox", [0, 0, 0, 0])]
        color = palette[index % len(palette)]
        width = 4 if component.get("selected", False) else 2
        draw.rectangle((x, y, x + w, y + h), outline=color, width=width)
        label = (
            f"#{index} score={component.get('score', 0.0):.1f} "
            f"dz={component.get('depth_gain_mm', 0.0):.1f}"
        )
        if component.get("world_centroid") is not None:
            centroid = component["world_centroid"]
            label += f" xyz=({centroid[0]:.2f},{centroid[1]:.2f},{centroid[2]:.2f})"
        draw.text((x + 4, max(y - 14, 2)), label, fill=color)

    mask_image = Image.fromarray((foreground.astype(np.uint8) * 255), mode="L").convert("RGB")
    depth_valid = np.isfinite(depth) & (depth > 1.0)
    depth_norm = np.zeros_like(depth, dtype=np.uint8)
    if np.count_nonzero(depth_valid) > 0:
        low = float(np.percentile(depth[depth_valid], 5.0))
        high = float(np.percentile(depth[depth_valid], 95.0))
        scale = max(high - low, 1e-6)
        depth_norm = np.clip((depth - low) / scale, 0.0, 1.0)
        depth_norm = (255.0 * (1.0 - depth_norm)).astype(np.uint8)
    depth_rgb = np.stack([depth_norm, depth_norm, depth_norm], axis=-1)
    depth_image = Image.fromarray(depth_rgb, mode="RGB")

    pad = 12
    canvas = Image.new("RGB", (overlay.width * 3 + pad * 4, overlay.height + 120), (248, 248, 250))
    canvas.paste(overlay, (pad, 56))
    canvas.paste(mask_image, (overlay.width + pad * 2, 56))
    canvas.paste(depth_image, (overlay.width * 2 + pad * 3, 56))
    cdraw = ImageDraw.Draw(canvas)
    cdraw.text((pad, 14), "RGB + component boxes", fill=(24, 24, 28))
    cdraw.text((overlay.width + pad * 2, 14), "Foreground mask", fill=(24, 24, 28))
    cdraw.text((overlay.width * 2 + pad * 3, 14), "Depth heatmap", fill=(24, 24, 28))
    cdraw.text(
        (pad, 34),
        f"estimated={_fmt_pose(estimated_pose)} oracle={_fmt_pose(oracle_pose)} "
        f"table_depth={component_diag.get('table_depth_mm', 0.0):.1f}mm",
        fill=(55, 55, 65),
    )

    image_path = output_root.parent / f"{output_root.name}_components.png"
    image_path.write_bytes(b"")
    canvas.save(image_path)

    components_path = output_root.parent / f"{output_root.name}_components.json"
    components_path.write_text(json.dumps(component_diag["components"], ensure_ascii=False, indent=2), encoding="utf-8")
    return {
        "components_png": str(image_path),
        "components_json": str(components_path),
    }


def _fmt_pose(pose: List[float] | None) -> str:
    if pose is None or len(pose) < 3:
        return "none"
    return f"({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f})"


if __name__ == "__main__":
    raise SystemExit(main())
