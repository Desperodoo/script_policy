"""Export per-skill real-view snapshots from aligned RoboTwin rollout artifacts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Sequence


DEFAULT_SKILLS = [
    "GoPregrasp",
    "ExecuteGraspPhase",
    "Lift",
    "PlaceApproach",
    "PlaceRelease",
    "OpenGripper",
    "CheckTaskSuccess",
]


def _load_grounding_rows(path: Path) -> List[Dict[str, Any]]:
    return json.loads(path.read_text(encoding="utf-8"))


def _load_gif_frames(path: Path) -> List[Any]:
    from PIL import Image

    image = Image.open(path)
    frames = []
    try:
        while True:
            frames.append(image.convert("RGB").copy())
            image.seek(image.tell() + 1)
    except EOFError:
        pass
    return frames


def export_skill_snapshots(
    gif_path: Path,
    grounding_path: Path,
    output_dir: Path,
    skills: Sequence[str],
) -> Dict[str, Any]:
    from PIL import Image, ImageDraw

    output_dir.mkdir(parents=True, exist_ok=True)
    rows = _load_grounding_rows(grounding_path)
    frames = _load_gif_frames(gif_path)
    if not frames:
        raise RuntimeError(f"No frames found in {gif_path}")

    selected: List[Dict[str, Any]] = []
    for skill in skills:
        match_index = next((index for index, row in enumerate(rows) if row.get("skill") == skill), None)
        if match_index is None or match_index >= len(frames):
            continue
        row = rows[match_index]
        frame = frames[match_index]
        file_name = f"{gif_path.stem}_{skill.lower()}.png"
        frame_path = output_dir / file_name
        frame.save(frame_path)
        selected.append(
            {
                "skill": skill,
                "frame_index": match_index,
                "frame_path": str(frame_path),
                "command_type": (row.get("command") or {}).get("type", ""),
                "is_grasped": bool(row.get("is_grasped", False)),
                "success": row.get("success", ""),
            }
        )

    manifest_path = output_dir / f"{gif_path.stem}_skill_snapshots.json"
    manifest_path.write_text(json.dumps(selected, ensure_ascii=False, indent=2), encoding="utf-8")

    outputs: Dict[str, Any] = {
        "manifest_json": str(manifest_path),
        "snapshot_count": len(selected),
    }
    if not selected:
        return outputs

    thumbs = [Image.open(item["frame_path"]).convert("RGB") for item in selected]
    thumb_w = 420
    thumb_h = max(1, int(thumbs[0].height * (thumb_w / thumbs[0].width)))
    label_h = 30
    header_h = 46
    canvas = Image.new("RGB", (thumb_w * len(thumbs), thumb_h + label_h + header_h), (248, 249, 251))
    draw = ImageDraw.Draw(canvas)
    draw.text((16, 14), f"RoboTwin Skill Snapshots: {gif_path.stem}", fill=(25, 32, 44))
    for idx, (thumb, item) in enumerate(zip(thumbs, selected)):
        x0 = idx * thumb_w
        canvas.paste(thumb.resize((thumb_w, thumb_h)), (x0, header_h))
        label = f"{item['frame_index']}: {item['skill']}"
        draw.text((x0 + 10, header_h + thumb_h + 6), label, fill=(25, 32, 44))
    sheet_path = output_dir / f"{gif_path.stem}_skill_snapshots.png"
    canvas.save(sheet_path)
    outputs["contact_sheet_png"] = str(sheet_path)
    return outputs


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Export per-skill real-view snapshots from RoboTwin rollout GIF.")
    parser.add_argument("--gif", required=True, help="Path to rollout GIF.")
    parser.add_argument("--grounding-json", required=True, help="Path to grounding JSON.")
    parser.add_argument(
        "--out-dir",
        default="script_runtime/artifacts/skill_snapshots",
        help="Directory to store per-skill snapshots.",
    )
    parser.add_argument(
        "--skills",
        default=",".join(DEFAULT_SKILLS),
        help="Comma-separated skills to export.",
    )
    return parser


def main() -> int:
    args = build_argparser().parse_args()
    skills = [part.strip() for part in args.skills.split(",") if part.strip()]
    outputs = export_skill_snapshots(
        gif_path=Path(args.gif),
        grounding_path=Path(args.grounding_json),
        output_dir=Path(args.out_dir),
        skills=skills,
    )
    print(json.dumps(outputs, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
