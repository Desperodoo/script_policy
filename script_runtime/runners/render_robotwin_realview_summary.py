"""Render a quick-look contact sheet from an existing real-view rollout GIF."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List


def _load_grounding_rows(path: Path | None) -> List[Dict[str, Any]]:
    if path is None or not path.exists():
        return []
    return json.loads(path.read_text(encoding="utf-8"))


def render_realview_summary(
    gif_path: Path,
    output_path: Path,
    grounding_path: Path | None = None,
    columns: int = 3,
) -> str:
    from PIL import Image, ImageDraw

    grounding_rows = _load_grounding_rows(grounding_path)
    image = Image.open(gif_path)
    frames: List[Image.Image] = []
    try:
        while True:
            frames.append(image.convert("RGB").copy())
            image.seek(image.tell() + 1)
    except EOFError:
        pass
    if not frames:
        raise RuntimeError(f"No frames found in {gif_path}")

    sample_count = min(6, len(frames))
    sample_indices = [round(index * (len(frames) - 1) / max(1, sample_count - 1)) for index in range(sample_count)]
    sampled_frames = [frames[index] for index in sample_indices]
    thumb_w = 420
    thumb_h = max(1, int(sampled_frames[0].height * (thumb_w / sampled_frames[0].width)))
    rows = (sample_count + columns - 1) // columns
    header_h = 48
    label_h = 28
    canvas = Image.new("RGB", (columns * thumb_w, rows * (thumb_h + label_h) + header_h), (248, 249, 251))
    draw = ImageDraw.Draw(canvas)
    draw.text((16, 16), f"RoboTwin Real-View Summary: {gif_path.name}", fill=(25, 32, 44))

    for slot, (frame_index, frame) in enumerate(zip(sample_indices, sampled_frames)):
        row = slot // columns
        col = slot % columns
        x0 = col * thumb_w
        y0 = header_h + row * (thumb_h + label_h)
        canvas.paste(frame.resize((thumb_w, thumb_h)), (x0, y0))
        if grounding_rows:
            grounding_index = min(frame_index, len(grounding_rows) - 1)
            grounding = grounding_rows[grounding_index]
            label = f"{frame_index}: {grounding.get('skill', 'idle')} | {grounding.get('command', {}).get('type', 'unknown')}"
        else:
            label = f"frame {frame_index}"
        draw.text((x0 + 10, y0 + thumb_h + 6), label, fill=(25, 32, 44))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_path)
    return str(output_path)


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Render contact sheet from an existing RoboTwin real-view GIF.")
    parser.add_argument("--gif", required=True, help="Path to rollout GIF.")
    parser.add_argument("--grounding-json", default="", help="Optional grounding JSON aligned to the rollout.")
    parser.add_argument(
        "--out",
        default="script_runtime/artifacts/robotwin_realview_summary.png",
        help="Output path for the summary PNG.",
    )
    return parser


def main() -> int:
    args = build_argparser().parse_args()
    grounding = Path(args.grounding_json) if args.grounding_json else None
    output = render_realview_summary(gif_path=Path(args.gif), grounding_path=grounding, output_path=Path(args.out))
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
