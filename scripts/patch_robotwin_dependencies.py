from __future__ import annotations

from pathlib import Path

import mplib
import sapien


def patch_sapien() -> None:
    urdf_loader = Path(sapien.__file__).resolve().parent / "wrapper" / "urdf_loader.py"
    text = urdf_loader.read_text(encoding="utf-8")
    updated = text.replace('with open(urdf_file, "r") as f:', 'with open(urdf_file, "r", encoding="utf-8") as f:')
    updated = updated.replace('srdf_file = urdf_file[:-4] + "srdf"', 'srdf_file = urdf_file[:-4] + ".srdf"')
    updated = updated.replace('with open(srdf_file, "r") as f:', 'with open(srdf_file, "r", encoding="utf-8") as f:')
    if updated != text:
        urdf_loader.write_text(updated, encoding="utf-8")
        print(f"[patch] Updated {urdf_loader}")
    else:
        print(f"[patch] No changes needed for {urdf_loader}")


def patch_mplib() -> None:
    planner = Path(mplib.__file__).resolve().parent / "planner.py"
    text = planner.read_text(encoding="utf-8")
    before = 'if np.linalg.norm(delta_twist) < 1e-4 or collide or not within_joint_limit:'
    after = 'if np.linalg.norm(delta_twist) < 1e-4 or not within_joint_limit:'
    updated = text.replace(before, after)
    if updated != text:
        planner.write_text(updated, encoding="utf-8")
        print(f"[patch] Updated {planner}")
    else:
        print(f"[patch] No changes needed for {planner}")


if __name__ == "__main__":
    patch_sapien()
    patch_mplib()
