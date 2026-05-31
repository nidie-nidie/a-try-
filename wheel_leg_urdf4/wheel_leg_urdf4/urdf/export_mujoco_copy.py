from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SOURCE_URDF = PACKAGE_DIR / "urdf" / "wheel_leg_urdf4.urdf"
SOURCE_MESH_DIR = PACKAGE_DIR / "meshes"
OUTPUT_DIR = PACKAGE_DIR / "mujoco_export"
OUTPUT_MESH_DIR = OUTPUT_DIR / "meshes"
OUTPUT_URDF = OUTPUT_DIR / "wheel_leg_urdf4_export.urdf"


def rebuild_mesh_links() -> None:
    OUTPUT_MESH_DIR.mkdir(parents=True, exist_ok=True)
    for entry in OUTPUT_MESH_DIR.iterdir():
        if entry.is_symlink() or entry.is_file():
            entry.unlink()

    for mesh in sorted(SOURCE_MESH_DIR.glob("*.STL")):
        target = OUTPUT_MESH_DIR / mesh.name
        os.symlink(mesh, target)


def build_urdf_copy() -> None:
    tree = ET.parse(SOURCE_URDF)
    root = tree.getroot()

    for link in root.findall("link"):
        name = link.get("name", "")
        if "dummy" not in name:
            continue
        for tag in ("visual", "collision"):
            for node in list(link.findall(tag)):
                link.remove(node)

    mujoco = root.find("mujoco")
    if mujoco is not None:
        root.remove(mujoco)

    mujoco = ET.Element("mujoco")
    compiler = ET.SubElement(mujoco, "compiler")
    compiler.set("meshdir", "meshes")
    compiler.set("strippath", "true")
    root.append(mujoco)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    tree.write(OUTPUT_URDF, encoding="utf-8", xml_declaration=True)


def main() -> None:
    rebuild_mesh_links()
    build_urdf_copy()
    print(OUTPUT_URDF)


if __name__ == "__main__":
    main()
