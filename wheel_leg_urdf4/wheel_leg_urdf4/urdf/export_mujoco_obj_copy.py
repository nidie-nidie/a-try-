from __future__ import annotations

import os
import struct
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SOURCE_URDF = PACKAGE_DIR / "urdf" / "wheel_leg_urdf4.urdf"
SOURCE_MESH_DIR = PACKAGE_DIR / "meshes"
OUTPUT_DIR = PACKAGE_DIR / "mujoco_export_obj"
OUTPUT_MESH_DIR = OUTPUT_DIR / "meshes"
OUTPUT_URDF = OUTPUT_DIR / "wheel_leg_urdf4_export_obj.urdf"
OUTPUT_XML = OUTPUT_DIR / "wheel_leg_urdf4_export_obj.xml"
BASE_STL = SOURCE_MESH_DIR / "base_link.STL"
BASE_OBJ = OUTPUT_MESH_DIR / "base_link_original.obj"


def stl_to_obj(src: Path, dst: Path) -> None:
    data = src.read_bytes()
    face_count = int.from_bytes(data[80:84], "little")
    offset = 84
    with dst.open("w", encoding="utf-8") as f:
        vertex_id = 1
        for _ in range(face_count):
            offset += 12  # normal
            verts = []
            for _ in range(3):
                x, y, z = struct.unpack_from("<3f", data, offset)
                offset += 12
                verts.append((x, y, z))
            offset += 2  # attribute byte count
            for x, y, z in verts:
                f.write(f"v {x} {y} {z}\n")
            f.write(f"f {vertex_id} {vertex_id + 1} {vertex_id + 2}\n")
            vertex_id += 3


def rebuild_mesh_dir() -> None:
    OUTPUT_MESH_DIR.mkdir(parents=True, exist_ok=True)
    for entry in OUTPUT_MESH_DIR.iterdir():
        if entry.is_symlink() or entry.is_file():
            entry.unlink()

    stl_to_obj(BASE_STL, BASE_OBJ)

    for mesh in sorted(SOURCE_MESH_DIR.glob("*.STL")):
        name = mesh.name
        if name == "base_link.STL" or "dummy" in name:
            continue
        os.symlink(mesh, OUTPUT_MESH_DIR / name)


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

    base_link = root.find("./link[@name='base_link']")
    if base_link is not None:
        for tag in ("visual", "collision"):
            node = base_link.find(tag)
            if node is None:
                continue
            mesh = node.find("./geometry/mesh")
            if mesh is not None:
                mesh.set("filename", "base_link_original.obj")

    old = root.find("mujoco")
    if old is not None:
        root.remove(old)

    mujoco = ET.Element("mujoco")
    compiler = ET.SubElement(mujoco, "compiler")
    compiler.set("meshdir", "meshes")
    compiler.set("strippath", "true")
    root.append(mujoco)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    tree.write(OUTPUT_URDF, encoding="utf-8", xml_declaration=True)


def main() -> None:
    rebuild_mesh_dir()
    build_urdf_copy()
    print(OUTPUT_URDF)


if __name__ == "__main__":
    main()
