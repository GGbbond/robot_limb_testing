#!/usr/bin/env python3
"""Generate lightweight visual-only STL meshes for the embedded viewport."""

import argparse
from pathlib import Path
import shutil
import struct

import numpy as np


STL_DTYPE = np.dtype([
    ("normal", "<f4", (3,)),
    ("vertices", "<f4", (3, 3)),
    ("attribute", "<u2"),
])


def read_binary_stl(path):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError("STL 文件过短：%s" % path)
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_count * STL_DTYPE.itemsize
    if len(data) != expected_size:
        raise ValueError("只支持二进制 STL：%s" % path)
    triangles = np.frombuffer(
        data, dtype=STL_DTYPE, count=triangle_count, offset=84)
    return triangles["vertices"].astype(np.float64)


def clustered_triangles(triangles, resolution):
    points = triangles.reshape(-1, 3)
    lower = points.min(axis=0)
    span = points.max(axis=0) - lower
    cell_size = max(float(np.max(span)) / float(resolution), 1.0e-9)
    cells = np.rint((points - lower) / cell_size).astype(np.int32)
    unique_cells, inverse = np.unique(cells, axis=0, return_inverse=True)
    vertices = np.zeros((len(unique_cells), 3), dtype=np.float64)
    np.add.at(vertices, inverse, points)
    vertices /= np.bincount(inverse)[:, None]
    faces = inverse.reshape(-1, 3)
    nondegenerate = (
        (faces[:, 0] != faces[:, 1]) &
        (faces[:, 1] != faces[:, 2]) &
        (faces[:, 0] != faces[:, 2])
    )
    faces = faces[nondegenerate]
    sorted_faces = np.sort(faces, axis=1)
    _keys, first_indices = np.unique(
        sorted_faces, axis=0, return_index=True)
    faces = faces[np.sort(first_indices)]
    result = vertices[faces]
    cross = np.cross(result[:, 1] - result[:, 0],
                     result[:, 2] - result[:, 0])
    area = np.linalg.norm(cross, axis=1)
    return result[area > 1.0e-12]


def write_binary_stl(path, triangles, source_name):
    edges_1 = triangles[:, 1] - triangles[:, 0]
    edges_2 = triangles[:, 2] - triangles[:, 0]
    normals = np.cross(edges_1, edges_2)
    lengths = np.linalg.norm(normals, axis=1)
    normals /= lengths[:, None]
    records = np.zeros(len(triangles), dtype=STL_DTYPE)
    records["normal"] = normals.astype(np.float32)
    records["vertices"] = triangles.astype(np.float32)
    header = ("BXI lightweight viewport mesh: " + source_name).encode("ascii")
    with open(path, "wb") as stream:
        stream.write(header[:80].ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(records)))
        stream.write(records.tobytes())


def generate(root, resolution=24, preserve_below=8000):
    data_dir = root / "src" / "bxi_example_py_elf3" / "data"
    source_dir = data_dir / "meshes"
    output_dir = data_dir / "meshes_view"
    output_dir.mkdir(parents=True, exist_ok=True)
    total_before = 0
    total_after = 0
    for source in sorted(source_dir.glob("*.STL")):
        destination = output_dir / source.name
        triangles = read_binary_stl(source)
        total_before += len(triangles)
        if len(triangles) <= preserve_below:
            shutil.copyfile(source, destination)
            reduced_count = len(triangles)
        else:
            reduced = clustered_triangles(triangles, resolution)
            write_binary_stl(destination, reduced, source.name)
            reduced_count = len(reduced)
        total_after += reduced_count
        print("%-28s %7d -> %7d" % (
            source.name, len(triangles), reduced_count))

    source_xml = data_dir / "elf3.xml"
    view_xml = data_dir / "elf3_view.xml"
    text = source_xml.read_text(encoding="utf-8")
    text = text.replace('meshdir="./meshes"', 'meshdir="./meshes_view"', 1)
    view_xml.write_text(text, encoding="utf-8")
    print("total triangles: %d -> %d (%.1f%%)" % (
        total_before, total_after, 100.0 * total_after / total_before))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--resolution", type=int, default=24)
    parser.add_argument("--preserve-below", type=int, default=8000)
    options = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    generate(root, options.resolution, options.preserve_below)


if __name__ == "__main__":
    main()
