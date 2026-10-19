"""Read the supplied numeric Xacro origins and STL bounds, without executing files."""
import ast
import math
import re
import struct
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
ROOT = Path(__file__).resolve().parents[5]
SOURCE = ROOT / "assets/landerpi/source_description"


def numeric_expression(text):
    """Only arithmetic on literal numbers; never execute xacro/file code."""
    node = ast.parse(text, mode="eval").body

    def walk(n):
        if isinstance(n, ast.Constant) and type(n.value) in (int, float):
            return n.value
        if isinstance(n, ast.UnaryOp) and isinstance(n.op, (ast.UAdd, ast.USub)):
            return walk(n.operand) * (-1 if isinstance(n.op, ast.USub) else 1)
        if isinstance(n, ast.BinOp) and isinstance(n.op, (ast.Add, ast.Sub, ast.Mult, ast.Div)):
            a, b = walk(n.left), walk(n.right)
            if isinstance(n.op, ast.Add):
                return a + b
            if isinstance(n.op, ast.Sub):
                return a - b
            if isinstance(n.op, ast.Mult):
                return a * b
            return a / b
        raise ValueError(f"Unsupported source expression: {text}")

    return float(walk(node))


def vector(text):
    text = re.sub(r"\$\{([^}]+)\}", lambda m: str(numeric_expression(m.group(1))), text)
    return np.array([float(x) for x in text.split()])


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def axis_matrix(axis, angle):
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew @ skew)


def fmt(values):
    return " ".join(f"{float(v):.12g}" for v in np.atleast_1d(values))


def quat(matrix):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(matrix, dtype=float).reshape(9))
    return fmt(q)


def write_xml(path, root):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def stl_bounds(path):
    """Axis-aligned bounds of the supplied binary STL, without CAD dependencies."""
    content = path.read_bytes()
    count = struct.unpack_from("<I", content, 80)[0]
    if len(content) != 84+50*count or count == 0:
        raise ValueError(f"Expected nonempty binary STL: {path}")
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3,3)), ("attribute", "<u2")])
    vertices = np.frombuffer(content,dtype=dtype,count=count,offset=84)["vertices"].reshape(-1,3)
    lower, upper = vertices.min(0), vertices.max(0)
    return (lower+upper)/2, np.maximum((upper-lower)/2,.0005)


def read_source():
    links, joints, groups = {}, {}, {}
    for filename, group in (("mecanum.xacro", "mecanum"), ("arm.urdf.xacro", "arm"), ("gripper.urdf.xacro", "gripper")):
        doc = ET.parse(SOURCE / "urdf" / filename).getroot()
        for link in doc.findall("link"):
            links[link.get("name")] = link
            groups[link.get("name")] = group
        for joint in doc.findall("joint"):
            joints[joint.find("child").get("link")] = joint
    return links, joints, groups


