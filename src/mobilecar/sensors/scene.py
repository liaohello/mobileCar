"""Build the browser scene description from the compiled MuJoCo model."""

from __future__ import annotations

import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from ..contracts.models import MeshAsset, SceneGeom, SceneLink, SceneManifest, SceneTransform


GEOM_TYPES = {
    int(mujoco.mjtGeom.mjGEOM_PLANE): "plane",
    int(mujoco.mjtGeom.mjGEOM_SPHERE): "sphere",
    int(mujoco.mjtGeom.mjGEOM_CAPSULE): "capsule",
    int(mujoco.mjtGeom.mjGEOM_ELLIPSOID): "ellipsoid",
    int(mujoco.mjtGeom.mjGEOM_CYLINDER): "cylinder",
    int(mujoco.mjtGeom.mjGEOM_BOX): "box",
    int(mujoco.mjtGeom.mjGEOM_MESH): "mesh",
}
JOINT_TYPES = {
    int(mujoco.mjtJoint.mjJNT_FREE): "free",
    int(mujoco.mjtJoint.mjJNT_BALL): "ball",
    int(mujoco.mjtJoint.mjJNT_SLIDE): "slide",
    int(mujoco.mjtJoint.mjJNT_HINGE): "hinge",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _xyzw(quat_wxyz) -> list[float]:
    w, x, y, z = (float(value) for value in quat_wxyz)
    return [x, y, z, w]


def _transform(position, quaternion_wxyz) -> SceneTransform:
    return SceneTransform(
        position_m=[float(value) for value in position],
        orientation_xyzw=_xyzw(quaternion_wxyz),
    )


def _mesh_map(model_dir: Path, model: mujoco.MjModel) -> tuple[list[MeshAsset], dict[int, str], dict[str, Path]]:
    xml_path = model_dir / "assets.xml"
    root = ET.parse(xml_path).getroot()
    files: dict[str, Path] = {}
    for node in root.findall(".//mesh"):
        name = node.get("name")
        file_name = node.get("file")
        if name and file_name:
            path = (model_dir / file_name).resolve()
            if not path.is_file():
                raise FileNotFoundError(f"mesh asset missing: {path}")
            files[name] = path
    assets: list[MeshAsset] = []
    mesh_id_to_asset: dict[int, str] = {}
    by_name: dict[str, MeshAsset] = {}
    for mesh_id in range(model.nmesh):
        name = model.mesh(mesh_id).name
        path = files.get(name)
        if path is None:
            raise RuntimeError(f"mesh {name!r} has no assets.xml source")
        asset_id = name.removesuffix("_mesh") or name
        asset = MeshAsset(
            asset_id=asset_id,
            uri=f"/api/v1/scene/assets/{asset_id}",
            sha256=_sha256(path),
            media_type="model/stl",
        )
        by_name[name] = asset
        assets.append(asset)
        mesh_id_to_asset[mesh_id] = asset.asset_id
    return assets, mesh_id_to_asset, files


def _body_joint(model: mujoco.MjModel, body_id: int) -> tuple[int | None, str, list[float]]:
    start = int(model.body_jntadr[body_id])
    count = int(model.body_jntnum[body_id])
    if count <= 0:
        return None, "fixed", [0.0, 0.0, 1.0]
    joint_id = start
    joint_type = JOINT_TYPES.get(int(model.jnt_type[joint_id]), "fixed")
    axis = [float(value) for value in model.jnt_axis[joint_id]]
    return joint_id, joint_type, axis


def _body_geoms(
    model: mujoco.MjModel,
    body_id: int,
    mesh_ids: dict[int, str],
) -> list[SceneGeom]:
    start = int(model.body_geomadr[body_id])
    count = int(model.body_geomnum[body_id])
    result: list[SceneGeom] = []
    for geom_id in range(start, start + count):
        geom_type = GEOM_TYPES.get(int(model.geom_type[geom_id]))
        if geom_type is None:
            continue
        mesh_asset_id = None
        if geom_type == "mesh":
            data_id = int(model.geom_dataid[geom_id])
            mesh_asset_id = mesh_ids.get(data_id)
            if mesh_asset_id is None:
                continue
            size = [float(value) for value in model.mesh_scale[data_id]]
        else:
            size = [float(value) for value in model.geom_size[geom_id]]
        name = model.geom(geom_id).name or f"geom_{geom_id}"
        result.append(
            SceneGeom(
                name=name,
                geom_type=geom_type,
                size_m=size,
                local_transform=_transform(model.geom_pos[geom_id], model.geom_quat[geom_id]),
                rgba=[float(value) for value in model.geom_rgba[geom_id]],
                mesh_asset_id=mesh_asset_id,
            )
        )
    return result


def build_scene_manifest(
    model: mujoco.MjModel,
    model_path: Path,
    *,
    model_id: str,
    scene_id: str,
) -> SceneManifest:
    model_path = model_path.resolve()
    assets, mesh_ids, _ = _mesh_map(model_path.parent, model)
    base_id = int(model.body("base_link").id)
    robot_ids: set[int] = {base_id}
    for body_id in range(1, model.nbody):
        current = body_id
        while current not in (0, base_id):
            current = int(model.body_parentid[current])
        if current == base_id:
            robot_ids.add(body_id)

    links: list[SceneLink] = []
    for body_id in sorted(robot_ids):
        parent_id = int(model.body_parentid[body_id])
        parent_name = None if parent_id == 0 else (model.body(parent_id).name or f"body_{parent_id}")
        joint_id, joint_type, axis = _body_joint(model, body_id)
        links.append(
            SceneLink(
                name=model.body(body_id).name or f"body_{body_id}",
                parent_name=parent_name,
                joint_name=None if joint_id is None else (model.joint(joint_id).name or f"joint_{joint_id}"),
                joint_type=joint_type,
                joint_axis=axis,
                local_transform=_transform(model.body_pos[body_id], model.body_quat[body_id]),
                geoms=_body_geoms(model, body_id, mesh_ids),
            )
        )

    world_links: list[SceneLink] = []
    for body_id in range(1, model.nbody):
        if body_id in robot_ids:
            continue
        ancestor = body_id
        while ancestor != 0 and int(model.body_parentid[ancestor]) != 0:
            ancestor = int(model.body_parentid[ancestor])
        if int(model.body_parentid[body_id]) != 0:
            continue
        joint_id, joint_type, axis = _body_joint(model, body_id)
        world_links.append(
            SceneLink(
                name=model.body(body_id).name or f"body_{body_id}",
                parent_name=None,
                joint_name=None if joint_id is None else (model.joint(joint_id).name or f"joint_{joint_id}"),
                joint_type=joint_type,
                joint_axis=axis,
                local_transform=_transform(model.body_pos[body_id], model.body_quat[body_id]),
                geoms=_body_geoms(model, body_id, mesh_ids),
            )
        )

    world_geoms = _body_geoms(model, 0, mesh_ids)
    if world_geoms:
        world_links.insert(
            0,
            SceneLink(
                name="world_static",
                parent_name=None,
                joint_name=None,
                joint_type="fixed",
                local_transform=SceneTransform(),
                geoms=world_geoms,
            ),
        )

    return SceneManifest(
        model_id=model_id,
        model_sha256=_sha256(model_path),
        scene_id=scene_id,
        links=links,
        world_links=world_links,
        mesh_assets=assets,
        camera_ids=[model.camera(i).name for i in range(model.ncam)],
        tcp_site="grasp_tcp",
    )

