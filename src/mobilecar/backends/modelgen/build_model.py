"""Generate modular MJCF from the supplied LanderPi description and YAML."""
import argparse
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
import yaml
from .parameters import DEFAULT_CONFIG, load_parameters
from .model.arm import build_arm, add_arm_actuators
from .model.source import (ROOT, SOURCE, read_source, vector, rpy_matrix, axis_matrix,
                           fmt, quat, stl_bounds, numeric_expression)

DEFAULT_OUTPUT = ROOT / "assets/landerpi/model"
DEFAULT_SCENE = ROOT / "assets/landerpi/scenes/mobile_pick.xml"
WHEEL_NAMES = ("front_left", "rear_left", "front_right", "rear_right")
SOURCE_WHEELS = ("wheel_left_front_link", "wheel_left_back_link", "wheel_right_front_link", "wheel_right_back_link")


def write_xml(path, root):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def build_model(params=None, output_dir=DEFAULT_OUTPUT, scene=DEFAULT_SCENE, navigation_sensor_mounts=None,
                lab=None):
    params = load_parameters() if params is None else params
    output = Path(output_dir).resolve()
    scene = Path(scene).resolve()
    if not scene.is_file():
        raise FileNotFoundError(scene)
    output.mkdir(parents=True, exist_ok=True)
    hw, sim = params["hardware"], params["simulation"]
    links, joints, groups = read_source()
    mesh_files = {}
    assets = ET.Element("mujocoinclude")
    asset = ET.SubElement(assets, "asset")
    for name, link in links.items():
        mesh = link.find("visual/geometry/mesh")
        if mesh is not None:
            source_file = SOURCE / "meshes" / groups[name] / mesh.get("filename").split("/")[-1]
            mesh_files[name] = source_file
            ET.SubElement(asset, "mesh", name=name+"_mesh", file=Path(os.path.relpath(source_file, output)).as_posix())
    write_xml(output / "assets.xml", assets)

    chassis = ET.Element("mujocoinclude")
    # base_link is 21.1 mm above wheel axes in the official assembly.
    base = ET.SubElement(chassis, "body", name="base_link", pos=fmt([0, 0, hw["wheel_radius_m"]+.0211+.002]))
    if not sim["base_fixed_for_debug"]:
        ET.SubElement(base, "freejoint", name="base_free")
    mass = sim["mass_distribution"]
    base_mass = hw["total_mass_kg"] - 4*mass["wheel_mass_kg"] - mass["fixed_arm_mass_kg"]
    dims = np.array(mass["chassis_equivalent_box_size_m"])
    inertia = base_mass/12 * np.array([dims[1]**2+dims[2]**2, dims[0]**2+dims[2]**2, dims[0]**2+dims[1]**2])
    ET.SubElement(base, "inertial", pos=fmt(mass["chassis_com_base_m"]), mass=str(base_mass), diaginertia=fmt(inertia))
    ET.SubElement(base, "geom", name="chassis_collision", type="box", pos="0 0 -0.016", size="0.096 0.048 0.017", group="3", rgba="0.2 0.2 0.2 0", contype="2", conaffinity="1", mass="0")
    ET.SubElement(base, "site", name="imu_site", pos="0 0 0.035", size="0.002", rgba="0 0 0 0")
    if not sim["fixed_arm"]:
        ET.SubElement(base, "site", name="base_frame", pos="0 0 0", size="0.003", rgba="0 0 1 1", group="4")
    ET.SubElement(base, "include", file="wheels.xml")

    def visual(body, name):
        if links[name].find("visual/geometry/mesh") is not None:
            color = "0.12 0.13 0.15 1" if any(s in name for s in ("black", "cam", "jaw", "link2", "link3", "link4")) else "0.65 0.68 0.72 1"
            origin = links[name].find("visual/origin")
            ET.SubElement(body, "geom", name=name+"_visual", type="mesh", mesh=name+"_mesh", pos=origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", quat=quat(rpy_matrix(vector(origin.get("rpy", "0 0 0")))) if origin is not None else "1 0 0 0", rgba=color, contype="0", conaffinity="0", mass="0", group="1")

    visual(base, "base_link")
    bodies = {"base_link": base}
    for name in ("back_shell_middle_link", "back_shell_black_link", "lidar_frame", "imu_frame"):
        if name not in joints:
            continue
        j = joints[name]
        origin = j.find("origin")
        body = ET.SubElement(bodies[j.find("parent").get("link")], "body", name=name, pos=origin.get("xyz"), quat=quat(rpy_matrix(vector(origin.get("rpy", "0 0 0")))))
        visual(body, name)
        bodies[name] = body
    if navigation_sensor_mounts is not None:
        for name, site in (("lidar", "d500_scan"), ("imu", "navigation_imu_site")):
            mount = navigation_sensor_mounts[name]
            parent = bodies.get(mount["parent_body"])
            if parent is None:
                raise ValueError(f"Unsupported sensor mounting body: {mount['parent_body']}")
            ET.SubElement(parent, "site", name=site, pos=fmt(mount["position_m"]),
                          quat=quat(rpy_matrix(mount["rpy_rad"])), size="0.002", rgba="0 0 0 0")
    arm_file = "fixed_arm.xml" if sim["fixed_arm"] else "arm.xml"
    ET.SubElement(bodies["back_shell_black_link"], "include", file=arm_file)
    write_xml(output / "chassis.xml", chassis)

    arm, arm_spec = build_arm(params, links, joints, groups, mesh_files, visual, lab=lab)
    write_xml(output / arm_file, arm)
    (output / "arm_spec.json").write_text(json.dumps(arm_spec, indent=2), encoding="utf-8")
    camera = sim["camera"]

    wheels = ET.Element("mujocoinclude")
    roll = sim["wheels"]
    n = roll["rollers_per_wheel"]
    radius, rr, half_length = hw["wheel_radius_m"], roll["roller_radius_m"], roll["roller_half_length_m"]
    hub_mass = mass["wheel_mass_kg"] - n*roll["roller_mass_kg"]
    positions = [(hw["wheelbase_m"]/2, hw["track_width_m"]/2), (-hw["wheelbase_m"]/2, hw["track_width_m"]/2), (hw["wheelbase_m"]/2, -hw["track_width_m"]/2), (-hw["wheelbase_m"]/2, -hw["track_width_m"]/2)]
    for index, (name, original, (x, y), handedness) in enumerate(zip(WHEEL_NAMES, SOURCE_WHEELS, positions, (1,-1,-1,1))):
        wheel = ET.SubElement(wheels, "body", name=name+"_wheel", pos=fmt([x, y, -.0211]))
        ET.SubElement(wheel, "joint", name=name+"_joint", type="hinge", axis="0 1 0", armature=str(roll["wheel_armature_kg_m2"]), damping="0.00002")
        hub_r = radius*.55
        ix = hub_mass*(3*hub_r**2+.018**2)/12
        ET.SubElement(wheel, "inertial", pos="0 0 0", mass=str(hub_mass), diaginertia=fmt([ix, hub_mass*hub_r**2/2, ix]))
        ET.SubElement(wheel, "geom", name=name+"_hub_contact", type="cylinder", size=fmt([hub_r,.009]), quat=quat(rpy_matrix([math.pi/2,0,0])), contype="2", conaffinity="1", mass="0", group="3", rgba="0 0 0 0")
        visual(wheel, original)
        for k in range(n):
            theta = 2*math.pi*k/n
            radial = np.array([math.sin(theta), 0, math.cos(theta)])
            tangent = np.array([math.cos(theta), 0, -math.sin(theta)])
            axis = (tangent + np.array([0, handedness, 0])) / math.sqrt(2)
            rotation = np.column_stack((radial, np.cross(axis, radial), axis))
            roller = ET.SubElement(wheel, "body", name=f"{name}_roller_{k}", pos=fmt((radius-rr)*radial), quat=quat(rotation))
            ET.SubElement(roller, "joint", name=f"{name}_roller_{k}_joint", axis="0 0 1", type="hinge", damping=str(roll["roller_bearing_damping_nm_s"]), frictionloss=str(roll["roller_bearing_friction_nm"]))
            roller_mass = roll["roller_mass_kg"]
            transverse = roller_mass*(rr**2+half_length**2)/5
            ET.SubElement(roller, "inertial", mass=str(roller_mass), pos="0 0 0", diaginertia=fmt([transverse, transverse, 2*roller_mass*rr**2/5]))
            ET.SubElement(roller, "geom", name=f"{name}_roller_{k}_contact", type="ellipsoid", size=fmt([rr, rr, half_length]), contype="2", conaffinity="1", condim="3", priority="1", friction=fmt([sim["contact"]["sliding_friction"], .001, .0001]), solref="0.004 1", solimp="0.9 0.95 0.001", mass="0", group="2", rgba="0.2 0.2 0.2 0")
    write_xml(output / "wheels.xml", wheels)

    sensors = ET.Element("mujocoinclude")
    sensor = ET.SubElement(sensors, "sensor")
    ET.SubElement(sensor, "gyro", name="imu_gyro", site="imu_site")
    ET.SubElement(sensor, "accelerometer", name="imu_accel", site="imu_site")
    if navigation_sensor_mounts is not None:
        ET.SubElement(sensor, "gyro", name="navigation_imu_gyro", site="navigation_imu_site")
        ET.SubElement(sensor, "accelerometer", name="navigation_imu_accel", site="navigation_imu_site")
    ET.SubElement(sensor, "framepos", name="base_position", objtype="body", objname="base_link")
    write_xml(output / "sensors.xml", sensors)
    actuators = ET.Element("mujocoinclude")
    act = ET.SubElement(actuators, "actuator")
    for name, limit in zip(WHEEL_NAMES, params["derived"]["wheel_torque_limits_nm"]):
        ET.SubElement(act, "motor", name=name+"_motor", joint=name+"_joint", gear="1", ctrllimited="true", ctrlrange=fmt([-limit,limit]))
    if not sim["fixed_arm"]:
        add_arm_actuators(act, params, arm_spec)
    write_xml(output / "actuators.xml", actuators)
    model = ET.Element("mujoco", model="landerpi_mecanum")
    ET.SubElement(model, "compiler", angle="radian", inertiafromgeom="false", autolimits="true")
    ET.SubElement(model, "option", timestep=str(sim["physics_timestep_s"]), gravity="0 0 -9.81", integrator="implicitfast", cone="elliptic", iterations="60")
    visual_options = ET.SubElement(model, "visual")
    ET.SubElement(visual_options, "global", offwidth=str(camera["width"]), offheight=str(camera["height"]))
    ET.SubElement(visual_options, "map", znear="0.01", zfar="10")
    ET.SubElement(model, "include", file="assets.xml")
    ET.SubElement(model, "include", file=Path(os.path.relpath(scene, output)).as_posix())
    wb = ET.SubElement(model, "worldbody")
    ET.SubElement(wb, "include", file="chassis.xml")
    ET.SubElement(model, "include", file="actuators.xml")
    ET.SubElement(model, "include", file="sensors.xml")
    if not sim["fixed_arm"]:
        equality = ET.SubElement(model, "equality")
        for name, coupling in arm_spec["mimic"].items():
            ET.SubElement(equality, "joint", name=name+"_coupling", joint1=name,
                          joint2=coupling["joint"],
                          polycoef=fmt([coupling["offset"], coupling["multiplier"], 0, 0, 0]),
                          solref="0.002 1", solimp="0.99 0.999 0.0001")
        contact = ET.SubElement(model, "contact")
        # Adjacent links are filtered by MuJoCo. Explicitly exclude the mounting assembly.
        ET.SubElement(contact, "exclude", body1="base_link", body2="link1")
    path = output / "robot.xml"
    write_xml(path, model)
    (output / "parameters.yaml").write_text(yaml.safe_dump(params, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--override", type=Path)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    args = parser.parse_args()
    path = build_model(load_parameters(args.config, args.override), args.output, args.scene)
    model = mujoco.MjModel.from_xml_path(str(path))
    print(json.dumps({"model": str(path), "mass_kg": float(model.body_mass.sum()), "joints": model.njnt, "actuators": model.nu}, indent=2))


if __name__ == "__main__":
    main()
