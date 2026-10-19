"""Fixed or articulated arm assembly with explicit inertias and gripper coupling."""

import xml.etree.ElementTree as ET

import numpy as np

from .source import vector, rpy_matrix, axis_matrix, fmt, quat, stl_bounds
from ..joint_coordinates import ArmCoordinates

ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 6))
GRIPPER_DRIVE = "gripper_base_joint"


def _tensor(inertial, scale):
    i = inertial.find("inertia")
    xx, yy, zz, xy, xz, yz = [float(i.get(k)) for k in
                               ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
    rotation = rpy_matrix(vector(inertial.find("origin").get("rpy", "0 0 0")))
    matrix = rotation @ (scale * np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])) @ rotation.T
    return fmt([matrix[0, 0], matrix[1, 1], matrix[2, 2],
                matrix[0, 1], matrix[0, 2], matrix[1, 2]])


def _mount_override(camera):
    """Declared G2a mount override (interface I1), or ``None`` for the baseline model.

    ``translation_m``/``rpy_rad`` live in the ``depth_cam_link`` parent frame; the
    rotation is applied about the shell origin, so the translation is a plain
    offset of the shell origin.  Unknown keys are rejected rather than ignored so
    a typo cannot silently produce a baseline model.
    """
    override = camera.get("mount_override")
    if override is None:
        return None
    if not isinstance(override, dict):
        raise ValueError("simulation.camera.mount_override must be a mapping")
    allowed = {"translation_m", "rpy_rad", "bracket_mass_kg", "bracket_radius_gyration_m", "status"}
    unknown = set(override)-allowed
    if unknown:
        raise ValueError(f"Unknown mount_override keys: {sorted(unknown)}")
    translation = np.asarray(override.get("translation_m", (0., 0., 0.)), dtype=float)
    rpy = np.asarray(override.get("rpy_rad", (0., 0., 0.)), dtype=float)
    mass = float(override.get("bracket_mass_kg", 0.))
    gyration = float(override.get("bracket_radius_gyration_m", 0.02))
    if translation.shape != (3,) or rpy.shape != (3,) or not (np.isfinite(translation).all() and np.isfinite(rpy).all()):
        raise ValueError("mount_override translation_m and rpy_rad must be finite three-vectors")
    if not np.isfinite(mass) or mass < 0 or not np.isfinite(gyration) or gyration < 0:
        raise ValueError("mount_override bracket mass/radius of gyration must be finite and non-negative")
    if not isinstance(override.get("status"), str) or not override["status"]:
        raise ValueError("mount_override requires a non-empty declaration status string")
    return translation, rpy_matrix(rpy), mass, gyration


def _apply_mount_override(position, rotation, camera):
    override = _mount_override(camera)
    if override is None:
        return position, rotation
    translation, delta_rotation, _, _ = override
    return np.asarray(position, dtype=float)+translation, delta_rotation @ np.asarray(rotation, dtype=float)


def _mount_bracket_mass(camera):
    override = _mount_override(camera)
    return 0. if override is None else override[2]


def _transform(position, rotation):
    value = np.eye(4)
    value[:3, :3] = np.asarray(rotation, dtype=float)
    value[:3, 3] = np.asarray(position, dtype=float)
    return value


def _inverse(t):
    result = np.eye(4)
    result[:3, :3] = t[:3, :3].T
    result[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return result


def _camera_to_tcp_mount(params, joints, initial, tcp, lab):
    """Derive the rigid TCP->camera transform from the existing assembly.

    The source description hangs the camera shell from ``link4``.  The lab
    requires the shell, bracket, inertial, collision proxy and optical frame to
    ride on the gripper.  The mounting numbers are therefore not invented: they
    are the transform that keeps the shell at its nominal-pose pose while
    changing its parent to the TCP carrier (``guide_rod_link``).  The result is
    explicitly marked ``derived_*_not_measured``.
    """
    def origin(name):
        joint = joints[name]
        o = joint.find("origin")
        return _transform(vector(o.get("xyz", "0 0 0")), rpy_matrix(vector(o.get("rpy", "0 0 0"))))

    joint5 = joints["link5"]
    q5 = float(initial.get("joint5", 0.0))
    axis = vector(joint5.find("axis").get("xyz"))
    link4_link5 = origin("link5") @ _transform(np.zeros(3), axis_matrix(axis, q5))
    link4_guide = link4_link5 @ origin("gripper_link") @ origin("guide_rod_link")
    link4_cam = origin("camera_connect_link") @ origin("depth_cam_link")
    guide_tcp = _transform(tcp, np.eye(3))
    guide_cam = guide_tcp @ _inverse(link4_guide @ guide_tcp) @ link4_cam
    return {
        "parent": "guide_rod_link",
        "position": guide_cam[:3, 3].copy(),
        "rotation": guide_cam[:3, :3].copy(),
        "tcp_to_camera": _inverse(guide_tcp) @ guide_cam,
        "source": lab.get("camera_source", "derived_from_existing_assembly_at_nominal_pose"),
        "status": lab.get("camera_status", "derived_not_measured"),
        "nominal_joint5_rad": q5,
    }


def build_arm(params, links, joints, groups, mesh_files, visual, lab=None):
    lab = {} if lab is None else dict(lab)
    sim = params["simulation"]
    dynamic = not sim["fixed_arm"]
    arm = ET.Element("mujocoinclude")
    names = [n for n in links if groups[n] in ("arm", "gripper")]
    raw_mass = sum(float(links[n].find("inertial/mass").get("value"))
                   for n in names if links[n].find("inertial") is not None)
    scale = sim["mass_distribution"]["fixed_arm_mass_kg"] / raw_mass
    coordinates = ArmCoordinates(params)
    initial = dict(zip(ARM_JOINTS, coordinates.to_model(sim["initial_arm_user_rad"])))
    spec = {"dynamic": dynamic, "initial_positions": {}, "mimic": {}, "limits": {},
            "source_limits": {}}
    bounds = {n: stl_bounds(p) for n, p in mesh_files.items() if n in names}
    opening = params["hardware"]["gripper_max_opening_m"]
    # The original translating jaws, rather than the small linkage levers, contact objects.
    left_center, left_half = bounds["left_jaw_link"]
    right_center, right_half = bounds["right_jaw_link"]
    left_origin = vector(joints["left_jaw_link"].find("origin").get("xyz"))
    right_origin = vector(joints["right_jaw_link"].find("origin").get("xyz"))
    source_gap = left_origin[1] + left_center[1] - left_half[1] - (
        right_origin[1] + right_center[1] + right_half[1])
    jaw_offset = (opening - source_gap) / 2
    # Fixed centre between the translating jaw contact sites, in the guide-rod
    # frame.  Computed before the body loop so the rigid camera mount can use
    # the same TCP definition.
    _lc = _pad_center(left_center, left_half, True, sim["gripper_servo"])
    _rc = _pad_center(right_center, right_half, False, sim["gripper_servo"])
    tcp = (left_origin + _lc + right_origin + _rc) / 2
    # G1 fixes the camera to the link4 branch. The old project's optional
    # camera-on-TCP experiment is intentionally not reachable from this model.
    camera_mount = None
    remap = {}
    if camera_mount is not None:
        remap["camera_connect_link"] = camera_mount
    drive_joint = joints["gripper_base_link"]
    coefficient = abs(float(joints["left_jaw_link"].find("mimic").get("multiplier")))
    closed_angle = float(drive_joint.find("limit").get("lower"))
    spec["gripper"] = {"max_width_m": opening,
                       "min_width_m": opening + 2 * coefficient * closed_angle,
                       "width_per_drive_rad_m": 2 * coefficient,
                       "source_gap_m": source_gap,
                       "jaw_origin_correction_m": jaw_offset}
    grip = spec["gripper"]
    grip["zero_width_m"] = params["calibration"]["gripper_zero_width_m"]
    grip["zero_model_rad"] = (grip["zero_width_m"]-opening)/(2*coefficient)
    for width in (grip["zero_width_m"], sim["initial_gripper_width_m"]):
        if not grip["min_width_m"] <= width <= opening:
            raise ValueError("Gripper zero/initial width must be within the model opening range")
    initial[GRIPPER_DRIVE] = (sim["initial_gripper_width_m"]-opening)/(2*coefficient)
    spec["user_coordinates"] = {"zero_model_rad": coordinates.zero.tolist(),
                                "direction": coordinates.direction.tolist(),
                                "status": params["calibration"]["arm_zero_status"]}
    bodies = {"back_shell_black_link": arm}
    pending = list(names)
    while pending:
        before = len(pending)
        for name in list(pending):
            joint = joints[name]
            parent = joint.find("parent").get("link")
            if name in remap:
                parent = remap[name]["parent"]
            if parent not in bodies:
                continue
            origin = joint.find("origin")
            position = vector(origin.get("xyz", "0 0 0"))
            rotation = rpy_matrix(vector(origin.get("rpy", "0 0 0")))
            if name in remap:
                # Rigid TCP mount: the shell keeps its nominal-pose placement
                # but now rides on the gripper carrier.
                position = remap[name]["position"]
                rotation = remap[name]["rotation"]
            kind, joint_name = joint.get("type"), joint.get("name")
            value = initial.get(joint_name, 0.)
            mimic = joint.find("mimic")
            if mimic is not None:
                multiplier, offset = float(mimic.get("multiplier", 1)), float(mimic.get("offset", 0))
                value = initial.get(mimic.get("joint"), 0.) * multiplier + offset
                spec["mimic"][joint_name] = {"joint": mimic.get("joint"),
                                             "multiplier": multiplier, "offset": offset}
            limit = joint.find("limit")
            if limit is not None and kind != "fixed":
                lower, upper = float(limit.get("lower")), float(limit.get("upper"))
                spec["source_limits"][joint_name] = [lower, upper]
                if not lower - 1e-8 <= value <= upper + 1e-8:
                    raise ValueError(f"Initial pose violates {joint_name} limits")
                effective = [lower, upper]
                if joint_name in ARM_JOINTS and lab.get("arm_limit_model_rad") is not None:
                    index = ARM_JOINTS.index(joint_name)
                    bundle_lower, bundle_upper = (float(v) for v in lab["arm_limit_model_rad"][index])
                    effective = [max(lower, bundle_lower), min(upper, bundle_upper)]
                    if effective[0] >= effective[1]:
                        raise ValueError(f"Effective {joint_name} limit is empty")
                    if not effective[0] - 1e-9 <= value <= effective[1] + 1e-9:
                        raise ValueError(f"Initial pose violates the effective {joint_name} limit")
                elif joint_name == "joint1" and lab.get("q1_limit_model_rad") is not None:
                    bundle_lower, bundle_upper = (float(v) for v in lab["q1_limit_model_rad"])
                    effective = [max(lower, bundle_lower), min(upper, bundle_upper)]
                    if effective[0] >= effective[1]:
                        raise ValueError("Effective joint1 limit is empty")
                    if not effective[0] - 1e-9 <= value <= effective[1] + 1e-9:
                        raise ValueError("Initial pose violates the effective joint1 harness limit")
                spec["limits"][joint_name] = effective
                if joint_name in ARM_JOINTS:
                    zero = coordinates.zero[ARM_JOINTS.index(joint_name)]
                    if not lower <= zero <= upper:
                        raise ValueError(f"Software zero violates {joint_name} limits")
            if name in ("left_jaw_link", "right_jaw_link"):
                position[1] += jaw_offset if name == "left_jaw_link" else -jaw_offset
            if not dynamic:
                if kind in ("revolute", "continuous"):
                    rotation = rotation @ axis_matrix(vector(joint.find("axis").get("xyz")), value)
                elif kind == "prismatic":
                    position += rotation @ vector(joint.find("axis").get("xyz")) * value
            if name == "depth_cam_link" and camera_mount is None:
                # G2a isolation hook (interface I1): re-pose the *whole* camera shell
                # (mesh + collision proxy + inertial + the child rgbd camera). The
                # optical element never moves relative to its shell.
                position, rotation = _apply_mount_override(position, rotation, sim["camera"])
            body = ET.SubElement(bodies[parent], "body", name=name, pos=fmt(position), quat=quat(rotation))
            if dynamic and kind != "fixed":
                axis = vector(joint.find("axis").get("xyz"))
                attrs = {"name": joint_name, "type": "slide" if kind == "prismatic" else "hinge",
                         "axis": fmt(axis), "limited": "true", "range": fmt(spec["limits"][joint_name])}
                if joint_name in ARM_JOINTS:
                    cfg = sim["arm_servo"]
                    attrs.update(damping=str(cfg["joint_damping_nm_s"]), armature=str(cfg["armature_kg_m2"]))
                else:
                    attrs.update(damping="0.2" if kind == "prismatic" else "0.00002",
                                 armature="0.000002")
                ET.SubElement(body, "joint", **attrs)
                spec["initial_positions"][joint_name] = value
            inertial = links[name].find("inertial")
            if inertial is not None:
                o = inertial.find("origin")
                body_mass = float(inertial.find("mass").get("value")) * scale
                tensor = _tensor(inertial, scale)
                if name == "depth_cam_link" and camera_mount is None:
                    override = _mount_override(sim["camera"])
                    if override is not None and override[2] > 0:
                        # Declared point-mass bracket at the shell origin: mass plus
                        # an explicit rotational term m*k^2 about its own centre.
                        # The bracket is modelled at the shell CoM offset; the m*k^2
                        # term is a declared simplification, never a silent zero.
                        body_mass += override[2]
                        delta = override[2]*override[3]**2
                        values = [float(v) for v in tensor.split()] if isinstance(tensor, str) else list(tensor)
                        values[0] += delta
                        values[1] += delta
                        values[2] += delta
                        tensor = fmt(values)
                ET.SubElement(body, "inertial", pos=o.get("xyz"),
                              mass=str(body_mass),
                              fullinertia=tensor)
                if name in bounds:
                    if dynamic:
                        _collision(body, name, bounds[name], sim["gripper_servo"])
                    else:
                        center, half = bounds[name]
                        ET.SubElement(body, "geom", name=name+"_collision", type="box",
                                      pos=fmt(center), size=fmt(half), contype="2", conaffinity="1",
                                      mass="0", group="3", rgba="0 0 0 0")
            visual(body, name)
            bodies[name] = body
            pending.remove(name)
        if len(pending) == before:
            raise ValueError(f"Unresolved official links: {pending}")
    camera = sim["camera"]
    ET.SubElement(bodies["depth_cam_link"], "camera", name="rgbd",
                  pos=fmt(camera["aperture_pos_depth_cam_m"]), xyaxes=fmt(camera["xyaxes_depth_cam"]),
                  fovy=str(camera["vertical_fov_deg"]))
    if dynamic:
        ET.SubElement(bodies["guide_rod_link"], "site", name="grasp_tcp", pos=fmt(tcp),
                      size="0.003", rgba="0 1 0 1", group="4")
        mount = joints["link1"].find("origin")
        ET.SubElement(arm, "site", name="arm_base", pos=mount.get("xyz", "0 0 0"),
                      quat=quat(rpy_matrix(vector(mount.get("rpy", "0 0 0")))),
                      size="0.003", rgba="1 0 0 1", group="4")
    spec["tcp"] = {"site": "grasp_tcp", "body": "guide_rod_link", "position": tcp.tolist()}
    if camera_mount is not None:
        spec["camera_mount"] = {
            "parent": camera_mount["parent"],
            "position": np.asarray(camera_mount["position"]).tolist(),
            "rotation": np.asarray(camera_mount["rotation"]).tolist(),
            "quat": quat(camera_mount["rotation"]),
            "tcp_to_camera": np.asarray(camera_mount["tcp_to_camera"]).tolist(),
            "nominal_joint5_rad": camera_mount["nominal_joint5_rad"],
            "source": camera_mount["source"],
            "status": camera_mount["status"],
        }
    else:
        spec["camera_mount"] = {"parent": "link4", "status": "source_topology_not_rigid_to_tcp"}
    spec["q1_bundle_deg"] = lab.get("q1_bundle_deg")
    spec["effective_limit_status"] = lab.get("limit_status", "source_limits")
    return arm, spec


def _pad_center(center, half, left, cfg):
    pad_half = np.array(cfg["finger_pad_size_m"]) / 2
    result = center.copy()
    result[1] = center[1] - half[1] + pad_half[1] if left else center[1] + half[1] - pad_half[1]
    result[2] = center[2] + half[2] - pad_half[2]
    return result


def _collision(body, name, bounds, cfg):
    center, half = bounds
    # Bits: environment=1, chassis/wheels=2, arm=4, gripper=8, object=16.
    gripper = name.startswith("gripper") or name in ("guide_rod_link", "left_jaw_link", "right_jaw_link")
    attrs = dict(name=name+"_collision", mass="0", group="3", rgba="0 0 0 0",
                 contype="8" if gripper else "4", conaffinity="23" if gripper else "31")
    if name in ("left_jaw_link", "right_jaw_link"):
        left = name == "left_jaw_link"
        pad_half = np.array(cfg["finger_pad_size_m"]) / 2
        pad = _pad_center(center, half, left, cfg)
        ET.SubElement(body, "geom", **{**attrs, "name": name+"_pad"}, type="box",
                      pos=fmt(pad), size=fmt(pad_half), condim="4", priority="2",
                      friction=fmt([cfg["sliding_friction"], 0.001, 0.0001]), solref="0.006 1")
        # Cover the rest of the visual finger's bounds without overlapping the
        # pad. Previously only the small distal pad could touch an object.
        lower, upper = center-half, center+half
        pad_lower, pad_upper = pad-pad_half, pad+pad_half
        pieces = []
        remaining_lower, remaining_upper = lower.copy(), upper.copy()
        for axis in range(3):
            for side, edge in ((0, pad_lower[axis]), (1, pad_upper[axis])):
                lo, hi = remaining_lower.copy(), remaining_upper.copy()
                if side == 0:
                    hi[axis] = min(edge, hi[axis])
                else:
                    lo[axis] = max(edge, lo[axis])
                if np.all(hi-lo > 1e-8):
                    pieces.append((lo, hi))
            remaining_lower[axis] = max(remaining_lower[axis], pad_lower[axis])
            remaining_upper[axis] = min(remaining_upper[axis], pad_upper[axis])
        for index, (lo, hi) in enumerate(pieces):
            ET.SubElement(body, "geom", **{**attrs, "name": f"{name}_shell_{index}"},
                          type="box", pos=fmt((lo+hi)/2), size=fmt((hi-lo)/2),
                          condim="4", friction=fmt([cfg["sliding_friction"], 0.001, 0.0001]),
                          solref="0.006 1")
        face = pad.copy()
        face[1] += -pad_half[1] if left else pad_half[1]
        ET.SubElement(body, "site", name="left_finger_contact" if left else "right_finger_contact",
                      pos=fmt(face), size="0.002", rgba="1 0.7 0 1", group="4")
    elif name in ("link2", "link3"):
        a, b = center.copy(), center.copy()
        a[2], b[2] = 0.005, 0.071
        ET.SubElement(body, "geom", **attrs, type="capsule", fromto=fmt(np.r_[a, b]), size="0.008")
    else:
        # Thin fixtures stay boxes; the gripper actuator box must not fill the finger gap.
        if name == "gripper_link":
            center, half = np.array([0.01, 0, 0.006]), np.array([0.015, 0.025, 0.012])
        ET.SubElement(body, "geom", **attrs, type="box", pos=fmt(center), size=fmt(half))


def add_arm_actuators(actuator, params, spec):
    cfg = params["simulation"]["arm_servo"]
    for index, name in enumerate(ARM_JOINTS):
        limit = cfg["torque_limits_nm"][index]
        ET.SubElement(actuator, "position", name=name+"_motor", joint=name, gear="1",
                      kp=str(cfg["kp_nm_per_rad"][index]), kv=str(cfg["kd_nm_s_per_rad"][index]),
                      ctrllimited="true", ctrlrange=fmt(spec["limits"][name]),
                      forcelimited="true", forcerange=fmt([-limit, limit]))
    cfg = params["simulation"]["gripper_servo"]
    limit = cfg["torque_limit_nm"]
    ET.SubElement(actuator, "position", name="gripper_motor", joint=GRIPPER_DRIVE, gear="1",
                  kp=str(cfg["kp_nm_per_rad"]), kv=str(cfg["kd_nm_s_per_rad"]),
                  ctrllimited="true", ctrlrange=fmt(spec["limits"][GRIPPER_DRIVE]),
                  forcelimited="true", forcerange=fmt([-limit, limit]))
