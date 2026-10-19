"""Read actual object contacts and contact forces for push/grasp acceptance."""

import mujoco
import numpy as np


def object_contacts(sim):
    try:
        object_geom = sim.model.geom("target_cube_contact").id
    except Exception:
        return {
            "left_force_n": 0.0,
            "right_force_n": 0.0,
            "support_force_n": 0.0,
            "other_force_n": 0.0,
            "bilateral_contact": False,
            "pairs": [],
            "available": False,
            "status": "NOT_APPLICABLE",
            "reason": "target_cube_not_present",
        }
    totals = {"left_force_n": 0., "right_force_n": 0., "support_force_n": 0., "other_force_n": 0.}
    pairs = []
    for index, contact in enumerate(sim.data.contact):
        if object_geom not in (contact.geom1, contact.geom2):
            continue
        other = contact.geom2 if contact.geom1 == object_geom else contact.geom1
        name = sim.model.geom(other).name
        force = np.zeros(6)
        mujoco.mj_contactForce(sim.model, sim.data, index, force)
        normal = max(0., float(force[0]))
        key = ("left_force_n" if name.startswith("left_jaw_link_") else
               "right_force_n" if name.startswith("right_jaw_link_") else
               "support_force_n" if name in ("pick_platform", "floor") else "other_force_n")
        totals[key] += normal
        pairs.append({"geom": name, "normal_force_n": normal, "distance_m": float(contact.dist)})
    totals["bilateral_contact"] = totals["left_force_n"] > .01 and totals["right_force_n"] > .01
    totals["pairs"] = pairs
    totals["available"] = True
    totals["status"] = "AVAILABLE"
    totals["reason"] = ""
    return totals
