"""读取标称 + 标定后的参数。纯离线：绝不打开串口。

这是 MJCF 生成器和控制器的共享入口。

结构：

    load_parameters()            读取 -> 合并 override -> 校验 -> 派生 -> 返回
    _validate_parameters()       整棵参数树的校验调度
    _validate_<section>()        每个参数分组一个校验函数
    _compute_derived()           派生量，只在校验通过后计算
"""

import argparse
import copy
import json
import math
from pathlib import Path

import yaml


DEFAULT_CONFIG = Path(__file__).resolve().parents[4] / "configs/robots/landerpi_nominal.yaml"

__all__ = ["DEFAULT_CONFIG", "load_parameters"]


# ---------------------------------------------------------------------------
# 基础 IO
# ---------------------------------------------------------------------------


def _read(path):
    value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected a YAML mapping")
    return value


def _merge(base, override, prefix=""):
    for key, value in override.items():
        location = f"{prefix}.{key}" if prefix else key
        if key not in base:
            raise ValueError(f"Unknown parameter: {location}")
        if isinstance(base[key], dict):
            if not isinstance(value, dict):
                raise ValueError(f"{location}: expected a mapping")
            _merge(base[key], value, location)
        else:
            base[key] = copy.deepcopy(value)


# ---------------------------------------------------------------------------
# 通用校验工具
# ---------------------------------------------------------------------------


def _finite_number(value, name):
    """校验任意有限数值；负数是合法值（例如陀螺仪零偏）。"""
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name}: expected a finite number")


def _number(value, name, allow_zero=False):
    """校验严格为正（或非负）的有限数值。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name}: expected a number")
    if not math.isfinite(value) or value < 0 or (value == 0 and not allow_zero):
        expected = "nonnegative" if allow_zero else "positive"
        raise ValueError(f"{name}: expected a finite {expected} number")


def _number_fields(mapping, keys, prefix="", allow_zero=False):
    """批量校验若干正数/非负数字段。"""
    for key in keys:
        name = f"{prefix}.{key}" if prefix else key
        _number(mapping[key], name, allow_zero=allow_zero)


def _vector(values, name, size):
    """校验定长向量，每个元素都必须是有限数值。"""
    if not isinstance(values, list) or len(values) != size:
        raise ValueError(f"{name}: expected {size} entries")
    for value in values:
        _finite_number(value, name)


def _scale_list(values, name, size=4):
    """校验按固件 wheel_order 排列的单轮缩放系数数组。

    数组必须完整填写：无法把残缺数组重新解释成默认的全 1，
    所以长度不对就是硬错误。
    """
    if not isinstance(values, list) or len(values) != size:
        raise ValueError(f"{name}: expected {size} entries in firmware wheel_order")
    for value in values:
        _number(value, name)


# ---------------------------------------------------------------------------
# 分模块校验
# ---------------------------------------------------------------------------


def _validate_hardware(hw):
    """校验机器人物理参数与电机实测数据。"""
    label = hw["motor_label"]

    _number_fields(
        hw,
        ("total_mass_kg", "wheel_radius_m", "wheelbase_m", "track_width_m"),
        prefix="hardware",
    )
    _number_fields(
        label,
        ("reference_voltage_v", "output_no_load_rpm", "reduction_ratio"),
        prefix="hardware.motor_label",
    )

    # 实测转速和对应的测量电压是一组标定数据；只提供其中一个会让派生 RPM
    # 失去唯一解，所以必须成对出现。
    measured_rpm = hw["measured_output_no_load_rpm"]
    measured_voltage = hw["measured_no_load_voltage_v"]
    if (measured_rpm is None) != (measured_voltage is None):
        raise ValueError("Measured no-load RPM and its voltage must be supplied together")
    if measured_rpm is not None:
        _number(measured_rpm, "hardware.measured_output_no_load_rpm")
        _number(measured_voltage, "hardware.measured_no_load_voltage_v")

    # 整车 COM/惯量需要重新分配到各个 MuJoCo body 上，这条标定路径尚未实现；
    # 与其静默忽略实测值，不如直接拒绝该 override。
    if (
        hw["measured_total_com_base_m"] is not None
        or hw["measured_total_inertia_at_com_kg_m2"] is not None
    ):
        raise ValueError(
            "Aggregate measured COM/inertia repartition is not implemented; "
            "use mass_distribution instead"
        )


def _validate_mass_distribution(hw, mass):
    """校验底盘 / 车轮 / 固定臂的质量分配。"""
    _number_fields(
        mass,
        ("wheel_mass_kg", "fixed_arm_mass_kg"),
        prefix="simulation.mass_distribution",
    )
    _vector(
        mass["chassis_com_base_m"],
        "simulation.mass_distribution.chassis_com_base_m",
        3,
    )
    _vector(
        mass["chassis_equivalent_box_size_m"],
        "simulation.mass_distribution.chassis_equivalent_box_size_m",
        3,
    )
    if min(mass["chassis_equivalent_box_size_m"]) <= 0:
        raise ValueError("Equivalent chassis box dimensions must be positive")

    allocated_mass = 4 * mass["wheel_mass_kg"] + mass["fixed_arm_mass_kg"]

    # 整车质量减去已分配质量后全部归给底盘，所以这部分必须严格为正，
    # 否则 MuJoCo 会拿到不可用的 body 质量。
    if hw["total_mass_kg"] <= allocated_mass:
        raise ValueError("Mass allocation leaves no positive chassis mass")


def _validate_wheels(hw, mass, wheels):
    """校验麦克纳姆轮与被动滚子参数。"""
    rollers_per_wheel = wheels["rollers_per_wheel"]
    if type(rollers_per_wheel) is not int or not 8 <= rollers_per_wheel <= 32:
        raise ValueError("rollers_per_wheel must be an integer in [8, 32]")

    _number_fields(
        wheels,
        (
            "roller_radius_m",
            "roller_half_length_m",
            "roller_mass_kg",
            "wheel_armature_kg_m2",
        ),
        prefix="simulation.wheels",
    )
    _number_fields(
        wheels,
        ("roller_bearing_damping_nm_s", "roller_bearing_friction_nm"),
        prefix="simulation.wheels",
        allow_zero=True,
    )

    if wheels["roller_radius_m"] >= hw["wheel_radius_m"]:
        raise ValueError("Roller radius must be smaller than wheel radius")

    total_roller_mass = wheels["roller_mass_kg"] * rollers_per_wheel

    # wheel_mass_kg 包含轮毂和全部滚子，因此滚子不能吃掉整轮质量。
    if total_roller_mass >= mass["wheel_mass_kg"]:
        raise ValueError("Roller mass allocation leaves no positive wheel hub mass")


def _validate_servo(servo):
    """校验仿真中的轮轴伺服参数。"""
    _number_fields(
        servo,
        ("kv_nm_per_rad_s", "torque_limit_nm"),
        prefix="simulation.wheel_servo",
    )
    _number(
        servo["command_delay_s"],
        "simulation.wheel_servo.command_delay_s",
        allow_zero=True,
    )
    for key in ("ki_nm_per_rad", "integral_torque_limit_nm"):
        _number(servo.get(key, 0.), "simulation.wheel_servo."+key, allow_zero=True)
    if servo.get("ki_nm_per_rad", 0.) > 0 and servo.get("integral_torque_limit_nm", 0.) <= 0:
        raise ValueError("Wheel PI requires a positive integral torque limit")
    _scale_list(
        servo["per_wheel_torque_scale"],
        "simulation.wheel_servo.per_wheel_torque_scale",
    )


def _validate_camera(camera):
    """校验 RGB-D 相机模型参数。"""
    for key in ("width", "height"):
        value = camera[key]
        if type(value) is not int or value <= 0:
            raise ValueError(f"simulation.camera.{key}: expected a positive integer")

    _number_fields(
        camera,
        ("fps", "depth_min_m", "depth_max_m", "vertical_fov_deg"),
        prefix="simulation.camera",
    )
    _vector(
        camera["aperture_pos_depth_cam_m"],
        "simulation.camera.aperture_pos_depth_cam_m",
        3,
    )

    # FOV 和景深分开校验，报错才能直接指出真正出错的字段。
    axes = camera["xyaxes_depth_cam"]
    _vector(axes, "simulation.camera.xyaxes_depth_cam", 6)
    x, y = axes[:3], axes[3:]
    if (not math.isclose(sum(v*v for v in x), 1, abs_tol=1e-6)
            or not math.isclose(sum(v*v for v in y), 1, abs_tol=1e-6)
            or abs(sum(a*b for a, b in zip(x, y))) > 1e-6):
        raise ValueError("Camera xyaxes must be orthonormal unit vectors")
    if not 0 < camera["vertical_fov_deg"] < 170:
        raise ValueError("simulation.camera.vertical_fov_deg must be in (0, 170)")
    if camera["depth_min_m"] >= camera["depth_max_m"]:
        raise ValueError(
            "simulation.camera.depth_min_m must be smaller than depth_max_m"
        )

    # Optional G2a isolation override (interface I1). Absent => baseline model
    # unchanged. Unknown keys are rejected so a typo cannot silently build the
    # default installation while a study believes it moved the camera.
    override = camera.get("mount_override")
    if override is not None:
        if not isinstance(override, dict):
            raise ValueError("simulation.camera.mount_override must be a mapping")
        allowed = {"translation_m", "rpy_rad", "bracket_mass_kg",
                   "bracket_radius_gyration_m", "status"}
        unknown = set(override)-allowed
        if unknown:
            raise ValueError(f"simulation.camera.mount_override: unknown keys {sorted(unknown)}")
        _vector(override.get("translation_m", [0, 0, 0]),
                "simulation.camera.mount_override.translation_m", 3)
        _vector(override.get("rpy_rad", [0, 0, 0]),
                "simulation.camera.mount_override.rpy_rad", 3)
        _number_fields(override, ("bracket_mass_kg",), prefix="simulation.camera.mount_override")
        if float(override["bracket_mass_kg"]) < 0:
            raise ValueError("simulation.camera.mount_override.bracket_mass_kg must be non-negative")
        if "bracket_radius_gyration_m" in override:
            _number_fields(override, ("bracket_radius_gyration_m",),
                           prefix="simulation.camera.mount_override")
            if float(override["bracket_radius_gyration_m"]) < 0:
                raise ValueError("mount_override.bracket_radius_gyration_m must be non-negative")
        status = override.get("status")
        if not isinstance(status, str) or not status:
            raise ValueError("simulation.camera.mount_override.status must be a non-empty string")


def _validate_calibration(calibration):
    """校验叠加在标称值之上的标定修正量。"""
    _vector(calibration["arm_zero_model_rad"], "calibration.arm_zero_model_rad", 5)
    _vector(calibration["arm_user_direction"], "calibration.arm_user_direction", 5)
    if any(type(value) is bool or value not in (-1, 1) for value in calibration["arm_user_direction"]):
        raise ValueError("calibration.arm_user_direction entries must be -1 or +1")
    _number(calibration["gripper_zero_width_m"], "calibration.gripper_zero_width_m")
    _vector(
        calibration["imu_gyro_bias_rad_s"],
        "calibration.imu_gyro_bias_rad_s",
        3,
    )
    _scale_list(
        calibration["effective_wheel_radius_scale"],
        "calibration.effective_wheel_radius_scale",
    )


def _validate_timing(sim):
    """校验物理步长与控制周期的时序关系。"""
    _number_fields(
        sim,
        ("supply_voltage_v", "physics_timestep_s", "controller_period_s"),
        prefix="simulation",
    )

    ratio = sim["controller_period_s"] / sim["physics_timestep_s"]

    # 控制器更新必须正好落在物理步边界上，否则控制周期会相对 MuJoCo 积分漂移。
    # 例如：物理 dt = 0.002 s，控制 dt = 0.010 s，比值 5，每次控制更新对应
    # 恰好 5 个物理步。
    if ratio < 1 or not math.isclose(ratio, round(ratio), abs_tol=1e-8):
        raise ValueError(
            "controller_period_s must be an integer multiple of physics_timestep_s"
        )


# ---------------------------------------------------------------------------
# 总校验
# ---------------------------------------------------------------------------


def _validate_simulation(hw, sim):
    """校验全部仿真侧参数。"""
    _validate_timing(sim)
    _validate_servo(sim["wheel_servo"])
    _validate_mass_distribution(hw, sim["mass_distribution"])
    _validate_wheels(hw, sim["mass_distribution"], sim["wheels"])
    _validate_camera(sim["camera"])
    for key in ("fixed_arm", "base_fixed_for_debug"):
        if type(sim[key]) is not bool:
            raise ValueError(f"simulation.{key}: expected a boolean")
    arm = sim["arm_servo"]
    for key in ("kp_nm_per_rad", "kd_nm_s_per_rad", "torque_limits_nm",
                "velocity_limits_rad_s", "acceleration_limits_rad_s2"):
        _vector(arm[key], f"simulation.arm_servo.{key}", 5)
        for value in arm[key]:
            _number(value, f"simulation.arm_servo.{key}")
    _number_fields(arm, ("joint_damping_nm_s", "armature_kg_m2"), "simulation.arm_servo")
    gripper = sim["gripper_servo"]
    _number_fields(gripper, ("kp_nm_per_rad", "kd_nm_s_per_rad", "torque_limit_nm",
                            "speed_m_s", "sliding_friction"), "simulation.gripper_servo")
    _vector(gripper["finger_pad_size_m"], "simulation.gripper_servo.finger_pad_size_m", 3)
    for value in gripper["finger_pad_size_m"]:
        _number(value, "simulation.gripper_servo.finger_pad_size_m")
    _number(hw["gripper_max_opening_m"], "hardware.gripper_max_opening_m")
    if hw["gripper_max_opening_m"] <= gripper["finger_pad_size_m"][1]:
        raise ValueError("Gripper opening must exceed finger pad thickness")

    _number(
        sim["contact"]["sliding_friction"],
        "simulation.contact.sliding_friction",
        allow_zero=True,
    )
    _vector(
        sim["initial_arm_user_rad"],
        "simulation.initial_arm_user_rad",
        5,
    )
    _number(sim["initial_gripper_width_m"], "simulation.initial_gripper_width_m")
    _number_fields(sim["manual_drive"], ("speed_m_s", "yaw_rate_rad_s"), "simulation.manual_drive")


def _validate_parameters(params):
    """校验完整的参数树。"""
    if params.get("schema_version") != 3:
        raise ValueError("Only schema_version=3 is supported; update custom configs with user-zero and initial-pose fields")

    hw, sim = params["hardware"], params["simulation"]

    _validate_hardware(hw)
    _validate_simulation(hw, sim)
    _validate_calibration(params["calibration"])

    for key, value in params["command_limits"].items():
        _number(value, f"command_limits.{key}")


# ---------------------------------------------------------------------------
# 派生参数
# ---------------------------------------------------------------------------


def _compute_derived(params):
    """根据通过校验的标称/标定数据计算派生量。"""
    hw, sim = params["hardware"], params["simulation"]
    label, servo = hw["motor_label"], sim["wheel_servo"]
    calibration = params["calibration"]

    measured_rpm = hw["measured_output_no_load_rpm"]
    measured_voltage = hw["measured_no_load_voltage_v"]

    # 有实测电机数据时优先使用；否则回退到电机标签上的空载规格。
    if measured_rpm is not None:
        rpm_per_v = measured_rpm / measured_voltage
        source = "measurement_scaled_linearly_with_voltage"
    else:
        rpm_per_v = label["output_no_load_rpm"] / label["reference_voltage_v"]
        source = "label_scaled_linearly_with_voltage_not_measured"

    rpm = rpm_per_v * sim["supply_voltage_v"]
    rad_s = rpm * 2 * math.pi / 60

    return {
        "no_load_estimate_source": source,
        "output_no_load_rpm_estimate": rpm,
        "output_no_load_rad_s_estimate": rad_s,
        "wheel_rim_speed_m_s_estimate": rad_s * hw["wheel_radius_m"],
        "mecanum_half_wheelbase_plus_half_track_m": (
            hw["wheelbase_m"] + hw["track_width_m"]
        )
        / 2,
        "effective_wheel_radii_m": [
            hw["wheel_radius_m"] * scale
            for scale in calibration["effective_wheel_radius_scale"]
        ],
        "wheel_torque_limits_nm": [
            servo["torque_limit_nm"] * scale
            for scale in servo["per_wheel_torque_scale"]
        ],
    }


# ---------------------------------------------------------------------------
# 对外入口
# ---------------------------------------------------------------------------


def load_parameters(config=DEFAULT_CONFIG, override=None):
    """读取、校验并派生机器人参数（SI 单位）。

    处理顺序：
        1. 读取标称配置。
        2. 应用可选的 override 配置。
        3. 校验全部参数。
        4. 计算派生物理量。

    返回：
        一份全新的参数字典；只有校验通过后才会挂上 derived 字段。
    """
    # 1. 标称配置。
    params = _read(config)

    # 2. 用户/标定 override。
    if override is not None:
        _merge(params, _read(override))

    # 3. 拒绝非法或物理上自相矛盾的配置。
    _validate_parameters(params)

    # 4. 只有校验通过后才计算派生量。
    params["derived"] = _compute_derived(params)

    return params


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--override", type=Path)
    args = parser.parse_args()
    try:
        result = load_parameters(args.config, args.override)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
