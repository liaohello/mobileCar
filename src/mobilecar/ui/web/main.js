import { MobileCarApi, episodeAssetUrl, randomId } from "./api.js";
import { SceneView } from "./scene-view.js";

const $ = (id) => document.getElementById(id);
const DRIVE_KEYS = new Set(["w", "a", "s", "d", "q", "e"]);
const PHASE_NAMES = ["SEARCH", "NAVIGATE", "OBSERVE", "APPROACH", "MANIPULATE", "VERIFY"];
const JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5"];

function isEditable(target) {
  if (!(target instanceof HTMLElement)) return false;
  return Boolean(
    target.closest("input, textarea, select, [contenteditable='true'], [role='textbox']"),
  );
}

function fixed(value, digits = 3, fallback = "—") {
  return Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : fallback;
}

function millimetres(value) {
  return Number.isFinite(Number(value)) ? `${(Number(value) * 1000).toFixed(1)} mm` : "—";
}

function shortId(value, length = 15) {
  if (!value) return "—";
  const text = String(value);
  return text.length <= length ? text : `${text.slice(0, length)}…`;
}

function utcNow() {
  return new Date().toISOString();
}

function setText(node, value) {
  if (typeof node === "string") node = $(node);
  if (node) node.textContent = value == null ? "—" : String(value);
}

function setTone(node, tone) {
  if (typeof node === "string") node = $(node);
  if (!node) return;
  node.dataset.tone = tone;
}

class MobileCarUi {
  constructor() {
    this.api = new MobileCarApi();
    this.sessionId = randomId("session");
    this.commandSeq = 0;
    this.runId = null;
    this.episodeId = null;
    this.backendGeneration = null;
    this.capabilities = null;
    this.manifest = null;
    this.snapshot = null;
    this.lastSnapshotAt = null;
    this.connected = false;
    this.pollInFlight = false;
    this.pollIntervalMs = 100;
    this.failedPolls = 0;
    this.keys = new Set();
    this.driveTimer = null;
    this.driveInFlight = false;
    this.heartbeatTimer = null;
    this.lastDriveCommandAt = 0;
    this.pendingPlan = null;
    this.currentPlan = null;
    this.targetTouched = false;
    this.target = [0.2, 0.0, 0.2];
    this.jointDirty = false;
    this.gripperDirty = false;
    this.currentEpisodeId = null;
    this.currentReplay = null;
    this.toastTimer = null;
    this.commandChain = Promise.resolve();
    this.heartbeatInFlight = false;
    this.planEpoch = 0;
    this.controlEpoch = 0;
    this.leaseIntentUntil = 0;
    this.g4a1Observation = null;
    this.g4a1Permission = null;
    this.g4a1Seq = 0;
    this.lastG4A1PollAt = 0;
    this.pendingG4A1PlanId = null;
    this.sceneView = new SceneView($("three-container"), {
      onTargetDrag: (position) => this.onTargetDragged(position),
      onNotice: (message) => this.toast(message),
    });
    this.bindEvents();
    this.addEvent("系统", "界面已启动，等待后端 bootstrap。", "pending");
    this.boot();
    this.startPolling();
    this.startHeartbeat();
  }

  bindEvents() {
    document.addEventListener("keydown", (event) => this.onKeyDown(event));
    document.addEventListener("keyup", (event) => this.onKeyUp(event));
    window.addEventListener("blur", () => this.releaseControls("窗口失焦"));
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) this.releaseControls("页面隐藏");
    });

    $("stop-button").addEventListener("click", () => this.stopAll());
    $("reset-button").addEventListener("click", () => this.resetEpisode());
    $("unlock-stop").addEventListener("click", () => this.unlockStop());
    $("cancel-arm").addEventListener("click", () => this.cancelMotion());
    $("execute-joints").addEventListener("click", () => this.executeJointGoal());
    $("execute-gripper").addEventListener("click", () => this.executeGripper());
    $("preview-tcp").addEventListener("click", () => this.previewTcp());
    $("execute-tcp").addEventListener("click", () => this.executeTcp());
    $("set-phase").addEventListener("click", () => this.sendTaskPhase());
    $("start-recording").addEventListener("click", () => this.startRecording());
    $("stop-recording").addEventListener("click", () => this.stopRecording());
    $("clear-events").addEventListener("click", () => $("event-list").replaceChildren());
    $("fit-view").addEventListener("click", () => this.sceneView.fitView());
    $("replay-open").addEventListener("click", () => this.openReplay());
    $("g4a2-link")?.addEventListener("click", (event) => {
      if (this.snapshot?.recording?.status === "RECORDING") {
        event.preventDefault();
        this.toast("请先停止并结束当前录制，再进入受限近场采集页面。");
      }
    });
    $("replay-close").addEventListener("click", () => $("replay-dialog").close());
    $("refresh-episodes").addEventListener("click", () => this.loadEpisodeIndex());
    $("validate-episode").addEventListener("click", () => this.loadReplayReport("validate"));
    $("load-assessment").addEventListener("click", () => this.loadReplayReport("assessment"));
    $("load-eligibility").addEventListener("click", () => this.loadReplayReport("eligibility"));
    $("g4a1-authorize").addEventListener("click", () => this.authorizeG4A1());
    $("g4a1-execute").addEventListener("click", () => this.executeG4A1());
    $("g4a1-revoke").addEventListener("click", () => this.revokeG4A1());
    $("operator-kind").addEventListener("change", () => {
      setText("operator-badge", $("operator-kind").value.toUpperCase());
    });
    $("gripper-slider").addEventListener("input", () => {
      this.gripperDirty = true;
      setText("gripper-value", millimetres(this.readGripper()));
    });
    for (const id of ["tcp-x", "tcp-y", "tcp-z"]) {
      $(id).addEventListener("input", () => this.onTargetInput());
    }
    $("tcp-frame").addEventListener("change", () => {
      this.onTargetInput();
      setText("target-frame-badge", $("tcp-frame").value.toUpperCase());
    });
    document.querySelectorAll("[data-drive-key]").forEach((button) => {
      const key = button.dataset.driveKey;
      button.addEventListener("pointerdown", (event) => {
        event.preventDefault();
        button.setPointerCapture(event.pointerId);
        this.pressDriveKey(key);
      });
      const release = () => this.releaseDriveKey(key);
      button.addEventListener("pointerup", release);
      button.addEventListener("pointercancel", release);
    });
  }

  async boot() {
    try {
      const [health, capabilities, manifest, snapshot] = await Promise.all([
        this.api.health(),
        this.api.capabilities(),
        this.api.scene(),
        this.api.snapshot(),
      ]);
      this.capabilities = capabilities;
      this.manifest = manifest;
      if (health.ui) {
        this.sceneView.configureRendering(health.ui);
        this.pollIntervalMs = Math.max(
          33,
          1000 / Number(health.ui.snapshot_hz || 15),
        );
        if (this.pollTimer) window.clearInterval(this.pollTimer);
        this.pollTimer = window.setInterval(
          () => this.pollSnapshot(),
          this.pollIntervalMs,
        );
      }
      this.sceneView.setManifest(manifest);
      this.buildJointControls();
      this.buildGripperControl();
      this.applySnapshot(snapshot);
      this.setConnected(true);
      setText("backend-name", capabilities.backend_kind);
      setText("model-hash", `model ${shortId(capabilities.model_sha256, 12)}`);
      this.addEvent(
        "bootstrap",
        `${capabilities.backend_kind} · ${capabilities.model_id} · run ${shortId(health.run_id, 12)}`,
        "ok",
      );
      if (capabilities.model_sha256 !== manifest.model_sha256) {
        this.addEvent("模型", "Capabilities 与 SceneManifest 模型哈希不一致。", "error");
      }
    } catch (error) {
      this.setConnected(false);
      this.addEvent("bootstrap", `后端不可用：${error.message}`, "error");
      this.toast(`无法连接应用：${error.message}`);
    }
  }

  startPolling() {
    this.pollSnapshot();
    this.pollTimer = window.setInterval(
      () => this.pollSnapshot(),
      this.pollIntervalMs,
    );
  }

  async pollSnapshot() {
    if (this.pollInFlight) return;
    this.pollInFlight = true;
    try {
      const snapshot = await this.api.snapshot();
      this.failedPolls = 0;
      this.setConnected(true);
      this.applySnapshot(snapshot);
    } catch (error) {
      this.failedPolls += 1;
      if (this.failedPolls >= 3) {
        this.setConnected(false);
        this.releaseControls("控制连接中断", false);
      }
    } finally {
      this.pollInFlight = false;
    }
  }

  setConnected(connected) {
    if (this.connected === connected) return;
    this.connected = connected;
    const dot = $("connection-dot");
    dot.classList.toggle("ok", connected);
    dot.classList.toggle("error", !connected);
    dot.classList.toggle("pending", false);
    setText("connection-text", connected ? "已连接" : "已断开");
    $("connection-banner").hidden = connected;
    if (connected) this.addEvent("连接", "快照流已连接；旧输入不会恢复。", "ok");
    else this.addEvent("连接", "快照流中断，驾驶输入已释放。", "error");
  }

  applySnapshot(snapshot) {
    if (!snapshot) return;
    const previousEpisode = this.episodeId;
    this.snapshot = snapshot;
    this.runId = snapshot.run_id;
    this.episodeId = snapshot.episode_id;
    this.backendGeneration = snapshot.backend_generation;
    this.lastSnapshotAt = performance.now();
    this.sceneView.updateSnapshot(snapshot);
    if (performance.now() - this.lastG4A1PollAt >= 250) {
      this.lastG4A1PollAt = performance.now();
      this.pollG4A1();
    }
    setText("snapshot-age", `${Math.max(0, performance.now() - this.lastSnapshotAt).toFixed(0)} ms`);
    setText("episode-short", shortId(snapshot.episode_id));
    setText("base-vx", fixed(snapshot.base?.vx_m_s, 3));
    setText("base-vy", fixed(snapshot.base?.vy_m_s, 3));
    setText("base-wz", fixed(snapshot.base?.wz_rad_s, 3));
    setText("base-stopped", snapshot.base?.stopped ? "是" : "否");
    setTone("base-stopped", snapshot.base?.stopped ? "ok" : "warn");
    setText("control-tick", shortId(snapshot.control_tick_id, 12));
    setText("tcp-actual", (snapshot.tcp?.position_m || []).map((v) => fixed(v, 3)).join("  "));
    setText("tcp-error", millimetres(snapshot.tcp?.position_error_m));
    setText(
      "orientation-error",
      Number.isFinite(snapshot.tcp?.orientation_error_rad)
        ? `${((snapshot.tcp.orientation_error_rad * 180) / Math.PI).toFixed(2)}°`
        : "—",
    );
    const margins = snapshot.arm?.joint_margin_rad || [];
    setText("joint-margin", margins.length ? `${((Math.min(...margins) * 180) / Math.PI).toFixed(2)}°` : "—");
    setText("generation", snapshot.backend_generation);
    setText("sim-time", `${fixed(snapshot.sim_time_s, 3)} s`);
    setText("wall-time", snapshot.wall_utc);
    setText("contact-summary", snapshot.contact_summary?.unexpected_contact ? "非预期" : "未见异常");
    setTone("contact-summary", snapshot.contact_summary?.unexpected_contact ? "error" : "ok");
    setText("owner-session", this.sessionId === snapshot.base_owner_session_id ||
      this.sessionId === snapshot.arm_owner_session_id ? "本会话" : shortId(snapshot.active_session_id));
    setText("owner-base", snapshot.base_owner_session_id ? shortId(snapshot.base_owner_session_id) : "未占用");
    setText("owner-arm", snapshot.arm_owner_session_id ? shortId(snapshot.arm_owner_session_id) : "未占用");
    setText("owner-operator", snapshot.active_operator_kind || "—");
    setText("safety-state", snapshot.safety_latched ? "已锁存" : "正常");
    setTone("safety-state", snapshot.safety_latched ? "error" : "ok");
    this.applyRecording(snapshot.recording);
    this.renderSensors(snapshot.sensors || []);
    this.syncJointInputs(snapshot);
    this.syncGripperInput(snapshot);
    if (previousEpisode && previousEpisode !== snapshot.episode_id) {
      this.planEpoch += 1;
      this.leaseIntentUntil = 0;
      this.pendingPlan = null;
      this.currentPlan = null;
      this.sceneView.showPlan(null);
      $("execute-tcp").disabled = true;
      this.addEvent("Episode", `切换到 ${snapshot.episode_id}；旧计划已作废。`, "info");
    }
    if (snapshot.safety_latched) {
      this.releaseDrive("软件停止锁存");
      if (this.pendingPlan || this.currentPlan) this.invalidatePlan("软件停止锁存");
      this.leaseIntentUntil = 0;
    }
    if (snapshot.task_phase && document.activeElement !== $("task-phase")) {
      $("task-phase").value = snapshot.task_phase;
    }
    if (
      snapshot.plan
      && (
        (!this.pendingPlan && !this.targetTouched)
        || this.pendingPlan?.plan_id === snapshot.plan.plan_id
      )
    ) {
      this.showPlan(snapshot.plan);
    }
    if (!this.targetTouched) {
      const tcp = snapshot.tcp?.position_m;
      if (tcp?.length === 3) this.setTargetInputs(tcp);
    }
    this.updateSnapshotAge();
  }

  updateSnapshotAge() {
    if (!this.lastSnapshotAt) return;
    setText("snapshot-age", `${Math.max(0, performance.now() - this.lastSnapshotAt).toFixed(0)} ms`);
  }

  applyRecording(recording = {}) {
    const status = recording.status || "IDLE";
    setText("recording-state", status);
    setTone("recording-state", status === "RECORDING" ? "record" : status === "FAILED" ? "error" : "ok");
    if (recording.error) this.addEvent("录制", recording.error, "error");
  }

  buildJointControls() {
    const limits = this.capabilities?.arm_user_limits_rad || JOINT_NAMES.map(() => [-2.0, 2.0]);
    const root = $("joint-sliders");
    root.replaceChildren();
    JOINT_NAMES.forEach((name, index) => {
      const [low, high] = limits[index] || [-2, 2];
      const row = document.createElement("label");
      row.className = "joint-row";
      row.innerHTML = `<span>${name}</span><input type="range" min="${low}" max="${high}" step="0.005" value="0" data-joint="${index}" /><output>0.0°</output>`;
      const input = row.querySelector("input");
      const output = row.querySelector("output");
      input.addEventListener("input", () => {
        this.jointDirty = true;
        output.textContent = `${((Number(input.value) * 180) / Math.PI).toFixed(1)}°`;
      });
      root.appendChild(row);
    });
  }

  buildGripperControl() {
    const slider = $("gripper-slider");
    slider.min = String(this.capabilities?.gripper_min_width_m ?? 0.005);
    slider.max = String(this.capabilities?.gripper_max_width_m ?? 0.05);
    slider.value = String((this.capabilities?.gripper_min_width_m + this.capabilities?.gripper_max_width_m) / 2 || 0.025);
    setText("gripper-value", millimetres(Number(slider.value)));
  }

  syncJointInputs(snapshot) {
    if (this.jointDirty) return;
    const values = snapshot.arm?.user_joint_positions_rad || [];
    document.querySelectorAll("[data-joint]").forEach((input, index) => {
      if (Number.isFinite(values[index])) {
        input.value = String(values[index]);
        input.nextElementSibling.textContent = `${((values[index] * 180) / Math.PI).toFixed(1)}°`;
      }
    });
  }

  syncGripperInput(snapshot) {
    if (this.gripperDirty) return;
    const width = snapshot.gripper?.width_m;
    if (Number.isFinite(width)) {
      $("gripper-slider").value = String(width);
      setText("gripper-value", millimetres(width));
    }
  }

  async pollG4A1() {
    try {
      const status = await this.api.g4a1Status();
      const observation = await this.api.g4a1Observation();
      this.g4a1Observation = observation;
      $("g4a1-rgb").src = `${observation.rgb_uri}?sample=${encodeURIComponent(observation.sample_id)}`;
      setText("g4a1-sample", shortId(observation.sample_id, 18));
      setText("g4a1-valid", `${(observation.valid_fraction * 100).toFixed(1)}%`);
      setText("g4a1-depth", millimetres(observation.center_depth_m));
      setText("g4a1-status", status.permission_active ? "AUTHORIZED" : "OBSERVE");
      setTone("g4a1-status", status.permission_active ? "ok" : "warn");
      if (!status.permission_active) this.g4a1Permission = null;
    } catch (_) {
      setText("g4a1-status", "NO RGB-D");
      setTone("g4a1-status", "warn");
    }
  }

  async authorizeG4A1() {
    const observation = await this.api.g4a1Observation();
    this.g4a1Observation = observation;
    const permission = await this.api.g4a1Permission({
      interface_profile: "g4a1-1.0",
      run_id: observation.run_id,
      episode_id: observation.episode_id,
      backend_generation: observation.backend_generation,
      session_id: this.sessionId,
      operator_id: $("operator-id").value,
      operator_kind: $("operator-kind").value,
      source: $("operator-kind").value,
      view_source: "link4_rgbd_proprio",
      observation_profile_id: "g4a1-link4-rgbd-proprio-v1",
      observation_profile_sha256: observation.observation_profile_sha256,
      action_profile_id: "g4a1-sixdim-absolute-goal-v1",
      action_profile_sha256: observation.action_profile_sha256,
    });
    if (!permission.accepted) throw new Error(permission.reason);
    this.g4a1Permission = permission;
    this.g4a1Seq = 0;
    this.addEvent("G4A-1", `近场权限 ${permission.permission_id}`, "ok");
  }

  async executeG4A1() {
    if (!this.g4a1Permission) await this.authorizeG4A1();
    let observation;
    for (let attempt = 0; attempt < 12; attempt += 1) {
      observation = await this.api.g4a1Observation();
      const age = observation.observation_backend_monotonic_s - observation.capture_backend_monotonic_s;
      if (age <= 0.35) break;
      await new Promise((resolve) => setTimeout(resolve, 100));
    }
    if (!observation || observation.observation_backend_monotonic_s - observation.capture_backend_monotonic_s > 0.4) {
      throw new Error("没有足够新鲜的 link4 RGB-D 样本");
    }
    this.g4a1Observation = observation;
    this.g4a1Seq += 1;
    const j = this.readJointGoal();
    const decision = observation.arrival_backend_monotonic_s + 0.000001;
    const receipt = await this.api.g4a1Action({
      interface_profile: "g4a1-1.0",
      run_id: observation.run_id,
      episode_id: observation.episode_id,
      backend_generation: observation.backend_generation,
      nearfield_epoch: this.g4a1Permission.nearfield_epoch,
      permission_id: this.g4a1Permission.permission_id,
      session_id: this.sessionId,
      action_frame_id: randomId("action_g4a1"),
      action_seq: this.g4a1Seq,
      teacher_plan_id: this.pendingG4A1PlanId,
      observation_sample_id: observation.sample_id,
      observation_capture_control_tick_id: observation.capture_control_tick_id,
      observation_arrival_backend_monotonic_s: observation.arrival_backend_monotonic_s,
      decision_backend_monotonic_s: decision,
      issued_backend_monotonic_s: decision,
      expires_backend_monotonic_s: decision + 0.4,
      observation_profile_sha256: observation.observation_profile_sha256,
      action_profile_sha256: observation.action_profile_sha256,
      source: "agent_ui",
      joint_order: JOINT_NAMES,
      units: ["rad", "rad", "rad", "rad", "rad", "m"],
      arm_user_joint_positions_rad: j,
      gripper_width_m: this.readGripper(),
    });
    if (!receipt.accepted) throw new Error(receipt.reason);
    this.addEvent("G4A-1", `六维动作 ${receipt.action_seq} 已下发`, "ok");
  }

  async revokeG4A1() {
    await this.api.g4a1Revoke("ui_cancel");
    this.g4a1Permission = null;
    this.addEvent("G4A-1", "近场权限已撤销，保持最后目标", "warn");
  }

  renderSensors(sensors) {
    const root = $("sensor-list");
    root.replaceChildren();
    if (!sensors.length) {
      root.innerHTML = '<span class="muted">没有传感器状态</span>';
      return;
    }
    for (const sensor of sensors) {
      const row = document.createElement("div");
      row.className = "sensor-row";
      const state = sensor.available && sensor.fresh ? "ok" : sensor.available ? "stale" : "missing";
      row.innerHTML = `<span class="sensor-state ${state}"></span><span>${sensor.kind}</span><strong>${sensor.quality || "unknown"}</strong>`;
      row.title = sensor.sample_id || "";
      root.appendChild(row);
    }
  }

  movement() {
    if (!this.connected || !this.snapshot || this.snapshot.safety_latched) return null;
    const x = (this.keys.has("w") ? 1 : 0) + (this.keys.has("s") ? -1 : 0);
    const y = (this.keys.has("a") ? 1 : 0) + (this.keys.has("d") ? -1 : 0);
    const yaw = (this.keys.has("q") ? 1 : 0) + (this.keys.has("e") ? -1 : 0);
    if (!x && !y && !yaw) return { vx: 0, vy: 0, wz: 0 };
    const linear = Math.hypot(x, y) || 1;
    return {
      vx: (x / linear) * (this.capabilities?.base_linear_limit_m_s ?? 0.05),
      vy: (y / linear) * (this.capabilities?.base_linear_limit_m_s ?? 0.05),
      wz: yaw * (this.capabilities?.base_yaw_limit_rad_s ?? 0.3),
    };
  }

  onKeyDown(event) {
    if (isEditable(event.target)) return;
    const key = event.key.toLowerCase();
    if (!DRIVE_KEYS.has(key)) return;
    event.preventDefault();
    if (!event.repeat) this.pressDriveKey(key);
  }

  onKeyUp(event) {
    const key = event.key.toLowerCase();
    if (!DRIVE_KEYS.has(key)) return;
    this.releaseDriveKey(key, { sendZero: this.keys.size === 1 });
  }

  pressDriveKey(key) {
    if (!this.connected || this.snapshot?.safety_latched) {
      this.toast(this.snapshot?.safety_latched ? "软件停止已锁存，请先解除。" : "控制连接未就绪。");
      return;
    }
    this.keys.add(key);
    this.ensureDriveTimer();
    this.sendDriveTick();
  }

  releaseDriveKey(key, { sendZero = true } = {}) {
    const had = this.keys.delete(key);
    if (!this.keys.size) {
      this.stopDriveTimer();
      if (sendZero && had) this.sendZeroBase("松键");
    }
  }

  releaseDrive(reason) {
    const hadInput = this.keys.size > 0;
    this.keys.clear();
    this.stopDriveTimer();
    if (hadInput) {
      this.addEvent("驾驶", `${reason}，全部驾驶输入释放。`, "warn");
      if (this.connected) this.sendZeroBase(reason);
    }
  }

  releaseControls(reason, sendStop = true) {
    const hadMotion = this.keys.size > 0 || Boolean(this.pendingPlan || this.currentPlan) || this.driveInFlight;
    this.keys.clear();
    this.stopDriveTimer();
    this.leaseIntentUntil = 0;
    this.invalidatePlan(reason);
    if (hadMotion) {
      this.addEvent("安全", `${reason}，驾驶、计划与租约意图已释放。`, "warn");
    }
    if (!sendStop || !this.connected) return;
    this.sendZeroBase(reason);
    this.sendCommand("cancel", { reason }, 0.5, { quiet: true, priority: true })
      .catch((error) => this.addEvent("安全", `取消动作失败：${error.message}`, "error"));
  }

  ensureDriveTimer() {
    if (this.driveTimer) return;
    this.driveTimer = window.setInterval(() => this.sendDriveTick(), 150);
  }

  stopDriveTimer() {
    if (this.driveTimer) window.clearInterval(this.driveTimer);
    this.driveTimer = null;
  }

  async sendDriveTick() {
    if (this.driveInFlight) return;
    const movement = this.movement();
    if (!movement) return;
    if (movement.vx === 0 && movement.vy === 0 && movement.wz === 0) return;
    if (this.pendingPlan || this.currentPlan) this.invalidatePlan("底盘驾驶接管");
    this.driveInFlight = true;
    try {
      await this.sendCommand(
        "set_base_twist",
        { frame_id: "base_link", vx_m_s: movement.vx, vy_m_s: movement.vy, wz_rad_s: movement.wz },
        0.5,
        { quiet: true, priority: true },
      );
      this.lastDriveCommandAt = performance.now();
    } catch (error) {
      this.addEvent("驾驶", error.message, "error");
    } finally {
      this.driveInFlight = false;
    }
  }

  sendZeroBase(reason) {
    this.sendCommand("zero_base", { reason }, 0.5, {
      quiet: true,
      priority: true,
    }).catch((error) => {
      this.addEvent("驾驶", `零速请求失败：${error.message}`, "error");
    });
  }

  startHeartbeat() {
    this.heartbeatTimer = window.setInterval(() => {
      if (!this.connected || !this.snapshot) return;
      if (this.heartbeatInFlight) return;
      const ownsBase = this.snapshot.base_owner_session_id === this.sessionId;
      const ownsArm = this.snapshot.arm_owner_session_id === this.sessionId;
      // A preview acquires arm ownership before the next snapshot poll. A long
      // plan must not lose its lease in that window, so renew while a reviewed
      // plan is pending/executing even if the snapshot owner fields lag.
      const planNeedsLease = Boolean(
        this.pendingPlan && !["REACHED", "FAILED", "CANCELLED"].includes(this.pendingPlan.status),
      );
      const intentNeedsLease = performance.now() < this.leaseIntentUntil;
      if (!ownsBase && !ownsArm && !planNeedsLease && !intentNeedsLease) return;
      if (performance.now() - this.lastDriveCommandAt < 240) return;
      this.heartbeatInFlight = true;
      this.api.renewLease(this.sessionId)
        .catch(() => {})
        .finally(() => {
          this.heartbeatInFlight = false;
        });
    }, 150);
  }

  sendCommand(type, payload = {}, ttl = 0.5, { quiet = false, priority = false } = {}) {
    if (!this.snapshot) return Promise.reject(new Error("快照尚未就绪"));
    const planEpoch = this.planEpoch;
    const controlEpoch = this.controlEpoch;
    const dispatch = async () => {
      if (
        controlEpoch !== this.controlEpoch &&
        !["stop", "cancel", "zero_base", "reset"].includes(type)
      ) {
        throw new Error("请求已被停止/复位事件作废");
      }
      if (planEpoch !== this.planEpoch && type === "preview_tcp_target") {
        throw new Error("预览请求已被更新的目标/接管事件作废");
      }
      this.commandSeq += 1;
      const request = {
        contract_version: "mobilecar-1.0",
        run_id: this.snapshot.run_id,
        backend_generation: this.snapshot.backend_generation,
        episode_id: this.snapshot.episode_id,
        session_id: this.sessionId,
        operator_id: $("operator-id").value.trim() || "agent_ui_operator",
        operator_kind: $("operator-kind").value,
        view_source: "third_person_3d",
        command_id: randomId("cmd"),
        command_seq: this.commandSeq,
        command_type: type,
        requested_ttl_s: ttl,
        payload,
      };
      const response = await this.api.submitCommand(request);
      this.handleCommandResponse(request, response, quiet, { planEpoch, controlEpoch });
      return response;
    };
    if (priority) return dispatch();
    // Concurrent keyboard ticks and heartbeats must not overtake each other:
    // command_seq is a session order, not a client wall-clock hint.
    const result = this.commandChain.catch(() => {}).then(dispatch);
    this.commandChain = result.catch(() => {});
    return result;
  }

  handleCommandResponse(request, response, quiet, epochs = {}) {
    const status = response.status || "UNKNOWN";
    const tone = ["REJECTED", "FAILED", "EXPIRED_BEFORE_DISPATCH", "LEASE_EXPIRED"].includes(status)
      ? "error"
      : "ok";
    if (!quiet) {
      this.addEvent(
        request.command_type,
        `${status}${response.reason ? ` · ${response.reason}` : ""}`,
        tone,
      );
    }
    if (response.decision?.approved === false) {
      setText("safety-reason", `拒绝：${response.decision.reason || response.reason || "未说明"}`);
      setTone("safety-reason", "error");
    } else if (!quiet) {
      setText("safety-reason", response.reason || `安全裁定：${status}`);
      setTone("safety-reason", tone);
    }
    const planIsCurrent = epochs.planEpoch === this.planEpoch;
    const controlIsCurrent = epochs.controlEpoch === this.controlEpoch;
    if (response.plan && planIsCurrent && controlIsCurrent) {
      this.pendingPlan = response.plan;
      this.showPlan(response.plan);
      $("execute-tcp").disabled = !response.plan.collision_free;
    } else if (response.plan && (!planIsCurrent || !controlIsCurrent)) {
      this.addEvent("TCP", "返回的计划属于已失效目标/停止周期，拒绝采用。", "warn");
    }
    if (status === "REJECTED" || status === "FAILED" || status === "EXPIRED_BEFORE_DISPATCH") {
      if (request.command_type === "execute_tcp_target") {
        this.invalidatePlan(response.reason || "执行被拒绝");
      }
      this.toast(response.reason || status);
    }
  }

  async stopAll() {
    this.releaseDrive("人工停止");
    this.controlEpoch += 1;
    this.leaseIntentUntil = 0;
    this.invalidatePlan("人工停止");
    try {
      await this.sendCommand(
        "stop",
        { latch: true, reason: "ui_stop" },
        0.5,
        { priority: true },
      );
      this.toast("停止请求已发送；夹爪保持当前策略。");
    } catch (error) {
      this.addEvent("停止", error.message, "error");
    }
  }

  async unlockStop() {
    try {
      await this.sendCommand(
        "stop",
        { latch: false, reason: "ui_software_stop_reset" },
        0.5,
        { priority: true },
      );
    } catch (error) {
      this.addEvent("安全", error.message, "error");
    }
  }

  async cancelMotion() {
    try {
      await this.sendCommand(
        "cancel",
        { reason: "operator_cancel" },
        0.5,
        { priority: true },
      );
      this.invalidatePlan("人工取消");
    } catch (error) {
      this.addEvent("取消", error.message, "error");
    }
  }

  async resetEpisode() {
    this.releaseControls("Reset");
    this.controlEpoch += 1;
    try {
      await this.sendCommand(
        "reset",
        { confirm: true },
        1.0,
        { priority: true },
      );
      this.invalidatePlan("Reset");
      this.addEvent("Episode", "Reset 完成；旧 Episode 目标与计划不再有效。", "warn");
    } catch (error) {
      this.addEvent("Episode", error.message, "error");
    }
  }

  readJointGoal() {
    return Array.from(document.querySelectorAll("[data-joint]")).map((input) => Number(input.value));
  }

  async executeJointGoal() {
    const values = this.readJointGoal();
    try {
      await this.sendCommand(
        "set_arm_joint_goal",
        { joint_names: JOINT_NAMES, joint_positions_rad: values },
        0.5,
      );
      this.jointDirty = false;
    } catch (error) {
      this.addEvent("五关节", error.message, "error");
    }
  }

  readGripper() {
    return Number($("gripper-slider").value);
  }

  async executeGripper() {
    try {
      await this.sendCommand("set_gripper_width_goal", { width_m: this.readGripper() }, 0.5);
      this.gripperDirty = false;
    } catch (error) {
      this.addEvent("夹爪", error.message, "error");
    }
  }

  onTargetInput() {
    this.target = ["tcp-x", "tcp-y", "tcp-z"].map((id) => Number($(id).value));
    if (this.target.some((value) => !Number.isFinite(value))) return;
    this.targetTouched = true;
    this.invalidatePlan();
    this.sceneView.setTarget(this.target, true);
  }

  onTargetDragged(position) {
    this.target = position;
    this.targetTouched = true;
    this.setTargetInputs(position);
    this.invalidatePlan();
    this.addEvent("TCP", `目标拖动至 ${position.map((v) => fixed(v, 3)).join(", ")}`, "info");
  }

  setTargetInputs(position) {
    const ids = ["tcp-x", "tcp-y", "tcp-z"];
    ids.forEach((id, index) => {
      if (document.activeElement !== $(id)) $(id).value = fixed(position[index], 3, "0");
    });
    this.target = [...position];
    this.sceneView.setTarget(position, true);
  }

  invalidatePlan(reason = "目标已修改") {
    this.planEpoch += 1;
    this.leaseIntentUntil = 0;
    this.pendingPlan = null;
    this.currentPlan = null;
    this.sceneView.showPlan(null);
    $("execute-tcp").disabled = true;
    setText("plan-id", "—");
    setText("plan-status", reason);
    $("plan-summary").textContent = `${reason}，需要重新预览安全计划。`;
    $("plan-summary").classList.add("empty");
  }

  tcpPayload() {
    const position = ["tcp-x", "tcp-y", "tcp-z"].map((id) => Number($(id).value));
    if (position.some((value) => !Number.isFinite(value))) throw new Error("TCP 坐标必须为有限数值");
    return { frame_id: $("tcp-frame").value, position_m: position };
  }

  async previewTcp() {
    try {
      this.target = this.tcpPayload().position_m;
      this.sceneView.setTarget(this.target, true);
      this.leaseIntentUntil = performance.now() + 30_000;
      // A human must inspect the preview before dispatch. Keep the initial
      // command lease within the backend's 30 s single-action budget.
      await this.sendCommand("preview_tcp_target", this.tcpPayload(), 30.0);
    } catch (error) {
      this.addEvent("TCP 预览", error.message, "error");
      this.toast(error.message);
    }
  }

  async executeTcp() {
    if (!this.pendingPlan) {
      this.toast("请先预览并检查同一目标。");
      return;
    }
    try {
      this.leaseIntentUntil = performance.now() + 30_000;
      await this.sendCommand(
        "execute_tcp_target",
        { ...this.tcpPayload(), plan_id: this.pendingPlan.plan_id },
        1.0,
      );
    } catch (error) {
      this.addEvent("TCP 执行", error.message, "error");
    }
  }

  showPlan(plan) {
    this.currentPlan = plan;
    this.sceneView.showPlan(plan);
    $("execute-tcp").disabled = !(plan.collision_free && plan.expires_backend_monotonic_s > 0);
    setText("plan-id", shortId(plan.plan_id, 24));
    setText("plan-status", plan.status || "PREVIEWED");
    setText("plan-position-priority", "position_first");
    setText("plan-collision", plan.collision_free ? "通过" : (plan.collision_details || []).join(", ") || "拒绝");
    setTone("plan-collision", plan.collision_free ? "ok" : "error");
    setText("plan-predicted-error", millimetres(plan.predicted_position_error_m));
    setText(
      "plan-predicted-orientation",
      `${((plan.predicted_orientation_error_rad * 180) / Math.PI).toFixed(2)}°`,
    );
    const summary = $("plan-summary");
    summary.classList.remove("empty");
    summary.innerHTML = "";
    const title = document.createElement("strong");
    title.textContent = `${plan.collision_free ? "检查通过" : "检查拒绝"} · ${plan.waypoints_model_rad?.length || 0} waypoints`;
    const detail = document.createElement("span");
    detail.textContent = `位置预测 ${millimetres(plan.predicted_position_error_m)} · 余量 ${((Math.min(...(plan.joint_margin_rad || [0])) * 180) / Math.PI).toFixed(2)}°`;
    summary.append(title, detail);
    if (plan.target_revision) setText("safety-reason", `目标修订 ${plan.target_revision} 已通过同一轨迹预检。`);
  }

  async sendTaskPhase() {
    try {
      await this.sendCommand("set_task_phase", { phase: $("task-phase").value }, 0.5);
    } catch (error) {
      this.addEvent("阶段", error.message, "error");
    }
  }

  async startRecording() {
    this.releaseControls("开始新录制");
    try {
      const response = await this.api.startEpisode({
        task_language: $("task-language").value.trim(),
        operator_id: $("operator-id").value.trim() || "agent_ui_operator",
        operator_kind: $("operator-kind").value,
        view_source: "third_person_3d",
        map_mode: "unavailable",
        require_recording: $("require-recording").checked,
      });
      if (!response.accepted) throw new Error(response.reason || "Recorder 拒绝开始");
      this.applyRecording({ status: response.recording_status, episode_id: response.episode_id });
      this.addEvent(
        "录制",
        `起始状态已确认 · ${shortId(response.episode_id)} · segment ${shortId(response.recording_segment_id)}`,
        "ok",
      );
    } catch (error) {
      this.addEvent("录制", error.message, "error");
      this.toast(error.message);
    }
  }

  async stopRecording() {
    this.releaseControls("结束录制");
    try {
      const response = await this.api.stopEpisode("OPERATOR_FINISHED");
      this.applyRecording({ status: response.recording_status, error: response.accepted ? null : response.reason });
      this.addEvent(
        "录制",
        response.accepted ? "停车、排空与片段结束请求已确认。" : `结束被拒绝：${response.reason}`,
        response.accepted ? "ok" : "error",
      );
    } catch (error) {
      this.addEvent("录制", error.message, "error");
    }
  }

  async openReplay() {
    const dialog = $("replay-dialog");
    // Keep the live safety controls outside the dialog operable: replay is
    // read-only, but stopping teleoperation must never be blocked by a modal.
    if (!dialog.open) dialog.show();
    await this.loadEpisodeIndex();
  }

  async loadEpisodeIndex() {
    const root = $("episode-list");
    root.innerHTML = '<span class="muted">读取索引…</span>';
    try {
      const response = await this.api.listEpisodes();
      const episodes = Array.isArray(response) ? response : response.episodes || [];
      root.replaceChildren();
      if (!episodes.length) {
        root.innerHTML = '<span class="muted">没有可用 Episode</span>';
        return;
      }
      for (const episode of episodes) {
        const id = episode.episode_id || episode.id;
        if (!id) continue;
        const button = document.createElement("button");
        button.className = "episode-item";
        button.dataset.episodeId = id;
        button.innerHTML = `<strong>${id}</strong><span>${episode.task_result || episode.audit_status || "unknown"}</span><small>${episode.operator_kind || ""}</small>`;
        button.addEventListener("click", () => this.openEpisode(id, button));
        root.appendChild(button);
      }
    } catch (error) {
      root.innerHTML = "";
      const message = document.createElement("span");
      message.className = "muted";
      message.textContent = `索引不可用：${error.message}`;
      root.appendChild(message);
    }
  }

  async openEpisode(episodeId, button) {
    document.querySelectorAll(".episode-item").forEach((node) => node.classList.remove("active"));
    button?.classList.add("active");
    this.currentEpisodeId = episodeId;
    $("replay-empty").hidden = true;
    $("replay-content").hidden = false;
    $("replay-report").textContent = "读取只读 Episode…";
    try {
      const detail = await this.api.readEpisode(episodeId);
      this.currentReplay = detail;
      const episode = detail.episode || detail.metadata || detail;
      setText("replay-audit", detail.validation?.audit_status || episode.audit_status || "unknown");
      setText("replay-result", detail.assessment?.task_result || episode.task_result || "unknown");
      setText("replay-operator", episode.operator_kind || detail.manifest?.operator_kind || "unknown");
      setText("replay-segments", episode.segments?.length || detail.segments?.length || episode.recording_segment_id || "—");
      $("replay-report").textContent = JSON.stringify(
        {
          contract: detail.episode?.contract_version || detail.manifest?.contract_version,
          audit: detail.validation?.audit_status,
          task_result: detail.assessment?.task_result || episode.task_result,
          warnings: detail.validation?.warnings,
          errors: detail.validation?.errors,
        },
        null,
        2,
      );
      this.renderReplayEvents(detail.events || detail.timeline || []);
      this.renderReplayMedia(detail, episode);
    } catch (error) {
      $("replay-report").textContent = `读取失败：${error.message}`;
    }
  }

  renderReplayEvents(events) {
    const root = $("replay-events");
    root.replaceChildren();
    setText("replay-event-count", `${events.length} events`);
    if (!events.length) {
      root.innerHTML = '<span class="muted">未提供事件索引。</span>';
      return;
    }
    events.slice(0, 500).forEach((event) => {
      const row = document.createElement("div");
      row.className = "replay-event";
      const time = event.measurement_time_s ?? event.backend_monotonic_s ?? event.sim_time_s;
      row.innerHTML = `<span class="mono">${Number.isFinite(time) ? Number(time).toFixed(3) : "—"}</span><strong>${event.event_type || event.type || "event"}</strong><span>${event.command_id || event.control_tick_id || ""}</span><span>${event.task_phase || ""}</span>`;
      root.appendChild(row);
    });
  }

  renderReplayMedia(detail, episode) {
    const root = $("replay-media");
    root.replaceChildren();
    const rgb = episode.latest_rgb_uri || detail.latest_rgb_uri || detail.media?.rgb?.[0];
    if (!rgb) return;
    const figure = document.createElement("figure");
    const image = document.createElement("img");
    image.loading = "lazy";
    image.alt = "Episode 原始 RGB";
    image.src = rgb.startsWith("/") ? rgb : episodeAssetUrl(this.currentEpisodeId, rgb);
    const caption = document.createElement("figcaption");
    caption.textContent = "原始 RGB · 只读回放（不含目标真值叠加）";
    figure.append(image, caption);
    root.appendChild(figure);
  }

  async loadReplayReport(kind) {
    if (!this.currentEpisodeId) return;
    const box = $("replay-report");
    box.textContent = "请求中…";
    try {
      const response =
        kind === "validate"
          ? await this.api.validateEpisode(this.currentEpisodeId)
          : kind === "assessment"
            ? await this.api.assessment(this.currentEpisodeId)
            : await this.api.eligibility(this.currentEpisodeId);
      box.textContent = JSON.stringify(response, null, 2);
      if (kind === "validate") setText("replay-audit", response.audit_status);
      if (kind === "assessment") setText("replay-result", response.task_result);
    } catch (error) {
      box.textContent = `${kind} 不可用：${error.message}`;
    }
  }

  addEvent(category, message, tone = "info") {
    const list = $("event-list");
    if (!list) return;
    const row = document.createElement("div");
    row.className = `event-row ${tone}`;
    const time = document.createElement("span");
    time.className = "event-time mono";
    time.textContent = new Date().toLocaleTimeString([], { hour12: false });
    const tag = document.createElement("strong");
    tag.textContent = category;
    const body = document.createElement("span");
    body.textContent = message;
    row.append(time, tag, body);
    list.prepend(row);
    while (list.children.length > 120) list.lastElementChild.remove();
  }

  toast(message) {
    const node = $("toast");
    node.textContent = message;
    node.classList.add("show");
    window.clearTimeout(this.toastTimer);
    this.toastTimer = window.setTimeout(() => node.classList.remove("show"), 3200);
  }

  dispose() {
    this.releaseDrive("界面关闭");
    window.clearInterval(this.pollTimer);
    window.clearInterval(this.heartbeatTimer);
    this.sceneView.dispose();
  }
}

window.mobileCarApp = new MobileCarUi();
