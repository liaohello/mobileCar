const $ = (id) => document.getElementById(id);
const JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"];
const sessionId = `human_${crypto && crypto.randomUUID ? crypto.randomUUID() : Math.random().toString(36).slice(2)}`;

// Observation bookkeeping: `observation` is the latest control observation,
// `displayedObservation` is the last frame that actually rendered on screen.
let observation = null;
let displayedObservation = null;
let permission = null;
let humanSession = null;
let intent = null;
// Display frame bound to the current human intent (the frame the user acted
// on). Deliberately separate from the control observation used by the action.
let intentSampleId = null;
let actionSeq = 0;
let targets = [0, 0, 0, 0, 0];
let targetsInitialized = false;
let lastSentTarget = null;
let displayMonotonicNs = null;
let streamTimer = null;
let streamActive = false;
let actionInFlight = false;
let stopInFlight = false;
let refreshInFlight = false;
let refreshTimer = null;
let recordingLifecycle = "idle";
let frameLifecycle = "waiting";
let currentEpisodeId = null;
let lastHealth = null;
let postures = [];
let previewMode = false;
// Six-dimensional action chain feedback: idle -> blocked -> sending ->
// accepted -> applying -> reached (or rejected / unchanged).
let chain = { state: "idle", reason: "", since: 0 };

const CHAIN_LABELS = {
  idle: "待命",
  blocked: "条件不满足",
  sending: "正在发送",
  accepted: "已接收",
  applying: "执行中",
  reached: "已到达",
  rejected: "被拒绝",
  unchanged: "目标未改变",
};

function log(message, kind = "info") {
  const line = `${new Date().toISOString()} [${kind}] ${message}\n`;
  $("g4a2-log").textContent += line;
  const lines = $("g4a2-log").textContent.split("\n");
  if (lines.length > 160) $("g4a2-log").textContent = lines.slice(-160).join("\n");
}

function setRecordingState(state, detail = "") {
  recordingLifecycle = state;
  const labels = {
    idle: "IDLE",
    starting: "正在开始",
    recording: "正在录制",
    finalizing: "正在结束",
    complete: "已结束",
    failed: "结束失败",
  };
  $("recording-lifecycle").textContent = labels[state] || state;
  $("recording-lifecycle").dataset.state = state;
  if (detail) $("human-stop-result").textContent = detail;
  updateControls();
}

function setFrameState(state, detail = "") {
  frameLifecycle = state;
  const labels = {
    waiting: "等待画面",
    live: "实时",
    paused: "画面已暂停",
    stale: "画面过期",
    error: "画面失败",
  };
  $("frame-lifecycle").textContent = labels[state] || state;
  $("frame-lifecycle").dataset.state = state;
  if (detail) $("human-image-status").textContent = detail;
  updateControls();
}

function setChain(state, reason = "") {
  if (chain.state === state && chain.reason === reason) return;
  chain = { state, reason, since: performance.now() };
  renderChain();
}

function renderChain() {
  const label = CHAIN_LABELS[chain.state] || chain.state;
  $("chain-lifecycle").textContent = label;
  $("chain-lifecycle").dataset.state = chain.state;
  const parts = [label];
  if (chain.reason) parts.push(chain.reason);
  $("human-chain-status").textContent = parts.join(" · ");
  $("human-chain-status").dataset.state = chain.state;
}

function clearSession() {
  stopStream();
  humanSession = null;
  permission = null;
  intent = null;
  intentSampleId = null;
  lastSentTarget = null;
  actionSeq = 0;
  setChain("idle", "");
  $("human-session").textContent = "未确认";
}

function busy() {
  return ["starting", "finalizing"].includes(recordingLifecycle);
}

// First unmet condition for an action, in operator-facing Chinese.
function blockedReason(what) {
  if (recordingLifecycle !== "recording") return "尚未开始真人录制";
  if (document.hidden) return "页面不可见";
  if (what === "confirm") return "";
  if (!humanSession) return "尚未确认本人操作";
  if (what === "authorize" || what === "send" || what === "stream") {
    if (!permission) return "尚未启用近场权限";
  }
  if (what === "send" || what === "stream") {
    if (frameLifecycle !== "live" || !displayedObservation) {
      return "没有新鲜成功显示的 RGB-D 画面";
    }
    if (streamActive && what === "send") return "5Hz 连续发送中";
  }
  return "";
}

function updateControls() {
  const recording = recordingLifecycle === "recording";
  const isBusy = busy();
  const canStop = recording || recordingLifecycle === "failed";
  $("human-record").disabled = recording || isBusy || recordingLifecycle === "failed";
  $("human-stop").disabled = !canStop || isBusy;
  $("human-confirm").disabled = !recording || isBusy;
  $("human-authorize").disabled = !recording || !humanSession || isBusy;
  $("human-send").disabled =
    !recording || !humanSession || !permission || frameLifecycle !== "live" || streamActive || isBusy;
  $("human-stream").disabled =
    !recording || !humanSession || !permission || frameLifecycle !== "live" || isBusy;
  $("human-cancel").disabled = !canStop || isBusy;
  // Inline disabled reasons so a greyed-out button always explains itself.
  const reasons = [
    ["human-confirm", blockedReason("confirm")],
    ["human-authorize", blockedReason("authorize")],
    ["human-send", blockedReason("send")],
    ["human-stream", blockedReason("stream")],
  ];
  for (const [id, reason] of reasons) {
    const node = $(id);
    if (node.disabled && reason) {
      node.title = reason;
      node.dataset.reason = reason;
    } else {
      node.title = "";
      delete node.dataset.reason;
    }
  }
  const sendReason = blockedReason("send");
  $("human-control-reason").textContent =
    recording && humanSession && permission && sendReason ? sendReason : "";
}

function captureInput() {
  if (recordingLifecycle !== "recording") throw new Error("当前没有正在录制的 Episode");
  if (frameLifecycle !== "live" || !displayedObservation || document.hidden) {
    throw new Error("请等待 link4 RGB 画面成功显示后再操作");
  }
  const now = Math.round(performance.now() * 1e6);
  if (now - displayMonotonicNs > 1.2e9) {
    throw new Error("显示画面已过期，请等待新帧");
  }
  return {
    observation: displayedObservation,
    displayNs: displayMonotonicNs,
    inputNs: now,
  };
}

async function api(path, options = {}) {
  const response = await fetch(path, {
    ...options,
    headers: { ...(options.body ? { "Content-Type": "application/json" } : {}), ...(options.headers || {}) },
    cache: "no-store",
  });
  const contentType = response.headers.get("content-type") || "";
  const body = contentType.includes("application/json") ? await response.json() : await response.text();
  if (!response.ok) {
    const detail = typeof body === "object" && body ? body.detail || body.reason : body;
    throw new Error(detail || `HTTP ${response.status}`);
  }
  return body;
}

function addSliders() {
  $("human-joints").replaceChildren();
  JOINTS.forEach((name, index) => {
    const row = document.createElement("div");
    row.className = "g4a2-row";
    row.innerHTML = `<label>${name} <input data-joint-index="${index}" type="range" min="-2.09" max="2.09" step="0.005" value="0" /></label><output>0.000 rad</output>`;
    const input = row.querySelector("input");
    input.addEventListener("input", () => {
      targets[index] = Number(input.value);
      row.querySelector("output").textContent = `${targets[index].toFixed(3)} rad`;
      renderJoints();
      if (["reached", "applying", "accepted"].includes(chain.state)) {
        setChain("idle", "目标已修改，尚未发送");
      }
    });
    $("human-joints").appendChild(row);
  });
}

async function displayFrame(next) {
  if (document.hidden) throw new Error("页面不可见");
  if (displayedObservation && displayedObservation.sample_id === next.sample_id && frameLifecycle === "live") return;
  const image = new Image(640, 480);
  image.alt = "link4 相机实时图像";
  image.id = "human-image";
  image.decoding = "async";
  const url = new URL(next.rgb_uri, window.location.href);
  let timeout;
  try {
    await Promise.race([
      new Promise((resolve, reject) => {
        image.onload = resolve;
        image.onerror = () => reject(new Error("RGB 图片加载失败，保留上一张画面"));
        image.src = url.href;
      }),
      new Promise((_, reject) => {
        timeout = setTimeout(() => reject(new Error("RGB 图片加载超时，保留上一张画面")), 4000);
      }),
    ]);
    if (image.decode) await image.decode();
    if (document.hidden) throw new Error("页面不可见");
    const previous = displayedObservation;
    const old = $("human-image");
    old.replaceWith(image);
    await new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve)));
    if (previous && (previous.episode_id !== next.episode_id || previous.backend_generation !== next.backend_generation)) {
      clearSession();
    }
    displayedObservation = next;
    displayMonotonicNs = Math.round(performance.now() * 1e6);
    image.dataset.sampleId = next.sample_id;
    $("human-sample").textContent = next.sample_id;
    $("human-valid").textContent = `${(next.valid_fraction * 100).toFixed(1)}%`;
    $("human-depth").textContent = Number.isFinite(next.center_depth_m)
      ? `${(next.center_depth_m * 1000).toFixed(1)} mm`
      : "该像素没有有效深度";
    previewMode = String(next.quality || "").endsWith("_preview");
    setFrameState("live", previewMode
      ? "预览画面（未录制，仅观看）· 机械臂 link4 相机，不是第三人称视角"
      : "画面已显示 · 这是机械臂 link4 相机，不是第三人称视角");
  } finally {
    clearTimeout(timeout);
    image.onload = null;
    image.onerror = null;
  }
}

async function refresh() {
  if (refreshInFlight) return;
  refreshInFlight = true;
  try {
    const health = await api("/api/v1/health");
    lastHealth = health;
    $("g4a2-status").textContent = `后端 ${health.backend_generation}`;
    // Observation first: preview works without recording (G4A-2R decoupling).
    try {
      const next = await api("/api/v1/g4a2/observation");
      observation = next;
      await displayFrame(next);
      if (!targetsInitialized) {
        targets = [...next.arm_user_joint_positions_rad];
        document.querySelectorAll("[data-joint-index]").forEach((input, index) => {
          input.value = targets[index];
          input.closest(".g4a2-row").querySelector("output").textContent = `${targets[index].toFixed(3)} rad`;
        });
        $("human-gripper").value = next.gripper_width_m;
        $("human-gripper-value").textContent = `${(next.gripper_width_m * 1000).toFixed(1)} mm`;
        targetsInitialized = true;
      }
      renderJoints();
      updateChainFromObservation();
    } catch (error) {
      if (["starting", "recording"].includes(recordingLifecycle)) {
        if (frameLifecycle === "live") {
          setFrameState("stale", `新画面暂不可用，保留上一张：${error.message}`);
        } else {
          setFrameState("waiting", `等待 link4 RGB 画面：${error.message}`);
        }
      } else if (displayedObservation) {
        // Outside recording the last frame stays on screen (preview may still
        // appear when the backend serves one).
        setFrameState("paused", "画面已暂停；记录结束后保留最后一帧");
      } else {
        setFrameState("waiting", "尚未获得 link4 RGB 画面");
      }
    }
    if (health.recording_status === "RECORDING") {
      if (recordingLifecycle !== "finalizing" && recordingLifecycle !== "failed") {
        setRecordingState("recording");
      }
    } else if (recordingLifecycle !== "finalizing") {
      if (recordingLifecycle === "failed") {
        setFrameState(displayedObservation ? "stale" : "error", "结束未确认；请重试停止");
        return;
      }
      if (["starting", "recording"].includes(recordingLifecycle)) {
        setRecordingState(health.recording_status === "COMPLETE" ? "complete" : "idle");
      }
      if (!displayedObservation) {
        setFrameState("waiting", "尚未获得 link4 RGB 画面");
      }
    }
  } catch (error) {
    $("g4a2-status").textContent = "后端不可用";
    setFrameState(displayedObservation ? "stale" : "waiting", `无法读取状态：${error.message}`);
  } finally {
    refreshInFlight = false;
    updateControls();
    clearTimeout(refreshTimer);
    refreshTimer = setTimeout(refresh, 300);
  }
}

function sameTarget(a, b) {
  return a.length === b.length && a.every((value, index) => Math.abs(value - b[index]) < 1e-9);
}

function currentTarget() {
  return [...targets, Number($("human-gripper").value)];
}

function renderJoints() {
  const actual = observation
    ? [...(observation.arm_user_joint_positions_rad || []), observation.gripper_width_m]
    : null;
  const rows = JOINTS.map((name, index) => {
    const target = targets[index];
    const actualValue = actual ? actual[index] : null;
    const error = actualValue === null ? null : actualValue - target;
    return `<tr><td>${name}</td><td>${target.toFixed(3)}</td><td>${
      actualValue === null ? "—" : actualValue.toFixed(3)
    }</td><td>${error === null ? "—" : `${error >= 0 ? "+" : ""}${error.toFixed(3)}`}</td></tr>`;
  });
  const gripTarget = Number($("human-gripper").value);
  const gripActual = actual ? actual[5] : null;
  const gripError = gripActual === null ? null : gripActual - gripTarget;
  rows.push(
    `<tr><td>夹爪</td><td>${(gripTarget * 1000).toFixed(1)} mm</td><td>${
      gripActual === null ? "—" : `${(gripActual * 1000).toFixed(1)} mm`
    }</td><td>${
      gripError === null ? "—" : `${gripError >= 0 ? "+" : ""}${(gripError * 1000).toFixed(1)} mm`
    }</td></tr>`,
  );
  $("human-joint-table").innerHTML =
    `<table class="g4a2-table"><thead><tr><th>关节</th><th>目标</th><th>实际</th><th>偏差</th></tr></thead><tbody>${rows.join(
      "",
    )}</tbody></table>`;
}

function updateChainFromObservation() {
  if (!observation) return;
  renderJoints();
  if (!["accepted", "applying"].includes(chain.state) || !lastSentTarget) return;
  const actual = [
    ...(observation.arm_user_joint_positions_rad || []),
    observation.gripper_width_m,
  ];
  if (actual.length !== 6 || actual.some((value) => !Number.isFinite(value))) return;
  const jointError = Math.max(
    ...actual.slice(0, 5).map((value, index) => Math.abs(value - lastSentTarget[index])),
  );
  const gripError = Math.abs(actual[5] - lastSentTarget[5]);
  if (jointError < 0.02 && gripError < 0.002) {
    setChain("reached", "实际关节与目标一致");
  } else {
    setChain(
      "applying",
      `最大关节偏差 ${jointError.toFixed(3)} rad · 夹爪偏差 ${(gripError * 1000).toFixed(1)} mm`,
    );
  }
}

async function confirmHuman() {
  if (recordingLifecycle !== "recording") throw new Error("请先开始真人录制");
  const seen = captureInput().observation;
  humanSession = await api("/api/v1/g4a2/session", {
    method: "POST",
    body: JSON.stringify({
      interface_profile: "g4a2-1.0",
      run_id: seen.run_id,
      episode_id: seen.episode_id,
      backend_generation: seen.backend_generation,
      session_id: sessionId,
      operator_id: $("human-operator").value,
      operator_kind: "human_teleop",
      source: "human_teleop",
      view_source: "link4_rgbd_proprio",
      explicit_human_confirmation: true,
      input_device: "server_browser",
      assist_mode: "manual",
    }),
  });
  if (!humanSession.accepted) throw new Error(humanSession.reason);
  $("human-session").textContent = humanSession.human_session_id;
  log(`human session ${humanSession.human_session_id}`, "ok");
  updateControls();
}

async function recordHuman() {
  if (["starting", "recording", "finalizing"].includes(recordingLifecycle)) return;
  clearSession();
  setRecordingState("starting", "正在准备录制；等待摄像机画面。");
  try {
    const response = await api("/api/v1/episodes/start", {
      method: "POST",
      body: JSON.stringify({
        operator_id: $("human-operator").value,
        operator_kind: "human_teleop",
        view_source: "link4_rgbd_proprio",
        map_mode: "unavailable",
        require_recording: true,
      }),
    });
    if (!response.accepted) throw new Error(response.reason || "录制未获接纳");
    currentEpisodeId = response.episode_id;
    setRecordingState("recording", `正在录制 · ${response.episode_id}`);
    if (displayedObservation) {
      setFrameState("paused", "录制已开始；等待新的 link4 RGB 画面");
    }
    log(`recording ${response.episode_id}`, "ok");
  } catch (error) {
    setRecordingState("failed", `开始录制失败：${error.message}`);
    throw error;
  } finally {
    updateControls();
    clearTimeout(refreshTimer);
    refreshTimer = setTimeout(refresh, 150);
  }
}

async function authorize() {
  if (!humanSession) throw new Error("请先点击确认本人操作");
  const seen = captureInput().observation;
  if (!observation) throw new Error("先等待 RGB-D 观测");
  permission = await api("/api/v1/g4a1/permission", {
    method: "POST",
    body: JSON.stringify({
      interface_profile: "g4a1-1.0",
      run_id: seen.run_id,
      episode_id: seen.episode_id,
      backend_generation: seen.backend_generation,
      session_id: sessionId,
      operator_id: $("human-operator").value,
      operator_kind: "human_teleop",
      source: "human_teleop",
      view_source: "link4_rgbd_proprio",
      observation_profile_id: "g4a1-link4-rgbd-proprio-v1",
      observation_profile_sha256: seen.observation_profile_sha256,
      action_profile_id: "g4a1-sixdim-absolute-goal-v1",
      action_profile_sha256: seen.action_profile_sha256,
    }),
  });
  if (!permission.accepted) throw new Error(permission.reason);
  log(`nearfield permission ${permission.permission_id}`, "ok");
  updateControls();
}

async function makeIntent(target, evidence) {
  if (!humanSession) throw new Error("请先点击确认本人操作");
  if (!evidence) throw new Error("缺少实际显示帧关联，请重新选择目标或点击发送");
  const seen = evidence.observation;
  intent = await api("/api/v1/g4a2/intent", {
    method: "POST",
    body: JSON.stringify({
      interface_profile: "g4a2-1.0",
      human_session_id: humanSession.human_session_id,
      run_id: seen.run_id,
      episode_id: seen.episode_id,
      backend_generation: seen.backend_generation,
      session_id: sessionId,
      display_sample_id: seen.sample_id,
      input_type: "sixdim_target",
      assist_mode: "manual",
      control_generator: "human_sixdim_sender",
      client_display_monotonic_ns: evidence.displayNs,
      client_intent_monotonic_ns: evidence.inputNs,
      raw_input: { q: target.slice(0, 5), gripper_width_m: target[5] },
    }),
  });
  if (!intent.accepted) throw new Error(intent.reason);
  intentSampleId = intent.display_sample_id || seen.sample_id;
  log(`human intent ${intent.human_intent_id} 绑定显示帧 ${intentSampleId}`, "ok");
}

async function sendFrame(evidence = null) {
  if (actionInFlight) return;
  const reason = blockedReason("send");
  if (reason) {
    setChain("blocked", reason);
    log(`发送被阻止：${reason}`, "warn");
    return;
  }
  actionInFlight = true;
  try {
    const seen = captureInput();
    const target = currentTarget();
    if (lastSentTarget && sameTarget(target, lastSentTarget) && chain.state === "reached") {
      setChain("unchanged", "目标与当前位置相同，无需重复发送");
      return;
    }
    if (!permission) await authorize();
    // A changed target is a new human choice: bind a fresh intent to the frame
    // the user is acting on now. An unchanged target keeps the existing
    // intent/display binding (no silent rebinding to a newer frame).
    const targetChanged = !intent || !lastSentTarget || !sameTarget(target, lastSentTarget);
    if (targetChanged) {
      await makeIntent(target, evidence || seen);
    }
    setChain("sending", "正在发送六维目标…");
    actionSeq += 1;
    const receipt = await api("/api/v1/g4a2/action", {
      method: "POST",
      body: JSON.stringify({
        interface_profile: "g4a2-1.0",
        run_id: seen.observation.run_id,
        episode_id: seen.observation.episode_id,
        backend_generation: seen.observation.backend_generation,
        nearfield_epoch: permission.nearfield_epoch,
        permission_id: permission.permission_id,
        session_id: sessionId,
        human_session_id: humanSession.human_session_id,
        human_intent_id: intent.human_intent_id,
        action_frame_id: `human_action_${crypto && crypto.randomUUID ? crypto.randomUUID() : Date.now()}`,
        action_seq: actionSeq,
        // Display frame stays bound to the human intent (the frame the user
        // acted on); the control observation is the latest known sample.
        display_sample_id: intentSampleId,
        observation_sample_id: seen.observation.sample_id,
        assist_mode: "manual",
        control_generator: "human_sixdim_sender",
        joint_order: JOINTS,
        units: ["rad", "rad", "rad", "rad", "rad", "m"],
        arm_user_joint_positions_rad: target.slice(0, 5),
        gripper_width_m: target[5],
        teacher_plan_id: null,
      }),
    });
    if (!receipt.accepted) {
      setChain("rejected", `被拒绝：${receipt.reason || "服务器未说明原因"}`);
      log(`action ${receipt.action_seq} rejected ${receipt.reason || ""}`, "error");
      return;
    }
    lastSentTarget = target;
    setChain("accepted", `已接收 · 控制 tick ${receipt.control_tick_id || "—"}`);
    log(`action ${receipt.action_seq} dispatched ${receipt.control_tick_id || ""}`, "ok");
    updateChainFromObservation();
  } catch (error) {
    setChain("rejected", `发送失败：${error.message}`);
    log(error.message, "error");
  } finally {
    actionInFlight = false;
    updateControls();
  }
}

async function startStream() {
  if (streamActive) return;
  if (!humanSession || recordingLifecycle !== "recording" || frameLifecycle !== "live") {
    throw new Error("请先开始录制、确认本人操作并等待实时画面");
  }
  if (!permission) await authorize();
  streamActive = true;
  const tick = async () => {
    const started = performance.now();
    try {
      await sendFrame(captureInput());
    } catch (error) {
      log(error.message, "error");
      stopStream();
    }
    if (streamActive) streamTimer = setTimeout(tick, Math.max(0, 200 - (performance.now() - started)));
  };
  tick();
  $("human-stream").textContent = "停止 5Hz";
  log("5Hz sender started", "warn");
  updateControls();
}

function stopStream() {
  streamActive = false;
  if (streamTimer) clearTimeout(streamTimer);
  streamTimer = null;
  $("human-stream").textContent = "开始 5Hz";
  updateControls();
}

async function stopAll() {
  if (stopInFlight || !["recording", "starting", "failed"].includes(recordingLifecycle)) return;
  stopInFlight = true;
  stopStream();
  setRecordingState("finalizing", "正在结束；正在撤销近场权限并保存 Episode……");
  try {
    try {
      await api("/api/v1/g4a1/revoke", {
        method: "POST",
        body: JSON.stringify({ reason: "human_stop", source: "human_teleop" }),
      });
    } catch (error) {
      log(`撤销近场权限失败：${error.message}`, "warn");
    }
    const response = await api("/api/v1/episodes/stop", {
      method: "POST",
      body: JSON.stringify({ reason: "OPERATOR_FINISHED" }),
    });
    if (!response.accepted) throw new Error(response.reason || "服务器未确认结束");
    currentEpisodeId = response.episode_id || currentEpisodeId;
    clearSession();
    setRecordingState("complete", `记录已保存 · Episode ${currentEpisodeId}`);
    setFrameState("paused", "画面已转为预览；记录已保存，最后一帧保留用于核对");
    log(`recording saved: ${currentEpisodeId}`, "ok");
  } catch (error) {
    setRecordingState("failed", `结束失败：${error.message}；请重试停止，不要开始新记录`);
    setFrameState(displayedObservation ? "stale" : "error", "结束未确认；保留最后一帧并暂停操作");
    log(`stop failed: ${error.message}`, "error");
  } finally {
    stopInFlight = false;
    updateControls();
    clearTimeout(refreshTimer);
    refreshTimer = setTimeout(refresh, 150);
  }
}

// --- Named postures (unified posture profile, exposed via capabilities) -----
const POSTURE_LABELS = {
  low_surround_scan: "低位环视姿态",
  nearfield_observation: "近场观察姿态",
  g4a1_baseline_frozen: "初始姿态",
};

function renderPostureButtons() {
  const host = $("human-postures");
  host.replaceChildren();
  for (const posture of postures) {
    const button = document.createElement("button");
    button.className = "secondary";
    button.textContent = POSTURE_LABELS[posture.purpose] || posture.name;
    button.title = `${posture.name} · 用户角 ${(posture.arm_user_joint_positions_rad || [])
      .map((value) => value.toFixed(2))
      .join(", ")} rad`;
    button.addEventListener("click", () => applyPosture(posture));
    host.appendChild(button);
  }
  if (!postures.length) {
    const note = document.createElement("span");
    note.className = "muted";
    note.textContent = "后端未提供姿态列表";
    host.appendChild(note);
  }
}

function applyPosture(posture) {
  const angles = posture.arm_user_joint_positions_rad || [];
  if (angles.length !== 5) return;
  targets = [...angles];
  document.querySelectorAll("[data-joint-index]").forEach((input, index) => {
    input.value = targets[index];
    input.closest(".g4a2-row").querySelector("output").textContent = `${targets[index].toFixed(3)} rad`;
  });
  if (Number.isFinite(posture.gripper_width_m)) {
    $("human-gripper").value = posture.gripper_width_m;
    $("human-gripper-value").textContent = `${(posture.gripper_width_m * 1000).toFixed(1)} mm`;
  }
  renderJoints();
  log(`已填入姿态 ${posture.name}（${POSTURE_LABELS[posture.purpose] || posture.purpose}）`, "info");
  sendFrame().catch((error) => log(error.message, "error"));
}

async function loadPostures() {
  try {
    const capabilities = await api("/api/v1/capabilities");
    postures = Array.isArray(capabilities.g4a2_postures) ? capabilities.g4a2_postures : [];
  } catch (_) {
    postures = [];
  }
  renderPostureButtons();
}

$("human-confirm").addEventListener("click", () => confirmHuman().catch((e) => log(e.message, "error")));
$("human-record").addEventListener("click", () => recordHuman().catch((e) => log(e.message, "error")));
$("human-authorize").addEventListener("click", () => authorize().catch((e) => log(e.message, "error")));
$("human-send").addEventListener("click", () => {
  try {
    sendFrame(captureInput()).catch((e) => log(e.message, "error"));
  } catch (e) {
    setChain("blocked", e.message);
    log(e.message, "error");
  }
});
$("human-stream").addEventListener("click", () => streamActive ? stopStream() : startStream().catch((e) => log(e.message, "error")));
$("human-cancel").addEventListener("click", () => stopAll());
$("human-stop").addEventListener("click", () => stopAll());
$("human-gripper").addEventListener("input", () => {
  $("human-gripper-value").textContent = `${(Number($("human-gripper").value) * 1000).toFixed(1)} mm`;
  renderJoints();
  if (["reached", "applying", "accepted"].includes(chain.state)) {
    setChain("idle", "目标已修改，尚未发送");
  }
});
$("human-teleop-link").addEventListener("click", (event) => {
  if (["starting", "recording", "finalizing"].includes(recordingLifecycle) || streamActive || permission) {
    event.preventDefault();
    setRecordingState(recordingLifecycle, "请先点击停止并结束，再进入整车遥操作；近场录制期间底盘保持停车。");
    log("请先结束当前近场记录，再切换整车页面。", "warn");
  }
});
window.addEventListener("blur", () => {
  stopStream();
  if (permission) {
    permission = null;
    api("/api/v1/g4a1/revoke", {
      method: "POST",
      body: JSON.stringify({ reason: "window_blur", source: "human_teleop" }),
    }).catch((e) => log(e.message, "error"));
  }
  log("窗口失焦：已停止发送；回到页面后需重新启用近场权限。", "warn");
});
document.addEventListener("visibilitychange", () => {
  if (document.hidden) {
    stopStream();
    if (frameLifecycle === "live") setFrameState("stale", "页面隐藏，画面已过期");
  }
});
addSliders();
setRecordingState("idle");
setFrameState("waiting");
setChain("idle", "");
renderJoints();
loadPostures();
refresh();
