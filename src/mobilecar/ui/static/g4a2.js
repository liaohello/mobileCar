const $ = (id) => document.getElementById(id);
const JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"];
const sessionId = `human_${crypto && crypto.randomUUID ? crypto.randomUUID() : Math.random().toString(36).slice(2)}`;

let observation = null;
let displayedObservation = null;
let permission = null;
let humanSession = null;
let intent = null;
let actionSeq = 0;
let targets = [0, 0, 0, 0, 0];
let targetsInitialized = false;
let targetEditEvidence = null;
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

function clearSession() {
  stopStream();
  humanSession = null;
  permission = null;
  intent = null;
  targetEditEvidence = null;
  lastSentTarget = null;
  actionSeq = 0;
  $("human-session").textContent = "未确认";
}

function updateControls() {
  const recording = recordingLifecycle === "recording";
  const busy = ["starting", "finalizing"].includes(recordingLifecycle);
  const canStop = recording || recordingLifecycle === "failed";
  $("human-record").disabled = recording || busy || recordingLifecycle === "failed";
  $("human-stop").disabled = !canStop || busy;
  $("human-confirm").disabled = !recording || frameLifecycle !== "live" || busy;
  $("human-authorize").disabled = !recording || !humanSession || frameLifecycle !== "live" || busy;
  $("human-send").disabled = !recording || !humanSession || frameLifecycle !== "live" || streamActive || busy;
  $("human-stream").disabled = !recording || !humanSession || frameLifecycle !== "live" || busy;
  $("human-cancel").disabled = !canStop || busy;
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
      try {
        targetEditEvidence = captureInput();
      } catch (_) {
        targetEditEvidence = null;
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
    setFrameState("live", "画面已显示 · 这是机械臂 link4 相机，不是第三人称视角");
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
    if (health.recording_status === "RECORDING") {
      if (recordingLifecycle !== "finalizing" && recordingLifecycle !== "failed") {
        setRecordingState("recording");
      }
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
      } catch (error) {
        if (frameLifecycle === "live") {
          setFrameState("stale", `新画面暂不可用，保留上一张：${error.message}`);
        } else {
          setFrameState("waiting", `等待 link4 RGB 画面：${error.message}`);
        }
      }
    } else if (recordingLifecycle !== "finalizing") {
      if (recordingLifecycle === "failed") {
        setFrameState(displayedObservation ? "stale" : "error", "结束未确认；请重试停止");
        return;
      }
      if (["starting", "recording"].includes(recordingLifecycle)) {
        setRecordingState(health.recording_status === "COMPLETE" ? "complete" : "idle");
      }
      setFrameState(displayedObservation ? "paused" : "waiting", displayedObservation
        ? "画面已暂停；记录结束后保留最后一帧"
        : "尚未获得 link4 RGB 画面");
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
    setFrameState(displayedObservation ? "paused" : "waiting", "录制已开始；等待新的 link4 RGB 画面");
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
  log(`human intent ${intent.human_intent_id}`, "ok");
}

async function sendFrame(evidence = null) {
  if (actionInFlight) return;
  if (recordingLifecycle !== "recording") throw new Error("请先开始真人录制");
  if (frameLifecycle !== "live") throw new Error("没有成功显示的新 RGB 帧");
  actionInFlight = true;
  try {
    const seen = captureInput();
    if (!humanSession) throw new Error("请先确认本人操作");
    const currentTarget = [...targets, Number($("human-gripper").value)];
    if (!permission) await authorize();
    if (!intent || !lastSentTarget || currentTarget.some((value, index) => value !== lastSentTarget[index])) {
      await makeIntent(currentTarget, evidence || seen);
      lastSentTarget = currentTarget;
    }
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
        observation_sample_id: seen.observation.sample_id,
        display_sample_id: seen.observation.sample_id,
        assist_mode: "manual",
        control_generator: "human_sixdim_sender",
        joint_order: JOINTS,
        units: ["rad", "rad", "rad", "rad", "rad", "m"],
        arm_user_joint_positions_rad: currentTarget.slice(0, 5),
        gripper_width_m: currentTarget[5],
        teacher_plan_id: null,
      }),
    });
    if (!receipt.accepted) throw new Error(receipt.reason);
    log(`action ${receipt.action_seq} dispatched ${receipt.control_tick_id || ""}`, "ok");
  } finally {
    actionInFlight = false;
  }
}

async function startStream() {
  if (streamActive) return;
  if (!humanSession || recordingLifecycle !== "recording" || frameLifecycle !== "live") {
    throw new Error("请先开始录制、确认本人操作并等待实时画面");
  }
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
    setFrameState("paused", "画面已暂停；记录已保存，最后一帧保留用于核对");
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

$("human-confirm").addEventListener("click", () => confirmHuman().catch((e) => log(e.message, "error")));
$("human-record").addEventListener("click", () => recordHuman().catch((e) => log(e.message, "error")));
$("human-authorize").addEventListener("click", () => authorize().catch((e) => log(e.message, "error")));
$("human-send").addEventListener("click", () => {
  try {
    sendFrame(captureInput()).catch((e) => log(e.message, "error"));
  } catch (e) {
    log(e.message, "error");
  }
});
$("human-stream").addEventListener("click", () => streamActive ? stopStream() : startStream().catch((e) => log(e.message, "error")));
$("human-cancel").addEventListener("click", () => stopAll());
$("human-stop").addEventListener("click", () => stopAll());
$("human-gripper").addEventListener("input", () => {
  $("human-gripper-value").textContent = `${(Number($("human-gripper").value) * 1000).toFixed(1)} mm`;
  try {
    targetEditEvidence = captureInput();
  } catch (_) {
    targetEditEvidence = null;
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
refresh();
