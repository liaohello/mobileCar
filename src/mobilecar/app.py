"""FastAPI application assembly for the G1 teleoperation service."""

from __future__ import annotations

import asyncio
import copy
import inspect
import json
import logging
import math
import mimetypes
import threading
import time
from contextlib import asynccontextmanager
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping
from uuid import uuid4

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response
from pydantic import BaseModel, ValidationError

from .config import AppConfig, load_config
from .contracts import (
    CommandRequest,
    CommandResponse,
    CommandStatus,
    CommandType,
    EpisodeControlResponse,
    EpisodeStartRequest,
    EventRecord,
    OperatorKind,
    RecordingState,
    RobotSnapshot,
    SceneManifest,
    SensorObservation,
    ValidationReport,
)
from .distribution import FanoutEventSink, NavigationTelemetrySink
from .ros_ipc import NavigationCommandSubmitter, NavigationIpcServer

LOGGER = logging.getLogger("mobilecar")


class EpisodeStopRequest(BaseModel):
    reason: str = "OPERATOR_FINISHED"
    task_result: str | None = None


class WebSocketSink:
    """Fan out backend events to the recorder and each connected browser."""

    def __init__(self, runtime: "Runtime") -> None:
        self.runtime = runtime

    def publish(self, event: EventRecord | Mapping[str, Any]) -> None:
        record = event if isinstance(event, EventRecord) else EventRecord.model_validate(event)
        self.runtime.record_event(record)

    def publish_observation(self, observation: SensorObservation | Mapping[str, Any]) -> None:
        record = (
            observation
            if isinstance(observation, SensorObservation)
            else SensorObservation.model_validate(observation)
        )
        self.runtime.record_observation(record)

    def recording_gate(self, episode_id: str) -> bool:
        return self.runtime.recording_gate(episode_id)


class Runtime:
    """Single application-owned assembly of backend, recorder and read-only replay."""

    def __init__(
        self,
        config: AppConfig,
        *,
        backend_kind: str | None = None,
        require_recording: bool | None = None,
        realtime: bool | None = None,
    ) -> None:
        data = copy.deepcopy(config.data)
        if backend_kind:
            data["backend"]["kind"] = backend_kind
        if require_recording is not None:
            data["recording"]["required_for_motion"] = bool(require_recording)
        if realtime is not None:
            data["backend"]["realtime"] = bool(realtime)
        self.config = AppConfig(config.path, data)
        self.loop: asyncio.AbstractEventLoop | None = None
        self.clients: set[asyncio.Queue[dict[str, Any]]] = set()
        self.clients_lock = threading.Lock()
        self.closed = False
        self.recording_error: str | None = None
        self._timeout_stop_started = False

        from .backends.factory import create_backend

        self.run_root = self.config.recording_root
        self.run_root.mkdir(parents=True, exist_ok=True)
        safety = copy.deepcopy(self.config.safety)
        safety["require_recording"] = bool(self.config.recording["required_for_motion"])
        self.backend = create_backend(
            self.config.backend,
            safety,
            self.config.data["acceptance"],
            self.run_root,
        )
        self.sink = WebSocketSink(self)
        self.navigation_control = NavigationCommandSubmitter(self.backend)
        self.navigation_ipc: NavigationIpcServer | None = None
        self.navigation_estimator = None
        self.navigation_telemetry = NavigationTelemetrySink()
        self.fanout_sink: FanoutEventSink | None = None
        navigation_config = self.config.navigation
        if bool(navigation_config.get("enabled", False)):
            from .estimation import (
                NavigationEstimationRuntime,
                load_planar_odometry,
            )

            navigation_profile = self.config.resolve_path(
                str(
                    navigation_config.get(
                        "profile_path",
                        "configs/profiles/g3a_navigation_v1.yaml",
                    )
                )
            )
            odometry = load_planar_odometry(navigation_profile)
            self.navigation_estimator = NavigationEstimationRuntime(
                odometry,
                self._publish_navigation_result,
            )
            ipc = NavigationIpcServer(
                self.config.navigation_socket_path,
                self.navigation_control.submit_control,
                queue_capacity=int(navigation_config.get("queue_capacity", 16)),
            )
            ipc.start()
            self.navigation_ipc = ipc
            self.fanout_sink = FanoutEventSink(
                recorder=self.sink,
                navigation_consumers=(
                    ("estimator", self.navigation_estimator),
                    ("telemetry", self.navigation_telemetry),
                    ("ros_bridge", ipc),
                ),
                queue_capacity=int(navigation_config.get("queue_capacity", 16)),
                recorder_queue_capacity=int(
                    self.config.recording["queue_capacity"]
                ),
            )
            self.backend.set_event_sink(self.fanout_sink)
        else:
            self.backend.set_event_sink(self.sink)
        self.recorder = _build_recorder(self.config)
        if hasattr(self.recorder, "set_failure_callback"):
            self.recorder.set_failure_callback(self._recording_failure)
        self.replay = _build_replay_reader(self.config)
        self.backend.start()
        LOGGER.info("backend started: %s", self.capabilities.backend_kind)

    def _publish_navigation_result(self, estimate, health) -> None:
        apply_health = getattr(self.backend, "apply_navigation_health", None)
        if callable(apply_health):
            try:
                apply_health(health)
            except Exception:
                LOGGER.exception("backend failed to apply navigation health")
        sink = self.fanout_sink
        if sink is None:
            return
        if estimate is not None:
            sink.publish_estimate(estimate)
        sink.publish_health(health)

    @property
    def capabilities(self):
        return self.backend.capabilities()

    def snapshot(self) -> RobotSnapshot:
        snapshot = self.backend.snapshot()
        status = self.recording_state()
        if status is not None:
            snapshot = snapshot.model_copy(update={"recording": status})
        return snapshot

    def scene_manifest(self) -> SceneManifest:
        scene = getattr(self.backend, "scene_manifest", None)
        if scene is None:
            raise HTTPException(status_code=501, detail="backend does not expose a scene manifest")
        return scene()

    def scene_asset(self, asset_id: str) -> bytes:
        getter = getattr(self.backend, "scene_asset", None)
        if getter is None:
            raise HTTPException(status_code=404, detail="unknown scene asset")
        try:
            return getter(asset_id)
        except (KeyError, FileNotFoundError) as exc:
            raise HTTPException(status_code=404, detail="unknown scene asset") from exc

    def renew_lease(self, session_id: str) -> dict[str, Any]:
        method = getattr(self.backend, "renew_lease", None)
        if method is None:
            raise HTTPException(status_code=501, detail="lease renewal is unsupported")
        return {"accepted": bool(method(session_id)), "session_id": session_id}

    def recording_state(self) -> RecordingState | None:
        status = getattr(self.recorder, "status", None)
        if status is None:
            return None
        try:
            value = status()
        except Exception:
            LOGGER.exception("recorder status failed")
            return RecordingState(
                status="FAILED",
                error="recorder status failed",
            )
        return value if isinstance(value, RecordingState) else RecordingState.model_validate(value)

    def recording_gate(self, episode_id: str) -> bool:
        method = getattr(self.recorder, "recording_gate", None)
        if method is not None:
            return bool(method(episode_id))
        status = self.recording_state()
        return bool(
            status is not None
            and status.episode_id == episode_id
            and status.command_capture_ready
            and status.status == "RECORDING"
        )

    def navigation_status(self) -> dict[str, Any]:
        status: dict[str, Any] = {
            "enabled": bool(self.config.navigation.get("enabled", False)),
            "interface_profile": "g3a-1.0",
            "profile_path": str(
                self.config.navigation.get(
                    "profile_path", "configs/profiles/g3a_navigation_v1.yaml"
                )
            ),
            "socket": str(self.config.navigation_socket_path),
        }
        backend_status = getattr(self.backend, "navigation_status", None)
        if callable(backend_status):
            status["backend"] = backend_status()
        if self.navigation_ipc is not None:
            status["ipc"] = asdict(self.navigation_ipc.stats())
        status["telemetry"] = self.navigation_telemetry.status()
        if self.fanout_sink is not None:
            fanout = self.fanout_sink.stats()
            status["fanout"] = {
                "closed": fanout.closed,
                "consumers": [asdict(item) for item in fanout.consumers],
            }
        estimator_status = getattr(self.navigation_estimator, "status", None)
        if callable(estimator_status):
            status["estimator"] = estimator_status()
        return status

    def record_event(self, event: EventRecord) -> None:
        publisher = getattr(self.recorder, "publish", None)
        if publisher is not None:
            try:
                publisher(event)
            except Exception as exc:
                self.recording_error = f"event publish failed: {exc}"
                LOGGER.exception("event publish failed")
        self.broadcast({"type": "event", "data": event.model_dump(mode="json")})
        if event.event_type == "episode_timeout" and not self._timeout_stop_started:
            self._timeout_stop_started = True
            threading.Thread(
                target=self.stop_episode,
                kwargs={"reason": "EPISODE_TIMEOUT"},
                name="episode-timeout-stop",
                daemon=True,
            ).start()

    def record_observation(self, observation: SensorObservation) -> None:
        publisher = getattr(self.recorder, "publish_observation", None)
        if publisher is not None:
            try:
                publisher(observation)
            except Exception as exc:
                self.recording_error = f"observation publish failed: {exc}"
                LOGGER.exception("observation publish failed")

    def submit(self, request: CommandRequest) -> CommandResponse:
        if self.closed:
            return _response(request, CommandStatus.REJECTED, "application is closed")
        if request.command_type == CommandType.START_RECORDING:
            episode_request = EpisodeStartRequest(
                operator_id=request.operator_id,
                operator_kind=request.operator_kind,
                view_source=request.view_source,
                require_recording=True,
            )
            result = self.start_episode(episode_request)
            return _response(
                request,
                CommandStatus.RECEIVED if result.accepted else CommandStatus.REJECTED,
                result.reason,
            )
        if request.command_type == CommandType.STOP_RECORDING:
            result = self.stop_episode(reason="USER_STOP")
            return _response(
                request,
                CommandStatus.APPLIED if result.accepted else CommandStatus.FAILED,
                result.reason,
            )
        if request.command_type == CommandType.RESET:
            state = self.recording_state()
            if state is not None and state.status in {"STARTING", "RECORDING", "DRAINING"}:
                return _response(
                    request,
                    CommandStatus.REJECTED,
                    "stop the active recording before resetting the Episode",
                )
        response = self.backend.submit(request)
        self.broadcast({"type": "event", "data": {"event_type": "command_response", "payload": response.model_dump(mode="json"), "wall_utc": _wall_utc(), "run_id": request.run_id, "episode_id": request.episode_id, "command_id": request.command_id}})
        return response

    def start_episode(self, request: EpisodeStartRequest) -> EpisodeControlResponse:
        try:
            current = self.recording_state()
            if current is not None and current.status in {"STARTING", "RECORDING", "DRAINING"}:
                return EpisodeControlResponse(
                    accepted=False,
                    reason=f"recording is already {current.status}",
                    episode_id=current.episode_id,
                    recording_segment_id=current.recording_segment_id,
                    recording_status=current.status,
                )
            before = self.backend.snapshot()
            reset_request = CommandRequest(
                run_id=before.run_id,
                backend_generation=before.backend_generation,
                episode_id=before.episode_id,
                session_id=f"recording_start_{uuid4().hex}",
                operator_id=request.operator_id,
                operator_kind=request.operator_kind,
                view_source=request.view_source,
                command_id=f"recording_reset_{uuid4().hex}",
                command_seq=1,
                command_type=CommandType.RESET,
                requested_ttl_s=1.0,
                payload={"confirm": True, "boundary_reason": "START_RECORDING"},
            )
            reset_response = self.backend.submit(reset_request)
            if reset_response.status not in {
                CommandStatus.APPLIED,
                CommandStatus.DISPATCHED,
                CommandStatus.RECEIVED,
            }:
                return EpisodeControlResponse(
                    accepted=False,
                    reason=f"Episode boundary reset rejected: {reset_response.reason}",
                )
            snapshot = self.backend.snapshot()
            self._timeout_stop_started = False
            result = self.recorder.start(request, snapshot)
            if isinstance(result, dict):
                result = EpisodeControlResponse.model_validate(result)
            self.broadcast({"type": "recording", "data": result.model_dump(mode="json")})
            return result
        except Exception as exc:
            LOGGER.exception("episode start failed")
            return EpisodeControlResponse(accepted=False, reason=str(exc))

    def _recording_failure(self, code: str, message: str) -> None:
        self.recording_error = f"{code}: {message}"
        LOGGER.error("recorder failure triggers controlled stop: %s", self.recording_error)

        def stop_backend() -> None:
            try:
                snapshot = self.backend.snapshot()
                request = CommandRequest(
                    run_id=snapshot.run_id,
                    backend_generation=snapshot.backend_generation,
                    episode_id=snapshot.episode_id,
                    session_id=f"recorder_failure_{uuid4().hex}",
                    operator_id="recorder_supervisor",
                    operator_kind=OperatorKind.SCRIPTED,
                    view_source="recording_failure",
                    command_id=f"recording_failure_stop_{uuid4().hex}",
                    command_seq=1,
                    command_type=CommandType.STOP,
                    requested_ttl_s=1.0,
                    payload={"latch": True, "reason": code},
                )
                self.backend.submit(request)
            except Exception:
                LOGGER.exception("controlled stop after recorder failure failed")

        threading.Thread(target=stop_backend, name="recorder-failure-stop", daemon=True).start()

    def stop_episode(self, *, reason: str) -> EpisodeControlResponse:
        snapshot = self.backend.snapshot()
        stop_request = CommandRequest(
            run_id=snapshot.run_id,
            backend_generation=snapshot.backend_generation,
            episode_id=snapshot.episode_id,
            session_id=f"app_stop_{uuid4().hex}",
            operator_id="mobilecar_application",
            operator_kind=OperatorKind.SCRIPTED,
            view_source="application_lifecycle",
            command_id=f"stop_{uuid4().hex}",
            command_seq=1,
            command_type=CommandType.STOP,
            requested_ttl_s=1.0,
            payload={"latch": True},
        )
        try:
            self.backend.submit(stop_request)
        except Exception:
            LOGGER.exception("backend stop failed before recording drain")

        deadline = time.monotonic() + float(self.config.safety["parking_confirm_timeout_s"])
        final_snapshot = self.backend.snapshot()
        while time.monotonic() < deadline:
            final_snapshot = self.backend.snapshot()
            if final_snapshot.base.stopped:
                break
            time.sleep(0.02)
        pause_sensor_capture = getattr(self.backend, "pause_sensor_capture", None)
        resume_sensor_capture = getattr(self.backend, "resume_sensor_capture", None)
        if pause_sensor_capture is not None:
            pause_sensor_capture(timeout_s=2.0)
        try:
            if self.fanout_sink is not None:
                self.fanout_sink.drain(timeout_s=2.0)
            result = self.recorder.stop(reason=reason, final_snapshot=final_snapshot)
            if isinstance(result, dict):
                result = EpisodeControlResponse.model_validate(result)
            if result.accepted and result.episode_id:
                self._finalize_episode_reports(result.episode_id, final_snapshot.run_id)
            self.broadcast({"type": "recording", "data": result.model_dump(mode="json")})
            return result
        except Exception as exc:
            LOGGER.exception("episode stop failed")
            return EpisodeControlResponse(accepted=False, reason=str(exc))
        finally:
            if resume_sensor_capture is not None:
                resume_sensor_capture()

    def _finalize_episode_reports(self, episode_id: str, run_id: str) -> None:
        try:
            from .data import assess_grasp_oracle, evaluate_eligibility, validate_episode

            validation = validate_episode(
                self.config.recording_root,
                episode_id,
                run_id=run_id,
                write_report=True,
            )
            acceptance = self.config.data["acceptance"]
            assessment = assess_grasp_oracle(
                self.config.recording_root,
                episode_id,
                run_id=run_id,
                lift_threshold_m=float(acceptance["lift_height_m"]),
                hold_threshold_s=float(acceptance["lift_hold_s"]),
                write=True,
            )
            evaluate_eligibility(
                self.config.recording_root,
                episode_id,
                run_id=run_id,
                write=True,
            )
            if bool(self.config.quality.get("g2a_quality_enabled", True)):
                self._write_g2a_reports(
                    episode_id,
                    run_id,
                    validation=validation,
                    assessment=assessment,
                )
        except Exception:
            LOGGER.exception("post-recording validation/assessment failed for %s", episode_id)

    @property
    def g2a_report_root(self) -> Path:
        return self.config.quality_root / "g2a"

    def _write_g2a_reports(
        self,
        episode_id: str,
        run_id: str,
        *,
        validation: ValidationReport,
        assessment: Any,
    ) -> None:
        """Write G2A reports separately from the historical G1 eligibility file."""

        try:
            from .data import (
                assess_g2a_quality,
                g2a_eligibility,
                write_g2a_eligibility,
            )

            episode_dir = self.config.recording_root / run_id / "episodes" / episode_id
            episode = json.loads((episode_dir / "episode.json").read_text(encoding="utf-8"))
            events = _read_jsonl(episode_dir / "events.jsonl")
            observations = _read_jsonl(episode_dir / "observations/index.jsonl")
            output_root = self.g2a_report_root / run_id
            profile_path = str(
                self.config.quality.get(
                    "profile_path", "configs/profiles/g2a_rgbd_5hz.yaml"
                )
            )
            action_profile_path = str(
                self.config.quality.get(
                    "action_profile_path",
                    "configs/profiles/g2a_action_teleop_v1.yaml",
                )
            )
            report = assess_g2a_quality(
                self.config.recording_root,
                episode_id,
                profile_path=profile_path,
                action_profile_path=action_profile_path,
                run_id=run_id,
                output_root=output_root,
                write=True,
                validation=validation,
            )
            eligibility = g2a_eligibility(
                report,
                episode,
                events,
                assessment.model_dump(mode="json")
                if hasattr(assessment, "model_dump")
                else dict(assessment or {}),
                observations,
            )
            write_g2a_eligibility(output_root, eligibility)
        except Exception:
            LOGGER.exception(
                "G2A quality/eligibility report failed for %s; G1 reports remain available",
                episode_id,
            )

    def list_episodes(self) -> Any:
        return self.replay.list_episodes()

    def read_episode(self, episode_id: str) -> Any:
        try:
            return self.replay.read_episode(episode_id)
        except (KeyError, FileNotFoundError) as exc:
            raise HTTPException(status_code=404, detail="episode not found") from exc

    def validate_episode(self, episode_id: str) -> ValidationReport:
        report = self.recorder.validate(episode_id)
        return report if isinstance(report, ValidationReport) else ValidationReport.model_validate(report)

    def assessment(self, episode_id: str) -> dict[str, Any]:
        detail = self.read_episode(episode_id)
        value = detail.get("assessment") if isinstance(detail, dict) else None
        if value is None:
            raise HTTPException(status_code=404, detail="assessment not generated")
        return value

    def eligibility(self, episode_id: str) -> dict[str, Any]:
        detail = self.read_episode(episode_id)
        value = detail.get("eligibility") if isinstance(detail, dict) else None
        if value is None:
            raise HTTPException(status_code=404, detail="eligibility not generated")
        return value

    def g2a_quality(self, episode_id: str) -> dict[str, Any]:
        return self._g2a_report(episode_id, "g2a_quality")

    def g2a_eligibility(self, episode_id: str) -> dict[str, Any]:
        return self._g2a_report(episode_id, "g2a_eligibility")

    def _g2a_report(self, episode_id: str, suffix: str) -> dict[str, Any]:
        matches = sorted(
            self.g2a_report_root.glob(f"*/{episode_id}.{suffix}.json"),
            key=lambda path: path.stat().st_mtime_ns,
            reverse=True,
        )
        if not matches:
            raise HTTPException(status_code=404, detail=f"{suffix} not generated")
        return json.loads(matches[0].read_text(encoding="utf-8"))

    def asset_file(self, episode_id: str, asset_path: str) -> Path:
        candidates = list(self.run_root.glob(f"*/episodes/{episode_id}"))
        if len(candidates) != 1:
            raise HTTPException(status_code=404, detail="episode not found")
        root = candidates[0].resolve()
        relative = Path(asset_path)
        requested = (
            (root / relative).resolve()
            if relative.parts and relative.parts[0] == "assets"
            else (root / "assets" / relative).resolve()
        )
        if not requested.is_relative_to(root) or not requested.is_file():
            raise HTTPException(status_code=404, detail="asset not found")
        return requested

    def attach_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop

    def add_client(self, queue: asyncio.Queue[dict[str, Any]]) -> None:
        with self.clients_lock:
            self.clients.add(queue)

    def remove_client(self, queue: asyncio.Queue[dict[str, Any]]) -> None:
        with self.clients_lock:
            self.clients.discard(queue)

    def broadcast(self, message: dict[str, Any]) -> None:
        loop = self.loop
        if loop is None or loop.is_closed():
            return

        def deliver() -> None:
            with self.clients_lock:
                clients = list(self.clients)
            for queue in clients:
                if queue.full():
                    try:
                        queue.get_nowait()
                    except asyncio.QueueEmpty:
                        pass
                try:
                    queue.put_nowait(message)
                except asyncio.QueueFull:
                    pass

        loop.call_soon_threadsafe(deliver)

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self.stop_episode(reason="BACKEND_CLOSED")
        except Exception:
            LOGGER.exception("final recording stop failed")
        try:
            self.backend.close()
        finally:
            try:
                if self.fanout_sink is not None:
                    self.fanout_sink.drain(timeout_s=2.0)
                    self.fanout_sink.close(timeout_s=2.0)
                if self.navigation_ipc is not None:
                    self.navigation_ipc.close()
                self.recorder.close(
                    timeout_s=float(self.config.recording["close_drain_timeout_s"])
                )
            finally:
                closer = getattr(self.replay, "close", None)
                if closer:
                    closer()


def _build_recorder(config: AppConfig):
    modules = (
        "mobilecar.data.recorder",
        "mobilecar.data",
    )
    cls = _find_class(modules, ("EpisodeRecorder", "Recorder"))
    kwargs = {
        "root": config.recording_root,
        "run_root": config.recording_root,
        "queue_capacity": int(config.recording["queue_capacity"]),
        "flush_interval_s": float(config.recording["flush_interval_s"]),
        "close_drain_timeout_s": float(config.recording["close_drain_timeout_s"]),
    }
    if config.recording.get("fault_after_events") is not None:
        kwargs["fault_after_events"] = int(config.recording["fault_after_events"])
    return _construct(cls, kwargs)


def _build_replay_reader(config: AppConfig):
    cls = _find_class(("mobilecar.data.replay", "mobilecar.data"), ("ReplayReader",))
    return _construct(cls, {"root": config.recording_root, "run_root": config.recording_root})


def _find_class(modules: tuple[str, ...], names: tuple[str, ...]):
    last_error: Exception | None = None
    for module_name in modules:
        try:
            module = __import__(module_name, fromlist=["*"])
        except ImportError as exc:
            last_error = exc
            continue
        for name in names:
            value = getattr(module, name, None)
            if value is not None:
                return value
    raise RuntimeError(f"cannot locate {names} in {modules}: {last_error}")


def _construct(cls, kwargs: dict[str, Any]):
    signature = inspect.signature(cls)
    if any(p.kind == inspect.Parameter.VAR_KEYWORD for p in signature.parameters.values()):
        return cls(**kwargs)
    supported = {key: value for key, value in kwargs.items() if key in signature.parameters}
    return cls(**supported)


def _response(request: CommandRequest, status: CommandStatus, reason: str = "") -> CommandResponse:
    return CommandResponse(
        command_id=request.command_id,
        command_seq=request.command_seq,
        status=status,
        reason=reason,
        backend_monotonic_s=time.monotonic(),
        sim_time_s=0.0,
    )


def _wall_utc() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.is_file():
        return rows
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            text = line.strip()
            if not text:
                continue
            value = json.loads(text)
            if isinstance(value, dict):
                rows.append(value)
    return rows


def _json_safe(value: Any) -> Any:
    """Represent non-finite sensor values as explicit JSON nulls."""

    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _runtime(request: Request) -> Runtime:
    return request.app.state.mobilecar


def create_app(
    config_path: str | Path | None = None,
    *,
    backend_kind: str | None = None,
    require_recording: bool | None = None,
    realtime: bool | None = None,
) -> FastAPI:
    config = load_config(config_path)

    @asynccontextmanager
    async def lifespan(application: FastAPI):
        runtime = Runtime(
            config,
            backend_kind=backend_kind,
            require_recording=require_recording,
            realtime=realtime,
        )
        runtime.attach_loop(asyncio.get_running_loop())
        application.state.mobilecar = runtime
        try:
            yield
        finally:
            await asyncio.to_thread(runtime.close)

    application = FastAPI(
        title="mobileCar G2A",
        version="0.2.0",
        lifespan=lifespan,
    )

    @application.get("/api/v1/health")
    def health(request: Request) -> dict[str, Any]:
        runtime = _runtime(request)
        snapshot = runtime.snapshot()
        g3a = bool(runtime.config.navigation.get("enabled", False))
        g4a1 = bool(runtime.config.backend.get("g4a1", {}).get("enabled", False))
        return {
            "status": "ok",
            "run_id": snapshot.run_id,
            "backend_generation": snapshot.backend_generation,
            "episode_id": snapshot.episode_id,
            "recording_status": snapshot.recording.status,
            "recording_required_for_motion": bool(
                runtime.config.recording["required_for_motion"]
            ),
            "interface_profile": (
                "g4a1-1.0" if g4a1 else ("g3a-1.0" if g3a else "g2a-1.0")
            ),
            "observation_profile_id": "g2a-rgbd-5hz-original",
            "navigation_profile_id": (
                "g3a-nav-50hz-laser10-v1" if g3a else None
            ),
            "ui": {
                "adaptive_rendering": bool(
                    runtime.config.ui.get("adaptive_rendering", True)
                ),
                "preferred_pixel_ratio": float(
                    runtime.config.ui.get("preferred_pixel_ratio", 1.0)
                ),
                "max_pixel_ratio": float(
                    runtime.config.ui.get("max_pixel_ratio", 1.5)
                ),
                "static_render_interval_s": float(
                    runtime.config.ui.get("static_render_interval_s", 0.1)
                ),
                "shadow_map_size": int(
                    runtime.config.ui.get("shadow_map_size", 512)
                ),
                "snapshot_hz": float(runtime.config.ui.get("snapshot_hz", 15)),
            },
        }

    @application.get("/api/v1/navigation")
    def navigation(request: Request):
        return _runtime(request).navigation_status()

    @application.get("/api/v1/navigation/scan")
    def navigation_scan(request: Request):
        value = _runtime(request).navigation_telemetry.latest_scan()
        if value is None:
            raise HTTPException(status_code=404, detail="navigation scan not available")
        return value

    @application.get("/api/v1/navigation/evidence")
    def navigation_evidence(request: Request):
        return _json_safe(
            _runtime(request).navigation_telemetry.latest_evidence()
        )

    @application.post("/api/v1/navigation/fault")
    def navigation_fault(payload: dict[str, Any], request: Request):
        runtime = _runtime(request)
        if not bool(runtime.config.navigation.get("test_control_enabled", False)):
            raise HTTPException(status_code=404, detail="navigation test control disabled")
        method = getattr(runtime.backend, "set_navigation_fault", None)
        if not callable(method):
            raise HTTPException(
                status_code=501, detail="backend navigation fault control unavailable"
            )
        fault = str(payload.get("fault", ""))
        active = payload.get("active")
        if not fault or not isinstance(active, bool):
            raise HTTPException(
                status_code=422, detail="fault and boolean active are required"
            )
        return method(fault, active)

    @application.get("/api/v1/navigation/reference/{control_tick_id}")
    def navigation_reference(control_tick_id: str, request: Request):
        runtime = _runtime(request)
        if not bool(runtime.config.navigation.get("test_control_enabled", False)):
            raise HTTPException(status_code=404, detail="navigation test control disabled")
        method = getattr(runtime.backend, "navigation_reference", None)
        if not callable(method):
            raise HTTPException(
                status_code=501, detail="backend navigation reference unavailable"
            )
        value = method(control_tick_id)
        if value is None:
            raise HTTPException(status_code=404, detail="capture reference unavailable")
        return _json_safe(value)

    @application.post("/api/v1/g3b/control")
    def g3b_control(payload: dict[str, Any], request: Request):
        runtime = _runtime(request)
        if not bool(runtime.config.navigation.get("test_control_enabled", False)):
            raise HTTPException(status_code=404, detail="G3B test control disabled")
        method = getattr(runtime.backend, "submit_g3b_control", None)
        if not callable(method):
            raise HTTPException(status_code=501, detail="G3B backend control unavailable")
        from .contracts.g3b import G3BControlMessage

        message = G3BControlMessage.model_validate(payload)
        return method(message).model_dump(mode="json")

    @application.get("/api/v1/g4a1/status")
    def g4a1_status(request: Request):
        method = getattr(_runtime(request).backend, "g4a1_status", None)
        if not callable(method):
            raise HTTPException(status_code=501, detail="G4A-1 backend unavailable")
        return method().model_dump(mode="json")

    @application.post("/api/v1/g4a2/session")
    def g4a2_session(payload: dict[str, Any], request: Request):
        method = getattr(_runtime(request).backend, "confirm_human_session", None)
        if not callable(method):
            raise HTTPException(status_code=501, detail="G4A-2 backend unavailable")
        from .contracts.g4a2 import HumanSessionRequest

        return method(HumanSessionRequest.model_validate(payload)).model_dump(
            mode="json"
        )

    @application.post("/api/v1/g4a2/intent")
    def g4a2_intent(payload: dict[str, Any], request: Request):
        method = getattr(_runtime(request).backend, "record_human_intent", None)
        if not callable(method):
            raise HTTPException(status_code=501, detail="G4A-2 backend unavailable")
        from .contracts.g4a2 import HumanIntentRequest

        return method(HumanIntentRequest.model_validate(payload)).model_dump(
            mode="json"
        )

    @application.post("/api/v1/g4a2/action")
    def g4a2_action(payload: dict[str, Any], request: Request):
        method = getattr(_runtime(request).backend, "submit_g4a2_action", None)
        if not callable(method):
            raise HTTPException(status_code=501, detail="G4A-2 backend unavailable")
        from .contracts.g4a2 import G4A2ActionRequest

        return method(G4A2ActionRequest.model_validate(payload)).model_dump(
            mode="json"
        )

    @application.get("/api/v1/g4a2/observation")
    def g4a2_observation(request: Request):
        method = getattr(_runtime(request).backend, "g4a1_observation", None)
        if not callable(method):
            raise HTTPException(status_code=501, detail="G4A-2 observation unavailable")
        try:
            return method().model_dump(mode="json")
        except ValueError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @application.post("/api/v1/g4a1/permission")
    def g4a1_permission(payload: dict[str, Any], request: Request):
        method = getattr(_runtime(request).backend, "authorize_nearfield", None)
        if not callable(method):
            raise HTTPException(status_code=501, detail="G4A-1 backend unavailable")
        from .contracts.g4a1 import NearfieldPermissionRequest

        return method(NearfieldPermissionRequest.model_validate(payload)).model_dump(
            mode="json"
        )

    @application.get("/api/v1/g4a1/observation")
    def g4a1_observation(request: Request):
        method = getattr(_runtime(request).backend, "g4a1_observation", None)
        if not callable(method):
            raise HTTPException(status_code=501, detail="G4A-1 backend unavailable")
        try:
            return method().model_dump(mode="json")
        except ValueError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @application.get("/api/v1/g4a1/rgbd/{sample_id}/{kind}")
    def g4a1_rgbd(sample_id: str, kind: str, request: Request):
        method = getattr(_runtime(request).backend, "g4a1_asset", None)
        if not callable(method):
            raise HTTPException(status_code=501, detail="G4A-1 backend unavailable")
        try:
            path = method(sample_id, kind)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail="G4A-1 asset not found") from exc
        media_type = "image/png" if kind in {"rgb", "mask"} else "application/octet-stream"
        return FileResponse(path, media_type=media_type)

    @application.post("/api/v1/g4a1/action")
    def g4a1_action(payload: dict[str, Any], request: Request):
        method = getattr(_runtime(request).backend, "submit_g4a1_action", None)
        if not callable(method):
            raise HTTPException(status_code=501, detail="G4A-1 backend unavailable")
        from .contracts.g4a1 import G4A1ActionRequest

        return method(G4A1ActionRequest.model_validate(payload)).model_dump(
            mode="json"
        )

    @application.post("/api/v1/g4a1/revoke")
    def g4a1_revoke(payload: dict[str, Any], request: Request):
        runtime = _runtime(request)
        if not bool(runtime.config.backend.get("g4a1", {}).get("test_control_enabled", False)):
            raise HTTPException(status_code=404, detail="G4A-1 test control disabled")
        method = getattr(runtime.backend, "revoke_nearfield", None)
        if not callable(method):
            raise HTTPException(status_code=501, detail="G4A-1 backend unavailable")
        return method(
            str(payload.get("reason", "manual_takeover")),
            str(payload.get("source", "test_injection")),
        )

    @application.post("/api/v1/g4a1/fault")
    def g4a1_fault(payload: dict[str, Any], request: Request):
        runtime = _runtime(request)
        if not bool(runtime.config.backend.get("g4a1", {}).get("test_control_enabled", False)):
            raise HTTPException(status_code=404, detail="G4A-1 test control disabled")
        method = getattr(runtime.backend, "set_g4a1_fault", None)
        if not callable(method):
            raise HTTPException(status_code=501, detail="G4A-1 backend unavailable")
        fault = str(payload.get("fault", ""))
        active = payload.get("active")
        if not fault or not isinstance(active, bool):
            raise HTTPException(status_code=422, detail="fault and boolean active are required")
        return method(fault, active)

    @application.post("/api/v1/g4a1/phase")
    def g4a1_phase(payload: dict[str, Any], request: Request):
        method = getattr(_runtime(request).backend, "mark_g4a1_phase", None)
        if not callable(method):
            raise HTTPException(status_code=501, detail="G4A-1 backend unavailable")
        phase = str(payload.get("phase", ""))
        try:
            return method(phase)
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc

    @application.get("/api/v1/capabilities")
    def capabilities(request: Request):
        return _runtime(request).capabilities

    @application.get("/api/v1/scene")
    def scene(request: Request):
        return _runtime(request).scene_manifest()

    @application.get("/api/v1/scene/assets/{asset_id}")
    def scene_asset(asset_id: str, request: Request):
        body = _runtime(request).scene_asset(asset_id)
        return Response(body, media_type="model/stl")

    @application.get("/api/v1/snapshot")
    def snapshot(request: Request):
        return _runtime(request).snapshot()

    @application.post("/api/v1/commands")
    def command(payload: CommandRequest, request: Request):
        return _runtime(request).submit(payload)

    @application.post("/api/v1/lease/renew")
    def renew_lease(payload: dict[str, str], request: Request):
        session_id = payload.get("session_id", "")
        if not session_id:
            raise HTTPException(status_code=422, detail="session_id is required")
        return _runtime(request).renew_lease(session_id)

    @application.post("/api/v1/episodes/start")
    def start_episode(payload: EpisodeStartRequest, request: Request):
        return _runtime(request).start_episode(payload)

    @application.post("/api/v1/episodes/stop")
    def stop_episode(payload: EpisodeStopRequest, request: Request):
        return _runtime(request).stop_episode(reason=payload.reason)

    @application.get("/api/v1/episodes")
    def episodes(request: Request):
        return _runtime(request).list_episodes()

    @application.get("/api/v1/episodes/{episode_id}")
    def episode(episode_id: str, request: Request):
        return _json_safe(_runtime(request).read_episode(episode_id))

    @application.post("/api/v1/episodes/{episode_id}/validate")
    def validate_episode(episode_id: str, request: Request):
        return _runtime(request).validate_episode(episode_id)

    @application.get("/api/v1/episodes/{episode_id}/assessment")
    def assessment(episode_id: str, request: Request):
        return _runtime(request).assessment(episode_id)

    @application.get("/api/v1/episodes/{episode_id}/eligibility")
    def eligibility(episode_id: str, request: Request):
        return _runtime(request).eligibility(episode_id)

    @application.get("/api/v1/episodes/{episode_id}/g2a-quality")
    def g2a_quality(episode_id: str, request: Request):
        return _runtime(request).g2a_quality(episode_id)

    @application.get("/api/v1/episodes/{episode_id}/g2a-eligibility")
    def g2a_eligibility(episode_id: str, request: Request):
        return _runtime(request).g2a_eligibility(episode_id)

    @application.get("/api/v1/episodes/{episode_id}/assets/{asset_path:path}")
    def episode_asset(episode_id: str, asset_path: str, request: Request):
        path = _runtime(request).asset_file(episode_id, asset_path)
        media_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        return FileResponse(path, media_type=media_type)

    @application.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        runtime: Runtime = websocket.app.state.mobilecar
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=8)
        runtime.add_client(queue)
        interval = 1.0 / float(runtime.config.ui["snapshot_hz"])
        try:
            await websocket.send_json(
                {
                    "type": "hello",
                    "data": {
                        "run_id": runtime.backend.snapshot().run_id,
                        "capabilities": runtime.capabilities.model_dump(mode="json"),
                    },
                }
            )
            await websocket.send_json(
                {"type": "snapshot", "data": runtime.snapshot().model_dump(mode="json")}
            )
            while True:
                try:
                    command_payload = await asyncio.wait_for(
                        websocket.receive_json(), timeout=interval
                    )
                except asyncio.TimeoutError:
                    command_payload = None
                if command_payload is not None:
                    try:
                        request_payload = CommandRequest.model_validate(command_payload)
                        response = await asyncio.to_thread(runtime.submit, request_payload)
                        await websocket.send_json(
                            {"type": "command_response", "data": response.model_dump(mode="json")}
                        )
                    except ValidationError as exc:
                        await websocket.send_json(
                            {"type": "error", "data": {"errors": exc.errors()}}
                        )
                try:
                    message = queue.get_nowait()
                    await websocket.send_json(message)
                except asyncio.QueueEmpty:
                    pass
                if command_payload is None:
                    await websocket.send_json(
                        {"type": "snapshot", "data": runtime.snapshot().model_dump(mode="json")}
                    )
        except WebSocketDisconnect:
            pass
        finally:
            runtime.remove_client(queue)

    from .ui import create_ui_router

    application.include_router(create_ui_router())
    return application


app = create_app()
