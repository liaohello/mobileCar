from mobilecar.contracts import (
    CommandResponse,
    CommandStatus,
    NavigationControlMessage,
)
from mobilecar.ros_ipc import NavigationCommandSubmitter


class _Snapshot:
    run_id = "run"
    backend_generation = 1
    episode_id = "episode"


class _Backend:
    def __init__(self):
        self.requests = []

    def snapshot(self):
        return _Snapshot()

    def submit(self, request):
        self.requests.append(request)
        return CommandResponse(
            command_id=request.command_id,
            command_seq=request.command_seq,
            status=CommandStatus.DISPATCHED,
            backend_monotonic_s=1.0,
            sim_time_s=0.0,
        )


def _message(message_type: str, **values):
    return NavigationControlMessage(
        message_type=message_type,
        run_id="run",
        episode_id="episode",
        backend_generation=1,
        navigation_epoch=4,
        session_id="ros_nav",
        **values,
    )


def test_cmd_vel_fallback_preserves_navigation_provenance():
    backend = _Backend()
    submitter = NavigationCommandSubmitter(backend)
    submitter.submit(_message("cmd_vel", vx_m_s=0.02, wz_rad_s=0.1))
    request = backend.requests[0]
    assert request.command_type == "set_base_twist"
    assert request.operator_kind == "autonomous_policy"
    assert request.payload["navigation_source"] == "ros_navigation_test"
    assert request.payload["navigation_epoch"] == 4


def test_release_and_stop_have_distinct_zero_semantics():
    backend = _Backend()
    submitter = NavigationCommandSubmitter(backend)
    submitter.submit(_message("release"))
    submitter.submit(_message("stop"))
    assert backend.requests[0].command_type == "zero_base"
    assert backend.requests[1].command_type == "stop"
    assert backend.requests[1].payload["latch"] is True


def test_preferred_narrow_backend_method_is_used():
    class _Narrow(_Backend):
        def __init__(self):
            super().__init__()
            self.controls = []

        def submit_navigation_control(self, message):
            self.controls.append(message)
            return CommandResponse(
                command_id="narrow",
                command_seq=1,
                status=CommandStatus.APPLIED,
                backend_monotonic_s=2.0,
                sim_time_s=1.0,
            )

    backend = _Narrow()
    response = NavigationCommandSubmitter(backend).submit(_message("release"))
    assert response.command_id == "narrow"
    assert backend.controls[0].navigation_epoch == 4
