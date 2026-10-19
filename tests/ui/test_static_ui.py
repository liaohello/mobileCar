from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
STATIC = ROOT / "src/mobilecar/ui/static"


def test_ui_build_contains_interactive_controls():
    index = (STATIC / "index.html").read_text(encoding="utf-8")
    script = (STATIC / "app.js").read_text(encoding="utf-8")
    assert 'data-testid="three-container"' in index
    assert 'data-testid="stop-button"' in index
    assert 'data-testid="replay-open"' in index
    assert "WEBGL" in script or "WebGLRenderer" in script
    assert (STATIC / "style.css").is_file()


def test_ui_does_not_import_mujoco_control_paths():
    sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (ROOT / "src/mobilecar/ui/web").glob("*.js")
    )
    assert "mj_step" not in sources
    assert "MjData" not in sources
    assert "/api/v1/commands" in sources


def test_ui_has_bounded_on_demand_render_and_priority_safety_commands():
    scene = (ROOT / "src/mobilecar/ui/web/scene-view.js").read_text(encoding="utf-8")
    main = (ROOT / "src/mobilecar/ui/web/main.js").read_text(encoding="utf-8")
    assert "_requestRender" in scene
    assert "configureRendering" in scene
    assert "getPixelRatio" in scene
    assert "shadowMapSize" in scene
    assert "priority: true" in main
    assert "leaseIntentUntil" in main
    assert "controlEpoch" in main


def test_g4a2_ui_separates_recording_and_frame_lifecycle():
    html = (STATIC / "g4a2.html").read_text(encoding="utf-8")
    script = (STATIC / "g4a2.js").read_text(encoding="utf-8")
    index = (STATIC / "index.html").read_text(encoding="utf-8")
    assert 'id="recording-lifecycle"' in html
    assert 'id="frame-lifecycle"' in html
    assert 'id="human-stop-result"' in html
    assert 'href="/g4a2"' in index
    assert "refreshInFlight" in script
    assert "image.decode" in script
    assert "setFrameState" in script
    assert "记录已保存" in script
    assert "结束失败" in script
