"""Static browser assembly owned by the UI work package."""

from __future__ import annotations

from pathlib import Path

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse


UI_ROOT = Path(__file__).resolve().parent
STATIC_ROOT = UI_ROOT / "static"


def create_ui_router() -> APIRouter:
    """Mount the built UI; API routes remain owned by the application assembly."""

    router = APIRouter()
    if not STATIC_ROOT.is_dir():
        return router

    @router.get("/", include_in_schema=False)
    def index() -> FileResponse:
        return FileResponse(STATIC_ROOT / "index.html")

    @router.get("/g4a2", include_in_schema=False)
    def g4a2_index() -> FileResponse:
        return FileResponse(STATIC_ROOT / "g4a2.html")

    @router.get("/static/{asset_path:path}", include_in_schema=False)
    def static_asset(asset_path: str) -> FileResponse:
        root = STATIC_ROOT.resolve()
        candidate = (root / asset_path).resolve()
        if not candidate.is_relative_to(root) or not candidate.is_file():
            raise HTTPException(status_code=404, detail="UI asset not found")
        return FileResponse(candidate)

    return router
