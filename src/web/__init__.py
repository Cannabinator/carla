"""Web asset helpers for the dashboard and visualization pages."""

from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parent
LEGACY_WEB_DIR = PACKAGE_DIR.parent / "visualization" / "web"
LEGACY_ASSETS_DIR = LEGACY_WEB_DIR / "assets"


def get_page_path(page_name: str) -> Path:
    """Return the path to a legacy HTML page by filename."""
    return LEGACY_WEB_DIR / page_name


__all__ = ["PACKAGE_DIR", "LEGACY_WEB_DIR", "LEGACY_ASSETS_DIR", "get_page_path"]