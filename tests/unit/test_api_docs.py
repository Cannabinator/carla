"""docs/openapi.json and docs/api.md must match the code: an API change has to show up in the diff."""

import pytest

pytest.importorskip("httpx")

from src.server import api_docs  # noqa: E402


def test_committed_api_docs_are_current():
    stale = [p.name for p, text in api_docs.expected_files().items() if not p.exists() or p.read_text() != text]
    assert not stale, f"{stale} out of date: run `python -m src.server.api_docs` and commit the result"


def test_every_route_has_a_summary_and_every_topic_is_documented():
    spec = api_docs.openapi()
    missing = [f"{m.upper()} {path}" for path, ops in spec["paths"].items() for m, op in ops.items()
               if not op.get("summary")]
    assert not missing, f"routes without a summary: {missing}"
    md = api_docs.render_markdown(spec)
    from src.server.topics import TOPICS
    assert all(f"`{name}`" in md for name in TOPICS)
    assert "/ws/lidar/{sensor_id}" in md and "4401" in md
