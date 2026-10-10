"""Session tokens, the disconnect grace period (D4 / C11) and the event log."""

import threading
from datetime import datetime, timedelta, timezone

import pytest

from src.server.events import EventLog
from src.server.session_manager import SessionManager
from src.server.world_config import WorldConfig


def make_session(sm, name="car"):
    return sm.create_session(name, "127.0.0.1", "vehicle.test", None)


# ── tokens ────────────────────────────────────────────────────────────────────────────────────────

def test_token_is_a_separate_secret_and_not_listed():
    sm = SessionManager()
    a, b = make_session(sm, "a"), make_session(sm, "b")
    assert a.session_token != a.session_id and a.session_token != b.session_token and len(a.session_token) >= 24
    assert "session_token" not in a.to_dict() and a.session_token not in str(a.to_dict())
    assert sm.get_by_token(a.session_token) is a and sm.get_by_token(b.session_token) is b
    assert sm.get_by_token("wrong") is None and sm.get_by_token(a.session_id) is None


def test_ended_session_token_stops_working():
    sm = SessionManager()
    a = make_session(sm)
    sm.end_session(a.session_id)
    assert sm.get_by_token(a.session_token) is None


# ── grace ─────────────────────────────────────────────────────────────────────────────────────────

def _age(session, seconds):
    session.last_seen = datetime.now(timezone.utc) - timedelta(seconds=seconds)


def test_grace_applies_only_without_connection_and_without_sign_of_life():
    sm = SessionManager()
    s = make_session(sm)
    assert sm.get_grace_expired(30) == []                    # just joined
    _age(s, 31)
    assert sm.get_grace_expired(30) == [s]                   # never connected, silent for 31 s
    sm.mark_connected(s.session_id)
    _age(s, 600)
    assert sm.get_grace_expired(30) == []                    # connected: never
    sm.mark_disconnected(s.session_id)                       # sets last_seen = now
    assert sm.get_grace_expired(30) == []
    _age(s, 29)
    assert sm.get_grace_expired(30) == []
    _age(s, 31)
    assert sm.get_grace_expired(30) == [s]
    sm.touch(s.session_id)                                   # e.g. REST telemetry poll
    assert sm.get_grace_expired(30) == []


def test_two_connections_are_counted():
    sm = SessionManager()
    s = make_session(sm)
    sm.mark_connected(s.session_id)
    sm.mark_connected(s.session_id)
    sm.mark_disconnected(s.session_id)
    assert s.ws_connections == 1 and s.to_dict()["connected"] is True
    sm.mark_disconnected(s.session_id)
    sm.mark_disconnected(s.session_id)                       # never negative
    assert s.ws_connections == 0 and s.to_dict()["connected"] is False


def test_ended_sessions_are_not_grace_expired():
    sm = SessionManager()
    s = make_session(sm)
    _age(s, 100)
    sm.end_session(s.session_id)
    assert sm.get_grace_expired(30) == []


# ── event log ─────────────────────────────────────────────────────────────────────────────────────

def test_event_log_sequence_and_since():
    log = EventLog(max_size=5)
    for i in range(3):
        log.add("k", frame=i, sim_time=i * 0.05, i=i)
    items, lost = log.since(0)
    assert [e["seq"] for e in items] == [1, 2, 3] and not lost and items[1]["data"] == {"i": 1}
    assert log.since(3) == ([], False)
    assert [e["seq"] for e in log.since(1)[0]] == [2, 3]


def test_event_log_reports_what_it_dropped():
    log = EventLog(max_size=3)
    for i in range(10):
        log.add("k", i=i)
    items, lost = log.since(0)
    assert [e["seq"] for e in items] == [8, 9, 10] and lost
    items, lost = log.since(7)
    assert [e["seq"] for e in items] == [8, 9, 10] and not lost
    assert log.last_seq == 10 and [e["seq"] for e in log.tail(2)] == [9, 10]


def test_event_log_limit_keeps_the_newest():
    log = EventLog(max_size=100)
    for _ in range(50):
        log.add("k")
    items, cut = log.since(0, limit=10)
    assert len(items) == 10 and items[-1]["seq"] == 50 and cut


def test_event_log_is_thread_safe():
    log = EventLog(max_size=100000)
    def work():
        for _ in range(500):
            log.add("k")
    ts = [threading.Thread(target=work) for _ in range(8)]
    [t.start() for t in ts]
    [t.join() for t in ts]
    seqs = [e["seq"] for e in log.since(0, limit=10**6)[0]]
    assert seqs == list(range(1, 4001))


# ── config ────────────────────────────────────────────────────────────────────────────────────────

def test_world_state_rate_must_be_whole_frames():
    assert WorldConfig(host="h").world_state_every_n_frames == 2
    with pytest.raises(ValueError, match="world_state_rate_hz"):
        WorldConfig(host="h", world_state_rate_hz=7.0)
    assert WorldConfig(host="h", world_state_rate_hz=20.0).world_state_every_n_frames == 1
    assert WorldConfig(host="h").session_disconnect_grace_s == 30.0 and WorldConfig(host="h").observer_public is True


def test_ended_sessions_are_listed_for_a_while_then_forgotten():
    sm = SessionManager()
    a, b = make_session(sm, "a"), make_session(sm, "b")
    sm.end_session(a.session_id)
    assert a.ended_at is not None and b.ended_at is None and a.to_dict()["ended_at"] and b.to_dict()["ended_at"] is None
    first = a.ended_at
    sm.end_session(a.session_id)                       # ending twice keeps the first time
    assert a.ended_at == first
    assert sm.prune_ended(600) == 0
    a.ended_at = datetime.now(timezone.utc) - timedelta(seconds=700)
    assert sm.prune_ended(600) == 1
    assert sm.get_session(a.session_id) is None and sm.get_session(b.session_id) is b
