"""The command line hands 'serve' options to start_server.py untouched."""

import sys

import start_server
from src import cli


def test_serve_forwards_leading_options(monkeypatch):
    seen = []
    monkeypatch.setattr(start_server, "main", lambda: seen.append(list(sys.argv)))
    monkeypatch.setattr(sys, "argv", ["keep"])
    assert cli.main(["serve", "--fake", "--port", "9999"]) == 0
    assert seen == [["start_server.py", "--fake", "--port", "9999"]]


def test_carla_address_defaults_come_from_the_environment(monkeypatch):
    monkeypatch.setenv("CARLA_HOST", "10.1.2.3")
    monkeypatch.setenv("CARLA_PORT", "2222")
    monkeypatch.setattr(sys, "argv", ["start_server.py"])
    args = start_server._parse_args()
    assert (args.carla_host, args.carla_port) == ("10.1.2.3", 2222)
