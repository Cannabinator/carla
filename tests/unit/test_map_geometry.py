"""build_map_geometry on a hand-made map: a straight 10 m lane, a lane that turns, and one junction."""

from types import SimpleNamespace

from src.server.map_geometry import build_map_geometry, lane_polyline


class WP:
    """A waypoint on a polyline; ``next(d)`` returns the waypoint d metres ahead along the polyline."""
    def __init__(self, path, i, junction=None):
        self.path, self.i, self.junction = path, i, junction
        x, y = path[i]
        self.transform = SimpleNamespace(location=SimpleNamespace(x=x, y=y, z=0.0))

    @property
    def is_junction(self):
        return self.junction is not None

    def get_junction(self):
        return self.junction

    def next(self, d):
        j = self.i + int(round(d))               # the path has one point per metre
        return [WP(self.path, j, self.junction)] if j < len(self.path) else []


def straight(n=11):
    return [(float(i), 0.0) for i in range(n)]


class FakeMap:
    name = "Town99"

    def __init__(self, edges):
        self.edges = edges

    def get_topology(self):
        return self.edges


def edge(path, junction=None):
    return (WP(path, 0, junction), WP(path, len(path) - 1, junction))


def test_lane_polyline_steps_and_ends_at_the_end_waypoint():
    path = straight(11)
    pts = lane_polyline(WP(path, 0), WP(path, 10), 2.0)
    assert pts[0] == (0.0, 0.0) and pts[-1] == (10.0, 0.0)
    xs = [p[0] for p in pts]
    assert xs == sorted(xs) and len(xs) == len(set(xs)) and 5 <= len(xs) <= 7
    assert all(b - a <= 2.0 + 1e-9 for a, b in zip(xs, xs[1:]))


def test_lane_that_ends_early_still_reaches_the_target():
    path = straight(5)
    end = WP([(9.0, 0.0)], 0)
    pts = lane_polyline(WP(path, 0), end, 2.0)
    assert pts[-1] == (9.0, 0.0)


def test_geometry_structure_dedup_and_junctions():
    j = SimpleNamespace(id=7, bounding_box=SimpleNamespace(location=SimpleNamespace(x=5.0, y=0.0),
                                                           extent=SimpleNamespace(x=3.0, y=2.0)))
    m = FakeMap([edge(straight(11)), edge(straight(11)), edge([(0.0, 5.0 + i) for i in range(6)], junction=j)])
    g = build_map_geometry(m, 2.0)
    assert g["map_name"] == "Town99" and len(g["lanes"]) == 2          # the duplicated edge is dropped
    assert g["junctions"] == [{"id": 7, "bbox": [2.0, -2.0, 8.0, 2.0]}]
    assert all(len(p) == 2 for lane in g["lanes"] for p in lane)
    assert build_map_geometry(m, 2.0)["version"] == g["version"]
    assert build_map_geometry(FakeMap([edge(straight(11))]), 2.0)["version"] != g["version"]


def test_empty_map():
    g = build_map_geometry(FakeMap([]))
    assert g["lanes"] == [] and g["junctions"] == []
