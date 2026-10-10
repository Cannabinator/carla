"""Command line: ``python -m src.cli run scenario.yaml [--fake] [--runs-dir runs]``; ``serve`` starts the web server."""

import argparse
import logging
import sys


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="carla-v2x", description="CARLA V2X + LiDAR research server")
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="run a scenario headless and record it")
    r.add_argument("scenario")
    r.add_argument("--runs-dir", default="runs")
    r.add_argument("--fake", action="store_true", help="use the deterministic synthetic world instead of CARLA")
    r.add_argument("-v", "--verbose", action="store_true")
    sub.add_parser("serve", help="start the web server; every option after 'serve' goes to start_server.py (--help lists them)")
    argv = sys.argv[1:] if argv is None else list(argv)
    if argv[:1] == ["serve"]:               # argparse cannot forward leading --options, so hand them over untouched
        import start_server
        sys.argv = ["start_server.py", *argv[1:]]
        start_server.main()
        return 0
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    from src.scenario import load_scenario, run_scenario
    manifest = run_scenario(load_scenario(args.scenario), args.runs_dir, fake=True if args.fake else None)
    print(f"run {manifest['run_id']}: {manifest['status']}, {manifest['frames_recorded']} V2X updates, "
          f"{manifest['ticks']} ticks -> {manifest['_dir']}")
    for name, info in manifest["files"].items():
        if "sha256" in info:
            print(f"  {name:20s} {info['lines']:>9} lines  {info['bytes']:>10} bytes  sha256 {info['sha256'][:16]}")
    return 0 if manifest["status"] == "finished" else 2


if __name__ == "__main__":
    sys.exit(main())
