# Acknowledgments

This project uses the following open-source libraries, resources, and standards.

## Core Dependencies

| Library | License | Link |
|---------|---------|------|
| **CARLA Simulator** | MIT License | https://carla.org/ |
| **NumPy** | BSD-3-Clause | https://numpy.org/ |
| **FastAPI** | MIT License | https://fastapi.tiangolo.com/ |
| **Uvicorn** | BSD-3-Clause | https://www.uvicorn.org/ |
| **Pydantic** | MIT License | https://pydantic.dev/ |
| **WebSockets** | BSD-3-Clause | https://websockets.readthedocs.io/ |

## Analysis and development (optional)

| Library | License | Link |
|---------|---------|------|
| **Pandas**, **Matplotlib** | BSD-3-Clause / PSF-style | https://pandas.pydata.org/, https://matplotlib.org/ |
| **pytest**, **httpx** | MIT / BSD-3-Clause | https://pytest.org/, https://www.python-httpx.org/ |
| **Playwright** | Apache-2.0 | https://playwright.dev/ |
| **Ruff**, **Pyright** | MIT | https://docs.astral.sh/ruff/, https://github.com/microsoft/pyright |

## Frontend libraries (vendored, not loaded from a CDN)

| Library | License | Where |
|---------|---------|-------|
| **Three.js** (+ OrbitControls) | MIT | `src/web/static/vendor/three/` |
| **uPlot** | MIT | `src/web/static/vendor/uplot/` |

Versions and checksums: `src/web/static/vendor/VERSIONS.md`. Three.js is Copyright (c) 2010-2023 Three.js Authors.

## Industry Standards

This implementation is based on published industry standards for V2V communication:

### SAE J2735 - Basic Safety Message (BSM)
- **Standard:** SAE J2735 - Dedicated Short Range Communications (DSRC) Message Set Dictionary
- **Publisher:** SAE International
- **Link:** https://www.sae.org/standards/content/j2735_202007/
- **Note:** Implementation based on public specification. No copyrighted code used.

### ETSI ITS-G5 - Cooperative Awareness Message (CAM)
- **Standard:** ETSI EN 302 637-2 - Intelligent Transport Systems (ITS); Vehicular Communications
- **Publisher:** European Telecommunications Standards Institute
- **Link:** https://www.etsi.org/deliver/etsi_en/302600_302699/30263702/
- **Note:** Implementation based on public specification. No copyrighted code used.

## CARLA Resources

- **Semantic Segmentation Colors:** Color scheme for LiDAR semantic tags based on CARLA Simulator documentation
- **Python API:** Uses CARLA Python Client (carla==0.9.16)
- **CARLA License:** MIT License - https://github.com/carla-simulator/carla/blob/master/LICENSE

## Algorithm Implementations

The LiDAR wire protocol v3 (`src/lidar/protocol.py`) is a custom binary format written for this project. The channel
model (`src/v2x/channel/`) uses textbook techniques (log-distance path loss, log-normal shadowing, a logistic
reception curve); its parameters are assumptions and its literature references are unverified, see
`docs/science/model.md` and `docs/science/limitations.md`. No third-party source code is copied.

---

## License Compatibility

All dependencies are compatible with this project's MIT License:
- MIT License ✓
- BSD-3-Clause ✓
- Apache-2.0 ✓
- PSF License ✓

---

*Last updated: January 2026*
