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

## Optional Dependencies

| Library | License | Link |
|---------|---------|------|
| **Open3D** | MIT License | http://www.open3d.org/ |
| **OpenCV** | Apache-2.0 | https://opencv.org/ |
| **Matplotlib** | PSF/BSD-style | https://matplotlib.org/ |
| **Pandas** | BSD-3-Clause | https://pandas.pydata.org/ |
| **Selenium** | Apache-2.0 | https://www.selenium.dev/ |

## Frontend Libraries (CDN)

| Library | License | Link |
|---------|---------|------|
| **Three.js** | MIT License | https://threejs.org/ |
| **OrbitControls** | MIT License | Part of Three.js |
| **PointerLockControls** | MIT License | Part of Three.js |

Three.js is Copyright (c) 2010-2024 Three.js Authors.
Full license: https://github.com/mrdoob/three.js/blob/dev/LICENSE

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

The following algorithms are original implementations using standard techniques:

| Algorithm | File | Description |
|-----------|------|-------------|
| Octree Downsampling | `src/utils/octree.py` | Voxel-based point cloud downsampling (common technique) |
| Binary Protocol | `src/utils/binary_protocol.py` | Custom binary serialization for LiDAR data |
| Lazy Evaluation | `src/utils/lazy.py` | Standard Python descriptor and memoization patterns |

These implementations do not derive from any copyrighted source code.

---

## License Compatibility

All dependencies are compatible with this project's MIT License:
- MIT License ✓
- BSD-3-Clause ✓
- Apache-2.0 ✓
- PSF License ✓

---

*Last updated: January 2026*
