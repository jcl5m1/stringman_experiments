# Stringman experiments

Experiments with [Stringman from Newfangled robotics](https://neufangled.com/)
for camera calibration, AprilTag detection, and extrinsic pose estimation.
This repository is intended as a growing collection of experiments, so the
main README is written as a landing page that summarizes the current work and
points to more detailed notes for each topic.

Related repo: [Stringman firmware](https://github.com/nhnifong/cranebot3-firmware#readme)

## Current experiment areas

### 1. 3D calibration viewer

[![3D viewer screenshot](images/3d_viewer.jpg)](https://jcl5m1.github.io/stringman_experiments/viewer.html)

The hosted viewer at [https://jcl5m1.github.io/stringman_experiments/viewer.html](https://jcl5m1.github.io/stringman_experiments/viewer.html)
shows a solved calibration as an interactive three.js scene. It renders camera
frusta, detected AprilTag poses, and the cube geometry in a common world
frame, using the saved JSON calibration plus the captured and reprojected image
assets in this repo.

Local preview:

```bash
python3 -m http.server
# then open http://localhost:8000/viewer.html
```

Viewer controls:
- mouse: orbit / zoom
- `1` `2` `3`: animate the view onto the solved pose + intrinsics of
  anchor0 / anchor1 / gripper (esc or drag to return to free orbit)
- `c` `d` `r`: switch frustum textures between captures/, annotated/
  (detected), reprojections/
- query params: `?calibration=detections/<prefix>_calibration.json`,
  `&cam=1..3`, `&source=captured|detected|reprojected`

### 2. Alternative camera calibration method

A separate approach in this repo uses the firmware camera calibration data as a
starting point and solves for camera extrinsics and tag/cube poses by
minimizing AprilTag corner reprojection error. That method is documented in
detail in [calibration.md](calibration.md), including the solve formulation,
the frame-tree output format, the reprojection overlays, and the observed
errors for the current batch.

The key idea is to treat camera intrinsics as mostly fixed and fit the camera
poses and tag/cube poses jointly, rather than relying on a standalone
chessboard-style calibration for every camera. This is especially useful for
comparing the firmware-derived intrinsics with the locally observed reprojection
performance and for testing whether the gripper camera pose can be constrained
through the cube mount geometry.

### 3. Planned and future experiments

This repository is designed to grow as more experiments are added. The current
structure leaves room for additional investigations such as:
- alternate tag detection pipelines
- different optimization objectives or loss functions
- more camera pose priors and constraint formulations
- comparison between firmware calibration and local refinement
- additional visualization and analysis scripts

## Documentation index

- [calibration.md](calibration.md): detailed explanation of the local extrinsic
  calibration workflow, reprojection overlays, and results
- [capture_frames.md](capture_frames.md): frame capture workflow
- [config.json](config.json): tag and cube geometry configuration
- [viewer.html](viewer.html): interactive 3D viewer implementation

