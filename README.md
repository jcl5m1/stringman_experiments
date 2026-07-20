Experiments with [Stringman from Newfangled robotics](https://neufangled.com/)

[String man git repo](https://github.com/nhnifong/cranebot3-firmware#readme)

## 3D calibration viewer

`viewer.html` renders a solved calibration (camera poses, AprilTag poses, cube)
as an interactive three.js scene, with each camera's frustum texture-mapped by
its captured image. It loads the calibration JSON and images via relative URLs,
so it works as-is on GitHub Pages or any static server (not from `file://`):

```
python3 -m http.server   # then open http://localhost:8000/viewer.html
```

- mouse: orbit / zoom
- `1` `2` `3`: animate the view onto the solved pose + intrinsics of
  anchor0 / anchor1 / gripper (esc or drag to return to free orbit)
- `c` `d` `r`: switch frustum textures between captures/, annotated/
  (detected), reprojections/
- query params: `?calibration=detections/<prefix>_calibration.json`,
  `&cam=1..3`, `&source=captured|detected|reprojected`

