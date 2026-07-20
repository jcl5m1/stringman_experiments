#!/usr/bin/env python3
"""
Calibrate anchor-camera and AprilTag extrinsics by bundle adjustment.

Minimizes the 2D reprojection error of detected AprilTag corners against
their 3D model points with scipy.optimize.least_squares:

  - World frame = tag 0 frame: tag 0's origin (lower-left corner) is fixed at
    (0,0,0), the tag lies in the z=0 plane facing +z, tag x-axis = world +X.
  - Every other tag is a rigid square with side length from config.json
    (apriltags.side_length_mm) and gets a 6-DOF pose (rvec + tvec).
  - Tag ID 1 lives on 4 sides of a rigid cube (config.json
    apriltags.marker_objects."1": width_mm, tags centered on the vertical
    faces, tops toward +z). The cube gets ONE 6-DOF pose for its center; each
    detection of ID 1 is assigned to the best-matching cube face.
  - Anchor cameras: solved 6-DOF extrinsics, initialized at (2,2,2) and
    (-2,-2,2) looking at the origin. Intrinsics are the cranebot3-firmware
    camera_cal values (see calibration.md).
  - Gripper camera: solved with intrinsics from cranebot3-firmware
    camera_cal_wide under the center-crop assumption (HACK, unverified - see
    calibration.md). It is rigidly mounted to the tag-1 cube: fixed offset
    loaded from config.json (apriltags.marker_objects."1".
    gripper_camera_offset_mm, in the cube frame whose +z points down in the
    world) with orientation = solved yaw about the cube z-axis, centered at
    the cube CENTER so the whole mount offset rotates with it, followed by a
    fixed x-tilt about the camera x-axis (gripper_camera_x_tilt_rad). The
    solve optimizes the yaw angle (gripper_theta) jointly with everything
    else; without the offset or cube detections: free 6-DOF from tag 0.

The solve runs in two stages, highest co-visibility first: stage 1 uses tags
seen by both anchors; stage 2 adds tags seen by only one anchor.

Inputs:  newest detections/<prefix>_detections.json (from detect_apriltags.py),
         config.json, captures/<prefix>_*.jpg (rendering only).
Outputs: detections/<prefix>_calibration.json, reprojections/<image>.jpg,
         and a popup window paging through the reprojection images.

Keys (popup): any key = next image, q/Esc = quit.

Requires: pip install opencv-contrib-python numpy scipy

Usage:
  python calibrate_extrinsics.py
  python calibrate_extrinsics.py --detections detections/20260720_083736_detections.json --no-ui
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

# Anchor intrinsics from cranebot3-firmware camera_cal (1920x1080), see calibration.md
K_ANCHOR = np.array([[1424.0, 0.0, 960.0], [0.0, 1424.0, 540.0], [0.0, 0.0, 1.0]])
DIST_ANCHOR = np.array([0.0115842, 0.18723804, -0.00126164, 0.00058383, -0.38807272])

# Gripper intrinsics: cranebot3-firmware camera_cal_wide (684x384) under the
# center-crop assumption - fx/fy unchanged, cx shifted by the 150px crop
# offset, distortion reused (it anchors at the principal point). HACK:
# unverified, see calibration.md.
K_GRIPPER = np.array(
    [
        [439.31834658631243, 0.0, 192.0],
        [0.0, 461.5621083718772, 192.0],
        [0.0, 0.0, 1.0],
    ]
)
DIST_GRIPPER = np.array(
    [
        -0.026228587204545444,
        -0.012309725227594465,
        -0.00033204923591180567,
        0.0015432535264626682,
        0.10759316594344916,
    ]
)

# per-camera (K, distortion) used for undistortion, solvePnP, and rendering
INTRINSICS = {
    "anchor0": (K_ANCHOR, DIST_ANCHOR),
    "anchor1": (K_ANCHOR, DIST_ANCHOR),
    "gripper": (K_GRIPPER, DIST_GRIPPER),
}

CAM_INIT_POS = {"anchor0": (2.0, 2.0, 2.0), "anchor1": (-2.0, -2.0, 2.0)}
GRIPPER_INIT_POS = (0.0, 0.0, 1.0)

ORIGIN_TAG = 0  # fixed at the world frame, never solved

CUBE_FACES = ["+X", "-X", "+Y", "-Y"]  # cube-face normals in the cube frame
OPPOSITE_FACE = {"+X": "-X", "-X": "+X", "+Y": "-Y", "-Y": "+Y"}

DETECTED_COLOR = (0, 255, 0)  # BGR
REPROJ_COLOR = (255, 0, 255)
CUBE_COLOR = (255, 255, 255)

WINDOW = "calibration reprojections"


# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------


def look_at(pos, view_dir=None, target=None):
    """world->cam (rvec, tvec) for a camera at pos. Optical axis from target
    (target - pos) or view_dir; image y down, world up +z when possible."""
    pos = np.asarray(pos, dtype=float)
    forward = np.asarray(target, dtype=float) - pos if target is not None else np.asarray(view_dir, dtype=float)
    forward /= np.linalg.norm(forward)
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, up)
    if np.linalg.norm(right) < 1e-6:  # looking straight along z
        right = np.cross(forward, np.array([0.0, 1.0, 0.0]))
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    R = np.stack([right, down, forward])  # rows = cam axes in world coords
    rvec, _ = cv2.Rodrigues(R)
    return rvec.flatten(), -R @ pos


def cam_center(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    return -R.T @ tvec


def rodrigues(rvec):
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3))
    return R


def mean_rotation(rotations):
    """SVD projection of the elementwise mean of rotation matrices."""
    M = np.mean(rotations, axis=0)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def tag_local_corners(side_m):
    """tl, tr, br, bl in the tag frame (origin at lower-left, matching the
    corner order saved by detect_apriltags.py)."""
    s = side_m
    return np.array([[0, s, 0], [s, s, 0], [s, 0, 0], [0, 0, 0]], dtype=float)


def cube_face_corners(cube_width_m, tag_side_m):
    """For each cube face: the tag's 4 corners (tl, tr, br, bl order) in the
    cube frame. Tags centered on vertical faces, tops toward +z."""
    h = cube_width_m / 2.0
    s = tag_side_m
    tag_y = np.array([0.0, 0.0, 1.0])
    local = [(0.0, s), (s, s), (s, 0.0), (0.0, 0.0)]  # tl, tr, br, bl offsets
    faces = {}
    normals = {"+X": [1, 0, 0], "-X": [-1, 0, 0], "+Y": [0, 1, 0], "-Y": [0, -1, 0]}
    for name in CUBE_FACES:
        n = np.array(normals[name], dtype=float)
        tag_x = np.cross(tag_y, n)  # so that tag_x x tag_y = n (outward)
        origin = n * h - (s / 2) * tag_x - (s / 2) * tag_y
        faces[name] = np.array([origin + x * tag_x + y * tag_y for x, y in local])
    return faces


def cube_wireframe(cube_width_m):
    """8 cube vertices and 12 edges (index pairs) in the cube frame."""
    h = cube_width_m / 2.0
    v = np.array([[x, y, z] for x in (-h, h) for y in (-h, h) for z in (-h, h)])
    edges = [
        (0, 1), (2, 3), (4, 5), (6, 7),
        (0, 2), (1, 3), (4, 6), (5, 7),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    return v, edges


# ---------------------------------------------------------------------------
# data loading
# ---------------------------------------------------------------------------


def load_inputs(detections_path, config_path):
    det_doc = json.loads(Path(detections_path).read_text())
    config = json.loads(Path(config_path).read_text())
    sizes_mm = {int(k): v for k, v in config["apriltags"]["side_length_mm"].items()}
    cube_cfg = config["apriltags"].get("marker_objects", {})
    return det_doc, sizes_mm, cube_cfg


def build_observations(det_doc, sizes_mm):
    """Flatten detections into observation dicts; undistort corners to
    normalized pinhole coordinates with each camera's own intrinsics.
    Cube (ID 1) detections become 'cube' obs."""
    observations = []
    for image_name, data in det_doc["images"].items():
        cam = image_name.rsplit(".", 1)[0].rsplit("_", 1)[-1]  # ..._anchor0.jpg -> anchor0
        if cam not in INTRINSICS:
            print(f"warning: no intrinsics for camera '{cam}', skipped {image_name}")
            continue
        K, dist = INTRINSICS[cam]
        dup_count = {}
        for tag in data["tags"]:
            tag_id = tag["id"]
            idx = dup_count.get(tag_id, 0)
            dup_count[tag_id] = idx + 1
            corners_px = np.array(tag["corners"], dtype=float)
            if tag_id not in sizes_mm:
                print(f"warning: tag {tag_id} in {image_name} has no size in config, skipped")
                continue
            corners_norm = cv2.undistortPoints(
                corners_px.reshape(-1, 1, 2), K, dist
            ).reshape(-1, 2)
            observations.append(
                {
                    "cam": cam,
                    "image": image_name,
                    "tag_id": tag_id,
                    "det_idx": idx,
                    "kind": "cube" if tag_id == CUBE_TAG_ID else "tag",
                    "corners_px": corners_px,
                    "corners_norm": corners_norm,
                }
            )
    return observations


# ---------------------------------------------------------------------------
# calibration state
# ---------------------------------------------------------------------------

CUBE_TAG_ID = 1


class Calibration:
    def __init__(self, observations, sizes_mm, cube_cfg):
        self.observations = observations
        self.sizes_m = {k: v / 1000.0 for k, v in sizes_mm.items()}
        self.cube_width_m = cube_cfg[str(CUBE_TAG_ID)]["width_mm"] / 1000.0
        self.cube_faces = cube_face_corners(
            self.cube_width_m, self.sizes_m[CUBE_TAG_ID]
        )
        self.cube_verts, self.cube_edges = cube_wireframe(self.cube_width_m)
        # rigid mount offset of the gripper camera in the cube frame, from
        # config.json (marker_objects."1".gripper_camera_offset_mm); None ->
        # gripper is solved as free 6-DOF instead of cube-constrained
        offset_mm = cube_cfg[str(CUBE_TAG_ID)].get("gripper_camera_offset_mm")
        self.gripper_cube_offset = (
            np.array(offset_mm, dtype=float) / 1000.0 if offset_mm is not None else None
        )
        # fixed pitch about the camera x-axis (radians), applied after the
        # solved yaw about the cube z-axis
        self.gripper_x_tilt = float(
            cube_cfg[str(CUBE_TAG_ID)].get("gripper_camera_x_tilt_rad", 0.0)
        )
        # face assignment per cube observation: obs index -> face name
        self.face_assignment = {}

        # initial camera poses
        self.cameras = {cam: look_at(pos, target=(0, 0, 0)) for cam, pos in CAM_INIT_POS.items()}
        self.gripper_theta = 0.0  # gripper yaw about the cube z-axis (mount constraint)

        # initial tag poses via solvePnP against the initial cameras
        self.tags = {}  # tag_id -> (rvec_tag2world, tvec_tag2world)
        self.cube_pose = None
        self._init_tag_poses()
        self._init_cube_pose()

    # -- initialization -----------------------------------------------------

    def _solve_pnp_world(self, obs):
        """Tag pose (rvec, tvec tag->world) from one observation, using the
        camera's current pose. None if solvePnP fails or the camera has no
        pose yet (e.g. gripper before its init)."""
        try:
            rvec_c2w, tvec_c2w = self.cam_pose(obs["cam"])
        except KeyError:
            return None
        K, dist = INTRINSICS[obs["cam"]]
        obj = tag_local_corners(self.sizes_m[obs["tag_id"]])
        ok, rvec_t2c, tvec_t2c = cv2.solvePnP(
            obj, obs["corners_px"], K, dist, flags=cv2.SOLVEPNP_IPPE
        )
        if not ok:
            return None
        R_c2w = rodrigues(rvec_c2w).T
        R_t2w = R_c2w @ rodrigues(rvec_t2c)
        t_t2w = R_c2w @ (tvec_t2c.flatten() - tvec_c2w)
        rvec_t2w, _ = cv2.Rodrigues(R_t2w)
        return rvec_t2w.flatten(), t_t2w

    def _init_tag_poses(self, only=None):
        by_id = {}
        for obs in self.observations:
            if obs["kind"] == "tag" and obs["tag_id"] != ORIGIN_TAG:
                if only is not None and obs["tag_id"] not in only:
                    continue
                by_id.setdefault(obs["tag_id"], []).append(obs)
        for tag_id, obs_list in by_id.items():
            poses = [p for o in obs_list if (p := self._solve_pnp_world(o)) is not None]
            if not poses:
                continue  # no usable observation yet (e.g. gripper-only tag)
            R = mean_rotation([rodrigues(p[0]) for p in poses])
            rvec, _ = cv2.Rodrigues(R)
            t = np.mean([p[1] for p in poses], axis=0)
            self.tags[tag_id] = (rvec.flatten(), t)

    def _cube_pose_from_face(self, obs, face_name):
        """Cube pose (rvec, tvec of the cube center) implied by one detection
        assigned to a specific face."""
        pnp = self._solve_pnp_world(obs)
        if pnp is None:
            return None
        rvec_t2w, t_t2w = pnp
        R_t2w = rodrigues(rvec_t2w)
        n_w = R_t2w @ np.array([0.0, 0.0, 1.0])  # tag normal = outward face normal
        y_w = R_t2w @ np.array([0.0, 1.0, 0.0])  # tag y = cube +z on every face
        s = self.sizes_m[CUBE_TAG_ID]
        h = self.cube_width_m / 2.0
        face_center_w = t_t2w + R_t2w @ np.array([s / 2, s / 2, 0.0])
        center_w = face_center_w - n_w * h
        # cube axes in world: z = tag_y; x or y = +/-normal depending on face
        z_c = y_w / np.linalg.norm(y_w)
        axis = {"+X": n_w, "-X": -n_w, "+Y": n_w, "-Y": -n_w}[face_name]
        if face_name in ("+X", "-X"):
            x_c = axis / np.linalg.norm(axis)
            y_c = np.cross(z_c, x_c)
        else:
            y_c = axis / np.linalg.norm(axis)
            x_c = np.cross(y_c, z_c)
        R_cube2w = mean_rotation([np.stack([x_c, y_c, z_c], axis=1)])
        rvec_cube2w, _ = cv2.Rodrigues(R_cube2w)
        return rvec_cube2w.flatten(), center_w

    def _init_cube_pose(self):
        cube_obs = [o for o in self.observations if o["kind"] == "cube"]
        if not cube_obs:
            return
        best = None
        for face_name in CUBE_FACES:
            pose = self._cube_pose_from_face(cube_obs[0], face_name)
            if pose is None:
                continue
            if len(cube_obs) == 2:
                # cameras are assumed to see opposing sides of the cube
                err = self._assignment_error(
                    pose, cube_obs, [face_name, OPPOSITE_FACE[face_name]]
                )
            else:
                err = self._cube_total_reproj(pose, cube_obs)
            if best is None or err < best[0]:
                best = (err, pose)
        self.cube_pose = best[1] if best else (np.zeros(3), np.zeros(3))
        self._reassign_faces()

    def init_gripper_pose(self):
        """Initial gripper world->cam pose from its observations of tag 0
        (whose world pose is fixed and exact) plus any other already-solved
        tags. Call after stage 1 so those tag poses are reliable."""
        obj_pts, img_pts = [], []
        for obs in self.observations:
            if obs["cam"] != "gripper" or obs["kind"] != "tag":
                continue
            if obs["tag_id"] == ORIGIN_TAG:
                pts_w = tag_local_corners(self.sizes_m[ORIGIN_TAG])
            elif obs["tag_id"] in self.tags:
                rvec, tvec = self.tags[obs["tag_id"]]
                pts_w = (
                    rodrigues(rvec) @ tag_local_corners(self.sizes_m[obs["tag_id"]]).T
                    + tvec.reshape(3, 1)
                ).T
            else:
                continue
            obj_pts.append(pts_w)
            img_pts.append(obs["corners_px"])
        if not obj_pts:
            print("warning: no gripper observations of known tags; gripper stays at guess")
            self.cameras["gripper"] = look_at(GRIPPER_INIT_POS, view_dir=(0, 0, -1))
            return
        obj = np.concatenate(obj_pts)
        img = np.concatenate(img_pts).reshape(-1, 1, 2)
        ok, rvec, tvec = cv2.solvePnP(obj, img, K_GRIPPER, DIST_GRIPPER)
        if not ok:
            print("warning: gripper solvePnP failed; gripper stays at guess")
            self.cameras["gripper"] = look_at(GRIPPER_INIT_POS, view_dir=(0, 0, -1))
            return
        self.cameras["gripper"] = (rvec.flatten(), tvec.flatten())
        pos = cam_center(*self.cameras["gripper"])
        print(f"gripper initial pose from tag {ORIGIN_TAG}: {[round(float(v), 3) for v in pos]}")

    def init_gripper_theta(self):
        """Initial gripper yaw about the cube z-axis (mount constraint): grid
        search for the theta with the lowest gripper reprojection error under
        the stage-1 cube pose."""
        gripper_obs = [o for o in self.observations if o["cam"] == "gripper"]
        best = None
        for deg in range(0, 360, 2):
            theta = np.deg2rad(deg)
            rvec_g, tvec_g = self._gripper_pose_from(self.cube_pose, theta)
            R_g, t_g = rodrigues(rvec_g), tvec_g.reshape(3, 1)
            total = 0.0
            for obs in gripper_obs:
                pts_w = self.obs_model_points(obs, self.tags, self.cube_pose).T
                p_c = R_g @ pts_w.T + t_g
                z = np.maximum(p_c[2], 1e-3)
                proj = (p_c[:2] / z).T
                total += float(np.sum((proj - obs["corners_norm"]) ** 2))
            if best is None or total < best[0]:
                best = (total, theta)
        self.gripper_theta = best[1]
        pos = cam_center(*self.cam_pose("gripper"))
        print(
            f"gripper mount constraint: theta init {np.rad2deg(self.gripper_theta):.0f} deg, "
            f"position {[round(float(v), 3) for v in pos]}"
        )

    # -- projection ----------------------------------------------------------

    def _gripper_pose_from(self, cube_pose, theta):
        """Gripper world->cam (rvec, tvec): rigid mount at
        self.gripper_cube_offset in the cube frame. The yaw theta about the
        cube z-axis is centered at the CUBE CENTER, so the whole mount
        (camera offset included) rotates with it; the fixed x-tilt then
        pitches the camera about its own x-axis."""
        R_c2w = rodrigues(cube_pose[0])
        R_yaw = rodrigues(np.array([0.0, 0.0, theta]))
        C_w = R_c2w @ (R_yaw @ self.gripper_cube_offset) + cube_pose[1]
        R_mount = R_yaw @ rodrigues(np.array([self.gripper_x_tilt, 0.0, 0.0]))
        R_w2c = R_mount @ R_c2w.T
        t_w2c = -R_w2c @ C_w
        rvec, _ = cv2.Rodrigues(R_w2c)
        return rvec.flatten(), t_w2c

    def cam_pose(self, cam):
        """(rvec, tvec) world->cam for cam. The gripper pose is derived from
        the cube pose + gripper_theta (mount constraint), not stored."""
        if (
            cam == "gripper"
            and self.cube_pose is not None
            and self.gripper_cube_offset is not None
        ):
            return self._gripper_pose_from(self.cube_pose, self.gripper_theta)
        return self.cameras[cam]

    def project_norm(self, points_w, cam):
        """world points -> normalized pinhole coords of cam. z clamped > 0 so
        points behind the camera yield large but finite residuals."""
        rvec, tvec = self.cam_pose(cam)
        p_c = rodrigues(rvec) @ points_w.T + tvec.reshape(3, 1)
        z = np.maximum(p_c[2], 1e-3)
        return (p_c[:2] / z).T

    def obs_model_points(self, obs, tag_poses, cube_pose):
        """3D world corners for an observation given current poses."""
        if obs["kind"] == "cube":
            face = self.face_assignment[id(obs)]
            rvec, tvec = cube_pose
            return rodrigues(rvec) @ self.cube_faces[face].T + tvec.reshape(3, 1)
        if obs["tag_id"] == ORIGIN_TAG:
            return tag_local_corners(self.sizes_m[ORIGIN_TAG]).T  # identity pose
        rvec, tvec = tag_poses[obs["tag_id"]]
        obj = tag_local_corners(self.sizes_m[obs["tag_id"]])
        return rodrigues(rvec) @ obj.T + tvec.reshape(3, 1)

    def _cube_total_reproj(self, cube_pose, cube_obs):
        """Sum of squared reprojection errors over cube observations, each
        against its best-matching face. Used to pick the initial assignment."""
        total = 0.0
        for obs in cube_obs:
            errs = []
            for face_name, corners_cube in self.cube_faces.items():
                rvec, tvec = cube_pose
                pts_w = rodrigues(rvec) @ corners_cube.T + tvec.reshape(3, 1)
                proj = self.project_norm(pts_w.T, obs["cam"])
                errs.append(np.sum((proj - obs["corners_norm"]) ** 2))
            total += min(errs)
        return total

    def _assignment_error(self, cube_pose, cube_obs, faces):
        """Sum of squared reprojection errors with a fixed per-observation
        face assignment."""
        total = 0.0
        rvec, tvec = cube_pose
        R = rodrigues(rvec)
        for obs, face_name in zip(cube_obs, faces):
            pts_w = R @ self.cube_faces[face_name].T + tvec.reshape(3, 1)
            proj = self.project_norm(pts_w.T, obs["cam"])
            total += float(np.sum((proj - obs["corners_norm"]) ** 2))
        return total

    def _reassign_faces(self):
        """Assign cube observations to the faces with the lowest reprojection
        error under the current cube pose. With exactly two cube observations,
        force opposing faces - the cameras are assumed to see opposite sides
        of the cube."""
        cube_obs = [o for o in self.observations if o["kind"] == "cube"]
        if len(cube_obs) == 2:
            best = None
            for face_name in CUBE_FACES:
                faces = [face_name, OPPOSITE_FACE[face_name]]
                err = self._assignment_error(self.cube_pose, cube_obs, faces)
                if best is None or err < best[0]:
                    best = (err, faces)
            for obs, face_name in zip(cube_obs, best[1]):
                self.face_assignment[id(obs)] = face_name
            return
        for obs in cube_obs:
            errs = {}
            for face_name in CUBE_FACES:
                self.face_assignment[id(obs)] = face_name
                proj = self.project_norm(
                    self.obs_model_points(obs, self.tags, self.cube_pose).T, obs["cam"]
                )
                errs[face_name] = np.sum((proj - obs["corners_norm"]) ** 2)
            self.face_assignment[id(obs)] = min(errs, key=errs.get)

    # -- optimization --------------------------------------------------------

    def solve(self, active_tag_ids, label, active_cams=None):
        """Joint least_squares over camera poses + the given tags (+ cube if
        any cube observation exists). active_cams limits which cameras'
        observations are used (cameras themselves come from self.cameras)."""
        var_tags = [t for t in active_tag_ids if t != ORIGIN_TAG]
        solve_cube = self.cube_pose is not None
        cams = sorted(self.cameras)
        # with a cube pose and a configured mount offset, the gripper pose is
        # derived from the cube pose + gripper_theta instead of free 6-DOF
        gripper_constrained = solve_cube and self.gripper_cube_offset is not None

        def pack():
            parts = []
            for cam in cams:
                parts.extend(self.cameras[cam])
            for tag_id in var_tags:
                parts.extend(self.tags[tag_id])
            if solve_cube:
                parts.extend(self.cube_pose)
            if gripper_constrained:
                parts.append(np.atleast_1d(self.gripper_theta))
            return np.concatenate(parts)

        def unpack(x):
            i = 0
            cam_poses = {}
            for cam in cams:
                cam_poses[cam] = (x[i : i + 3], x[i + 3 : i + 6])
                i += 6
            tag_poses = {}
            for tag_id in var_tags:
                tag_poses[tag_id] = (x[i : i + 3], x[i + 3 : i + 6])
                i += 6
            cube_pose = (x[i : i + 3], x[i + 3 : i + 6]) if solve_cube else self.cube_pose
            i += 6 if solve_cube else 0
            theta = x[i] if gripper_constrained else self.gripper_theta
            if gripper_constrained:
                cam_poses["gripper"] = self._gripper_pose_from(cube_pose, theta)
            return cam_poses, tag_poses, cube_pose, theta

        active_obs = [
            o
            for o in self.observations
            if (o["kind"] == "cube" or o["tag_id"] in active_tag_ids)
            and (active_cams is None or o["cam"] in active_cams)
        ]

        def residuals(x):
            cam_poses, tag_poses, cube_pose, theta = unpack(x)
            res = []
            for obs in active_obs:
                if obs["kind"] == "cube":
                    rvec, tvec = cube_pose
                    pts_w = (
                        rodrigues(rvec) @ self.cube_faces[self.face_assignment[id(obs)]].T
                        + tvec.reshape(3, 1)
                    )
                elif obs["tag_id"] == ORIGIN_TAG:
                    pts_w = tag_local_corners(self.sizes_m[ORIGIN_TAG]).T
                else:
                    rvec, tvec = tag_poses[obs["tag_id"]]
                    pts_w = rodrigues(rvec) @ tag_local_corners(
                        self.sizes_m[obs["tag_id"]]
                    ).T + tvec.reshape(3, 1)
                rvec_c, tvec_c = cam_poses[obs["cam"]]
                p_c = rodrigues(rvec_c) @ pts_w + tvec_c.reshape(3, 1)
                z = np.maximum(p_c[2], 1e-3)
                proj = (p_c[:2] / z).T
                # scale by the camera's fx so residuals are roughly pixels
                res.append(
                    (proj - obs["corners_norm"]).flatten() * INTRINSICS[obs["cam"]][0][0, 0]
                )
            return np.concatenate(res)

        result = least_squares(
            residuals,
            pack(),
            method="trf",
            x_scale="jac",
            loss="soft_l1",
            f_scale=1.0,
            verbose=2,
        )
        cam_poses, tag_poses, cube_pose, theta = unpack(result.x)
        if gripper_constrained:
            # derived from cube_pose + theta; don't store a stale copy
            del cam_poses["gripper"]
        self.cameras = cam_poses
        self.tags.update(tag_poses)
        self.cube_pose = cube_pose
        self.gripper_theta = float(theta)
        rms = np.sqrt(2 * result.cost / len(residuals(result.x)))
        print(f"[{label}] converged: {result.message}")
        print(f"[{label}] approx RMS residual: {rms:.3f} px over {len(active_obs)} observations")
        return result


# ---------------------------------------------------------------------------
# reporting / rendering
# ---------------------------------------------------------------------------


def rms_px_for_obs(cal, obs):
    """Honest pixel RMS for one observation via cv2.projectPoints with
    distortion on the original image coordinates."""
    pts_w = cal.obs_model_points(obs, cal.tags, cal.cube_pose).T
    rvec, tvec = cal.cam_pose(obs["cam"])
    K, dist = INTRINSICS[obs["cam"]]
    proj, _ = cv2.projectPoints(pts_w, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((proj - obs["corners_px"]) ** 2, axis=1))))


def draw_gripper_solution(out, cal, rvec, tvec, K, dist):
    """Draw the solved gripper camera (center dot, label, and a small frustum
    matching its intrinsics) and the L-shaped mount offset from the cube
    center: first the cube-frame z segment, then the y segment, as two white
    line segments."""
    grip_rvec, grip_tvec = cal.cam_pose("gripper")
    grip_C = cam_center(grip_rvec, grip_tvec)
    R_c2w = rodrigues(cal.cube_pose[0])
    cube_C = cal.cube_pose[1]
    R_yaw = rodrigues(np.array([0.0, 0.0, cal.gripper_theta]))
    z_part = R_c2w @ (R_yaw @ np.array([0.0, 0.0, cal.gripper_cube_offset[2]]))
    y_part = R_c2w @ (R_yaw @ np.array([0.0, cal.gripper_cube_offset[1], 0.0]))
    elbow = cube_C + z_part
    # frustum at depth d matching the gripper intrinsics: rays through the
    # image corners of the 384x384 frame
    fx, fy = K_GRIPPER[0, 0], K_GRIPPER[1, 1]
    cx, cy = K_GRIPPER[0, 2], K_GRIPPER[1, 2]
    d = 0.2  # frustum depth, meters
    img_corners = np.array([[0, 0], [384, 0], [384, 384], [0, 384]], dtype=float)
    corners_cam = (
        np.stack(
            [
                (img_corners[:, 0] - cx) / fx,
                (img_corners[:, 1] - cy) / fy,
                np.ones(4),
            ]
        )
        * d
    )
    R_g_c2w = rodrigues(grip_rvec).T  # gripper cam -> world
    corners_w = (R_g_c2w @ corners_cam + grip_C.reshape(3, 1)).T
    pts, _ = cv2.projectPoints(
        np.vstack([cube_C, elbow, grip_C, corners_w]), rvec, tvec, K, dist
    )
    p = pts.reshape(-1, 2).astype(int)
    cv2.line(out, tuple(p[0]), tuple(p[1]), (255, 255, 255), 2)  # z offset
    cv2.line(out, tuple(p[1]), tuple(p[2]), (255, 255, 255), 2)  # y offset
    cv2.circle(out, tuple(p[2]), 5, (255, 255, 255), -1)  # gripper camera center
    frustum = p[3:]
    for i in range(4):  # edges from center + image-plane rectangle
        cv2.line(out, tuple(p[2]), tuple(frustum[i]), (255, 255, 255), 1)
        cv2.line(out, tuple(frustum[i]), tuple(frustum[(i + 1) % 4]), (255, 255, 255), 1)
    cv2.putText(
        out, "gripper", tuple(p[2] + [8, -8]), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
        (255, 255, 255), 2,
    )


def render_reprojection(image, cal, cam, observations, note=None):
    """Draw detected corners (green dots) and solved reprojections (magenta
    outlines + X marks); cube also gets a white wireframe. The source image
    is dimmed to 33% so the annotations stand out."""
    out = cv2.convertScaleAbs(image, alpha=0.33, beta=0)
    rvec, tvec = cal.cam_pose(cam)
    K, dist = INTRINSICS[cam]
    rms_values = []
    for obs in observations:
        rms_values.append(rms_px_for_obs(cal, obs))
        for pt in obs["corners_px"]:
            cv2.circle(out, tuple(np.round(pt).astype(int)), 3, DETECTED_COLOR, -1)
        pts_w = cal.obs_model_points(obs, cal.tags, cal.cube_pose).T
        proj, _ = cv2.projectPoints(pts_w, rvec, tvec, K, dist)
        proj = proj.reshape(-1, 2).astype(int)
        cv2.polylines(out, [proj], True, REPROJ_COLOR, 1)
        for pt in proj:
            cv2.drawMarker(out, tuple(pt), REPROJ_COLOR, cv2.MARKER_TILTED_CROSS, 8, 1)
        if obs["kind"] == "cube":
            verts_w = (
                rodrigues(cal.cube_pose[0]) @ cal.cube_verts.T + cal.cube_pose[1].reshape(3, 1)
            ).T
            vproj, _ = cv2.projectPoints(verts_w, rvec, tvec, K, dist)
            vproj = vproj.reshape(-1, 2).astype(int)
            for a, b in cal.cube_edges:
                cv2.line(out, tuple(vproj[a]), tuple(vproj[b]), CUBE_COLOR, 1)
    if cam != "gripper" and cal.cube_pose is not None and cal.gripper_cube_offset is not None:
        draw_gripper_solution(out, cal, rvec, tvec, K, dist)
    if rms_values:
        text = f"rms {np.mean(rms_values):.2f} px ({len(rms_values)} tags)"
        cv2.putText(out, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    if note:
        cv2.putText(out, note, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return out


def save_results_json(path, cal, observations, detections_path, stages):
    def pose_entry(rvec, tvec):
        return {
            "rvec": [round(float(v), 6) for v in rvec],
            "position_m": [round(float(v), 4) for v in tvec],
        }

    cams = {}
    for cam in sorted({o["cam"] for o in observations}):
        rvec, tvec = cal.cam_pose(cam)
        obs_list = [o for o in observations if o["cam"] == cam]
        rms = [rms_px_for_obs(cal, o) for o in obs_list]
        entry = {
            "position_m": [round(float(v), 4) for v in cam_center(rvec, tvec)],
            "rvec_world_to_cam": [round(float(v), 6) for v in rvec],
            "tvec_world_to_cam": [round(float(v), 6) for v in tvec],
            "rms_px": round(float(np.mean(rms)), 3) if rms else None,
        }
        if cam == "gripper" and cal.gripper_cube_offset is not None:
            entry["mount_constraint"] = {
                "offset_in_cube_frame_m": [round(float(v), 4) for v in cal.gripper_cube_offset],
                "x_tilt_rad": round(float(cal.gripper_x_tilt), 4),
                "x_tilt_deg": round(float(np.rad2deg(cal.gripper_x_tilt)), 2),
                "source": "config.json apriltags.marker_objects.\"1\"",
                "note": "rigid mount in the tag-1 cube frame; solved yaw about the "
                "cube z-axis, fixed x-tilt about the camera x-axis, y rotation 0; "
                "pose derived from cube pose + theta",
                "theta_rad": round(float(cal.gripper_theta), 4),
                "theta_deg": round(float(np.rad2deg(cal.gripper_theta)), 1),
            }
        cams[cam] = entry
    tags = {}
    for tag_id, (rvec, tvec) in sorted(cal.tags.items()):
        obs_list = [o for o in observations if o["kind"] == "tag" and o["tag_id"] == tag_id]
        rms = [rms_px_for_obs(cal, o) for o in obs_list]
        entry = pose_entry(rvec, tvec)
        entry["rms_px"] = round(float(np.mean(rms)), 3) if rms else None
        tags[str(tag_id)] = entry
    tags[str(ORIGIN_TAG)] = {
        "position_m": [0.0, 0.0, 0.0],
        "rvec": [0.0, 0.0, 0.0],
        "fixed": "world frame anchor",
    }
    cube = None
    if cal.cube_pose is not None:
        rvec, tvec = cal.cube_pose
        cube_obs = [o for o in observations if o["kind"] == "cube"]
        rms = [rms_px_for_obs(cal, o) for o in cube_obs]
        cube = {
            "center_m": [round(float(v), 4) for v in tvec],
            "rvec": [round(float(v), 6) for v in rvec],
            "width_m": cal.cube_width_m,
            "face_assignment": {
                o["image"]: cal.face_assignment[id(o)] for o in cube_obs
            },
            "rms_px": round(float(np.mean(rms)), 3) if rms else None,
        }
    all_rms = [rms_px_for_obs(cal, o) for o in observations]
    doc = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "source_detections": str(detections_path),
        "intrinsics": {
            "anchor": {
                "resolution": [1920, 1080],
                "K": K_ANCHOR.tolist(),
                "distortion": DIST_ANCHOR.tolist(),
                "source": "cranebot3-firmware camera_cal (see calibration.md)",
            },
            "gripper": {
                "resolution": [384, 384],
                "K": K_GRIPPER.tolist(),
                "distortion": DIST_GRIPPER.tolist(),
                "source": "cranebot3-firmware camera_cal_wide under center-crop "
                "assumption (HACK, unverified - see calibration.md)",
            },
        },
        "world_frame": "tag 0 frame: origin at tag 0 lower-left corner, z-up",
        "overall_rms_px": round(float(np.mean(all_rms)), 3) if all_rms else None,
        "stages": stages,
        "cameras": cams,
        "tags": tags,
        "cube": cube,
    }
    Path(path).write_text(json.dumps(doc, indent=2) + "\n")
    print(f"saved {path}")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--detections", help="detections JSON (default: newest in --detections-dir)")
    parser.add_argument("--detections-dir", default="detections")
    parser.add_argument("--config", default="config.json")
    parser.add_argument("--captures", default="captures")
    parser.add_argument("--reproj-dir", default="reprojections")
    parser.add_argument("--no-ui", action="store_true", help="skip the popup window")
    args = parser.parse_args()

    if args.detections:
        detections_path = Path(args.detections)
    else:
        candidates = sorted(Path(args.detections_dir).glob("*_detections.json"))
        if not candidates:
            parser.error(f"no *_detections.json in {args.detections_dir}")
        detections_path = candidates[-1]
    print(f"using detections: {detections_path}")
    prefix = detections_path.name.split("_detections.json")[0]

    det_doc, sizes_mm, cube_cfg = load_inputs(detections_path, args.config)
    anchor_cams = sorted(CAM_INIT_POS)
    observations = build_observations(det_doc, sizes_mm)
    if not observations:
        parser.error("no usable observations in the detections file")

    cal = Calibration(observations, sizes_mm, cube_cfg)

    # co-visibility: tags seen by >= 2 cameras (cube counts via its faces)
    cam_of = {}
    for obs in observations:
        cam_of.setdefault(obs["cam"], set()).add(
            "cube" if obs["kind"] == "cube" else obs["tag_id"]
        )
    counts = {}
    for cam, ids in cam_of.items():
        for tid in ids:
            counts[tid] = counts.get(tid, 0) + 1
    stage1_ids = sorted(t for t in counts if counts[t] >= 2 and t != "cube")
    stage2_ids = sorted(
        t for t in {o["tag_id"] for o in observations if o["kind"] == "tag"}
        if t not in stage1_ids
    )

    stages = []
    print(f"\n=== stage 1: anchor cameras + co-visible tags {stage1_ids} + cube ===")
    cal.solve(stage1_ids, "stage 1", active_cams=anchor_cams)
    cal._reassign_faces()
    cal._init_tag_poses(only=stage2_ids)  # re-init against solved stage-1 cameras
    if cal.cube_pose is not None and cal.gripper_cube_offset is not None:
        cal.init_gripper_theta()  # mount constraint: yaw search about cube axis
    else:
        cal.init_gripper_pose()  # free 6-DOF fallback when no cube is observed
    missing = [t for t in stage2_ids if t not in cal.tags]
    if missing:
        cal._init_tag_poses(only=missing)  # tags only visible to the gripper
    still_missing = [t for t in stage2_ids if t not in cal.tags]
    if still_missing:
        print(f"warning: no init possible for tags {still_missing}, using origin guess")
        for t in still_missing:
            cal.tags[t] = (np.zeros(3), np.zeros(3))
    stages.append({"stage": 1, "tags": stage1_ids, "cube": True, "cameras": anchor_cams})

    print(f"\n=== stage 2: all cameras, all tags (adding {stage2_ids}) ===")
    cal.solve(stage1_ids + stage2_ids, "stage 2")
    cal._reassign_faces()
    stages.append(
        {"stage": 2, "tags": stage1_ids + stage2_ids, "cube": True,
         "cameras": sorted({o["cam"] for o in observations})}
    )

    print("\nper-camera RMS (px):")
    for cam in sorted({o["cam"] for o in observations}):
        rms = [rms_px_for_obs(cal, o) for o in observations if o["cam"] == cam]
        print(f"  {cam}: {np.mean(rms):.3f}")
    print("per-tag RMS (px):")
    for tag_id in sorted({o["tag_id"] for o in observations}):
        obs_list = [o for o in observations if o["tag_id"] == tag_id]
        rms = [rms_px_for_obs(cal, o) for o in obs_list]
        print(f"  tag {tag_id}: {np.mean(rms):.3f}")

    json_path = Path(args.detections_dir) / f"{prefix}_calibration.json"
    save_results_json(json_path, cal, observations, detections_path, stages)

    # render reprojection images for every image in the detections file
    reproj_dir = Path(args.reproj_dir)
    reproj_dir.mkdir(parents=True, exist_ok=True)
    rendered = []
    for image_name in det_doc["images"]:
        cam = image_name.rsplit(".", 1)[0].rsplit("_", 1)[-1]
        image_path = Path(args.captures) / image_name
        image = cv2.imread(str(image_path))
        if image is None:
            print(f"{image_name}: unreadable, skipped")
            continue
        img_obs = [o for o in observations if o["image"] == image_name]
        note = (
            "gripper intrinsics = center-crop assumption (see calibration.md)"
            if cam == "gripper"
            else None
        )
        out = render_reprojection(image, cal, cam, img_obs, note=note)
        out_path = reproj_dir / image_name
        cv2.imwrite(str(out_path), out)
        print(f"saved {out_path}")
        rendered.append((image_name, out))

    if not args.no_ui:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 1440, 810)
        for name, image in rendered:
            cv2.imshow(WINDOW, image)
            key = cv2.waitKey(0) & 0xFF
            if key in (ord("q"), 27):
                break
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
