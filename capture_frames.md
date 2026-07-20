# capture_frames.py

Capture one full-resolution frame from each Stringman camera (2 anchors +
gripper) and save them as JPEGs.

## Usage

```bash
# stop stringman-headless first (see "Why headless must be stopped" below)
brew install ffmpeg            # required, not a pip package
.venv/bin/pip install websockets

# easiest: discover components once, then capture from the JSON
.venv/bin/python discover_components.py                 # writes config.json
.venv/bin/python capture_frames.py --components config.json

# or pass IPs directly
.venv/bin/python capture_frames.py \
    gripper=192.168.68.140 \
    anchor0=192.168.68.139 \
    anchor1=192.168.68.141
# or bare IPs (files are labelled cam1, cam2, ...):
.venv/bin/python capture_frames.py 192.168.68.140 192.168.68.139 -o captures/
```

Each positional argument is `label=<ip>`; output is
`<out-dir>/<YYYYMMDD_HHMMSS>_<label>.jpg` (one timestamp per run, default dir
`captures/`). Dependencies: `ffmpeg` on PATH, Python package
`websockets` (already installed in this repo's `.venv`).

# discover_components.py

Finds Stringman components on the LAN without mDNS and writes
`config.json` for `capture_frames.py --components`.

```bash
.venv/bin/python discover_components.py                      # scan local /24
.venv/bin/python discover_components.py --subnet 192.168.68.0/24
.venv/bin/python discover_components.py --ips 192.168.68.139 192.168.68.140 192.168.68.141
```

How it works: TCP-probe every host in the subnet on port `8765` (the
component websocket port), then briefly connect each answering websocket and
classify from its JSON updates — `grip_sensors` → gripper; `line_record`,
`last_raw_encoder`, or `spoolN` → anchor; any of those keys (or `nf_robot_v`
/ `video_ready`) confirms a cranebot. Note that mDNS-based naming is not
used: arpeggio components don't send `nf_robot_v` reliably and service names
don't map cleanly to IPs, so key-sniffing the update stream is the robust
signal. Labels are assigned `gripper`, `anchor0`, `anchor1` (anchors ordered
by IP).

Caveats:

- The probe briefly attaches to each component, which starts and stops its
  camera (that is the component's design: the camera runs only while a
  client is connected). Don't run discovery in a tight loop — the Pi Zeros
  are slow to recycle the camera and repeated rapid connect/disconnect
  cycles can knock a component server over (it stops on unexpected
  exceptions) until systemd restarts it.
- Flaky wifi is the norm; the port probe retries 3 times with a 1.5 s
  timeout. If a component is missing from the JSON, just re-run.
- The JSON is a snapshot keyed to IPs; if your DHCP reassigns addresses,
  re-run discovery.

# reboot_components.py

Recovery tool for when component servers wedge or drop off the network
(which happens - Pi Zeros on wifi). For each component it checks port 8765;
anything not serving gets `sudo reboot` over ssh (user `pi`, key auth via
`~/.ssh/config`), then it polls until every component serves 8765 again.

```bash
python3 reboot_components.py                # IPs from config.json
python3 reboot_components.py --no-reboot    # just wait until all are up
python3 reboot_components.py --timeout 900 --interval 15
```

Components already serving are left alone; components fully off-network
can't be rebooted remotely (no ping, no ssh) and need a physical power
cycle. Exits 0 when all are up, 1 on timeout. No third-party dependencies -
system `python3` is fine.

## High-level Stringman data flow

Stringman is a distributed robot; there is no ROS. Three network tiers matter
for getting at images:

```
anchors (Pi Zero 2W) ----\
                          >-- tcp/8888 mpegts video --> stringman-headless --+--> local UI / AI clients
gripper (Pi Zero 2W)  ---/        (ws/8765 control)      (observer.py)      |
                                                                 ws/4245 telemetry (protobuf)
                                                                 MJPEG/HTTP video re-serve
                                                                 optional cloud relay (RTMP -> MediaMTX -> WebRTC/HLS/RTSP)
```

**Components.** Each anchor and the gripper is a Raspberry Pi Zero 2W running
a small websocket server on port `8765` (JSON messages). Components advertise
themselves via mDNS as `_http._tcp.local.` with names like
`123.cranebot-anchor-arpeggio-service.<mac>`. When a client connects to the
websocket, the component starts its camera (`rpicam-vid`) and serves an
H.264/mpegts stream on `tcp://0.0.0.0:8888` for exactly as long as the client
stays connected; when the client disconnects, the camera stops. Stream
parameters as shipped:

| component | resolution | fps | notes |
|---|---|---|---|
| anchor    | 1920x1080  | 20  | `--vflip --hflip` (mounted upside down) |
| gripper (arpeggio) | 684x384 | 60 | full-FOV 16:9 mode of the wide camera |

Both the websocket and the video socket accept **a single client**.

**stringman-headless** (`observer.py`) is that client in normal operation. It
discovers components via mDNS, consumes the raw streams for AprilTag
detection and object recognition, fuses visual + encoder estimates in a
Kalman filter, and runs the control loop. For consumers it offers:

- a **telemetry websocket** on `ws://<host>:4245` exchanging protobufs
  (`TelemetryBatchUpdate` out / `ControlBatchUpdate` in, schemas in
  `src/nf_robot/protos/`). It carries robot state and `VideoReady`
  announcements — but **no image pixels**; the author explicitly decided
  against putting frames on this socket (`docs/video_data_flow.md`).
- **re-served MJPEG-over-HTTP video** (`/stream.mjpeg`): gripper on port
  4246, anchor N on 4247+N, reprojected floor view on 8747, target heatmap on
  8748. Convenient, but downscaled and throttled (~5 fps gripper at 384x384,
  ~2 fps anchors) — fine for UIs and inference, not for full-quality capture.
- an optional **cloud relay** (`--telemetry_env=production`): video is pushed
  via RTMP to MediaMTX at media.neufangled.com and re-served as WebRTC
  (WHEP), HLS, or RTSP; telemetry is brokered through the same control plane.
  Nothing is stored server-side.

The lerobot integration (`src/nf_robot/ml/stringman_lerobot.py`) is just
another telemetry client: it reads state + `VideoReady` from ws/4245 and
pulls the actual frames from the announced stream URIs with PyAV.

## Design rationale

**Why the raw component streams (tcp/8888).** They are the only source of
un-downscaled, unthrottled frames: 1920x1080 for anchors, 684x384@60 for the
gripper. Every downstream option (headless MJPEG re-serves, cloud relay) is a
scaled, rate-limited re-encode. Note the raw frames are still
H.264-compressed by the Pi's hardware encoder; true sensor-max stills
(4608x2592) would require a different `rpicam` invocation on the component
itself (see `start_stream.sh` in the firmware repo) and are out of scope here.

**Why stringman-headless must be stopped.** Each component allows one
websocket client, and `rpicam-vid ... -o tcp://0.0.0.0:8888?listen=1` accepts
one TCP connection. While headless runs, it holds both. The script does not
check for this; a hung connect or ffmpeg timeout is the symptom.

**Why the websocket dance.** The camera does not run until a client connects
to the component's websocket, and it stops the moment the client disconnects.
So the script (1) opens `ws://<ip>:8765`, (2) waits for the periodic JSON
update containing `"video_ready": [port, timestamp]` — the component's own
signal that `rpicam-vid` is up (it parses the encoder's "Output #0, mpegts"
ready line), (3) captures, then (4) closes the socket so the camera stops.

**Why connect-then-grab.** Cameras take a second or two to start, so all
websockets are connected first; only once every stream reports ready does the
script grab one frame from each — concurrently, in threads — so the three
images are near-simultaneous rather than seconds apart.

**Why ffmpeg via subprocess.** The stream is H.264 in an mpegts container
over raw TCP; ffmpeg handles probing/decoding robustly with zero Python
imaging dependencies, and it matches the firmware repo's own experiment
style (`experiments/cap_from_stream.py`). The only Python dependency is
`websockets`.

## Limitations

- Exclusive access: nothing else (headless, another capture) can be attached
  to a component while this script holds it.
- "Simultaneous" means concurrent grabs of live streams, not hardware-synced
  shutters; expect small timing skew.
- Failure of one camera doesn't block the others; failures are listed on
  stderr and the exit code is non-zero.
