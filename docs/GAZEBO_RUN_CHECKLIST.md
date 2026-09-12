# Pre-Gazebo validation checklist

Status on 2026-09-10: **ALL LIVE GATES PENDING**. The commands and thresholds below are project acceptance criteria, not measured results. Run one experiment at a time and preserve every terminal log, JSONL, DataFlash BIN and configuration snapshot under a timestamped directory.

Update 2026-09-13: three exploratory velocity-level Gazebo/SITL runs now have
JSONL evidence (right-angle, narrow-gate and slalom; see
`docs/RUN_3_MPPI_MAPS_QUICKSTART_VI.md`). They do **not** mark the gates below
as passed because they were not executed as a complete checklist run with all
required environment, DataFlash, frame-sanity and abort-gate artifacts.

## Run identity and log directory

```bash
cd /Users/nguyenthanhlam/Projects/ardupilot_gazebo
RUN_ID=$(date +%Y%m%d_%H%M%S)
RUN_DIR="$PWD/output/log/$RUN_ID"
mkdir -p "$RUN_DIR"
git rev-parse HEAD | tee "$RUN_DIR/project_commit.txt"
git -C /Users/nguyenthanhlam/Projects/ardupilot rev-parse HEAD | tee "$RUN_DIR/ardupilot_commit.txt"
cp mppi_ardupilot/config.yaml mppi_ardupilot/rigid_pa_mppi_config.yaml \
  config/mppi_velocity.parm config/mppi_attitude_sitl.parm \
  config/ablation_mppi_unknown.yaml config/ablation_pa_mppi_unknown.yaml "$RUN_DIR/"
```

Expected identity for the 2026-09-13 evidence: project base commit `910c6a824fad0ca38615f93a0a17fc2c36d591cf` plus the recorded dirty-tree diff; ArduPilot `f808f78ce5a518ca96f2fb36420608b2b6254367`. Abort if the firmware commit differs until the source audit is repeated.

## Gate table

Each gate starts as `[ ] PENDING`. Mark it only after attaching the named logs and evaluating the metric.

### 0. Environment check — [ ] PENDING

Command:

```bash
export GZ_VERSION=harmonic
export GZ_PARTITION=ardupilot_warehouse_demo
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds:${GZ_SIM_RESOURCE_PATH:-}"
{
  gz sim --versions
  /opt/miniconda3/envs/ardupilot-rviz/bin/python - <<'PY'
import numpy, torch, pymavlink, yaml
print("numpy", numpy.__version__)
print("torch", torch.__version__)
print("pymavlink", pymavlink.__version__)
print("yaml", yaml.__version__)
PY
} 2>&1 | tee "$RUN_DIR/00_environment.log"
```

Metric/pass threshold: every import succeeds; Gazebo reports the intended major version; `build/libArduPilotPlugin.dylib` (macOS) or `.so` (Linux) exists. Abort: missing plugin, mixed Gazebo ABI, or software rendering so slow that sensor rate cannot be maintained.

### 1. Offline regression — [ ] PENDING FOR THIS RUN

Command:

```bash
KMP_DUPLICATE_LIB_OK=TRUE OMP_NUM_THREADS=1 \
  /opt/miniconda3/envs/ardupilot-rviz/bin/python -m unittest \
  tests.test_mppi_core -v 2>&1 | tee "$RUN_DIR/01_unit_tests.log"
```

Metric/pass threshold: 41/41 pass with no warning hidden by filtering. Abort: any failure, NaN or nondeterministic seeded test.

### 2. Gazebo server — [ ] PENDING

Command (terminal GZ-SERVER):

```bash
cd /Users/nguyenthanhlam/Projects/ardupilot_gazebo
export GZ_PARTITION=ardupilot_warehouse_demo
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds:${GZ_SIM_RESOURCE_PATH:-}"
gz sim -v2 -r "$PWD/worlds/iris_warehouse_sensor.sdf" -s \
  2>&1 | tee "$RUN_DIR/02_gazebo_server.log"
```

Expected telemetry: server runs without plugin/model errors. Metric/pass threshold: real-time factor is nonzero and `/iris/odometry` plus `/sensor_suite/lidar/points` appear within 10 s. Abort: NaN physics, model falling before arming, repeated sensor/plugin errors.

Optional GUI (separate terminal):

```bash
export GZ_PARTITION=ardupilot_warehouse_demo
gz sim -v1 -g --gui-config "$PWD/config/gazebo_runway_camera.config" \
  2>&1 | tee "$RUN_DIR/02_gazebo_gui.log"
```

### 3. ArduPilot SITL — [ ] PENDING

Start velocity-mode firmware first (terminal SITL):

```bash
cd /Users/nguyenthanhlam/Projects/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f JSON -N -w --console --map \
  --custom-location=-35.363262,149.165237,584,0 \
  --add-param-file=/Users/nguyenthanhlam/Projects/ardupilot_gazebo/config/mppi_velocity.parm \
  -A "--serial1=tcp:2" 2>&1 | tee "$RUN_DIR/03_sitl_velocity.log"
```

Expected telemetry: heartbeat and EKF initialization; TCP 5762 listening. Pass: mode changes work and no pre-arm sensor failure remains. Abort: wrong commit, repeated JSON backend timeout, EKF unhealthy.

For the later rigid gate, restart SITL cleanly and append:

```text
--add-param-file=/Users/nguyenthanhlam/Projects/ardupilot_gazebo/config/mppi_attitude_sitl.parm
```

Never change `GUID_OPTIONS` in a running experiment without restarting and recording the parameter set.

### 4. Sensor/topic verification — [ ] PENDING

Command:

```bash
{
  gz topic -l
  timeout 5 gz topic -e -t /iris/odometry
  timeout 5 gz topic -e -t /sensor_suite/lidar/points
} 2>&1 | tee "$RUN_DIR/04_topics.log"
```

On macOS without GNU `timeout`, use `gtimeout` from coreutils. Expected telemetry: changing odometry timestamps and non-empty point cloud. Pass: both update for at least 5 s; scan age remains below `stale_after_s`. Abort: static timestamps, empty scans, or frame/topic mismatch.

### 5. MAVLink heartbeat and parameters — [ ] PENDING

In MAVProxy:

```text
watch HEARTBEAT
param show OA_TYPE
param show AVOID_ENABLE
param show PRX1_TYPE
param show GUID_OPTIONS
```

Save MAVProxy output as `$RUN_DIR/05_mavlink_parameters.log`. Velocity pass: `OA_TYPE=0`, `AVOID_ENABLE=0`, `PRX1_TYPE=0`, `GUID_OPTIONS=0`. Rigid pass after restart: the first three remain zero and `GUID_OPTIONS=8`; `MOT_THST_HOVER=0.38`. Abort: missing heartbeat, unexpected OA active, or rigid gate does not read back exactly 8.

### 6. EKF/state verification — [ ] PENDING

MAVProxy:

```text
watch EKF_STATUS_REPORT
watch LOCAL_POSITION_NED
watch ATTITUDE
```

Expected telemetry: finite position/velocity/yaw and healthy EKF flags. Pass: no reset or position jump greater than 0.5 m while stationary for 10 s. Abort: EKF unhealthy, non-finite state, timestamp age over 0.5 s.

### 7. Frame sanity — [ ] PENDING

With the vehicle disarmed, move/rotate the Gazebo model only if the simulator workflow safely permits it, or use a short standard-controller maneuver. Record both `/iris/odometry` and `LOCAL_POSITION_NED`.

Pass contract:

- Gazebo ENU `(+x,+y,+z)` corresponds to MAVLink NED `(+y,+x,-z)` for this world alignment.
- Positive physical CCW yaw/up-axis maps to negative NED yaw-rate.
- Body FLU rate `(p,q,r)` maps to FRD `(p,-q,-r)`.
- Quaternion order at the planner boundary is `(w,x,y,z)` and remains unit length within `1e-3`.

Metric: transformed paired samples differ by at most 0.10 m, 0.10 m/s and 3 degrees after accounting for origin. Abort: any sign ambiguity; do not continue to body-rate control. Log: `$RUN_DIR/07_frame_sanity.jsonl` plus terminal notes.

### 8. Standard takeoff and hover gate — [ ] PENDING

MAVProxy:

```text
mode guided
arm throttle
takeoff 20
```

Use standard ArduPilot position/velocity control only. Pass after 10 s settled: altitude error ≤0.5 m, horizontal drift ≤0.5 m over 10 s, roll/pitch ≤5 degrees, EKF healthy. Abort immediately to `mode land` for altitude error >1.5 m, roll/pitch >15 degrees, motor saturation, or estimator reset. Logs: `$RUN_DIR/08_standard_hover_mavproxy.log` and DataFlash BIN.

### 9. Low-amplitude body-rate/thrust gate — [ ] PENDING

Restart SITL with both parm files. Reconfirm `GUID_OPTIONS=8`. Start with a 1 m clear-air goal:

```bash
MAVLINK20=1 KMP_DUPLICATE_LIB_OK=TRUE OMP_NUM_THREADS=1 \
  /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py --planner rigid-pa-mppi \
  --config mppi_ardupilot/rigid_pa_mppi_config.yaml \
  --experimental-attitude-control --state-source odom --goal "1,0,20" \
  --mav tcp:127.0.0.1:5762 \
  --diag-jsonl "$RUN_DIR/09_rigid_body_gate.jsonl" \
  2>&1 | tee "$RUN_DIR/09_rigid_body_gate.log"
```

Pass over 20 s: altitude deviation ≤0.75 m, roll/pitch ≤10 degrees, normalized thrust in `[0.1,0.8]`, no NaN, no stale/timeout gate, rate tracking has the correct sign, planner p95 <20 ms. Abort to `mode land`: altitude deviation >1.5 m, roll/pitch >20 degrees, thrust stays saturated >0.5 s, wrong rate sign, stale telemetry, or mode change.

### 10. Clear-air goal — [ ] PENDING

Use velocity mode first; only repeat with rigid mode after Gate 9 passes:

```bash
MAVLINK20=1 KMP_DUPLICATE_LIB_OK=TRUE OMP_NUM_THREADS=1 \
  /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py --planner mppi \
  --config mppi_ardupilot/config.yaml --goal "5,0,20" \
  --diag-jsonl "$RUN_DIR/10_clear_air_mppi.jsonl" \
  2>&1 | tee "$RUN_DIR/10_clear_air_mppi.log"
```

Pass: goal distance <1.0 m within 20 s; altitude deviation ≤0.75 m; zero deadline misses. Abort: hard-brake with no obstacle, divergence for 3 consecutive seconds, or any Gate 9 abort condition.

### 11. Static obstacle baseline — [ ] PENDING

```bash
MAVLINK20=1 KMP_DUPLICATE_LIB_OK=TRUE OMP_NUM_THREADS=1 \
  /opt/miniconda3/envs/ardupilot-rviz/bin/python \
  scripts/mppi_velocity_avoidance.py --planner mppi \
  --config mppi_ardupilot/config.yaml \
  --goal "16,10,20;30,0,20" \
  --diag-jsonl "$RUN_DIR/11_static_mppi.jsonl" \
  2>&1 | tee "$RUN_DIR/11_static_mppi.log"
```

Pass: final goal <1.0 m, measured clearance ≥5.0 m, no collision/contact and no deadline miss. Abort: clearance <1.0 m, hard-brake, map/state stale, or divergence. This validates the baseline pipeline only; it does not demonstrate a perception-aware advantage.

### 12. Unknown-environment PA-MPPI — [ ] PENDING

Use a prepared C-wall/U-shaped world whose geometry is hidden from the initial sensor view. Before running, archive the exact world file. Use the same start, goal, dynamics, safety limits, horizon, base cost and seed for both algorithms.

```bash
# Vanilla: perception coefficients disabled
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/mppi_velocity_avoidance.py --planner pa-mppi \
  --config config/ablation_mppi_unknown.yaml --seed 7 \
  --diag-jsonl "$RUN_DIR/12_cwall_vanilla_seed7.jsonl"

# PA-MPPI: only perception coefficients enabled
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/mppi_velocity_avoidance.py --planner pa-mppi \
  --config config/ablation_pa_mppi_unknown.yaml --seed 7 \
  --diag-jsonl "$RUN_DIR/12_cwall_pa_seed7.jsonl"
```

Repeat the documented seed set; do not retune algorithms independently. Pass for each run: no collision, goal <1.0 m, clearance above the shared threshold. Record success, time-to-goal, path length, minimum clearance, control smoothness, cycles, compute mean/p95/worst, deadline misses, mapped free volume and time until goal first becomes visible. Abort uses the same safety conditions for both algorithms.

### 13. Ablation and plots — [ ] PENDING

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python scripts/plot_mppi_experiment.py \
  --run vanilla="$RUN_DIR/12_cwall_vanilla_seed7.jsonl" \
  --run pa_mppi="$RUN_DIR/12_cwall_pa_seed7.jsonl" \
  --output-dir "$RUN_DIR/plots" \
  2>&1 | tee "$RUN_DIR/13_plotting.log"
```

Pass: plots are generated only from recorded fields; manually verify trajectory, altitude, velocity, rate tracking, thrust, clearance/goal and latency plots against the raw JSONL. Report every seed, including failures. No claim of PA-MPPI benefit is allowed from the simple waypoint-guided wall case alone.

## Post-run archive

Copy the ArduPilot DataFlash `.BIN`, world file, terminal logs and `git diff --binary` into `RUN_DIR`; create `MANIFEST.sha256`. A gate is not considered passed if its raw log is missing, even when a video exists.
