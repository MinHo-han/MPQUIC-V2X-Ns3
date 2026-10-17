# NR V2X MPQUIC Simulation

ns-3 based simulation of MPQUIC-over-NR-V2X for digital-twin video streaming.  
Combines NR Uu (5G cellular) and NR Sidelink PC5 (V2X, 3GPP Rel. 16 Mode 2) into a dual-path QUIC transport.

---

## Software Versions

| Component | Version |
|-----------|---------|
| ns-3 | **3.42** |
| 5G-LENA NR module (CTTC) | **NR-v3.0** |
| 3GPP NR Sidelink | **Rel. 16** (PC5 Mode 2, sensing-based) |
| SUMO | 1.19.0 |
| Python | 3.12.x |
| CMake | ≥ 3.16 |
| GCC | ≥ 11 (tested: 13.3.0) |
| OS | Ubuntu 22.04 / 24.04 |

---

## Dependencies

```bash
sudo apt update
sudo apt install -y \
    g++ python3 python3-dev cmake ninja-build \
    libsqlite3-dev libxml2-dev libgsl-dev \
    pkg-config git
```

---

## Build

This repository includes the full ns-3 3.42 source tree together with the `contrib/nr` (5G-LENA NR-v3.0) and `contrib/mpquic` modules.  
Cloning the repository is sufficient — no separate ns-3 installation is needed.

```bash
git clone https://github.com/MinHo-han/MPQUIC-V2X-Ns3.git ns-3-v2x
cd ns-3-v2x
python3 ns3 configure --enable-examples
python3 ns3 build
```

The compiled binary is located at:
```
build/contrib/nr/examples/ns3.42-cttc-nr-v2x-mpquic-default
```

---

## SUMO Mobility Traces

Each `.tcl` file is a vehicle mobility trace exported from SUMO and converted to ns-2 format.  
When `--traceFile` is not specified, the simulation falls back to a `ConstantVelocity` mobility model.

### Nagoya (`sumo_scenarios/nagoya/`)

Urban tile C extracted from the Nagoya PLATEAU 2022 CityGML dataset (59.6% building footprint, high-density inner-city).  
Building footprint CSV is required by `BuildingsChannelConditionModel` for geometric LOS/NLOS classification.

| `--topoMode` | Mobility trace | Buildings CSV | Vehicles |
|---|---|---|---|
| `nagoya_tile_c` | `nagoya_tile_c_mobility.tcl` | `nagoya_tile_c_buildings.csv` | 50 |

### Gangnam (`sumo_scenarios/gangnam_large/`)

Three sub-scenarios derived from the Gangnam-gu, Seoul road network.  
Building footprint CSV is required by `BuildingsChannelConditionModel` for geometric LOS/NLOS classification.

| `--topoMode` | Mobility trace | Buildings CSV | Vehicles | Characteristics |
|---|---|---|---|---|
| `gangnam_corridor` | `gangnam_corridor_mobility.tcl` | `gangnam_corridor_buildings.csv` | 42 | Dense corridor, mixed LOS/NLOS |
| `gangnam_burst` | `gangnam_2km_mobility.tcl` | `gangnam_corridor_buildings.csv` | 43 | 2 km stretch, burst-loss focus |
| `gangnam_nlos` | `gangnam_nlos_mobility.tcl` | `gangnam_nlos_buildings.csv` | 37 | Deep NLOS, high building density |

> **Note**: All Nagoya and Gangnam modes require `--traceFile` to point to the corresponding `.tcl` file.

### Customizing gNB / RSU Positions

To define your own infrastructure layout, place a file named `<topoMode>_infra.csv`  
in the **same directory as your `.tcl` trace file**.  
The simulator detects it automatically at startup — no extra argument needed.

**Format** — one infrastructure node per line:

```
# lines starting with '#' are comments
# z is optional — defaults: gNB=25 m, RSU=6 m
gnb,x,y[,z]
rsu,x,y[,z]
```

**Coordinate bounds per scenario** (place all nodes within these ranges):

| `--topoMode` | X range (m) | Y range (m) | Area |
|---|---|---|---|
| `nagoya_tile_c` | 0 – 2000 | 0 – 2000 | 2 km × 2 km |
| `gangnam_corridor` | 4000 – 6000 | 4000 – 6000 | 2 km × 2 km |
| `gangnam_burst` | 4000 – 7000 | 6000 – 8000 | 3 km × 2 km |
| `gangnam_nlos` | 2000 – 4000 | 2000 – 4000 | 2 km × 2 km |

Template CSV files (commented examples only) are provided alongside each trace file.  
Uncomment and adjust the coordinates to match your deployment.

---

## Running a Simulation

### Basic example

```bash
BIN=./build/contrib/nr/examples/ns3.42-cttc-nr-v2x-mpquic-default

$BIN \
  --schedulerType=MinRtt \
  --topoMode=nagoya_tile_c \
  --traceFile=sumo_scenarios/nagoya/nagoya_tile_c_mobility.tcl \
  --nVehicles=50 \
  --nSenders=16 \
  --videoPreset=sd \
  --simTime=120 \
  --seed=1 \
  --outputDir=./results/test
```

### Key arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--schedulerType` | `MinRtt` | MPQUIC path selection algorithm |
| `--topoMode` | `highway` | Node placement scenario |
| `--traceFile` | _(none)_ | SUMO ns-2 mobility trace; omit for straight-line constant velocity |
| `--nVehicles` | `50` | Total number of vehicles |
| `--nSenders` | `0` (=all) | Number of video-sending vehicles |
| `--videoPreset` | `sd` | `lo` (0.5 Mbps) / `sd` (1.0 Mbps) / `hd` (3.0 Mbps) |
| `--simTime` | `120` | Simulation duration (seconds) |
| `--seed` | `1` | RNG seed |
| `--congLevel` | `none` | Uu background load: `none` / `light` / `medium` / `heavy` |
| `--outputDir` | `.` | Directory for output files |

### `--schedulerType` options

Three schedulers are available.

| Value | Class | Description |
|-------|-------|-------------|
| `RoundRobin` | `MpQuicRoundRobinScheduler` | Cycles through active paths in order |
| `MinRtt` | `MpQuicMinRttScheduler` | Always sends on the path with the lowest RTT |
| `Redundant` | `MpQuicRedundantScheduler` | Copies every packet to all active paths |

### `--topoMode` options

| Value | Description |
|-------|-------------|
| `highway` | Straight road, no buildings (default) |
| `nagoya_tile_c` | Nagoya inner-city tile C, 4 gNBs + 4 RSUs, with buildings |
| `gangnam_corridor` | Gangnam corridor, mixed LOS/NLOS, with buildings |
| `gangnam_burst` | Gangnam 2 km stretch, burst-loss focus, with buildings |
| `gangnam_nlos` | Gangnam deep NLOS, high building density |

> **Note**: All Nagoya and Gangnam modes require `--traceFile` to point to the corresponding `.tcl` file.

---

## Output Files

The following files are written to `--outputDir`:

```
{outputDir}/
├── {tag}_frames.csv           # Per-frame reception log (E2E latency, late_flag, etc.)
├── {tag}_veh{i}_epoch.csv     # Epoch aggregates (DMF, RTT, position, path stats, etc.)
├── {tag}_veh{i}_telemetry.csv # Raw per-epoch telemetry log
└── run_meta.json              # Run parameter metadata
```

Key columns in `_frames.csv`:

| Column | Description |
|--------|-------------|
| `frame_id` | Stream sequence number |
| `path_id` | Receiving path (0 = Uu, 1 = PC5) |
| `capture_ts_ms` | Frame transmission timestamp (ms) |
| `server_rx_ts_ms` | Application-layer drain timestamp (ms) |
| `late_flag` | Deadline miss flag (0 = on-time, 1 = late) |

---

## Example Sweep

```bash
# 36 runs: 3 schedulers × 3 presets × 4 congestion levels × 1 seed
# topoMode=nagoya_tile_c, nVehicles=50, nSenders=16, simTime=120

BIN=./build/contrib/nr/examples/ns3.42-cttc-nr-v2x-mpquic-default

for SCHED in RoundRobin MinRtt Redundant; do
  for PRESET in lo sd hd; do
    for CONG in none light medium heavy; do
      OUTDIR="./results/nagoya__${PRESET}__${CONG}__${SCHED}"
      $BIN \
        --schedulerType=${SCHED} \
        --topoMode=nagoya_tile_c \
        --traceFile=sumo_scenarios/nagoya/nagoya_tile_c_mobility.tcl \
        --nVehicles=50 --nSenders=16 \
        --congLevel=${CONG} --videoPreset=${PRESET} \
        --seed=1 --simTime=120 \
        --outputDir=${OUTDIR} &
    done
  done
done
wait
```

---

## Real-Device Prototype

A hardware prototype of MPQUIC running on real V2X equipment is available at:

**[https://github.com/TwinSteer/MPQUIC](https://github.com/TwinSteer/MPQUIC)**

That repository contains the on-board implementation for physical V2X devices and was jointly developed with the team at **Ajou University, South Korea** as part of the same project.  
This ns-3 repository covers the simulation framework only.

