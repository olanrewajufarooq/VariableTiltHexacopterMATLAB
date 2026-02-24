# Hexacopter MATLAB Simulation

A MATLAB framework for simulating and controlling a rigid-body hexacopter.
It provides SE(3) dynamics, trajectory generation, multiple controllers,
and optional parameter adaptation with live visualization and plots.

Note: The repository name is legacy; the current model is a standard
hexacopter (allocation/tilt-specific logic has been removed).

## Features

- Rigid-body hexacopter dynamics with optional ground contact
- Trajectory generators (hover, circle, Lissajous, helix, etc.)
- Controllers: PD, feedlinearization, feedforward
- Optional parameter adaptation (mass/CoG/inertia)
- Live visualization with URDF or fallback stick model
- Logging, summary plots, and tracking metrics

## Requirements

- MATLAB (recent version)
- Robotics System Toolbox (optional, for URDF visualization)

## Quick Start

1. Open MATLAB and set the working directory to the repo root.
2. Run the startup helper to add the project to the path.

```matlab
run('run/startup.m')
```

## Demos

Two ready-to-run scripts live in `run/`:

1. Nominal demo (no adaptation)

```matlab
run('run/run_nominal_demo.m')
```

2. Adaptive demo (payload drop + parameter estimation)

```matlab
run('run/run_adaptive_demo.m')
```

Each demo produces summary plots and saves logs under `results/`.

## Configuration Basics

The workflow is to create a `vt.config.Config` object, set parameters,
then run a `vt.sim.SimRunner`:

```matlab
cfg = vt.config.Config();
cfg.setTrajectory('lissajous3d', 1.2);
cfg.setTrajectoryMethod('precomputed');
cfg.setController('Feedforward');
cfg.setPotentialType('liealgebra');
cfg.setAdaptation('none');
cfg.setSimParams(0.005, 30);
cfg.done();

sim = vt.sim.SimRunner(cfg);
sim.setup();
sim.run();
sim.save(true, 'summary');
```

## Project Structure

- `src/+vt/+sim` - simulation runner
- `src/+vt/+plant` - hexacopter plant dynamics
- `src/+vt/+ctrl` - controllers, potentials, adaptation
- `src/+vt/+traj` - trajectory generators
- `src/+vt/+plot` - live and summary plotting
- `src/+vt/+metrics` - tracking metrics
- `run/` - demo scripts

## Notes

- Live view and URDF options are configured in the demo scripts.
- If Robotics System Toolbox is unavailable, the viewer falls back to a lightweight stick model.
