# Hexacopter MATLAB Simulation Framework

A comprehensive MATLAB framework for simulating, controlling, and analyzing rigid-body hexacopter dynamics with advanced trajectory planning, multiple controller options, and parameter adaptation capabilities.

## Project Overview

This framework provides a complete simulation environment for hexacopter research and development, featuring:

- **Advanced Dynamics**: SE(3) rigid-body dynamics with optional ground contact modeling
- **Trajectory Generation**: Multiple predefined trajectories (hover, circle, Lissajous, helix, etc.)
- **Controller Suite**: PD, feedback linearization, feedforward controllers with adaptive capabilities
- **Parameter Adaptation**: Real-time estimation of mass, center of gravity, and inertia properties
- **Batch Simulations**: Multi-trajectory and multi-gain sweep runs with automated reporting
- **Visualization**: Live 3D visualization with URDF support or fallback stick model
- **Analysis Tools**: Comprehensive logging, summary plots, and tracking metrics
- **CI/CD**: Tag-driven GitHub Actions pipeline for automated paper result reproduction

> **Note**: Despite the repository name, this is a standard hexacopter simulation framework. The tilt-specific logic has been removed, and the model now represents a conventional hexacopter with variable allocation capabilities.

## Architecture

The simulation pipeline flows through four main stages:

```
Config  -->  SimRunner.setup()  -->  SimRunner.run()  -->  plot / save
```

**Key classes and their roles:**

| Class | Package | Role |
|-------|---------|------|
| `Config` | `vt.config` | Fluent configuration builder — vehicle, sim, trajectory, controller, viz, payload |
| `SimRunner` | `vt.sim` | Orchestrates setup, simulation loop, logging, batch execution, and results persistence |
| `HexacopterPlant` | `vt.plant` | SE(3) rigid-body dynamics integrator with ground contact |
| `WrenchController` | `vt.ctrl` | Computes body wrench using PD / FeedLin / Feedforward modes |
| `PreComputedTrajectory` | `vt.traj` | Analytical trajectory generator (9 built-in types) |
| `ModelReferenceTrajectory` | `vt.traj` | Filter-based reference model trajectory |
| `EuclideanAdaptation` | `vt.ctrl.adapt` | Online estimation of mass, CoG, and inertia (10-parameter regressor) |
| `Plotter` | `vt.plot` | Live updating and summary figure generation |
| `UrdfViewer` | `vt.plot` | 3D URDF visualization with stick-model fallback |
| `TrackingMetrics` | `vt.metrics` | Position, orientation, and parameter estimation error analysis |
| `Logger` | `vt.core` | Structured time-series data collection |

**Design patterns**: Factory pattern (trajectories, adaptation, potentials), abstract base classes, fluent builder (Config), handle semantics.

## Getting Started

### Prerequisites

- **MATLAB**: R2020b or later recommended
- **Robotics System Toolbox**: Optional, for URDF visualization
- **System Requirements**: Standard MATLAB installation with sufficient memory for 3D visualization

### Installation

```bash
git clone https://github.com/olanrewajufarooq/VariableTiltHexacopterMATLAB.git
cd VariableTiltHexacopterMATLAB
```

```matlab
% From MATLAB: change into the repo folder and add paths
cd 'path/to/VariableTiltHexacopterMATLAB'
startup
```

### Paper Result Reproduction

```matlab
% For adaptive control with parameter estimation (Paper 1)
run_adaptive_paper_ICUAS

% For modeling and control allocation (Paper 2)
run_nominal_paper_ICUAS
```

### Running Demos

```matlab
% Quick nominal simulation
run_nominal_demo

% Adaptive simulation with payload drop
run_adaptive_demo
```

## Configuration Guide

The main entry point is `vt.config.Config` (a fluent configuration object). Configure what you need, then call `cfg.done()` to finalize time steps and trajectory timing.

### Basic Configuration

```matlab
startup

cfg = vt.config.Config();

% Trajectory
cfg.setTrajectory('lissajous3d', 1.2);        % name, cycles

% Controller
cfg.setController('Feedforward');             % 'PD' | 'Feedforward' | 'FeedLin'

% Timing
cfg.setSimParams(0.005, 30);                  % sim_dt, duration (s)

cfg.done();
```

### Adaptive Payload-Drop Example

```matlab
startup

duration = 30;
cfg = vt.config.Config();

cfg.setTrajectory('lissajous3d', 1.25);
cfg.setTrajectoryMethod('precomputed');

cfg.setController('Feedforward');
cfg.setPotentialType('liealgebra');
cfg.setAdaptation('euclidean');

cfg.setSimParams(0.005, duration);
cfg.setAdaptationParams(0.005);
cfg.setControlParams(0.01);

% payload: mass (kg), CoG (m), drop time (s), startWithTrueValues
cfg.setPayload(1.5, [0.115; 0.05; -0.05], 2*duration/3, false);

cfg.done();
```

### Configuration Reference

```matlab
% Trajectory
cfg.setTrajectory('lissajous3d', 1.2);        % name, cycles
cfg.setTrajectoryMethod('precomputed');       % 'precomputed' | 'modelreference'

% Controller
cfg.setController('Feedforward');             % 'PD' | 'Feedforward' | 'FeedLin'
cfg.setPotentialType('liealgebra');           % 'liealgebra' | 'separate'
cfg.setAdaptation('none');                    % 'none' | 'euclidean' | 'geo-aware'

% Timing
cfg.setSimParams(0.005, 30);                  % sim_dt, duration (s)
cfg.setControlParams(0.005);                  % control_dt (s)
cfg.setAdaptationParams(0.005);               % adaptation_dt (s)

% Enabling Live Visualization
cfg.enableLiveView(true);
cfg.setLiveSummary(true);
cfg.setLiveUpdateRate(100);                   % update every N control steps
cfg.setLiveUrdfEmbedding(false);
cfg.setPlotLayout('column-major');            % 'row-major' | 'column-major'

% Tuning Gains
cfg.setKpGains([5.5 5.5 5.5 5.5 5.5 5.5]);     % 6x1 proportional gain vector
cfg.setKdGains([2.05 2.05 2.05 2.05 2.05 2.05]); % 6x1 derivative gain vector
cfg.setAdaptiveGains(4e-3);                    % adaptive gain (scalar or matrix)

% Payload
cfg.setPayload(1.5, [0.115; 0.05; -0.05], 2*30/3, false);
```

## Batch Simulations

Batch mode runs multiple trajectories and/or multiple gain configurations in a single call. Pass a cell array of trajectory names and a matrix of adaptive gains (one row per run):

```matlab
cfg = vt.config.Config();

% Multiple trajectories — each is run with every gain row
cfg.setTrajectory( ...
    {'circle', 'infinity3d', 'lissajous3d', 'helix3d', 'poly3d'}, ...
    2.25, true);

% Multiple gain configurations — one row per run per trajectory
cfg.setAdaptiveGains(1e-2 * [ ...
    0,   0,   0,   0,   0,   0,  0,   0,   0,   0; ...   % baseline (no adaptation)
    8,   8,  12, 0.4, 0.4, 0.4, 36,  12,  12,  12; ...   % moderate gains
    8,   8,  12, 0.4, 0.4, 0.4, 36, 120, 120, 120]);     % high CoG gains

cfg.setAdaptation('euclidean');
cfg.setController('Feedforward');
cfg.setSimParams(0.005, 60);
cfg.done();

sim = vt.sim.SimRunner(cfg);
sim.setup();
sim.run();
sim.plot('all');
```

This produces `5 trajectories x 3 gain rows = 15 runs`, each saved in its own subdirectory under `results/adaptive/<timestamp>_multi_traj/`.

## Features

### Dynamics Engine

- **SE(3) Rigid-Body Dynamics**: Full 6-DOF motion on the SE(3) manifold via Euler-Poincare equations
- **Ground Contact**: Optional spring-damper-friction ground interaction modeling

### Trajectory Generation

| Trajectory | Description |
|-----------|-------------|
| `hover` | Stationary position maintenance |
| `circle` | Circular path following |
| `infinity` | Planar figure-eight |
| `infinity3d` | 3D figure-eight |
| `infinity3dmod` | Modified 3D figure-eight with amplitude modulation |
| `lissajous3d` | Complex 3D Lissajous patterns |
| `helix3d` | Helical spiral motion |
| `poly3d` | Polynomial trajectory |
| `takeoffland` | Takeoff and landing maneuver |

### Controller Options

- **PD Controller**: Proportional-Derivative control with potential-based error
- **Feedforward Controller**: Full state feedforward with Coriolis and inertia compensation
- **FeedLin**: Nonlinear feedback linearization with Coriolis compensation
- **Adaptive Controllers**: Real-time parameter estimation (Euclidean adaptation)

### Visualization

- **URDF Support**: Advanced 3D visualization with Robotics System Toolbox
- **Fallback Model**: Lightweight stick model when URDF unavailable
- **Live Updates**: Real-time state visualization during simulation
- **Summary Plots**: Comprehensive post-simulation analysis figures

## Project Structure

```
VariableTiltHexacopterMATLAB/
├── src/                              # Core framework source code
│   └── +vt/                          # Main package namespace
│       ├── +config/                  # Configuration (Config.m)
│       ├── +core/                    # Logging (Logger.m)
│       ├── +ctrl/                    # Controllers
│       │   ├── WrenchController.m    # Wrench-level controller
│       │   ├── +adapt/              # Adaptation strategies
│       │   │   ├── AdaptationBase.m
│       │   │   ├── AdaptationFactory.m
│       │   │   ├── EuclideanAdaptation.m
│       │   │   ├── GeoAwareAdaptation.m
│       │   │   └── NoAdaptation.m
│       │   └── +potential/          # Potential functions
│       │       ├── PotentialBase.m
│       │       ├── PotentialFactory.m
│       │       ├── LieAlgebraPotential.m
│       │       └── SeparatePotential.m
│       ├── +metrics/                 # Tracking metrics (TrackingMetrics.m)
│       ├── +plant/                   # Hexacopter dynamics (HexacopterPlant.m)
│       ├── +plot/                    # Visualization
│       │   ├── Plotter.m
│       │   ├── UrdfViewer.m
│       │   └── defaultAxisLimits.m
│       ├── +se3/                     # SE(3) Lie group utilities
│       │   ├── Ad.m, Ad_inv.m        # Adjoint operators
│       │   ├── adV.m                 # Lie bracket
│       │   ├── expSE3.m, logSE3.m   # Exponential / logarithmic maps
│       │   ├── hat3.m, vee3.m       # so(3) isomorphisms
│       │   ├── hat6.m, vee6.m       # se(3) isomorphisms
│       │   └── invSE3.m             # SE(3) inverse
│       ├── +sim/                     # Simulation runner (SimRunner.m)
│       ├── +traj/                    # Trajectory generators
│       │   ├── TrajectoryBase.m
│       │   ├── TrajectoryFactory.m
│       │   ├── PreComputedTrajectory.m
│       │   └── ModelReferenceTrajectory.m
│       └── +utils/                   # Utility functions
│           ├── baseParams.m, addPayload.m
│           ├── rotm2rpy.m, rpy2rotm.m
│           ├── getGeneralizedInertia.m, inertiaFromParams.m
│           └── cleanNearZero.m
├── run/                              # Demo and CI entry-point scripts
│   ├── run_nominal_demo.m
│   ├── run_adaptive_demo.m
│   ├── run_nominal_paper_ICUAS.m
│   ├── run_adaptive_paper_ICUAS.m
│   ├── ci_nominal_release.m
│   ├── ci_adaptive_release.m
│   └── startup.m
├── assets/                           # Robot model assets
│   └── hexacopter_description/
│       └── urdf/
│           └── variable_tilt_hexacopter.urdf
├── results/                          # Simulation outputs (git-ignored)
├── .github/workflows/                # CI/CD pipeline
│   └── release-results.yml
├── startup.m                         # Environment path initialization
├── LICENSE                           # MIT License
└── README.md
```

## Simulation Outputs

### Single-Run Results

Results are saved automatically under `results/`, split by nominal vs adaptive runs:

```
results/
├── nominal/
│   └── yyyymmdd_HHMMSS_<traj>_<ctrl>_<potential>/
│       ├── sim_data.mat
│       ├── summary_nominal.png
│       ├── live_summary.png        # if live summary enabled
│       └── urdf_view.png           # if URDF view created
└── adaptive/
    └── yyyymmdd_HHMMSS_<traj>_<ctrl>_<potential>/
        ├── sim_data.mat
        ├── summary_adaptive.png
        ├── live_summary.png
        └── urdf_view.png
```

`sim_data.mat` contains `logs`, `metrics`, `runInfo`, and `cfg` (plus `est` for adaptive runs).

### Batch Results

Batch runs produce a nested directory structure:

```
results/adaptive/
└── yyyymmdd_HHMMSS_multi_traj/
    ├── adaptive_report.txt           # Aggregated batch summary
    ├── command_window.txt            # Aggregated console logs
    ├── t01_circle/
    │   ├── run_001/
    │   │   ├── command_window.txt
    │   │   ├── logs.mat
    │   │   ├── metrics.txt
    │   │   └── figures/              # PNG plots
    │   ├── run_002/
    │   └── ...
    ├── t02_inf3d/
    ├── t03_liss3d/
    └── ...
```

### Summary Plots

`sim.plot('summary')` generates a single summary figure:

- **3D Path**: desired vs actual
- **XY Path**: top-down view
- **Altitude**: z vs time
- **Position + Orientation**: stacked time-series
- **Linear + Angular Velocity**: stacked time-series
- **Force + Torque (Wrench)**: stacked time-series

For adaptive runs, the summary also includes:

- **Mass + CoG Estimates**
- **Principal Inertia Estimates**
- **Off-Diagonal Inertia Estimates**

`sim.plot('all')` additionally saves standalone and stacked figures (PNG) into the same results folder.

## CI/CD: Release Automation

The repository includes a GitHub Actions workflow (`.github/workflows/release-results.yml`) that automatically runs simulations and publishes results when you push a version tag.

**Tag patterns:**

| Tag | Runs |
|-----|------|
| `vX.X.X` | Both nominal and adaptive simulations |
| `vX.X.X-nom` | Nominal simulations only |
| `vX.X.X-adapt` | Adaptive simulations only |

**Pipeline steps:**
1. Checks out repository and sets up MATLAB
2. Runs `ci_nominal_release` and/or `ci_adaptive_release` (headless, off-screen rendering)
3. Aggregates command-window logs into release notes
4. Packages results as a `.zip` and creates a GitHub Release

## Customization

### Adding New Controllers

1. Add a new controller type string to `src/+vt/+config/Config.m` validation in `setController(...)`
2. Implement the new control law in `src/+vt/+ctrl/WrenchController.m` (add a new `case`)
3. Run a demo and save with `sim.plot('summary')`

### Custom Trajectories

1. Create a new trajectory class under `src/+vt/+traj/` (subclass `vt.traj.TrajectoryBase`)
2. Register it in `src/+vt/+traj/TrajectoryFactory.m`
3. Add the name to `src/+vt/+config/Config.m` validation in `setTrajectory(...)`

### Visualization Customization

```matlab
cfg.enableLiveView(true);
cfg.setLiveSummary(true);
cfg.setLiveUpdateRate(200);           % update every N control steps
cfg.setLiveUrdfEmbedding(true);       % reserve an axes slot for URDF viewer
cfg.setPlotLayout('row-major');       % 'row-major' | 'column-major'

% Optional (advanced): tweak axis behavior directly
cfg.viz.dynamicAxis = true;
cfg.viz.axisPadding = 2.0;
```

## Troubleshooting

### Common Issues

#### MATLAB Path Issues

```matlab
% Ensure all paths are added
startup

% Verify package structure
which vt.sim.SimRunner
```

#### Visualization Problems

- **URDF Not Found**: Fallback to stick model automatically
- **Graphics Performance**: Increase live update interval, disable URDF embedding, or disable live view
- **3D Rendering Issues**: Check MATLAB graphics drivers

#### Simulation Stability

- **Large Time Steps**: Reduce timestep for stiff systems
- **Numerical Issues**: Check initial conditions and parameter values
- **Integration Errors**: Verify ODE solver settings

### Performance Optimization

```matlab
% Optimize for speed
cfg.setSimParams(0.02, 15);           % Larger timestep, shorter duration
cfg.enableLiveView(false);            % Disable live visualization

% Or keep live view but reduce update load
cfg.setLiveUrdfEmbedding(false);
cfg.setLiveUpdateRate(500);
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please follow these guidelines:

1. **Fork** the repository
2. **Create** a feature branch
3. **Document** changes in the README
4. **Submit** a pull request

### Development Setup

```matlab
% Add repo paths
startup

% Run a demo
run_nominal_demo
```

## References

Relevant references are cited in the associated papers. Please refer to the papers for detailed bibliographic information.

## Support

For questions and support:

- **Issues**: [GitHub Issues](https://github.com/olanrewajufarooq/VariableTiltHexacopterMATLAB/issues)
- **Discussions**: [GitHub Discussions](https://github.com/olanrewajufarooq/VariableTiltHexacopterMATLAB/discussions)
- **Email**: <g202404900@kfupm.edu.sa>, <olanrewajufarooq@yahoo.com>

## Version History

### v1.1.0 (Current)

- Batch simulation support (multi-trajectory, multi-gain sweeps)
- Ground-origin start for non-hover trajectories
- Trajectory hover-start overrides
- Per-run results persistence with aggregated reports
- CI/CD release automation via GitHub Actions

### v1.0.0

- Initial stable release
- Complete simulation framework
- Multiple controller options
- Advanced trajectory generation
- Parameter adaptation capabilities

### Planned Features

- [ ] Geometric-Aware Adaptive Control
- [ ] Sensor Noise Modeling
- [ ] State Estimation Integration

## Citations

If you use this framework in your research, please cite one of our papers:

### Paper 1: Indirect Geometric Adaptive Control on SE(3) for a Hexacopter With Online Generalized Inertia and CoG Estimation

```bibtex
@misc{olanrewaju_indirect_geometric_adaptive_control,
  title  = {Indirect Geometric Adaptive Control on SE(3) for a Hexacopter With Online Generalized Inertia and CoG Estimation},
  author = {Farooq Olanrewaju and Aymen Benyahia and Ramy Rashad and Sami El-Ferik},
  note   = {Please replace this entry with the final publication details (venue/year/DOI) once available.}
}
```

### Paper 2: From Modeling to Control Allocation: A Geometric Approach to Variable-Tilt Hexacopter

```bibtex
@misc{olanrewaju_modeling_to_control_allocation_variable_tilt_hexacopter,
  title  = {From Modeling to Control Allocation: A Geometric Approach to Variable-Tilt Hexacopter},
  author = {Farooq Olanrewaju and Ziad Shoeib and Ahmed Abdelrazeq and MD Tarique bin Hamid and Ramy Rashad},
  note   = {Please replace this entry with the final publication details (venue/year/DOI) once available.}
}
```

**Note**: The first paper focuses on adaptive control with online parameter estimation, while the second paper covers the complete modeling and control allocation approach for variable-tilt hexacopters.
